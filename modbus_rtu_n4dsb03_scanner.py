from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Iterable, Sequence

import serial


DEFAULT_PRIORITY_SLAVES = (1, 2, 3, 4, 5, 8, 10, 16, 32, 64)
DEFAULT_START_ADDRESSES = (0, 1)
DEFAULT_QUANTITY = 10


@dataclass
class ProbeResult:
    slave: int
    fc: int
    start_address: int
    regs: list[int]


@dataclass
class TempCandidate:
    slave: int
    fc: int
    address: int
    raw: int
    temperature_c: float


def _format_bits16(value: int) -> str:
    return format(value & 0xFFFF, "016b")


def _parse_csv_ints(value: str) -> list[int]:
    if not value.strip():
        return []
    return [int(x.strip()) for x in value.split(",") if x.strip()]


def _parse_fc_list(value: str) -> list[int]:
    out = _parse_csv_ints(value)
    allowed = {3, 4}
    filtered = [fc for fc in out if fc in allowed]
    return filtered if filtered else [4, 3]


def _crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def _build_read_frame(slave: int, fc: int, start_address: int, quantity: int) -> bytes:
    payload = bytes(
        [
            slave & 0xFF,
            fc & 0xFF,
            (start_address >> 8) & 0xFF,
            start_address & 0xFF,
            (quantity >> 8) & 0xFF,
            quantity & 0xFF,
        ]
    )
    crc = _crc16_modbus(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def _read_exact_or_timeout(ser: serial.Serial, n: int) -> bytes:
    data = b""
    while len(data) < n:
        chunk = ser.read(n - len(data))
        if not chunk:
            break
        data += chunk
    return data


def _read_registers(
    ser: serial.Serial,
    slave: int,
    fc: int,
    start_address: int,
    quantity: int,
) -> list[int] | None:
    frame = _build_read_frame(slave=slave, fc=fc, start_address=start_address, quantity=quantity)
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()

    # Normal response: slave, fc, byte_count, data..., crc_lo, crc_hi
    header = _read_exact_or_timeout(ser, 3)
    if len(header) != 3:
        return None

    r_slave, r_fc, third = header[0], header[1], header[2]
    if r_slave != slave:
        return None

    if r_fc == (fc | 0x80):
        # Exception response: one-byte exception code + CRC
        tail = _read_exact_or_timeout(ser, 3)
        if len(tail) != 3:
            return None
        packet = header + tail
        body, crc_recv = packet[:-2], int.from_bytes(packet[-2:], "little")
        if _crc16_modbus(body) != crc_recv:
            return None
        return None

    if r_fc != fc:
        return None

    byte_count = third
    tail = _read_exact_or_timeout(ser, byte_count + 2)
    if len(tail) != byte_count + 2:
        return None

    packet = header + tail
    body, crc_recv = packet[:-2], int.from_bytes(packet[-2:], "little")
    if _crc16_modbus(body) != crc_recv:
        return None

    if byte_count % 2 != 0:
        return None

    regs: list[int] = []
    raw = packet[3 : 3 + byte_count]
    for i in range(0, len(raw), 2):
        regs.append((raw[i] << 8) | raw[i + 1])
    return regs


def _s16(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


def _decode_temp_tenths(raw: int) -> float:
    return _s16(raw) / 10.0


def dump_register_block(probe: ProbeResult, hide_zero: bool = False) -> None:
    print("\nRegister dump:")
    print("  addr   raw    hex   int16   temp(0.1C)  bits")
    for i, raw in enumerate(probe.regs):
        addr = probe.start_address + i
        signed = _s16(raw)
        temp01 = signed / 10.0
        if hide_zero and raw == 0:
            continue
        print(
            f"  {addr:5d} {raw:5d} 0x{raw:04X} {signed:7d} {temp01:10.1f} {_format_bits16(raw)}"
        )


def scan_slaves(
    ser: serial.Serial,
    slave_range: Iterable[int],
    start_addresses: Sequence[int],
    quantity: int,
    functions: Sequence[int],
) -> list[ProbeResult]:
    found: list[ProbeResult] = []

    for slave in slave_range:
        for fc in functions:
            for start in start_addresses:
                regs = _read_registers(
                    ser,
                    slave=slave,
                    fc=fc,
                    start_address=start,
                    quantity=quantity,
                )
                if regs is None:
                    continue
                found.append(ProbeResult(slave=slave, fc=fc, start_address=start, regs=regs))
                break
            if found and found[-1].slave == slave:
                break
    return found


def build_scan_order(scan_from: int, scan_to: int, quick: bool) -> list[int]:
    all_slaves = list(range(scan_from, scan_to + 1))
    if not quick:
        return all_slaves

    ordered: list[int] = []
    in_range_priority = [s for s in DEFAULT_PRIORITY_SLAVES if scan_from <= s <= scan_to]
    ordered.extend(in_range_priority)
    ordered.extend(s for s in all_slaves if s not in in_range_priority)
    return ordered


def read_temperature_candidates(
    probe: ProbeResult,
    include_zero: bool = False,
) -> list[TempCandidate]:
    # N4DSB03 usually returns temperature as signed int16 in 0.1 C.
    # We treat each received register as potential temperature and filter by DS18B20 range.
    out: list[TempCandidate] = []
    for i, raw in enumerate(probe.regs):
        t = _decode_temp_tenths(raw)
        if not include_zero and raw == 0:
            continue
        if -55.0 <= t <= 125.0:
            out.append(
                TempCandidate(
                    slave=probe.slave,
                    fc=probe.fc,
                    address=probe.start_address + i,
                    raw=raw,
                    temperature_c=t,
                )
            )
    return out


def run(args: argparse.Namespace) -> int:
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            bytesize=args.bytesize,
            parity=args.parity,
            stopbits=args.stopbits,
            timeout=args.timeout,
            write_timeout=max(0.2, args.timeout),
        )
    except serial.SerialException as exc:
        print(f"ERROR: failed to open {args.port}: {exc}")
        return 2

    try:
        scan_from = args.scan_from
        scan_to = args.scan_to
        if scan_from < 1 or scan_to > 247 or scan_from > scan_to:
            print("ERROR: slave range must be within 1..247 and scan_from <= scan_to")
            return 2

        scan_order = build_scan_order(scan_from, scan_to, args.quick)
        probe_results = scan_slaves(
            ser,
            slave_range=scan_order,
            start_addresses=args.start_addresses,
            quantity=args.quantity,
            functions=args.functions,
        )

        if not probe_results:
            print("No Modbus RTU slaves found with current probe set.")
            return 1

        print("Found slave candidates:")
        for item in probe_results:
            preview = ", ".join(str(v) for v in item.regs[:4])
            print(f"  slave={item.slave:3d} via FC{item.fc} start={item.start_address} regs=[{preview} ...]")

        target_slave = args.slave if args.slave else probe_results[0].slave
        target_probe = next((p for p in probe_results if p.slave == target_slave), None)
        if target_probe is None:
            print(f"No response block found for requested slave {target_slave}")
            return 1

        print(f"\nUsing slave {target_slave} for D1/temperature probing")

        if args.dump:
            dump_register_block(target_probe, hide_zero=args.hide_zero)

        d1_candidates = read_temperature_candidates(
            target_probe,
            include_zero=args.include_zero_temp,
        )

        if not d1_candidates:
            print("No plausible temperature values decoded from the tested D1 registers.")
            print("Try --start-addresses 0,1 and --quantity 20 if your firmware maps more fields.")
            return 1

        print("\nTemperature candidates (signed int16 in 0.1 C, range -55..125 C):")
        for c in d1_candidates:
            print(
                f"  slave={c.slave:3d} FC{c.fc} reg={c.address:5d} raw={c.raw:6d} -> {c.temperature_c:7.1f} C"
            )

        if d1_candidates:
            best = d1_candidates[0]
            print(
                f"\nBest guess for D1: slave={best.slave} FC{best.fc} reg={best.address} temp={best.temperature_c:.1f} C"
            )

        return 0
    finally:
        ser.close()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Modbus RTU scanner for N4DSB03-like modules with DS18B20 temperature probing"
    )
    p.add_argument("--port", default="COM10")
    p.add_argument("--baudrate", type=int, default=9600)
    p.add_argument("--parity", default="N")
    p.add_argument("--stopbits", type=int, default=1)
    p.add_argument("--bytesize", type=int, default=8)
    p.add_argument("--timeout", type=float, default=0.25)
    p.add_argument(
        "--quick",
        action="store_true",
        help="Try common slave IDs first, then continue sequentially",
    )

    p.add_argument("--scan-from", type=int, default=1)
    p.add_argument("--scan-to", type=int, default=247)
    p.add_argument(
        "--start-addresses",
        type=_parse_csv_ints,
        default=list(DEFAULT_START_ADDRESSES),
        help="Comma-separated start addresses to probe (usually 0,1)",
    )
    p.add_argument("--quantity", type=int, default=DEFAULT_QUANTITY)
    p.add_argument(
        "--functions",
        type=_parse_fc_list,
        default=[4, 3],
        help="Comma-separated function codes to try (3,4)",
    )
    p.add_argument(
        "--dump",
        action="store_true",
        help="Print detailed table for all received registers",
    )
    p.add_argument(
        "--hide-zero",
        action="store_true",
        help="Hide rows where raw register value equals 0 in dump table",
    )
    p.add_argument(
        "--include-zero-temp",
        action="store_true",
        help="Include raw=0 values in temperature candidates",
    )

    p.add_argument(
        "--slave",
        type=int,
        default=None,
        help="If known, skip auto-pick and use this slave address for D1 probing",
    )
    return p


if __name__ == "__main__":
    parser = build_parser()
    raise SystemExit(run(parser.parse_args()))
