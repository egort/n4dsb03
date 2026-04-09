from __future__ import annotations

import argparse

import serial

from modbus_rtu_n4dsb03_scanner import _crc16_modbus, _read_registers


def build_write_single_frame(slave: int, address: int, value: int) -> bytes:
    payload = bytes(
        [
            slave & 0xFF,
            0x06,
            (address >> 8) & 0xFF,
            address & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ]
    )
    crc = _crc16_modbus(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def write_single_register(ser: serial.Serial, slave: int, address: int, value: int) -> tuple[int, int] | None:
    frame = build_write_single_frame(slave=slave, address=address, value=value)
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()

    reply = b""
    while len(reply) < 8:
        chunk = ser.read(8 - len(reply))
        if not chunk:
            break
        reply += chunk

    if len(reply) != 8:
        return None

    body = reply[:-2]
    recv_crc = int.from_bytes(reply[-2:], "little")
    if _crc16_modbus(body) != recv_crc:
        return None

    if reply[0] != slave or reply[1] != 0x06:
        return None

    r_addr = (reply[2] << 8) | reply[3]
    r_value = (reply[4] << 8) | reply[5]
    return r_addr, r_value


def read_holding(ser: serial.Serial, slave: int, address: int) -> int | None:
    regs = _read_registers(ser, slave=slave, fc=3, start_address=address, quantity=1)
    if not regs:
        return None
    return regs[0]


def main() -> int:
    p = argparse.ArgumentParser(description="Write one Modbus RTU holding register")
    p.add_argument("--port", default="COM10")
    p.add_argument("--baudrate", type=int, default=9600)
    p.add_argument("--parity", default="N")
    p.add_argument("--bytesize", type=int, default=8)
    p.add_argument("--stopbits", type=int, default=1)
    p.add_argument("--timeout", type=float, default=0.12)
    p.add_argument("--slave", type=int, required=True)
    p.add_argument("--address", type=int, required=True)
    p.add_argument("--value", type=int, required=True)
    args = p.parse_args()

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baudrate,
        bytesize=args.bytesize,
        parity=args.parity,
        stopbits=args.stopbits,
        timeout=args.timeout,
        write_timeout=max(0.2, args.timeout),
    )
    try:
        result = write_single_register(ser, args.slave, args.address, args.value)
        if result is None:
            print("WRITE_FAILED")
            return 1
        r_addr, r_value = result
        print(f"WRITE_OK addr={r_addr} value={r_value}")
        verify = read_holding(ser, args.slave, args.address)
        print(f"READBACK {verify}")
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
