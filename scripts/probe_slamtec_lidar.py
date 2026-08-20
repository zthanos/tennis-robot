#!/usr/bin/env python3
"""Read Slamtec device information without starting the LiDAR scan or motor."""

from __future__ import annotations

import argparse
import json
import struct
from dataclasses import asdict, dataclass


SYNC_BYTE = 0xA5
SYNC_BYTE_2 = 0x5A
GET_DEVICE_INFO = bytes((SYNC_BYTE, 0x50))
DEVICE_INFO_RESPONSE_TYPE = 0x04
DEVICE_INFO_PAYLOAD_SIZE = 20


@dataclass(frozen=True)
class DeviceInfo:
    model_id: int
    firmware_major: int
    firmware_minor: int
    hardware_revision: int
    serial_number: str


def parse_response_descriptor(data: bytes) -> tuple[int, int, int]:
    """Return payload size, send mode, and response type."""

    if len(data) != 7:
        raise ValueError(f"response descriptor must be 7 bytes, got {len(data)}")
    if data[:2] != bytes((SYNC_BYTE, SYNC_BYTE_2)):
        raise ValueError(f"invalid response sync bytes: {data[:2].hex()}")
    size_and_mode = struct.unpack_from("<I", data, 2)[0]
    return size_and_mode & 0x3FFFFFFF, size_and_mode >> 30, data[6]


def parse_device_info(payload: bytes) -> DeviceInfo:
    if len(payload) != DEVICE_INFO_PAYLOAD_SIZE:
        raise ValueError(
            f"device-info payload must be {DEVICE_INFO_PAYLOAD_SIZE} bytes, "
            f"got {len(payload)}"
        )
    return DeviceInfo(
        model_id=payload[0],
        firmware_minor=payload[1],
        firmware_major=payload[2],
        hardware_revision=payload[3],
        serial_number=payload[4:].hex().upper(),
    )


def read_exact(serial_port: object, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = serial_port.read(size - len(chunks))
        if not chunk:
            raise TimeoutError(
                f"serial response timed out after {len(chunks)}/{size} bytes"
            )
        chunks.extend(chunk)
    return bytes(chunks)


def query_device_info(port: str, baudrate: int, timeout_s: float) -> DeviceInfo:
    try:
        import serial
    except ImportError as exc:  # pragma: no cover - depends on target image
        raise RuntimeError("pyserial is required: install python3-serial") from exc

    # GET_DEVICE_INFO is a read-only protocol request. It does not issue the
    # start-scan command and does not toggle DTR to start legacy LiDAR motors.
    with serial.Serial(
        port=port,
        baudrate=baudrate,
        timeout=timeout_s,
        write_timeout=timeout_s,
        exclusive=True,
        dsrdtr=False,
        rtscts=False,
    ) as connection:
        connection.reset_input_buffer()
        connection.write(GET_DEVICE_INFO)
        connection.flush()
        descriptor = read_exact(connection, 7)
        payload_size, send_mode, response_type = parse_response_descriptor(descriptor)
        if response_type != DEVICE_INFO_RESPONSE_TYPE:
            raise ValueError(f"unexpected response type: 0x{response_type:02X}")
        if send_mode != 0:
            raise ValueError(f"device-info response must be single-send, got {send_mode}")
        if payload_size != DEVICE_INFO_PAYLOAD_SIZE:
            raise ValueError(f"unexpected device-info payload size: {payload_size}")
        return parse_device_info(read_exact(connection, payload_size))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "port", help="enumerated serial path, preferably /dev/serial/by-id/..."
    )
    parser.add_argument(
        "--baudrate", type=int, required=True, help="documented model baud rate"
    )
    parser.add_argument(
        "--timeout", type=float, default=2.0, help="serial timeout in seconds"
    )
    args = parser.parse_args()
    if args.baudrate <= 0:
        parser.error("--baudrate must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    result = asdict(query_device_info(args.port, args.baudrate, args.timeout))
    result.update({"port": args.port, "baudrate": args.baudrate})
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
