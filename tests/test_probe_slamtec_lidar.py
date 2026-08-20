import pytest

from scripts.probe_slamtec_lidar import (
    DEVICE_INFO_RESPONSE_TYPE,
    parse_device_info,
    parse_response_descriptor,
)


def test_parse_device_info_response() -> None:
    serial = bytes.fromhex("00112233445566778899AABBCCDDEEFF")
    info = parse_device_info(bytes((0x81, 0x07, 0x02, 0x06)) + serial)

    assert info.model_id == 0x81
    assert info.firmware_major == 2
    assert info.firmware_minor == 7
    assert info.hardware_revision == 6
    assert info.serial_number == "00112233445566778899AABBCCDDEEFF"


def test_parse_single_send_descriptor() -> None:
    descriptor = bytes.fromhex("A55A1400000004")

    assert parse_response_descriptor(descriptor) == (
        20,
        0,
        DEVICE_INFO_RESPONSE_TYPE,
    )


@pytest.mark.parametrize(
    "descriptor",
    [b"", bytes.fromhex("A55A14000000"), bytes.fromhex("00001400000004")],
)
def test_rejects_invalid_descriptor(descriptor: bytes) -> None:
    with pytest.raises(ValueError):
        parse_response_descriptor(descriptor)


def test_rejects_invalid_device_info_payload_size() -> None:
    with pytest.raises(ValueError, match="20 bytes"):
        parse_device_info(b"short")
