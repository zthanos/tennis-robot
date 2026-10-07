from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MOTION_SKETCH = ROOT / "arduino/motion/motion_mega/motion_mega.ino"
INTAKE_SKETCH = (
    ROOT
    / "arduino/collector/07_dual_intake_mega_bench/07_dual_intake_mega_bench.ino"
)
UNIFIED_SKETCH = (
    ROOT / "arduino/motion/motion_intake_mega/motion_intake_mega.ino"
)
GYRO_SKETCH = (
    ROOT / "arduino/motion/08_mpu6050_mega_bench/08_mpu6050_mega_bench.ino"
)


def _pin_value(token: str) -> int:
    if token.startswith("A"):
        # Arduino Mega analog aliases: A0=D54, ..., A15=D69.
        return 54 + int(token[1:])
    return int(token)


def _assigned_pins(source: str, stop_marker: str) -> dict[str, int]:
    pin_section = source.split(stop_marker, 1)[0]
    assignments = re.findall(
        r"(?:const\s+uint8_t|,)\s*([A-Z][A-Z0-9_]*)\s*=\s*(A\d+|\d+)",
        pin_section,
    )
    return {name: _pin_value(token) for name, token in assignments}


def test_intake_reservations_do_not_overlap_drive_motion_pins() -> None:
    motion = _assigned_pins(
        MOTION_SKETCH.read_text(encoding="utf-8"), "// ---- Tuning"
    )
    intake = _assigned_pins(
        INTAKE_SKETCH.read_text(encoding="utf-8"), "const unsigned long BAUD"
    )

    # The E-stop status and armed LED are deliberately shared system safety
    # nets. Every motor, encoder and IR signal must otherwise be unique.
    shared_safety_names = {"ESTOP_STATUS_PIN", "ARMED_LED_PIN"}
    intake_functional = {
        name: pin for name, pin in intake.items() if name not in shared_safety_names
    }

    overlap = set(motion.values()) & set(intake_functional.values())
    assert overlap == set()
    assert len(intake_functional.values()) == len(set(intake_functional.values()))


def test_intake_pin_contract_keeps_i2c_and_spi_available() -> None:
    intake = _assigned_pins(
        INTAKE_SKETCH.read_text(encoding="utf-8"), "const unsigned long BAUD"
    )
    used = set(intake.values())

    assert not used.intersection({20, 21})  # I2C MPU6050
    assert not used.intersection({50, 51, 52, 53})  # SPI expansion
    assert intake["LEFT_RPWM_PIN"] == 44
    assert intake["LEFT_LPWM_PIN"] == 45
    assert intake["RIGHT_RPWM_PIN"] == 46
    assert intake["RIGHT_LPWM_PIN"] == 11
    assert intake["LEFT_EN_PIN"] == 40
    assert intake["RIGHT_EN_PIN"] == 42
    assert intake["IR_ENTRY_PIN"] == 36
    assert intake["IR_EXIT_PIN"] == 37


def test_unified_sketch_matches_common_perfboard_pin_contract() -> None:
    source = UNIFIED_SKETCH.read_text(encoding="utf-8")
    pins = _assigned_pins(source, "const unsigned long BAUD")

    expected = {
        "DRIVE_LEFT_RPWM_PIN": 5,
        "DRIVE_LEFT_LPWM_PIN": 6,
        "DRIVE_LEFT_EN_PIN": 30,
        "DRIVE_RIGHT_RPWM_PIN": 9,
        "DRIVE_RIGHT_LPWM_PIN": 10,
        "DRIVE_RIGHT_EN_PIN": 31,
        "DRIVE_LF_ENC_A_PIN": 2,
        "DRIVE_LF_ENC_B_PIN": 22,
        "DRIVE_LR_ENC_A_PIN": 3,
        "DRIVE_LR_ENC_B_PIN": 23,
        "DRIVE_RF_ENC_A_PIN": 18,
        "DRIVE_RF_ENC_B_PIN": 24,
        "DRIVE_RR_ENC_A_PIN": 19,
        "DRIVE_RR_ENC_B_PIN": 25,
        "INTAKE_LEFT_RPWM_PIN": 44,
        "INTAKE_LEFT_LPWM_PIN": 45,
        "INTAKE_LEFT_EN_PIN": 40,
        "INTAKE_RIGHT_RPWM_PIN": 46,
        "INTAKE_RIGHT_LPWM_PIN": 11,
        "INTAKE_RIGHT_EN_PIN": 42,
        # As-built: left encoder on A10/A11, right on A8/A9.
        "INTAKE_LEFT_ENC_A_PIN": 64,
        "INTAKE_LEFT_ENC_B_PIN": 65,
        "INTAKE_RIGHT_ENC_A_PIN": 62,
        "INTAKE_RIGHT_ENC_B_PIN": 63,
        "IR_ENTRY_PIN": 36,
        "IR_EXIT_PIN": 37,
        "START_ARM_PIN": 32,
        "ESTOP_STATUS_PIN": 33,
        "ARMED_LED_PIN": 34,
        "IMU_SDA_PIN": 20,
        "IMU_SCL_PIN": 21,
    }

    assert pins == expected
    assert len(pins.values()) == len(set(pins.values()))
    assert "ISR(PCINT2_vect)" in source
    assert "#include <Wire.h>" in source
    assert "HOST_TIMEOUT_MS = 300" in source
    assert "MAX_INTAKE_TEST_PWM = 90" in source


def test_gyro_bench_sketch_uses_mega_i2c_and_probes_both_addresses() -> None:
    source = GYRO_SKETCH.read_text(encoding="utf-8")

    assert "#include <Wire.h>" in source
    assert "probe(0x68) || probe(0x69)" in source
    assert "Wire.setClock(100000UL)" in source
    assert "MPU6050_NOT_FOUND" in source
