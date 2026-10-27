from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/run_intake_ir_cycle.py"
SKETCH = (
    ROOT
    / "arduino/collector/07_dual_intake_mega_bench/07_dual_intake_mega_bench.ino"
)


def _load_script():
    spec = importlib.util.spec_from_file_location("run_intake_ir_cycle", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_data_parser_reads_ir_and_state_fields() -> None:
    module = _load_script()
    parsed = module.parse_data_line("DATA,100,2,60,-60,1,-1,20.0,-20.0,1,0")
    assert parsed == module.IntakeTelemetry(
        mega_ms=100,
        state=2,
        left_pwm=60,
        right_pwm=-60,
        left_rpm=20.0,
        right_rpm=-20.0,
        entry_broken=True,
        exit_broken=False,
    )
    assert module.parse_data_line("EVENT,100,RUN_STARTED") is None
    assert module.parse_data_line("DATA,broken") is None


def test_pi_and_mega_share_four_second_exit_stop_contract() -> None:
    module = _load_script()
    source = SKETCH.read_text(encoding="utf-8")
    assert module.MAX_TIMEOUT_S == 4.0
    assert module.EXPECTED_FEATURE in source
    assert "RUN_AUTO_BENCH" in source
    assert "runMode == RUN_AUTO_BENCH ? RUN_AUTO_BENCH : RUN_AUTO_BALL" in source
    assert "if (runMode == RUN_AUTO_BENCH) return;" in source
    assert 'finishRun(F("BALL_EXIT"));' in source
    assert 'F("BALL_TIMEOUT")' in source
    assert "const int8_t LEFT_INWARD_SIGN = -1;" in source
    assert "const int8_t RIGHT_INWARD_SIGN = 1;" in source
