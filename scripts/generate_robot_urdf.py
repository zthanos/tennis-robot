"""Generate the tennis robot URDF from its xacro source."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = PROJECT_ROOT / "ros2_ws" / "src" / "tennis_robot" / "urdf" / "tennis_robot.urdf.xacro"
DEFAULT_OUTPUT = PROJECT_ROOT / "runtime" / "tennis_robot.urdf"
DEFAULT_CONTROLLERS = (
    PROJECT_ROOT / "ros2_ws" / "src" / "tennis_robot" / "config" / "controllers.yaml"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--sim-mode",
        default="true",
        choices=["true", "false"],
        help="ros2_control hardware backend: true=GazeboSimSystem, false=real robot.",
    )
    parser.add_argument(
        "--controllers-config",
        type=Path,
        default=DEFAULT_CONTROLLERS,
        help="Absolute path to controllers.yaml baked into the gz_ros2_control plugin.",
    )
    parser.add_argument(
        "--xacro-command",
        default="xacro",
        help="xacro executable to use; defaults to PATH lookup.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Fail if the output URDF is missing or does not match the rendered xacro.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = args.source.resolve()
    output = args.output.resolve()

    if not source.exists():
        print(f"Source xacro not found: {source}", file=sys.stderr)
        return 2

    xacro_exe = shutil.which(args.xacro_command)
    if xacro_exe is None:
        print(
            "xacro executable not found. Install ros-humble-xacro or source a ROS "
            "environment that provides xacro.",
            file=sys.stderr,
        )
        return 2

    env = os.environ.copy()
    env.setdefault("PYTHONUTF8", "1")
    controllers_config = args.controllers_config.resolve()
    result = subprocess.run(
        [
            xacro_exe,
            str(source),
            f"sim_mode:={args.sim_mode}",
            f"controllers_config:={controllers_config}",
        ],
        check=False,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr, end="")
        return result.returncode

    ET.fromstring(result.stdout)
    if args.check:
        if not output.exists():
            print(f"Generated URDF is missing: {output}", file=sys.stderr)
            return 1
        current = output.read_text(encoding="utf-8")
        if current != result.stdout:
            print(
                f"Generated URDF is stale: {output.relative_to(PROJECT_ROOT)}",
                file=sys.stderr,
            )
            return 1
        print(f"Generated URDF is current: {output.relative_to(PROJECT_ROOT)}")
        return 0

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(result.stdout, encoding="utf-8")
    print(f"Generated {output.relative_to(PROJECT_ROOT)} from {source.relative_to(PROJECT_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
