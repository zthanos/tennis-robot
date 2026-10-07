#!/usr/bin/env python3
"""Generate and execute one deterministic native Gazebo flywheel capability case."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
BENCH_XACRO = ROOT / "ros2_ws/src/tennis_robot/urdf/flywheel_launcher_bench.urdf.xacro"
CONTROLLERS = ROOT / "ros2_ws/src/tennis_robot/config/flywheel_launcher_bench_controllers.yaml"
BASE_WORLD = ROOT / "gazebo/worlds/flywheel_launcher_geometry_bench.sdf"
BALL_MODEL = ROOT / "gazebo/models/tennis_ball_compliant/model.sdf"
DEFAULT_BUILD = Path("/tmp/tennis_ball_contact_system_capability_build")


def local_to_world(local_x: float, local_y: float, local_z: float) -> tuple[float, float, float]:
    pitch = -math.radians(20.0)
    return (
        math.cos(pitch) * local_x + math.sin(pitch) * local_z,
        local_y,
        0.350 - math.sin(pitch) * local_x + math.cos(pitch) * local_z,
    )


def render_bench_model(temp_dir: Path) -> ET.Element:
    urdf = temp_dir / "flywheel_launcher_bench.urdf"
    subprocess.run(
        [
            "xacro", "-o", str(urdf), str(BENCH_XACRO),
            f"controllers_config:={CONTROLLERS}",
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    converted = subprocess.run(
        ["gz", "sdf", "-p", str(urdf)],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    model = ET.fromstring(converted.stdout).find("model")
    if model is None:
        raise RuntimeError("gz sdf conversion did not produce a bench model")
    for plugin in list(model.findall("plugin")):
        if "gz_ros2_control" in plugin.attrib.get("filename", ""):
            model.remove(plugin)
    # URDF conversion produces a dynamic root link. Anchor only that datum to
    # the world; marking the whole model static would also freeze the wheels.
    anchor = ET.SubElement(model, "joint", {"name": "bench_world_anchor", "type": "fixed"})
    ET.SubElement(anchor, "parent").text = "world"
    ET.SubElement(anchor, "child").text = "launcher_bench_datum_link"
    return model


def set_rotating_properties(model: ET.Element, wheel_mass: float, spin_inertia: float) -> None:
    hub_mass = 0.02676364627607291
    total_mass = wheel_mass + hub_mass
    transverse = 0.5 * spin_inertia + total_mass * 0.050**2 / 12.0
    for side in ("left", "right"):
        link = model.find(f"./link[@name='flywheel_{side}_link']")
        if link is None:
            raise RuntimeError(f"missing flywheel_{side}_link after conversion")
        link.find("./inertial/mass").text = f"{total_mass:.17g}"
        inertia = link.find("./inertial/inertia")
        inertia.find("ixx").text = f"{transverse:.17g}"
        inertia.find("iyy").text = f"{transverse:.17g}"
        inertia.find("izz").text = f"{spin_inertia:.17g}"
        # The calibrated analytical contact plugin owns tyre contact. Remove
        # only that native cylinder before physics initialization; keep the
        # shaft, hub collar, pilot and retainer as physical collision geometry.
        tyre_token = f"flywheel_{side}_col_collision"
        for collision in list(link.findall("collision")):
            if tyre_token in collision.attrib.get("name", ""):
                link.remove(collision)


def build_world(args: argparse.Namespace, output_dir: Path, temp_dir: Path) -> Path:
    tree = ET.parse(BASE_WORLD)
    world = tree.getroot().find("world")
    physics = world.find("physics")
    physics.find("max_step_size").text = f"{args.timestep:.17g}"
    physics.find("real_time_factor").text = "0"

    bench = render_bench_model(temp_dir)
    set_rotating_properties(bench, args.wheel_mass, args.spin_inertia)
    world.append(bench)

    ball = ET.parse(BALL_MODEL).getroot().find("model")
    contact_plugin = ball.find("plugin")
    ET.SubElement(contact_plugin, "telemetry_csv").text = str(output_dir / "contacts.csv")
    if args.friction is not None:
        ET.SubElement(contact_plugin, "friction_coefficient").text = f"{args.friction:.17g}"

    hold = local_to_world(-0.080, 0.0, 0.0)
    ball.find("pose").text = " ".join(f"{value:.17g}" for value in (*hold, 0.0, 0.0, 0.0))
    injection_direction = local_to_world(1.0, 0.0, 0.0)
    injection_velocity = (
        injection_direction[0],
        injection_direction[1],
        injection_direction[2] - 0.350,
    )
    capability = ET.SubElement(
        ball,
        "plugin",
        {
            "filename": "flywheel_capability_control_system",
            "name": "tennis_ball_contact_system::FlywheelCapabilityControlSystem",
        },
    )
    values = {
        "ball_link": "ball_link",
        "left_wheel_link": "flywheel_left_link",
        "right_wheel_link": "flywheel_right_link",
        "left_target_rad_s": f"{args.left_target:.17g}",
        "right_target_rad_s": f"{args.right_target:.17g}",
        "effort_limit_nm": "0.62",
        "speed_kp_nm_per_rad_s": "0.05",
        "joint_damping_nm_s_rad": "0.002",
        "minimum_injection_time_s": f"{args.minimum_injection_time:.17g}",
        "settle_tolerance_rad_s": "1.0",
        "settle_duration_s": "0.2",
        "hold_position_world": " ".join(f"{value:.17g}" for value in hold),
        "injection_velocity_world": " ".join(
            f"{args.injection_speed * value:.17g}" for value in injection_velocity
        ),
        "state_csv": str(output_dir / "state.csv"),
        "state_topic": "/flywheel/capability_state",
    }
    for name, value in values.items():
        ET.SubElement(capability, name).text = value
    world.append(ball)

    output = output_dir / "world.sdf"
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--left-target", type=float, required=True)
    parser.add_argument("--right-target", type=float, required=True)
    parser.add_argument("--friction", type=float)
    parser.add_argument("--timestep", type=float, default=0.001)
    parser.add_argument("--duration", type=float, default=7.0)
    parser.add_argument("--minimum-injection-time", type=float, default=4.0)
    parser.add_argument("--injection-speed", type=float, default=1.0)
    parser.add_argument("--wheel-mass", type=float, default=0.90)
    parser.add_argument("--spin-inertia", type=float, default=0.006751162108290868)
    parser.add_argument("--plugin-build", type=Path, default=DEFAULT_BUILD)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="flywheel_capability_") as raw_temp:
        world = build_world(args, args.output_dir.resolve(), Path(raw_temp))
    env = os.environ.copy()
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join(
        [str(args.plugin_build.resolve()), env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")]
    ).rstrip(os.pathsep)
    env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
        [str((ROOT / "ros2_ws/src").resolve()), str((ROOT / "gazebo/models").resolve()),
         env.get("GZ_SIM_RESOURCE_PATH", "")]
    ).rstrip(os.pathsep)
    env["GZ_PARTITION"] = f"flywheel_capability_{os.getpid()}"
    iterations = int(math.ceil(args.duration / args.timestep))
    result = subprocess.run(
        ["gz", "sim", "-s", "-r", "--iterations", str(iterations), str(world)],
        cwd=ROOT,
        env=env,
        check=False,
        capture_output=True,
        text=True,
        timeout=max(60.0, args.duration * 20.0),
    )
    (args.output_dir / "gazebo.log").write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr)
        return result.returncode
    for name in ("state.csv", "contacts.csv"):
        path = args.output_dir / name
        if not path.exists() or path.stat().st_size == 0:
            print(f"missing native telemetry: {path}")
            return 3
    print(f"completed {iterations} native Gazebo iterations in {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
