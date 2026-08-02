from __future__ import annotations

import importlib.util
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def _load(name: str, relative_path: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


probe = _load("network_probe", "scripts/network/network_probe.py")
report = _load("network_probe_report", "scripts/network/network_probe_report.py")


def test_default_interface_uses_lowest_metric_up_route():
    text = """Iface Destination Gateway Flags RefCnt Use Metric Mask
eth0 00000000 0101A8C0 0003 0 0 100 00000000
wlan0 00000000 0101A8C0 0003 0 0 600 00000000
down0 00000000 0101A8C0 0000 0 0 1 00000000
"""
    assert probe.default_interface(text) == "eth0"


def test_parse_snmp_pairs_protocol_rows():
    parsed = probe.parse_proc_snmp(
        """Ip: Forwarding DefaultTTL
Ip: 1 64
Udp: InDatagrams NoPorts InErrors OutDatagrams RcvbufErrors
Udp: 10 2 3 20 4
"""
    )
    assert parsed["Udp"] == {
        "InDatagrams": 10,
        "NoPorts": 2,
        "InErrors": 3,
        "OutDatagrams": 20,
        "RcvbufErrors": 4,
    }


def test_counter_delta_handles_monotonic_growth_and_counter_reset():
    assert probe.counter_delta(
        {"rx_bytes": 100, "tx_bytes": 50},
        {"rx_bytes": 140, "tx_bytes": 7},
    ) == {"rx_bytes": 40, "tx_bytes": 7}


def test_summary_computes_rates_drops_udp_and_http():
    samples = [
        {
            "monotonic_s": 10.0,
            "interface": {
                "rx_bytes": 100,
                "tx_bytes": 50,
                "rx_packets": 10,
                "tx_packets": 5,
                "multicast": 2,
                "rx_dropped": 1,
                "tx_dropped": 0,
            },
            "udp": {"InDatagrams": 4, "OutDatagrams": 7, "InErrors": 0},
            "http": [{"ok": True, "payload_bytes": 1000, "latency_ms": 5.0}],
        },
        {
            "monotonic_s": 12.0,
            "interface": {
                "rx_bytes": 500,
                "tx_bytes": 250,
                "rx_packets": 50,
                "tx_packets": 25,
                "multicast": 10,
                "rx_dropped": 3,
                "tx_dropped": 1,
            },
            "udp": {"InDatagrams": 24, "OutDatagrams": 27, "InErrors": 2},
            "http": [{"ok": True, "payload_bytes": 2000, "latency_ms": 15.0}],
        },
    ]
    summary = probe.summarize_samples(samples)
    assert summary["duration_s"] == 2.0
    assert summary["rates"]["rx_bytes_per_s"]["median"] == 200.0
    assert summary["rates"]["tx_packets_per_s"]["median"] == 10.0
    assert summary["rates"]["multicast_packets_per_s"]["median"] == 4.0
    assert summary["interface_delta"]["rx_dropped"] == 2
    assert summary["udp_delta"]["InErrors"] == 2
    assert summary["http"]["total_payload_bytes"] == 3000
    assert summary["http"]["latency_ms_p50"] == 10.0


def _document(label: str, rx: float, tx: float) -> dict:
    return {
        "schema": "tennis_robot/network_probe/v1",
        "label": label,
        "interface": "eth0",
        "summary": {
            "duration_s": 120.0,
            "rates": {
                "rx_bytes_per_s": {"median": rx},
                "tx_bytes_per_s": {"median": tx},
                "rx_packets_per_s": {"median": 10.0},
                "tx_packets_per_s": {"median": 5.0},
                "multicast_packets_per_s": {"median": 1.0},
            },
            "interface_delta": {"rx_dropped": 0, "tx_dropped": 0},
            "udp_delta": {"InErrors": 0},
            "http": {},
        },
    }


def test_report_is_deterministic_and_uses_named_baseline():
    comparison = report.compare_documents(
        [_document("stack", 400.0, 200.0), _document("baseline", 100.0, 50.0)],
        "baseline",
    )
    assert [row["label"] for row in comparison["scenarios"]] == [
        "baseline",
        "stack",
    ]
    stack = comparison["scenarios"][1]
    assert stack["ratio_to_baseline"]["rx_bytes_per_s"] == 4.0
    assert stack["ratio_to_baseline"]["tx_bytes_per_s"] == 4.0
    markdown = report.markdown_report(comparison)
    assert "stack" in markdown
    assert "0.39" in markdown


def test_isolated_domain_bridge_is_directional_and_excludes_bulk_topics():
    path = ROOT / "config/network/pc42_pi43_domain_bridge.yaml"
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert config["from_domain"] == 42
    assert config["to_domain"] == 43
    topics = config["topics"]

    assert topics["/scan"] == {"type": "sensor_msgs/msg/LaserScan"}
    assert topics["/perception/ball_detections"] == {
        "type": "tennis_robot_msgs/msg/BallDetectionArray"
    }
    assert topics["/cmd_vel_nav"]["reversed"] is True
    assert topics["/collector/cmd"]["reversed"] is True
    assert topics["/tf_static"]["qos"] == {
        "reliability": "reliable",
        "durability": "transient_local",
    }
    assert topics["/telemetry/sensor_snapshot"]["qos"] == {
        "reliability": "best_effort"
    }

    forbidden = {
        "/camera/image_raw",
        "/camera/depth",
        "/camera/intake_debug/image_raw",
        "/camera/intake_debug/depth",
        "/gz/pose_info",
        "/gz/roller_contact_0",
        "/gz/roller_contact_1",
        "/sim/ball_markers",
        "/sim/roller_contact_markers",
    }
    assert forbidden.isdisjoint(topics)

    runner = (ROOT / "scripts/network/run_pi_isolated.sh").read_text(
        encoding="utf-8"
    )
    assert "--wait-for-publisher false" in runner
    assert "--wait-for-subscription false" in runner
    assert "--auto-remove" not in runner
    assert "export ROS2CLI_DISABLE_DAEMON=1" in runner
    assert "net.core.rmem_default" in runner
    assert "install_udp_buffer_profile.sh" in runner


def test_udp_buffer_profile_matches_distributed_launch_preflight():
    profile = (ROOT / "config/network/99-tennis-robot-udp-buffers.conf").read_text(
        encoding="utf-8"
    )
    native_runner = (ROOT / "run_native.sh").read_text(encoding="utf-8")

    assert "net.core.rmem_default = 4194304" in profile
    assert "net.core.rmem_max = 4194304" in profile
    assert "required_udp_rmem=4194304" in native_runner
    assert "install_udp_buffer_profile.sh" in native_runner


def test_pc_isolated_profile_collapses_local_participants_before_lan():
    config = yaml.safe_load(
        (ROOT / "config/network/pc41_lan42_domain_bridge.yaml").read_text(
            encoding="utf-8"
        )
    )
    runner = (ROOT / "scripts/network/run_pc_isolated.sh").read_text(
        encoding="utf-8"
    )

    assert config["from_domain"] == 41
    assert config["to_domain"] == 42
    assert config["topics"]["/scan"] == {"type": "sensor_msgs/msg/LaserScan"}
    assert config["topics"]["/cmd_vel_nav"]["reversed"] is True
    assert "GZ_IP=127.0.0.1" in runner
    assert "ROS2CLI_DISABLE_DAEMON=1" in runner
    assert 'ROS_DOMAIN_ID="$PC_LOCAL_DOMAIN_ID"' in runner
    assert "--wait-for-publisher false" in runner
