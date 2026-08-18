#!/usr/bin/env python3
"""Read-only host/network sampler for controlled PC/Pi qualification runs.

The probe does not open sockets unless ``--http-url`` is explicitly supplied.
By default it only reads Linux kernel counters and writes one JSON artifact.
It never changes ROS, DDS, interface, routing, QoS, or sensor configuration.
"""

from __future__ import annotations

import argparse
import gzip
import json
import os
from pathlib import Path
import platform
import re
import socket
import statistics
import subprocess
import time
from typing import Iterable
from urllib import request


SCHEMA = "tennis_robot/network_probe/v1"
INTERFACE_COUNTERS = (
    "rx_bytes",
    "tx_bytes",
    "rx_packets",
    "tx_packets",
    "rx_dropped",
    "tx_dropped",
    "rx_errors",
    "tx_errors",
    "multicast",
    "collisions",
)
UDP_COUNTERS = (
    "InDatagrams",
    "NoPorts",
    "InErrors",
    "OutDatagrams",
    "RcvbufErrors",
    "SndbufErrors",
    "InCsumErrors",
    "IgnoredMulti",
)


def default_interface(route_text: str) -> str | None:
    """Return the lowest-metric usable IPv4 default-route interface."""

    candidates: list[tuple[int, str]] = []
    for line in route_text.splitlines()[1:]:
        fields = line.split()
        if len(fields) < 8 or fields[1] != "00000000":
            continue
        try:
            flags = int(fields[3], 16)
            metric = int(fields[6])
        except ValueError:
            continue
        if flags & 0x1:  # RTF_UP
            candidates.append((metric, fields[0]))
    return min(candidates)[1] if candidates else None


def parse_proc_snmp(text: str) -> dict[str, dict[str, int]]:
    """Parse the paired header/value rows in ``/proc/net/snmp``."""

    parsed: dict[str, dict[str, int]] = {}
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    for index in range(0, len(lines) - 1):
        header_proto, _, header_values = lines[index].partition(":")
        value_proto, _, value_values = lines[index + 1].partition(":")
        if not header_proto or header_proto != value_proto:
            continue
        names = header_values.split()
        values = value_values.split()
        if len(names) != len(values):
            continue
        try:
            parsed[header_proto] = {
                name: int(value) for name, value in zip(names, values)
            }
        except ValueError:
            continue
    return parsed


def counter_delta(before: dict[str, int], after: dict[str, int]) -> dict[str, int]:
    """Return monotonic counter deltas; a reset is represented by the new value."""

    result: dict[str, int] = {}
    for key in sorted(set(before) | set(after)):
        old = int(before.get(key, 0))
        new = int(after.get(key, 0))
        result[key] = new - old if new >= old else new
    return result


def _percentile(values: Iterable[float], percentile: float) -> float | None:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percentile
    lower = int(position)
    upper = min(len(ordered) - 1, lower + 1)
    fraction = position - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def rate_summary(samples: list[dict]) -> dict:
    """Summarize rates from consecutive counter samples."""

    rates: dict[str, list[float]] = {
        "rx_bytes_per_s": [],
        "tx_bytes_per_s": [],
        "rx_packets_per_s": [],
        "tx_packets_per_s": [],
        "multicast_packets_per_s": [],
    }
    for previous, current in zip(samples, samples[1:]):
        elapsed = float(current["monotonic_s"]) - float(previous["monotonic_s"])
        if elapsed <= 0:
            continue
        delta = counter_delta(previous["interface"], current["interface"])
        for output_name, counter_name in (
            ("rx_bytes_per_s", "rx_bytes"),
            ("tx_bytes_per_s", "tx_bytes"),
            ("rx_packets_per_s", "rx_packets"),
            ("tx_packets_per_s", "tx_packets"),
            ("multicast_packets_per_s", "multicast"),
        ):
            rates[output_name].append(delta.get(counter_name, 0) / elapsed)

    summary: dict[str, dict[str, float | None]] = {}
    for name, values in rates.items():
        summary[name] = {
            "median": statistics.median(values) if values else None,
            "p95": _percentile(values, 0.95),
            "peak": max(values) if values else None,
        }
    return summary


def summarize_samples(samples: list[dict]) -> dict:
    if not samples:
        return {"sample_count": 0}
    duration = max(
        0.0,
        float(samples[-1]["monotonic_s"]) - float(samples[0]["monotonic_s"]),
    )
    interface_delta = counter_delta(
        samples[0]["interface"], samples[-1]["interface"]
    )
    udp_delta = counter_delta(samples[0]["udp"], samples[-1]["udp"])
    http_samples = [
        result
        for sample in samples
        for result in sample.get("http", [])
        if result.get("ok")
    ]
    return {
        "sample_count": len(samples),
        "duration_s": duration,
        "interface_delta": interface_delta,
        "udp_delta": udp_delta,
        "rates": rate_summary(samples),
        "http": {
            "successful_requests": len(http_samples),
            "total_payload_bytes": sum(
                int(result.get("payload_bytes", 0)) for result in http_samples
            ),
            "latency_ms_p50": (
                statistics.median(float(result["latency_ms"]) for result in http_samples)
                if http_samples
                else None
            ),
            "latency_ms_p95": _percentile(
                (float(result["latency_ms"]) for result in http_samples), 0.95
            ),
        },
    }


class LinuxCounterReader:
    def __init__(
        self,
        interface: str,
        *,
        sys_class_net: Path = Path("/sys/class/net"),
        proc_snmp: Path = Path("/proc/net/snmp"),
    ) -> None:
        if not re.fullmatch(r"[A-Za-z0-9_.:-]+", interface):
            raise ValueError(f"invalid interface name: {interface!r}")
        self.interface = interface
        self._statistics = sys_class_net / interface / "statistics"
        self._proc_snmp = proc_snmp
        if not self._statistics.is_dir():
            raise ValueError(f"network interface does not exist: {interface}")

    def sample(self) -> dict:
        interface = {}
        for name in INTERFACE_COUNTERS:
            path = self._statistics / name
            try:
                interface[name] = int(path.read_text(encoding="ascii").strip())
            except (FileNotFoundError, ValueError):
                interface[name] = 0
        try:
            udp_all = parse_proc_snmp(
                self._proc_snmp.read_text(encoding="ascii", errors="replace")
            ).get("Udp", {})
        except FileNotFoundError:
            udp_all = {}
        udp = {name: int(udp_all.get(name, 0)) for name in UDP_COUNTERS}
        return {
            "wall_time_s": time.time(),
            "monotonic_s": time.monotonic(),
            "interface": interface,
            "udp": udp,
        }


def sample_http(urls: list[str], timeout_s: float) -> list[dict]:
    results = []
    for url in urls:
        started = time.monotonic()
        try:
            http_request = request.Request(
                url, headers={"Accept-Encoding": "gzip"}
            )
            with request.urlopen(http_request, timeout=timeout_s) as response:
                body = response.read()
                content_encoding = response.headers.get("Content-Encoding", "").lower()
                decoded_size = len(gzip.decompress(body)) if content_encoding == "gzip" else len(body)
                results.append(
                    {
                        "url": url,
                        "ok": True,
                        "status": int(response.status),
                        # payload_bytes is the actual HTTP body transferred.
                        "payload_bytes": len(body),
                        "decoded_payload_bytes": decoded_size,
                        "content_encoding": content_encoding or "identity",
                        "latency_ms": (time.monotonic() - started) * 1000.0,
                    }
                )
        except Exception as exc:  # diagnostic artifact must record, not abort
            results.append(
                {
                    "url": url,
                    "ok": False,
                    "error": f"{type(exc).__name__}: {exc}",
                    "latency_ms": (time.monotonic() - started) * 1000.0,
                }
            )
    return results


def command_snapshot(command: list[str], timeout_s: float = 15.0) -> dict:
    try:
        completed = subprocess.run(
            command,
            check=False,
            text=True,
            capture_output=True,
            timeout=timeout_s,
        )
        return {
            "command": command,
            "returncode": completed.returncode,
            "stdout": completed.stdout,
            "stderr": completed.stderr,
        }
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return {
            "command": command,
            "returncode": None,
            "error": f"{type(exc).__name__}: {exc}",
        }


def capture_ros_graph(topics: list[str]) -> dict:
    """Capture ROS graph metadata without subscribing to topic payloads."""

    graph = {
        "nodes": command_snapshot(["ros2", "node", "list"]),
        "topics": command_snapshot(["ros2", "topic", "list", "-t"]),
        "topic_info": {},
    }
    for topic in topics:
        graph["topic_info"][topic] = command_snapshot(
            ["ros2", "topic", "info", "--verbose", topic]
        )
    return graph


def _detect_interface(explicit: str | None) -> str:
    if explicit:
        return explicit
    route_path = Path("/proc/net/route")
    interface = default_interface(
        route_path.read_text(encoding="ascii", errors="replace")
    )
    if not interface:
        raise RuntimeError("no usable default-route interface; pass --interface")
    return interface


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", required=True, help="Scenario label, e.g. D-stack-ui-closed")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--interface", help="Defaults to the lowest-metric IPv4 default route")
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--interval-s", type=float, default=1.0)
    parser.add_argument(
        "--http-url",
        action="append",
        default=[],
        help="Optional endpoint to measure once per interval; repeatable",
    )
    parser.add_argument("--http-timeout-s", type=float, default=2.0)
    parser.add_argument(
        "--capture-ros-graph",
        action="store_true",
        help="Run metadata-only ros2 graph commands at the end",
    )
    parser.add_argument(
        "--ros-topic",
        action="append",
        default=[],
        help="Topic for ros2 topic info --verbose; repeatable",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.duration_s < 0:
        raise SystemExit("--duration-s must be >= 0")
    if args.interval_s <= 0:
        raise SystemExit("--interval-s must be > 0")
    interface = _detect_interface(args.interface)
    reader = LinuxCounterReader(interface)
    samples = []
    started = time.monotonic()
    interrupted = False
    try:
        while True:
            sample = reader.sample()
            if args.http_url:
                sample["http"] = sample_http(args.http_url, args.http_timeout_s)
            samples.append(sample)
            elapsed = time.monotonic() - started
            if elapsed >= args.duration_s:
                break
            time.sleep(min(args.interval_s, max(0.0, args.duration_s - elapsed)))
    except KeyboardInterrupt:
        interrupted = True

    artifact = {
        "schema": SCHEMA,
        "label": args.label,
        "host": {
            "hostname": socket.gethostname(),
            "platform": platform.platform(),
            "kernel": platform.release(),
            "pid": os.getpid(),
        },
        "interface": interface,
        "configuration": {
            "duration_s": args.duration_s,
            "interval_s": args.interval_s,
            "http_urls": args.http_url,
            "capture_ros_graph": bool(args.capture_ros_graph),
            "ros_topics": args.ros_topic,
        },
        "interrupted": interrupted,
        "samples": samples,
        "summary": summarize_samples(samples),
    }
    if args.capture_ros_graph:
        artifact["ros_graph"] = capture_ros_graph(args.ros_topic)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(artifact, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(args.output)
    print(json.dumps({"output": str(args.output), **artifact["summary"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
