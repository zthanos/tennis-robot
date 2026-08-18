#!/usr/bin/env python3
"""Compare network_probe/v1 artifacts without producing network traffic."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


SCHEMA = "tennis_robot/network_probe_report/v1"


def _metric(document: dict, rate: str, statistic: str = "median") -> float | None:
    value = (
        ((document.get("summary") or {}).get("rates") or {})
        .get(rate, {})
        .get(statistic)
    )
    return float(value) if value is not None else None


def _ratio(value: float | None, baseline: float | None) -> float | None:
    if value is None or baseline is None or baseline <= 0:
        return None
    return value / baseline


def compare_documents(documents: list[dict], baseline_label: str) -> dict:
    by_label = {str(document["label"]): document for document in documents}
    if len(by_label) != len(documents):
        raise ValueError("scenario labels must be unique")
    if baseline_label not in by_label:
        raise ValueError(f"baseline label not found: {baseline_label}")
    baseline = by_label[baseline_label]
    scenarios = []
    for label in sorted(by_label):
        document = by_label[label]
        summary = document.get("summary") or {}
        rates = {
            name: _metric(document, name)
            for name in (
                "rx_bytes_per_s",
                "tx_bytes_per_s",
                "rx_packets_per_s",
                "tx_packets_per_s",
                "multicast_packets_per_s",
            )
        }
        scenarios.append(
            {
                "label": label,
                "interface": document.get("interface"),
                "duration_s": summary.get("duration_s"),
                "rates_median": rates,
                "ratio_to_baseline": {
                    name: _ratio(value, _metric(baseline, name))
                    for name, value in rates.items()
                },
                "interface_delta": summary.get("interface_delta") or {},
                "udp_delta": summary.get("udp_delta") or {},
                "http": summary.get("http") or {},
            }
        )
    return {
        "schema": SCHEMA,
        "baseline_label": baseline_label,
        "scenarios": scenarios,
    }


def markdown_report(report: dict) -> str:
    lines = [
        "# Network probe comparison",
        "",
        f"Baseline: `{report['baseline_label']}`",
        "",
        "| Scenario | RX KiB/s | TX KiB/s | RX pkt/s | TX pkt/s | Multicast pkt/s | Drops | UDP errors |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for scenario in report["scenarios"]:
        rates = scenario["rates_median"]
        interface_delta = scenario["interface_delta"]
        udp_delta = scenario["udp_delta"]
        drops = int(interface_delta.get("rx_dropped", 0)) + int(
            interface_delta.get("tx_dropped", 0)
        )
        udp_errors = sum(
            int(udp_delta.get(name, 0))
            for name in ("InErrors", "RcvbufErrors", "SndbufErrors", "InCsumErrors")
        )

        def render(value: float | None, divisor: float = 1.0) -> str:
            return "—" if value is None else f"{value / divisor:.2f}"

        lines.append(
            "| {label} | {rx} | {tx} | {rxp} | {txp} | {multi} | {drops} | {udp} |".format(
                label=scenario["label"],
                rx=render(rates["rx_bytes_per_s"], 1024.0),
                tx=render(rates["tx_bytes_per_s"], 1024.0),
                rxp=render(rates["rx_packets_per_s"]),
                txp=render(rates["tx_packets_per_s"]),
                multi=render(rates["multicast_packets_per_s"]),
                drops=drops,
                udp=udp_errors,
            )
        )
    return "\n".join(lines) + "\n"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-label", required=True)
    parser.add_argument("--input", type=Path, action="append", required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-markdown", type=Path)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    documents = [
        json.loads(path.read_text(encoding="utf-8")) for path in args.input
    ]
    for document in documents:
        if document.get("schema") != "tennis_robot/network_probe/v1":
            raise SystemExit(f"unsupported probe artifact: {document.get('schema')}")
    report = compare_documents(documents, args.baseline_label)
    encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(encoded, encoding="utf-8")
    if args.output_markdown:
        args.output_markdown.parent.mkdir(parents=True, exist_ok=True)
        args.output_markdown.write_text(markdown_report(report), encoding="utf-8")
    if not args.output_json and not args.output_markdown:
        print(markdown_report(report), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
