"""Optional OpenTelemetry setup for simulation controllers."""

from __future__ import annotations

import logging
import os
from contextlib import nullcontext
from dataclasses import dataclass
from typing import Any

LOGGER = logging.getLogger(__name__)


def _env_flag(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


@dataclass
class SimulationTelemetry:
    tracer: Any
    frame_counter: Any
    detection_counter: Any
    detection_area_histogram: Any
    loop_duration_histogram: Any
    enabled: bool

    def start_span(self, name: str):
        if not self.enabled:
            return nullcontext()
        return self.tracer.start_as_current_span(name)

    def add_frame(self) -> None:
        if self.enabled:
            self.frame_counter.add(1)

    def add_detection(self, area_px: int) -> None:
        if not self.enabled:
            return
        self.detection_counter.add(1)
        self.detection_area_histogram.record(area_px)

    def record_loop_duration(self, duration_ms: float) -> None:
        if self.enabled:
            self.loop_duration_histogram.record(duration_ms)


class _NoopMetric:
    def add(self, *_args: Any, **_kwargs: Any) -> None:
        return None

    def record(self, *_args: Any, **_kwargs: Any) -> None:
        return None


def setup_telemetry(service_name: str) -> SimulationTelemetry:
    if not _env_flag("OTEL_ENABLED"):
        return _noop_telemetry()

    try:
        from opentelemetry import metrics, trace
        from opentelemetry.exporter.otlp.proto.http.metric_exporter import OTLPMetricExporter
        from opentelemetry.exporter.otlp.proto.http.trace_exporter import OTLPSpanExporter
        from opentelemetry.sdk.metrics import MeterProvider
        from opentelemetry.sdk.metrics.export import ConsoleMetricExporter, PeriodicExportingMetricReader
        from opentelemetry.sdk.resources import Resource
        from opentelemetry.sdk.trace import TracerProvider
        from opentelemetry.sdk.trace.export import BatchSpanProcessor, ConsoleSpanExporter
    except ImportError:
        LOGGER.exception("OpenTelemetry is enabled but packages are not installed.")
        return _noop_telemetry()

    resource = Resource.create(
        {
            "service.name": service_name,
            "service.namespace": "tennis-robot",
            "deployment.environment": os.getenv("APP_ENV", "simulation"),
            "robot.simulator": os.getenv("SIMULATOR", "webots"),
        }
    )
    exporter = os.getenv("OTEL_EXPORTER", "otlp").strip().lower()

    trace_provider = TracerProvider(resource=resource)
    metric_exporter = ConsoleMetricExporter() if exporter == "console" else OTLPMetricExporter()
    span_exporter = ConsoleSpanExporter() if exporter == "console" else OTLPSpanExporter()
    trace_provider.add_span_processor(BatchSpanProcessor(span_exporter))
    trace.set_tracer_provider(trace_provider)

    metric_reader = PeriodicExportingMetricReader(
        metric_exporter,
        export_interval_millis=int(os.getenv("OTEL_METRIC_EXPORT_INTERVAL_MS", "5000")),
    )
    meter_provider = MeterProvider(resource=resource, metric_readers=[metric_reader])
    metrics.set_meter_provider(meter_provider)

    meter = metrics.get_meter(service_name)
    return SimulationTelemetry(
        tracer=trace.get_tracer(service_name),
        frame_counter=meter.create_counter(
            "robot.camera.frames",
            unit="1",
            description="Camera frames processed by the controller.",
        ),
        detection_counter=meter.create_counter(
            "robot.vision.ball.detections",
            unit="1",
            description="Tennis ball detections produced by the vision pipeline.",
        ),
        detection_area_histogram=meter.create_histogram(
            "robot.vision.ball.area",
            unit="px",
            description="Detected tennis ball bounding-box area.",
        ),
        loop_duration_histogram=meter.create_histogram(
            "robot.control.loop.duration",
            unit="ms",
            description="Controller loop duration.",
        ),
        enabled=True,
    )


def _noop_telemetry() -> SimulationTelemetry:
    metric = _NoopMetric()
    return SimulationTelemetry(
        tracer=None,
        frame_counter=metric,
        detection_counter=metric,
        detection_area_histogram=metric,
        loop_duration_histogram=metric,
        enabled=False,
    )
