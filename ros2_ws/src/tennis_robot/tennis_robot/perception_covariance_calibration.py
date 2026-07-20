"""Pure calibrated measurement-covariance artifact contract."""
from __future__ import annotations
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

class CalibrationError(ValueError): pass

def compute_artifact_sha256(data: dict) -> str:
    """Canonical content hash of an artifact dict, excluding the hash field itself."""
    body = {key: value for key, value in dict(data).items() if key != "artifact_sha256"}
    encoded = json.dumps(body, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()

@dataclass(frozen=True)
class PerceptionSpatialValidationConfig:
    max_rgb_depth_timestamp_delta_s: float; max_detection_to_tf_age_s: float
    covariance_psd_relative_tolerance: float; min_position_covariance_trace_m2: float; max_position_covariance_trace_m2: float
    def __post_init__(self):
        values=(self.max_rgb_depth_timestamp_delta_s,self.max_detection_to_tf_age_s,self.covariance_psd_relative_tolerance,self.min_position_covariance_trace_m2,self.max_position_covariance_trace_m2)
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in values) or self.max_rgb_depth_timestamp_delta_s<=0 or self.max_detection_to_tf_age_s<=0 or not 0<=self.covariance_psd_relative_tolerance<1 or not 0<=self.min_position_covariance_trace_m2<=self.max_position_covariance_trace_m2 or self.max_position_covariance_trace_m2<=0: raise CalibrationError("invalid spatial validation config")
    def to_dict(self) -> dict:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}
    @classmethod
    def from_dict(cls, data: dict) -> "PerceptionSpatialValidationConfig":
        if not isinstance(data, dict) or set(data) != set(cls.__dataclass_fields__):
            raise CalibrationError("invalid spatial validation config fields")
        return cls(**data)

def validate_spatial_metadata(rgb_s, depth_s, covariance, config):
    if not isinstance(config, PerceptionSpatialValidationConfig): return "validation_config_invalid"
    if not all(math.isfinite(v) for v in (rgb_s, depth_s)) or depth_s <= 0: return "matched_depth_stamp_invalid"
    if abs(rgb_s-depth_s)>config.max_rgb_depth_timestamp_delta_s: return "rgb_depth_delta_exceeded"
    if len(covariance)!=9 or any(not math.isfinite(v) for v in covariance): return "covariance_nonfinite"
    if any(abs(covariance[i*3+j]-covariance[j*3+i])>1e-12 for i in range(3) for j in range(3)): return "covariance_nonsymmetric"
    # Principal-minor PSD check for the declared 3x3 covariance.
    a,b,c,d,e,f,g,h,i=covariance; det=a*(e*i-f*h)-b*(d*i-f*g)+c*(d*h-e*g)
    scale=max(1.0, *(abs(v) for v in covariance))
    if a < 0 or a*e-b*d < -config.covariance_psd_relative_tolerance*scale or det < -config.covariance_psd_relative_tolerance*scale: return "covariance_not_psd"
    trace=a+e+i
    if not config.min_position_covariance_trace_m2<=trace<=config.max_position_covariance_trace_m2: return "covariance_trace_out_of_range"
    return None

@dataclass(frozen=True)
class CalibrationArtifact:
    calibration_id: str; model_id: str; model_version: str; platform: str
    calibrated_at: str; floor: tuple[float, float, float]
    range_coeff: tuple[float, float, float]; quality_coeff: tuple[float, float, float]
    range_min_m: float; range_max_m: float; quality_min: float; quality_max: float
    evidence_reference: str; acceptance_metrics: dict; artifact_sha256: str

    @classmethod
    def from_dict(cls, data: dict) -> "CalibrationArtifact":
        expected = {"schema_version","calibration_id","model_id","model_version","platform","calibrated_at","parameters","range_validity_domain_m","depth_quality_validity_domain","evidence_reference","acceptance_metrics","artifact_sha256"}
        if set(data) != expected or data["schema_version"] != "perception-covariance-calibration/v1": raise CalibrationError("invalid artifact schema")
        if data["artifact_sha256"] != compute_artifact_sha256(data): raise CalibrationError("artifact sha256 mismatch")
        p, r, q = data["parameters"], data["range_validity_domain_m"], data["depth_quality_validity_domain"]
        try: artifact = cls(data["calibration_id"], data["model_id"], data["model_version"], data["platform"], data["calibrated_at"], tuple(p["axis_variance_floor_m2"]), tuple(p["axis_range_variance_per_m2"]), tuple(p["axis_quality_variance_m2"]), float(r["min"]), float(r["max"]), float(q["min"]), float(q["max"]), data["evidence_reference"], data["acceptance_metrics"], data["artifact_sha256"])
        except (KeyError, TypeError, ValueError) as exc: raise CalibrationError("invalid artifact values") from exc
        artifact.validate(); return artifact

    def to_dict(self) -> dict:
        return {
            "schema_version": "perception-covariance-calibration/v1",
            "calibration_id": self.calibration_id, "model_id": self.model_id,
            "model_version": self.model_version, "platform": self.platform,
            "calibrated_at": self.calibrated_at,
            "parameters": {
                "axis_variance_floor_m2": list(self.floor),
                "axis_range_variance_per_m2": list(self.range_coeff),
                "axis_quality_variance_m2": list(self.quality_coeff),
            },
            "range_validity_domain_m": {"min": self.range_min_m, "max": self.range_max_m},
            "depth_quality_validity_domain": {"min": self.quality_min, "max": self.quality_max},
            "evidence_reference": self.evidence_reference,
            "acceptance_metrics": self.acceptance_metrics,
            "artifact_sha256": self.artifact_sha256,
        }

    def validate(self) -> None:
        if self.model_id != "range_depth_quality_diagonal_v1" or not all(isinstance(v, str) and v for v in (self.calibration_id,self.model_version,self.platform,self.calibrated_at,self.evidence_reference)): raise CalibrationError("invalid artifact identity")
        if not isinstance(self.artifact_sha256, str) or len(self.artifact_sha256) != 64 or any(c not in "0123456789abcdef" for c in self.artifact_sha256): raise CalibrationError("invalid artifact sha256")
        if not isinstance(self.acceptance_metrics, dict) or not self.acceptance_metrics: raise CalibrationError("acceptance metrics required")
        for values in (self.floor,self.range_coeff,self.quality_coeff):
            if len(values) != 3 or any(not isinstance(v,(int,float)) or not math.isfinite(v) or v < 0 for v in values): raise CalibrationError("invalid calibrated coefficients")
        if not 0 <= self.range_min_m < self.range_max_m or not 0 <= self.quality_min <= self.quality_max <= 1: raise CalibrationError("invalid validity domain")

@dataclass(frozen=True)
class CovarianceResult:
    covariance: tuple[float, ...] | None; reason: str | None

class RangeDepthQualityDiagonalModel:
    def __init__(self, artifact: CalibrationArtifact): artifact.validate(); self.artifact = artifact
    def evaluate(self, range_m: float, depth_quality: float) -> CovarianceResult:
        if not all(math.isfinite(v) for v in (range_m, depth_quality)): return CovarianceResult(None, "invalid_input")
        a = self.artifact
        if not a.range_min_m <= range_m <= a.range_max_m or not a.quality_min <= depth_quality <= a.quality_max: return CovarianceResult(None, "calibration_out_of_domain")
        variances = tuple(a.floor[i] + a.range_coeff[i] * range_m ** 2 + a.quality_coeff[i] * (1 - depth_quality) ** 2 for i in range(3))
        return CovarianceResult((variances[0],0.,0.,0.,variances[1],0.,0.,0.,variances[2]), None)

def evaluate_producer_spatial_covariance(
    model: RangeDepthQualityDiagonalModel | None,
    camera_optical_xyz_m: tuple[float, float, float],
    depth_quality: float,
) -> CovarianceResult:
    """Pure producer-model boundary; never supplies a default covariance."""
    if model is None or len(camera_optical_xyz_m) != 3:
        return CovarianceResult(None, "calibration_unavailable")
    if not all(math.isfinite(value) for value in (*camera_optical_xyz_m, depth_quality)):
        return CovarianceResult(None, "invalid_input")
    range_m = math.sqrt(sum(value * value for value in camera_optical_xyz_m))
    return model.evaluate(range_m, depth_quality)

@dataclass(frozen=True)
class SpatialCalibrationRuntime:
    healthy: bool; health_reason: str; artifact_id: str | None
    artifact_version: str | None; model: RangeDepthQualityDiagonalModel | None

def load_spatial_calibration_runtime(
    path: str | Path | None, *, expected_platform: str,
    expected_calibration_id: str | None = None,
    expected_model_version: str | None = None,
    required_path: str | Path | None = None,
) -> SpatialCalibrationRuntime:
    """Load one explicit artifact without any default or cross-platform fallback."""
    if not path:
        return SpatialCalibrationRuntime(False, "calibration_missing", None, None, None)
    try:
        resolved_path = Path(path).resolve(strict=True)
        if required_path and resolved_path != Path(required_path).resolve(strict=True):
            return SpatialCalibrationRuntime(False, "calibration_identity_mismatch", None, None, None)
        artifact = load_artifact(resolved_path)
    except (CalibrationError, OSError):
        return SpatialCalibrationRuntime(False, "calibration_invalid", None, None, None)
    if artifact.platform != expected_platform:
        return SpatialCalibrationRuntime(False, "calibration_platform_mismatch", None, None, None)
    if expected_calibration_id and artifact.calibration_id != expected_calibration_id:
        return SpatialCalibrationRuntime(False, "calibration_identity_mismatch", None, None, None)
    if expected_model_version and artifact.model_version != expected_model_version:
        return SpatialCalibrationRuntime(False, "calibration_identity_mismatch", None, None, None)
    return SpatialCalibrationRuntime(
        True, "healthy", artifact.calibration_id, artifact.model_version,
        RangeDepthQualityDiagonalModel(artifact),
    )

def load_artifact(path: str | Path) -> CalibrationArtifact:
    try:
        with Path(path).open(encoding="utf-8") as stream: return CalibrationArtifact.from_dict(json.load(stream))
    except (OSError, json.JSONDecodeError) as exc: raise CalibrationError("unable to load artifact") from exc
