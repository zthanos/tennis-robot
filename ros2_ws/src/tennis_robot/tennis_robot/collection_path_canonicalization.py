"""Pure Python mirror of ``CollectionPathCanonicalizationV1`` (Phase 6B).

Byte-for-byte reproduction of the C++
``canonicalize_collection_path_v1`` / ``collection_path_sha256_v1``
(``tennis_robot_collection_controller/src/collection_path_canonicalization.cpp``)
so the Python follower can compute the exact ``path_sha256`` the C++ controller
will recompute on the received ``nav_msgs/Path`` — the hash both sides bind the
execution context to.

Wire layout (all integers big-endian, all floats big-endian IEEE-754 float64):

    u32(len(frame_id_utf8)) | frame_id_utf8
    u32(len(poses))
    per pose: f64 x, y, z, qx, qy, qz, qw

The digest is ``sha256`` of that byte string, lowercase hex.  Non-finite
floats raise :class:`CanonicalizationError`, mirroring the C++ throw.  This
core takes plain values (a frame id string + :class:`CanonicalPose` sequence),
never ROS types, so it is offline-testable.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
import struct

_U32_MAX = 0xFFFFFFFF


class CanonicalizationError(ValueError):
    """The path cannot be canonicalized (non-finite float or length overflow)."""


@dataclass(frozen=True)
class CanonicalPose:
    """One flattened path pose: position xyz + quaternion xyzw."""

    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def canonicalize_collection_path_v1(frame_id: str, poses: "tuple[CanonicalPose, ...]") -> bytes:
    """Return the canonical byte string for ``frame_id`` + ordered ``poses``."""
    if not isinstance(frame_id, str):
        raise CanonicalizationError("frame_id must be a string")
    try:
        frame_bytes = frame_id.encode("utf-8")
    except UnicodeEncodeError as exc:  # lone surrogates etc.
        raise CanonicalizationError("path frame_id is not valid UTF-8") from exc
    poses = tuple(poses)
    if len(frame_bytes) > _U32_MAX or len(poses) > _U32_MAX:
        raise CanonicalizationError("path field exceeds canonicalization length limit")

    output = bytearray()
    output += struct.pack(">I", len(frame_bytes))
    output += frame_bytes
    output += struct.pack(">I", len(poses))
    for pose in poses:
        values = (pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw)
        for value in values:
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
                raise CanonicalizationError("path contains non-finite float64")
        output += struct.pack(">7d", *values)
    return bytes(output)


def collection_path_sha256_v1(frame_id: str, poses: "tuple[CanonicalPose, ...]") -> str:
    """Return the lowercase-hex sha256 of the canonical path bytes."""
    return hashlib.sha256(canonicalize_collection_path_v1(frame_id, poses)).hexdigest()
