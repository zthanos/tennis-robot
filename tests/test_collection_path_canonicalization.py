"""Phase 6B Part 1: pure sha256 canonicalization parity.

The C++ ``collection_path_sha256_v1`` is exercised for real in the container
parity harness.  Here we pin the pure Python against an independent inline
reference implementation of the *same* wire format (the one the isolated
launch test uses against the real controller), plus the documented edge cases.
"""

import hashlib
import os
import struct
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"))

from tennis_robot.collection_path_canonicalization import (
    CanonicalPose,
    CanonicalizationError,
    canonicalize_collection_path_v1,
    collection_path_sha256_v1,
)


def _reference_sha256(frame_id, poses):
    """Independent inline reference of the C++ v1 wire format."""
    raw = struct.pack(">I", len(frame_id.encode())) + frame_id.encode()
    raw += struct.pack(">I", len(poses))
    for pose in poses:
        raw += struct.pack(">7d", pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw)
    return hashlib.sha256(raw).hexdigest()


def _pose(x, y, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return CanonicalPose(x, y, z, qx, qy, qz, qw)


SAMPLE = (
    _pose(0.0, 0.0),
    _pose(2.0, 0.0),
    _pose(3.3, -1.25, qz=0.3826834323650898, qw=0.9238795325112867),
    _pose(3.8, 0.0),
)


def test_matches_independent_reference_for_a_multi_pose_path():
    assert collection_path_sha256_v1("map", SAMPLE) == _reference_sha256("map", SAMPLE)


def test_layout_is_u32_frame_len_then_frame_then_u32_count_then_7f64_per_pose():
    poses = (_pose(1.0, 2.0),)
    blob = canonicalize_collection_path_v1("map", poses)
    assert blob[:4] == struct.pack(">I", 3)  # len("map")
    assert blob[4:7] == b"map"
    assert blob[7:11] == struct.pack(">I", 1)  # pose count
    assert blob[11:] == struct.pack(">7d", 1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    assert len(blob) == 4 + 3 + 4 + 56


def test_frame_id_length_prefix_disambiguates_frames():
    assert collection_path_sha256_v1("map", SAMPLE) != collection_path_sha256_v1("odom", SAMPLE)


def test_hash_is_lowercase_hex_sha256():
    digest = collection_path_sha256_v1("map", SAMPLE)
    assert len(digest) == 64
    assert all(character in "0123456789abcdef" for character in digest)


def test_empty_pose_list_still_hashes_deterministically():
    assert collection_path_sha256_v1("map", ()) == _reference_sha256("map", ())


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_float_raises(bad):
    with pytest.raises(CanonicalizationError):
        canonicalize_collection_path_v1("map", (_pose(bad, 0.0),))


def test_pose_order_is_significant():
    reversed_sample = tuple(reversed(SAMPLE))
    assert collection_path_sha256_v1("map", SAMPLE) != collection_path_sha256_v1("map", reversed_sample)
