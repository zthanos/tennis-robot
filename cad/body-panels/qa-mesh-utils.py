#!/usr/bin/env python3
"""Mesh sampling helpers shared by the active concept-F QA only."""

import struct
import numpy as np
from scipy.spatial import cKDTree

SPACING = 0.4


def load(path):
    data = open(path, "rb").read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        vertices = [list(map(float, line.split()[1:4]))
                    for line in data.decode().splitlines()
                    if line.strip().startswith("vertex")]
        return np.array(vertices).reshape(-1, 3, 3)
    count = struct.unpack("<I", data[80:84])[0]
    rows = np.frombuffer(data[84:84 + count * 50],
                         dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rows["v"].reshape(-1, 3, 3).astype(float)


def volume_cm3(triangles):
    return abs(np.einsum("ij,ij->i", triangles[:, 0],
                         np.cross(triangles[:, 1], triangles[:, 2])).sum()) / 6000


def sample(triangles, spacing=SPACING, box=None):
    if box is not None:
        low, high = box
        keep = (np.all(triangles.max(1) >= low, 1)
                & np.all(triangles.min(1) <= high, 1))
        triangles = triangles[keep]
    if len(triangles) == 0:
        return np.zeros((0, 3))
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    counts = np.maximum(1, np.ceil(area / (spacing * spacing * 0.5)).astype(int))
    index = np.repeat(np.arange(len(triangles)), counts)
    r1, r2 = np.random.rand(len(index)), np.random.rand(len(index))
    root = np.sqrt(r1)
    points = ((1 - root)[:, None] * a[index]
              + (root * (1 - r2))[:, None] * b[index]
              + (root * r2)[:, None] * c[index])
    edges = np.concatenate([a, b, c, (a + b) / 2, (b + c) / 2, (c + a) / 2])
    return np.concatenate([points, edges])


def _point_triangle_distance(points, triangles):
    a, b, c = triangles[:, 0][None], triangles[:, 1][None], triangles[:, 2][None]
    p = points[:, None, :]
    ab, ac, ap = b - a, c - a, p - a
    d1, d2 = (ab * ap).sum(-1), (ac * ap).sum(-1)
    bp = p - b
    d3, d4 = (ab * bp).sum(-1), (ac * bp).sum(-1)
    cp = p - c
    d5, d6 = (ab * cp).sum(-1), (ac * cp).sum(-1)
    va, vb, vc = d3*d6-d5*d4, d5*d2-d1*d6, d1*d4-d3*d2
    denominator = np.where(abs(va+vb+vc) < 1e-12, 1e-12, va+vb+vc)
    v, w = vb/denominator, vc/denominator
    projected = a + ab*v[..., None] + ac*w[..., None]

    def edge(origin, vector, parameter):
        return origin + vector*np.clip(parameter, 0, 1)[..., None]

    with np.errstate(divide="ignore", invalid="ignore"):
        eab = edge(a, ab, d1/np.where(d1-d3 == 0, 1e-12, d1-d3))
        eac = edge(a, ac, d2/np.where(d2-d6 == 0, 1e-12, d2-d6))
        bc = c-b
        ebc = edge(b, bc, ((p-b)*bc).sum(-1)/np.maximum((bc*bc).sum(-1), 1e-12))
    inside = (v >= 0) & (w >= 0) & (v+w <= 1)
    interior = np.where(inside, np.linalg.norm(p-projected, axis=-1), np.inf)
    return np.minimum.reduce([interior] + [np.linalg.norm(p-e, axis=-1)
                                           for e in (eab, eac, ebc)]).min(1)


def min_dist(panel, target, margin=80.0):
    coarse = sample(panel, 2.5)
    low, high = coarse.min(0)-margin, coarse.max(0)+margin
    target_points = sample(target, 2.5, box=(low, high))
    if len(target_points) == 0:
        return float("inf"), None
    distance, _ = cKDTree(target_points).query(coarse, k=1)
    best = (float("inf"), None)
    target_min, target_max = target.min(1), target.max(1)
    seen = []
    for index in np.argsort(distance)[:400]:
        centre = coarse[index]
        if any(np.linalg.norm(centre-old) < 15 for old in seen):
            continue
        seen.append(centre)
        if len(seen) > 6:
            break
        radius = 6.0
        box = (centre-radius, centre+radius)
        fine = sample(panel, 0.4, box=box)
        fine = fine[np.all((fine >= box[0]) & (fine <= box[1]), 1)]
        reach = distance[index]+radius+4
        nearby = target[np.all(target_max >= centre-reach, 1)
                        & np.all(target_min <= centre+reach, 1)]
        for start in range(0, len(fine), 2000):
            chunk = fine[start:start+2000]
            if not len(chunk) or not len(nearby):
                continue
            values = np.concatenate([_point_triangle_distance(chunk, nearby[i:i+300])[:, None]
                                     for i in range(0, len(nearby), 300)], 1).min(1)
            local = int(np.argmin(values))
            if values[local] < best[0]:
                best = (float(values[local]), chunk[local].round(1).tolist())
    return best
