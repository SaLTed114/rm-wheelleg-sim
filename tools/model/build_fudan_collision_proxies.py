#!/usr/bin/env python3
"""Build convex self-collision proxy pieces around the Fudan leg pivots."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import struct

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
ASSET_DIR = ROOT / "models" / "MJCF" / "fudan"
EXCLUSION_RADIUS = 0.025
SECTOR_COUNT = 8


@dataclass(frozen=True)
class ProxySpec:
    source_name: str
    body_name: str
    pivot_xy: tuple[float, float]
    contype: int
    conaffinity: int
    sectors: tuple[int, ...]


PROXY_SPECS = (
    ProxySpec(
        "lf0", "Right_rear_link", (0.09435, 0.0), 2, 0,
        (0, 1, 3, 4, 5, 6, 7),
    ),
    ProxySpec(
        "l22", "Right_front_child2_link", (0.0, -0.11232), 0, 2,
        (2, 5, 6),
    ),
    ProxySpec(
        "rf0", "Left_rear_link", (0.09435, 0.0), 4, 0,
        (0, 1, 2, 3, 4, 5, 7),
    ),
    ProxySpec(
        "r22", "Left_front_child2_link", (0.0, 0.11232), 0, 4,
        (2, 3, 6),
    ),
)


def proxy_mesh_name(spec: ProxySpec, sector: int) -> str:
    return f"{spec.source_name}_self_collision_{sector}"


def proxy_file_name(spec: ProxySpec, sector: int) -> str:
    return f"{proxy_mesh_name(spec, sector)}.STL"


def read_binary_stl(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    if len(raw) < 84:
        raise RuntimeError(f"invalid binary STL: {path}")
    triangle_count = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) != 84 + 50 * triangle_count:
        raise RuntimeError(f"unsupported non-binary STL: {path}")
    record = np.dtype([
        ("normal", "<f4", (3,)),
        ("vertices", "<f4", (3, 3)),
        ("attribute", "<u2"),
    ])
    triangles = np.frombuffer(
        raw, dtype=record, count=triangle_count, offset=84)
    return triangles["vertices"].astype(np.float64)


def clip_to_tangent_halfspace(
    triangle: np.ndarray,
    pivot: np.ndarray,
    direction: np.ndarray,
) -> list[np.ndarray]:
    def distance(point: np.ndarray) -> float:
        return float(np.dot(point[:2] - pivot, direction) - EXCLUSION_RADIUS)

    polygon = [vertex for vertex in triangle]
    clipped: list[np.ndarray] = []
    previous = polygon[-1]
    previous_distance = distance(previous)
    for current in polygon:
        current_distance = distance(current)
        previous_inside = previous_distance >= 0.0
        current_inside = current_distance >= 0.0
        if previous_inside != current_inside:
            ratio = previous_distance / (previous_distance - current_distance)
            clipped.append(previous + ratio * (current - previous))
        if current_inside:
            clipped.append(current)
        previous = current
        previous_distance = current_distance

    if len(clipped) < 3:
        return []
    return [
        np.asarray((clipped[0], clipped[index], clipped[index + 1]))
        for index in range(1, len(clipped) - 1)
    ]


def split_proxy(
    triangles: np.ndarray,
    pivot_xy: tuple[float, float],
) -> list[list[np.ndarray]]:
    angles = np.arange(SECTOR_COUNT, dtype=np.float64) * (
        2.0 * math.pi / SECTOR_COUNT)
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    pivot = np.asarray(pivot_xy, dtype=np.float64)
    pieces: list[list[np.ndarray]] = [[] for _ in range(SECTOR_COUNT)]

    for triangle in triangles:
        centroid = np.mean(triangle[:, :2], axis=0) - pivot
        sector = int(np.argmax(directions @ centroid))
        pieces[sector].extend(clip_to_tangent_halfspace(
            triangle, pivot, directions[sector]))
    return pieces


def write_binary_stl(path: Path, triangles: list[np.ndarray]) -> None:
    if not triangles:
        raise RuntimeError(f"empty collision proxy: {path}")
    header = b"Fudan self-collision proxy".ljust(80, b"\0")
    with path.open("wb") as output:
        output.write(header)
        output.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
            norm = float(np.linalg.norm(normal))
            if norm > 0.0:
                normal /= norm
            values = [*normal, *triangle[0], *triangle[1], *triangle[2]]
            output.write(struct.pack("<12fH", *values, 0))


def build_collision_proxy_assets(asset_dir: Path = ASSET_DIR) -> None:
    for spec in PROXY_SPECS:
        for sector in range(SECTOR_COUNT):
            (asset_dir / proxy_file_name(spec, sector)).unlink(
                missing_ok=True)
        source = asset_dir / f"{spec.source_name}_Link.STL"
        pieces = split_proxy(read_binary_stl(source), spec.pivot_xy)
        for sector in spec.sectors:
            write_binary_stl(
                asset_dir / proxy_file_name(spec, sector), pieces[sector])


if __name__ == "__main__":
    build_collision_proxy_assets()
