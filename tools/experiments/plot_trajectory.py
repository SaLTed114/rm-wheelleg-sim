#!/usr/bin/env python3
"""Plot planned and actual benchmark trajectories as a dependency-free SVG."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def load_paths(trace: Path) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    with trace.open(newline="", encoding="ascii") as source:
        rows = [row for row in csv.DictReader(source)
                if row["phase"].startswith("figure_eight_")]
    if len(rows) < 2:
        raise ValueError("trace contains no complete figure-eight trajectory")

    start_x = float(rows[0]["axle_x"])
    start_y = float(rows[0]["axle_y"])
    start_heading = float(rows[0]["psi"])
    cosine = math.cos(start_heading)
    sine = math.sin(start_heading)
    actual: list[tuple[float, float]] = []
    planned: list[tuple[float, float]] = [(0.0, 0.0)]
    planned_x = 0.0
    planned_y = 0.0
    planned_heading = 0.0
    previous_time = float(rows[0]["simulation_time"])

    for row in rows:
        dx = float(row["axle_x"]) - start_x
        dy = float(row["axle_y"]) - start_y
        actual.append((cosine * dx + sine * dy, -sine * dx + cosine * dy))

        current_time = float(row["simulation_time"])
        timestep = current_time - previous_time
        previous_time = current_time
        yaw_rate = float(row["ref_dpsi"])
        velocity = float(row["ref_ds"])
        midpoint_heading = planned_heading + 0.5 * yaw_rate * timestep
        planned_x += velocity * math.cos(midpoint_heading) * timestep
        planned_y += velocity * math.sin(midpoint_heading) * timestep
        planned_heading += yaw_rate * timestep
        planned.append((planned_x, planned_y))
    return planned, actual


def write_svg(
    output: Path,
    planned: list[tuple[float, float]],
    actual: list[tuple[float, float]],
) -> None:
    width = 850
    height = 760
    margin = 80.0
    all_points = planned + actual
    minimum_x = min(point[0] for point in all_points)
    maximum_x = max(point[0] for point in all_points)
    minimum_y = min(point[1] for point in all_points)
    maximum_y = max(point[1] for point in all_points)
    span = max(maximum_x - minimum_x, maximum_y - minimum_y, 0.1) * 1.15
    center_x = 0.5 * (minimum_x + maximum_x)
    center_y = 0.5 * (minimum_y + maximum_y)
    scale = min(width - 2.0 * margin, height - 2.0 * margin) / span

    def screen(point: tuple[float, float]) -> tuple[float, float]:
        return (
            width / 2.0 + (point[0] - center_x) * scale,
            height / 2.0 - (point[1] - center_y) * scale,
        )

    def polyline(points: list[tuple[float, float]]) -> str:
        stride = max(1, len(points) // 1800)
        selected = points[::stride]
        if selected[-1] != points[-1]:
            selected.append(points[-1])
        return " ".join(
            f"{x:.2f},{y:.2f}" for x, y in
            (screen(point) for point in selected))

    planned_end = planned[-1]
    actual_end = actual[-1]
    planned_closure = math.hypot(*planned_end)
    actual_closure = math.hypot(*actual_end)
    start_screen = screen((0.0, 0.0))
    actual_end_screen = screen(actual_end)
    document = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="#f4f0e8"/>
<style>text{{font-family:Verdana,sans-serif;fill:#24231f}} .title{{font-size:21px;font-weight:bold}} .label{{font-size:13px}}</style>
<text x="45" y="38" class="title">Measured figure-eight trajectory</text>
<text x="45" y="62" class="label">planned closure {planned_closure:.3f} m; actual closure {actual_closure:.3f} m</text>
<line x1="45" y1="88" x2="85" y2="88" stroke="#2563eb" stroke-width="4"/><text x="95" y="93" class="label">planned reference</text>
<line x1="245" y1="88" x2="285" y2="88" stroke="#c24132" stroke-width="4"/><text x="295" y="93" class="label">actual axle path</text>
<rect x="{margin}" y="110" width="{width - 2 * margin}" height="{height - 110 - margin}" fill="#fffdf8" stroke="#817b70"/>
<polyline points="{polyline(planned)}" fill="none" stroke="#2563eb" stroke-width="3" stroke-dasharray="8 6"/>
<polyline points="{polyline(actual)}" fill="none" stroke="#c24132" stroke-width="3"/>
<circle cx="{start_screen[0]:.2f}" cy="{start_screen[1]:.2f}" r="7" fill="#238636"/><text x="{start_screen[0] + 10:.2f}" y="{start_screen[1] - 8:.2f}" class="label">start</text>
<circle cx="{actual_end_screen[0]:.2f}" cy="{actual_end_screen[1]:.2f}" r="7" fill="#c24132"/><text x="{actual_end_screen[0] + 10:.2f}" y="{actual_end_screen[1] + 18:.2f}" class="label">actual end</text>
<text x="45" y="{height - 25}" class="label">Coordinates are relative to the measured entry pose; axes use equal scale.</text>
</svg>
'''
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(document, encoding="ascii")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()
    try:
        planned, actual = load_paths(arguments.trace)
        write_svg(arguments.output, planned, actual)
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    print(f"Wrote {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
