#!/usr/bin/env python3
"""Plot the touchdown transient of the landing-suspension candidates."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Callable


COLORS = ("#292524", "#1d4ed8", "#0f766e", "#ca8a04", "#dc2626", "#7c3aed")


def short_name(name: str) -> str:
    if "hold_extended" in name:
        return "position hold"
    suffix = name.split("landing_suspension_", 1)[-1]
    return suffix.replace("_", " ").upper()


def read_results(root: Path, window: float) -> list[dict]:
    with (root / "platform_summary.csv").open(
        newline="", encoding="ascii"
    ) as source:
        summaries = list(csv.DictReader(source))

    results = []
    for summary in summaries:
        path = root / summary["case"] / "trace.csv"
        with path.open(newline="", encoding="ascii") as source:
            rows = [
                row for row in csv.DictReader(source)
                if row["phase"] == "platform_drop_post_touchdown"
            ]
        if not rows:
            raise ValueError(f"missing post-touchdown samples: {path}")
        start = float(rows[0]["simulation_time"])
        rows = [
            row for row in rows
            if float(row["simulation_time"]) - start <= window + 1.0e-12
        ]
        results.append({
            "summary": summary,
            "rows": rows,
            "start": start,
            "suspension": "landing_suspension_" in summary["case"],
        })
    return results


def number(row: dict[str, str], column: str) -> float:
    return float(row[column])


def maximum_side(row: dict[str, str], prefix: str, absolute: bool = False) -> float:
    values = (number(row, prefix + "_l"), number(row, prefix + "_r"))
    if absolute:
        values = tuple(abs(value) for value in values)
    return max(values)


def compression_mm(row: dict[str, str]) -> float:
    values = []
    for side in ("l", "r"):
        if int(row["suspension_contact_" + side]):
            values.append(max(
                0.0,
                number(row, "suspension_capture_" + side) -
                number(row, "leg_length_" + side),
            ) * 1000.0)
    return max(values, default=0.0)


def draw_panel(
    doc: list[str],
    results: list[dict],
    x: float,
    y: float,
    width: float,
    height: float,
    window: float,
    title: str,
    value: Callable[[dict[str, str]], float],
    suspension_only: bool = False,
    reference: float | None = None,
) -> None:
    selected = [result for result in results
                if result["suspension"] or not suspension_only]
    series = [
        (result, [value(row) for row in result["rows"]])
        for result in selected
    ]
    all_values = [item for _, values in series for item in values]
    if reference is not None:
        all_values.append(reference)
    value_min, value_max = min(all_values), max(all_values)
    span = max(value_max - value_min, 1.0e-6)
    value_min -= 0.08 * span
    value_max += 0.08 * span

    def x_screen(time: float) -> float:
        return x + width * time / window

    def y_screen(item: float) -> float:
        return y + height * (value_max - item) / (value_max - value_min)

    doc.append(
        f'<rect x="{x}" y="{y}" width="{width}" height="{height}" '
        'fill="#fffdf8" stroke="#8b8478"/>'
    )
    doc.append(f'<text x="{x}" y="{y - 11}" class="label">{title}</text>')
    for tick in range(4):
        time = window * tick / 3.0
        tick_x = x_screen(time)
        doc.append(
            f'<line x1="{tick_x:.2f}" y1="{y}" x2="{tick_x:.2f}" '
            f'y2="{y + height}" stroke="#e4ded2"/>'
        )
        doc.append(
            f'<text x="{tick_x:.2f}" y="{y + height + 20}" '
            f'text-anchor="middle" class="axis">{time * 1000.0:.0f}</text>'
        )
    for tick in range(3):
        item = value_min + (value_max - value_min) * tick / 2.0
        tick_y = y_screen(item)
        doc.append(
            f'<line x1="{x}" y1="{tick_y:.2f}" x2="{x + width}" '
            f'y2="{tick_y:.2f}" stroke="#e4ded2"/>'
        )
        doc.append(
            f'<text x="{x - 8}" y="{tick_y + 4:.2f}" text-anchor="end" '
            f'class="axis">{item:.2f}</text>'
        )
    if reference is not None and value_min <= reference <= value_max:
        reference_y = y_screen(reference)
        doc.append(
            f'<line x1="{x}" y1="{reference_y:.2f}" x2="{x + width}" '
            f'y2="{reference_y:.2f}" stroke="#9a3412" '
            'stroke-width="2" stroke-dasharray="8 6"/>'
        )

    for result, values in series:
        index = results.index(result)
        rows = result["rows"]
        points = " ".join(
            f'{x_screen(number(row, "simulation_time") - result["start"]):.2f},'
            f'{y_screen(item):.2f}'
            for row, item in zip(rows, values)
        )
        stroke_width = 3 if not result["suspension"] else 2
        doc.append(
            f'<polyline points="{points}" fill="none" '
            f'stroke="{COLORS[index]}" stroke-width="{stroke_width}"/>'
        )


def write_svg(root: Path, output: Path, window: float) -> None:
    results = read_results(root, window)
    if len(results) > len(COLORS):
        raise ValueError("more landing cases than plot colors")

    width, height = 1500, 1110
    left, column_gap, panel_width = 92.0, 105.0, 605.0
    right_column = left + panel_width + column_gap
    top, panel_height, row_gap = 245.0, 205.0, 95.0
    doc = [f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="#f1eee5"/>
<style>text{{font-family:Verdana,sans-serif;fill:#28251f}} .title{{font-size:23px;font-weight:bold}} .label{{font-size:14px}} .axis{{font-size:12px;fill:#625d53}} .note{{font-size:13px;fill:#57534e}}</style>
<text x="42" y="38" class="title">200 mm landing transient: position hold vs axial suspension</text>
<text x="42" y="65" class="label">Touchdown window: first {window * 1000.0:.0f} ms. Left panels include the position-hold baseline; right panels compare impedance candidates only.</text>
<text x="42" y="91" class="note">The common initial normal-force peak is the first contact sample. Requested and applied force reveal whether the impedance candidates remain cap-dominated.</text>''']

    for index, result in enumerate(results):
        legend_x = 42.0 + (index % 3) * 455.0
        legend_y = 126.0 + (index // 3) * 31.0
        doc.append(
            f'<line x1="{legend_x}" y1="{legend_y}" '
            f'x2="{legend_x + 34}" y2="{legend_y}" '
            f'stroke="{COLORS[index]}" stroke-width="4"/>'
            f'<text x="{legend_x + 44}" y="{legend_y + 5}" '
            f'class="label">{short_name(result["summary"]["case"])}</text>'
        )

    panels = (
        (left, top, "Base vertical velocity [m/s]", lambda row: number(row, "base_vertical_velocity"), False, None),
        (right_column, top, "Retraction from touchdown length [mm]", compression_mm, True, None),
        (left, top + panel_height + row_gap, "Maximum wheel normal force [N]", lambda row: maximum_side(row, "ground_normal_force"), False, None),
        (right_column, top + panel_height + row_gap, "Requested axial force [N]", lambda row: maximum_side(row, "suspension_requested_force", True), True, 240.0),
        (left, top + 2.0 * (panel_height + row_gap), "Base height [m]", lambda row: number(row, "base_z"), False, None),
        (right_column, top + 2.0 * (panel_height + row_gap), "Applied axial force [N]", lambda row: maximum_side(row, "suspension_applied_force", True), True, 240.0),
    )
    for panel in panels:
        draw_panel(doc, results, panel[0], panel[1], panel_width,
                   panel_height, window, *panel[2:])

    doc.append(
        f'<text x="{width / 2:.0f}" y="{height - 19}" '
        'text-anchor="middle" class="label">Time after first lower-ground contact [ms]</text>'
    )
    doc.append('</svg>\n')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(doc), encoding="ascii")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_directory", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--window", type=float, default=0.3,
        help="seconds after first touchdown to plot (default: 0.3)",
    )
    arguments = parser.parse_args()
    if not math.isfinite(arguments.window) or arguments.window <= 0.0:
        parser.error("--window must be a finite positive number")
    try:
        write_svg(arguments.result_directory, arguments.output, arguments.window)
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    print(f"Wrote {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
