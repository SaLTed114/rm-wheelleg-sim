#!/usr/bin/env python3
"""Plot drop benchmark pitch and leg-angle responses as an SVG."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


POLICIES = ("length_only", "leg_lqr")
COLORS = {
    "length_only": "#d97706",
    "leg_lqr": "#16856b",
}
PITCH_RATES = (-0.5, 0.0, 0.5)


def case_name(policy: str, pitch_rate: float) -> str:
    if pitch_rate > 0.0:
        rate = "pos"
    elif pitch_rate < 0.0:
        rate = "neg"
    else:
        rate = "zero"
    return f"{policy}_pitch_rate_{rate}_{abs(pitch_rate):.1f}".replace(".", "p")


def load_case(root: Path, policy: str, pitch_rate: float) -> list[dict[str, str]]:
    trace = root / case_name(policy, pitch_rate) / "trace.csv"
    with trace.open(newline="", encoding="ascii") as source:
        rows = list(csv.DictReader(source))
    if not rows:
        raise ValueError(f"empty trace: {trace}")
    return rows


def write_svg(root: Path, output: Path) -> None:
    traces = {
        (policy, rate): load_case(root, policy, rate)
        for policy in POLICIES for rate in PITCH_RATES
    }
    width, height = 1180, 920
    left, right = 82.0, 35.0
    panel_top, panel_height, panel_gap = 125.0, 210.0, 55.0
    plot_width = width - left - right
    time_max = max(
        float(row["release_elapsed"])
        for rows in traces.values() for row in rows)
    with (root / "summary.csv").open(newline="", encoding="ascii") as source:
        summary = next(csv.DictReader(source))
    clearance_mm = 1000.0 * float(summary["wheel_clearance_target"])

    def x_screen(time: float) -> float:
        return left + plot_width * time / time_max

    documents = [f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="#f3efe5"/>
<style>text{{font-family:Verdana,sans-serif;fill:#25241f}} .title{{font-size:22px;font-weight:bold}} .label{{font-size:13px}} .axis{{font-size:12px;fill:#5d5a51}}</style>
<text x="42" y="38" class="title">{clearance_mm:.0f} mm wheel-clearance drop comparison</text>
<text x="42" y="62" class="label">Solid: +0.5 rad/s; dashed: 0; dotted: -0.5 rad/s. Vertical marks show first wheel touchdown.</text>''']
    legend_x = 42.0
    for policy in POLICIES:
        color = COLORS[policy]
        documents.append(
            f'<line x1="{legend_x}" y1="91" x2="{legend_x + 38}" y2="91" '
            f'stroke="{color}" stroke-width="4"/><text x="{legend_x + 47}" '
            f'y="96" class="label">{policy}</text>')
        legend_x += 255.0

    panels = (
        ("pitch", "Pitch [deg]", lambda row: math.degrees(float(row["pitch"]))),
        ("pitch_rate", "Pitch rate [rad/s]", lambda row: float(row["pitch_rate"])),
        ("leg", "Common leg angle [deg]", lambda row: math.degrees(
            0.5 * (float(row["theta_l"]) + float(row["theta_r"])))),
    )
    dash = {-0.5: "2 5", 0.0: "9 6", 0.5: ""}
    for panel_index, (_, label, value) in enumerate(panels):
        y_top = panel_top + panel_index * (panel_height + panel_gap)
        values = [value(row) for rows in traces.values() for row in rows]
        value_min, value_max = min(values), max(values)
        margin = max(0.1, 0.12 * (value_max - value_min))
        value_min -= margin
        value_max += margin

        def y_screen(number: float) -> float:
            return y_top + panel_height * (value_max - number) / (value_max - value_min)

        documents.append(
            f'<rect x="{left}" y="{y_top}" width="{plot_width}" height="{panel_height}" '
            'fill="#fffdf8" stroke="#8a8579"/>')
        if value_min <= 0.0 <= value_max:
            zero_y = y_screen(0.0)
            documents.append(
                f'<line x1="{left}" y1="{zero_y:.2f}" x2="{left + plot_width}" '
                f'y2="{zero_y:.2f}" stroke="#bbb5a8" stroke-width="1"/>')
        documents.append(
            f'<text x="{left}" y="{y_top - 10}" class="label">{label}</text>')
        documents.append(
            f'<text x="{left - 8}" y="{y_top + 5}" text-anchor="end" class="axis">{value_max:.2f}</text>')
        documents.append(
            f'<text x="{left - 8}" y="{y_top + panel_height}" text-anchor="end" class="axis">{value_min:.2f}</text>')

        for policy in POLICIES:
            for rate in PITCH_RATES:
                rows = traces[(policy, rate)]
                stride = max(1, len(rows) // 1300)
                selected = rows[::stride]
                if selected[-1] is not rows[-1]:
                    selected.append(rows[-1])
                points = " ".join(
                    f'{x_screen(float(row["release_elapsed"])):.2f},'
                    f'{y_screen(value(row)):.2f}' for row in selected)
                dash_attribute = (
                    f' stroke-dasharray="{dash[rate]}"' if dash[rate] else "")
                documents.append(
                    f'<polyline points="{points}" fill="none" stroke="{COLORS[policy]}" '
                    f'stroke-width="2.2"{dash_attribute}/>')
                touchdown = next(
                    (row for row in rows if row["touchdown_latched"] == "1"), None)
                if touchdown is not None:
                    touch_x = x_screen(float(touchdown["release_elapsed"]))
                    documents.append(
                        f'<line x1="{touch_x:.2f}" y1="{y_top}" x2="{touch_x:.2f}" '
                        f'y2="{y_top + 10}" stroke="{COLORS[policy]}" stroke-width="2"/>')

        if panel_index == len(panels) - 1:
            for tick in range(6):
                time = time_max * tick / 5.0
                x = x_screen(time)
                documents.append(
                    f'<line x1="{x:.2f}" y1="{y_top + panel_height}" x2="{x:.2f}" '
                    f'y2="{y_top + panel_height + 6}" stroke="#5d5a51"/>')
                documents.append(
                    f'<text x="{x:.2f}" y="{y_top + panel_height + 23}" '
                    f'text-anchor="middle" class="axis">{time:.1f}</text>')
            documents.append(
                f'<text x="{left + plot_width / 2}" y="{y_top + panel_height + 43}" '
                'text-anchor="middle" class="label">Time since release [s]</text>')
    documents.append('</svg>\n')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(documents), encoding="ascii")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_directory", type=Path)
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()
    try:
        write_svg(arguments.result_directory, arguments.output)
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    print(f"Wrote {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
