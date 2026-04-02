#!/usr/bin/env python3
"""
Read hybrid_task_datalog_*.csv and plot in one figure (2 subplots):
  - Top: desired vs current velocity, task-frame x
  - Bottom: desired vs current force, task-frame z
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path


def load_csv(path: Path) -> tuple[list[float], list[float], list[float], list[float]]:
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header row")
        fields = [h.strip() for h in reader.fieldnames]
        required = ("vdes_x", "vcur_x", "fdes_z", "fcur_z")
        for name in required:
            if name not in fields:
                raise ValueError(
                    f"Missing column {name!r}. Found: {reader.fieldnames}"
                )
        vdes_x: list[float] = []
        vcur_x: list[float] = []
        fdes_z: list[float] = []
        fcur_z: list[float] = []
        for row in reader:
            vdes_x.append(float(row["vdes_x"]))
            vcur_x.append(float(row["vcur_x"]))
            fdes_z.append(float(row["fdes_z"]))
            fcur_z.append(float(row["fcur_z"]))
    return vdes_x, vcur_x, fdes_z, fcur_z


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Plot velocity x and force z from hybrid task CSV datalog."
    )
    parser.add_argument(
        "csv",
        type=Path,
        help="Path to hybrid_task_datalog_*.csv",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=None,
        metavar="SEC",
        help="If set, x-axis is time (s) = sample_index * dt; else x-axis is sample index",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Optional output path for PNG (e.g. out.png)",
    )
    args = parser.parse_args()

    if not args.csv.is_file():
        print(f"error: not a file: {args.csv}", file=sys.stderr)
        return 1

    try:
        vdes_x, vcur_x, fdes_z, fcur_z = load_csv(args.csv)
    except (ValueError, KeyError, OSError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    n = len(vdes_x)
    if n == 0:
        print("error: no data rows", file=sys.stderr)
        return 1

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print(
            "error: matplotlib is required (e.g. pip install matplotlib)",
            file=sys.stderr,
        )
        return 1

    if args.dt is not None:
        x = [i * args.dt for i in range(n)]
        xlabel = "Time (s)"
    else:
        x = list(range(n))
        xlabel = "Sample"

    fig, (ax_vel, ax_force) = plt.subplots(
        2,
        1,
        figsize=(9, 7),
        sharex=True,
        constrained_layout=True,
    )

    ax_vel.plot(x, vdes_x, label=r"$v_{\mathrm{des},x}$")
    ax_vel.plot(x, vcur_x, label=r"$v_{\mathrm{cur},x}$")
    ax_vel.set_ylabel("Velocity (m/s)")
    ax_vel.set_title("Task-frame velocity — X")
    ax_vel.legend()
    ax_vel.grid(True, alpha=0.3)

    ax_force.plot(x, fdes_z, label=r"$f_{\mathrm{des},z}$")
    ax_force.plot(x, fcur_z, label=r"$f_{\mathrm{cur},z}$")
    ax_force.set_xlabel(xlabel)
    ax_force.set_ylabel("Force (N)")
    ax_force.set_title("Task-frame force — Z")
    ax_force.legend()
    ax_force.grid(True, alpha=0.3)

    fig.suptitle("Hybrid task datalog", fontsize=11, y=1.02)

    if args.save is not None:
        out = args.save
        if not out.suffix:
            out = out.with_suffix(".png")
        fig.savefig(out, dpi=150, bbox_inches="tight")
        print(f"Wrote {out}")

    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
