#!/usr/bin/env python3
"""Plot x tracking, force, acceleration, and joint torque from a Cartesian dynamics run."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


PANELS = (
    ("Base X travel from contact start", "Travel (mm)", (
        ("pdes_x", "Reference", "--"), ("pcur_x", "Measured", "-"),
    )),
    ("Base X velocity", "Velocity (m/s)", (
        ("vdes_x", "Reference", "--"), ("vcur_x", "Measured", "-"),
    )),
    ("Base Z force (robot on environment)", "Force (N)", (
        ("fdes_z", "Reference", "--"), ("fcur_z", "Measured", "-"),
        ("f_feedback_z", "Feedback after filter and clamp", ":"),
        ("f_cmd_z", "Force controller command", "-."),
    )),
    ("Base X force", "Force (N)", (
        ("wrench_x", "Model wrench command", "--"), ("fcur_x", "Measured", "-"),
    )),
    ("Base X acceleration", "Acceleration (m/s²)", (
        ("a_requested_x", "Requested", "--"), ("a_cmd_x", "After limit", "-"),
        ("a_integral_x", "Integral contribution", ":"),
    )),
    ("Joint torque commands including gravity", "Torque (N·m)", (
        ("tau_cmd_2", "Joint 2", "-"), ("tau_cmd_4", "Joint 4", "-"),
    )),
)


def load_csv(path: Path) -> dict[str, list[float]]:
    columns = ("time_s",) + tuple(
        name for _, _, traces in PANELS for name, _, _ in traces
    )
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fieldnames = set(reader.fieldnames or [])
        missing = set(columns) - fieldnames - {"a_integral_x"}
        if missing:
            raise ValueError(f"Missing CSV columns: {', '.join(sorted(missing))}")
        columns = tuple(name for name in columns if name in fieldnames)
        data: dict[str, list[float]] = {name: [] for name in columns}
        for line, row in enumerate(reader, start=2):
            try:
                values = [float(row[name]) for name in columns]
            except (ValueError, TypeError) as exc:
                raise ValueError(f"Invalid or missing number on CSV line {line}") from exc
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"Nonfinite number on CSV line {line}")
            if data["time_s"] and values[0] < data["time_s"][-1]:
                raise ValueError(f"Timestamp decreases on CSV line {line}")
            for name, value in zip(columns, values):
                data[name].append(value)
    if not data["time_s"]:
        raise ValueError("CSV has no data rows")
    return data


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="Path to an identified or nominal Cartesian run CSV")
    parser.add_argument("--save", type=Path, help="Save figure (defaults to PNG if no extension)")
    parser.add_argument("--no-show", action="store_true", help="Save without opening a window; requires --save")
    args = parser.parse_args()
    if args.no_show and args.save is None:
        parser.error("--no-show requires --save")

    try:
        data = load_csv(args.csv)
        import matplotlib
        if args.no_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except (ValueError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except ImportError:
        print("error: matplotlib is required (python3 -m pip install matplotlib)", file=sys.stderr)
        return 1

    x_start = data["pdes_x"][0]
    for name in ("pdes_x", "pcur_x"):
        data[name] = [1000.0 * (x - x_start) for x in data[name]]
    time = data["time_s"]
    marker = "." if len(time) == 1 else None
    fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True, constrained_layout=True)
    fig.suptitle(args.csv.name)
    for ax, (title, units, traces) in zip(axes.flat, PANELS):
        for name, label, style in traces:
            if name not in data:
                continue
            ax.plot(time, data[name], style, label=label, marker=marker, linewidth=1.2)
        ax.set_title(title)
        ax.set_ylabel(units)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize="small")
    for ax in axes[-1]:
        ax.set_xlabel("Time since contact start (s)")

    if args.save:
        output = args.save if args.save.suffix else args.save.with_suffix(".png")
        try:
            fig.savefig(output, dpi=150, bbox_inches="tight")
        except (OSError, ValueError) as exc:
            plt.close(fig)
            print(f"error: {exc}", file=sys.stderr)
            return 1
        print(f"Wrote {output}")
    if not args.no_show:
        plt.show()
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
