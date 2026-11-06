#!/usr/bin/env python3
"""Show joint tracking data in one figure without saving files."""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import re
import sys

DATA_DIR = Path(__file__).resolve().parents[1] / "data"


def find_logs():
    patterns = (
        "dyncomp_control/dyncomp_tracking_*.csv",
        "comptorq_control/comptorq_tracking_*.csv",
        "joint_computed_torque/joint_tracking_*.csv",
    )
    return [path for pattern in patterns for path in DATA_DIR.glob(pattern)]


def load_csv(path, selected=None):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        names = reader.fieldnames or []
        available = sorted(int(m.group(1)) for name in names
                           for m in [re.fullmatch(r"q_des_rad_j(\d+)", name)] if m)
        joints = list(dict.fromkeys(selected)) if selected else available
        if not joints or any(j not in available for j in joints):
            raise ValueError(f"Requested joints unavailable; CSV contains {available}")
        groups = ("q_des_rad", "q_meas_rad", "dq_des_rad_s", "dq_meas_rad_s", "ddq_des_rad_s2")
        columns = ["time_s"] + [f"{group}_j{j}" for group in groups for j in joints]
        missing = set(columns) - set(names)
        if missing:
            raise ValueError("Missing columns: " + ", ".join(sorted(missing)))
        data = {name: [] for name in columns}
        for line, row in enumerate(reader, 2):
            try:
                values = [float(row[name]) for name in columns]
            except (ValueError, TypeError) as exc:
                raise ValueError(f"Invalid number at line {line}") from exc
            if not all(math.isfinite(v) for v in values):
                raise ValueError(f"Nonfinite number at line {line}")
            if data["time_s"] and values[0] < data["time_s"][-1]:
                raise ValueError(f"Timestamp decreases at line {line}")
            for name, value in zip(columns, values):
                data[name].append(value)
    if not data["time_s"]:
        raise ValueError("CSV contains no samples")
    # Compute from raw signals so the sign convention is explicit and old CSVs work.
    for j in joints:
        for prefix, desired, measured in (("q_error_rad", "q_des_rad", "q_meas_rad"),
                                          ("dq_error_rad_s", "dq_des_rad_s", "dq_meas_rad_s")):
            data[f"{prefix}_j{j}"] = [d-m for d,m in zip(data[f"{desired}_j{j}"], data[f"{measured}_j{j}"])]
    return data, joints


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?", type=Path, help="CSV path; defaults to most recently modified tracking log")
    parser.add_argument("--joints", nargs="+", type=int, default=[2, 4],
                        help="1-based joint numbers to plot (default: 2 4)")
    parser.add_argument("--show", action="store_true", help=argparse.SUPPRESS)  # Compatibility; always show.
    args = parser.parse_args()
    try:
        path = args.csv
        if path is None:
            logs = find_logs()
            if not logs:
                raise ValueError(f"No tracking CSV files in {DATA_DIR}")
            path = max(logs, key=lambda p: p.stat().st_mtime_ns)
        data, joints = load_csv(path, args.joints)
        import matplotlib.pyplot as plt
        plots = [
            ("position", "Position (rad)", "q_des_rad", "q_meas_rad"),
            ("velocity", "Velocity (rad/s)", "dq_des_rad_s", "dq_meas_rad_s"),
            ("position_error", "Desired − measured (rad)", "q_error_rad", None),
            ("velocity_error", "Desired − measured (rad/s)", "dq_error_rad_s", None),
            ("desired_acceleration", "Acceleration (rad/s²)", "ddq_des_rad_s2", None),
        ]
        fig, axes = plt.subplots(len(joints), len(plots),
                                 figsize=(18, max(3, min(14, 2.5*len(joints)))),
                                 sharex=True, squeeze=False, constrained_layout=True)
        fig.suptitle(path.stem)
        for col, (stem, ylabel, first, second) in enumerate(plots):
            for row, j in enumerate(joints):
                ax = axes[row, col]
                ax.plot(data["time_s"], data[f"{first}_j{j}"],
                        label="Desired" if second or stem == "desired_acceleration" else "Error", linewidth=1.3)
                if second:
                    ax.plot(data["time_s"], data[f"{second}_j{j}"], label="Measured", linewidth=1, alpha=.85)
                if "error" in stem:
                    ax.axhline(0, color="black", linewidth=.7, linestyle="--")
                if row == 0:
                    ax.set_title(stem.replace("_", " ").title())
                    ax.legend(loc="upper right", fontsize=8)
                ax.set_ylabel(f"J{j}: {ylabel}", fontsize=8)
                ax.tick_params(labelsize=8)
                ax.grid(True, alpha=.3)
                if row == len(joints)-1:
                    ax.set_xlabel("Time (s)")
        print(f"Input: {path}\nSamples: {len(data['time_s'])}; joints: {joints}")
        plt.show()
    except ImportError:
        print("error: matplotlib is required. Use the id Conda environment or install matplotlib.", file=sys.stderr)
        return 1
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
