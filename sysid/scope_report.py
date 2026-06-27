# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "matplotlib"]
# ///
"""
Quick-look report for CSV files exported from the dashboard scope.

Usage:
  python3 sysid/scope_report.py /path/to/scope.csv
  python3 sysid/scope_report.py /path/to/scope.csv --dt-ms 30 --out-dir sysid/data/scope_reports
  python3 sysid/scope_report.py /path/to/scope.csv --figures motor --motor-panels actual,error,step --motors 2,4,6
  python3 sysid/scope_report.py /path/to/scope.csv --figures custom --panels pose.z-yaw,pose.cross,motor.error --cols 2

Outputs:
  - <stem>_motor_scope.png: motor actual/target/error report
  - <stem>_pose_scope.png: 6DoF actual/target/error report
  - <stem>_scope.summary.json: numeric feature summary
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np


MOTOR_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]

POSE_COLS = [
    ("pose_x", "pose_tgt_x", "X mm"),
    ("pose_y", "pose_tgt_y", "Y mm"),
    ("pose_z_off", "pose_tgt_z_off", "Z off mm"),
    ("pose_roll", "pose_tgt_roll", "Roll deg"),
    ("pose_pitch", "pose_tgt_pitch", "Pitch deg"),
    ("pose_yaw", "pose_tgt_yaw", "Yaw deg"),
]
POSE_AXIS_BY_NAME = {
    "x": ("pose_x", "pose_tgt_x", "X mm"),
    "y": ("pose_y", "pose_tgt_y", "Y mm"),
    "z": ("pose_z_off", "pose_tgt_z_off", "Z off mm"),
    "z_off": ("pose_z_off", "pose_tgt_z_off", "Z off mm"),
    "roll": ("pose_roll", "pose_tgt_roll", "Roll deg"),
    "pitch": ("pose_pitch", "pose_tgt_pitch", "Pitch deg"),
    "yaw": ("pose_yaw", "pose_tgt_yaw", "Yaw deg"),
}

DEFAULT_MOTOR_PANELS = ["actual", "target", "error", "step"]
DEFAULT_POSE_PANELS = ["z-yaw", "cross", "error", "kinetic"]
PANEL_IDS = [
    "motor.actual",
    "motor.target",
    "motor.error",
    "motor.step",
    "motor.kinetic",
    "pose.z-yaw",
    "pose.cross",
    "pose.error",
    "pose.kinetic",
]


def finite(v: float) -> bool:
    return isinstance(v, (int, float)) and math.isfinite(v)


def parse_float(s: str | None) -> float:
    if s is None or s == "":
        return math.nan
    try:
        return float(s)
    except ValueError:
        return math.nan


def load_csv(path: Path) -> tuple[list[str], list[dict[str, float]]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = [{k: parse_float(v) for k, v in row.items()} for row in reader]
        return list(reader.fieldnames or []), rows


def series(rows: list[dict[str, float]], key: str) -> np.ndarray:
    return np.array([r.get(key, math.nan) for r in rows], dtype=float)


def csv_list(value: str | None, default: list[str]) -> list[str]:
    if value is None or value.strip() == "":
        return list(default)
    return [x.strip() for x in value.split(",") if x.strip()]


def parse_motors(value: str | None) -> list[int]:
    if value is None or value.strip().lower() in ("", "all"):
        return [1, 2, 3, 4, 5, 6]
    out = []
    for part in value.split(","):
        motor = int(part.strip().lstrip("mM"))
        if motor < 1 or motor > 6:
            raise ValueError(f"motor index out of range: {part}")
        out.append(motor)
    return out


def parse_pose_axes(value: str | None) -> list[tuple[str, str, str]]:
    names = csv_list(value, ["x", "y", "z", "roll", "pitch", "yaw"])
    out = []
    for name in names:
        key = name.lower().replace("-", "_")
        if key not in POSE_AXIS_BY_NAME:
            raise ValueError(f"unknown pose axis: {name}")
        out.append(POSE_AXIS_BY_NAME[key])
    return out


def parse_figsize(value: str | None, default: tuple[float, float]) -> tuple[float, float]:
    if not value:
        return default
    parts = [float(x.strip()) for x in value.lower().replace("x", ",").split(",") if x.strip()]
    if len(parts) != 2 or parts[0] <= 0 or parts[1] <= 0:
        raise ValueError("--figsize must be WIDTH,HEIGHT, for example 14,10")
    return (parts[0], parts[1])


def stats(values: Iterable[float]) -> dict[str, float | int | None]:
    a = np.array([x for x in values if finite(float(x))], dtype=float)
    if len(a) == 0:
        return {
            "n": 0,
            "mean": None,
            "rms": None,
            "sd": None,
            "min": None,
            "p05": None,
            "p50": None,
            "p95": None,
            "max": None,
        }
    return {
        "n": int(len(a)),
        "mean": round(float(np.mean(a)), 4),
        "rms": round(float(np.sqrt(np.mean(a * a))), 4),
        "sd": round(float(np.std(a)), 4),
        "min": round(float(np.min(a)), 4),
        "p05": round(float(np.percentile(a, 5)), 4),
        "p50": round(float(np.percentile(a, 50)), 4),
        "p95": round(float(np.percentile(a, 95)), 4),
        "max": round(float(np.max(a)), 4),
    }


def percentile_abs(values: Iterable[float], p: float) -> float | None:
    a = np.array([abs(float(x)) for x in values if finite(float(x))], dtype=float)
    return round(float(np.percentile(a, p)), 4) if len(a) else None


def target_rows(rows: list[dict[str, float]]) -> list[int]:
    return [i for i, r in enumerate(rows) if finite(r.get("tgt1", math.nan))]


def top_kinetic(rows: list[dict[str, float]], count: int, dt_s: float) -> list[dict[str, float | int | None]]:
    ranked = sorted(
        range(len(rows)),
        key=lambda i: rows[i].get("kinetic", -math.inf) if finite(rows[i].get("kinetic", math.nan)) else -math.inf,
        reverse=True,
    )
    out = []
    for i in ranked[:count]:
        r = rows[i]
        prev = rows[i - 1] if i > 0 else None
        vz = None
        vyaw = None
        if prev:
            z0, z1 = prev.get("pose_tgt_z_off", math.nan), r.get("pose_tgt_z_off", math.nan)
            y0, y1 = prev.get("pose_tgt_yaw", math.nan), r.get("pose_tgt_yaw", math.nan)
            if finite(z0) and finite(z1):
                vz = round(float((z1 - z0) / dt_s), 3)
            if finite(y0) and finite(y1):
                vyaw = round(float((y1 - y0) / dt_s), 3)
        out.append(
            {
                "sample": int(i),
                "time_s": round(i * dt_s, 3),
                "kinetic": round(float(r.get("kinetic", math.nan)), 4),
                "target_z_velocity_per_s": vz,
                "target_yaw_velocity_per_s": vyaw,
                "z_error": round(float(r["pose_z_off"] - r["pose_tgt_z_off"]), 4)
                if finite(r.get("pose_z_off", math.nan)) and finite(r.get("pose_tgt_z_off", math.nan))
                else None,
                "yaw_error": round(float(r["pose_yaw"] - r["pose_tgt_yaw"]), 4)
                if finite(r.get("pose_yaw", math.nan)) and finite(r.get("pose_tgt_yaw", math.nan))
                else None,
            }
        )
    return out


def summarize(path: Path, rows: list[dict[str, float]], dt_s: float) -> dict:
    idx = target_rows(rows)
    target_span = [idx[0], idx[-1]] if idx else None
    summary = {
        "schema": "stewart.scope_report.v1",
        "source": str(path),
        "samples": len(rows),
        "dt_ms_assumed": round(dt_s * 1000, 3),
        "duration_s_assumed": round(max(0, len(rows) - 1) * dt_s, 3),
        "target_rows": len(idx),
        "target_span_samples": target_span,
        "blank_target_rows": len(rows) - len(idx),
        "pose": {},
        "pose_error_on_target_rows": {},
        "hold_error_deg_on_target_rows": {},
        "motor_step_deg_per_sample": {},
        "target_velocity_units_per_s": {},
        "top_kinetic": top_kinetic(rows, 12, dt_s),
    }

    for pose_key, target_key, _label in POSE_COLS:
        summary["pose"][pose_key] = stats(series(rows, pose_key))
        errs = [
            rows[i].get(pose_key, math.nan) - rows[i].get(target_key, math.nan)
            for i in idx
            if finite(rows[i].get(pose_key, math.nan)) and finite(rows[i].get(target_key, math.nan))
        ]
        summary["pose_error_on_target_rows"][pose_key] = {
            **stats(errs),
            "p95_abs": percentile_abs(errs, 95),
            "max_abs": round(float(max(map(abs, errs))), 4) if errs else None,
        }
        vel = np.diff(series(rows, target_key)) / dt_s
        summary["target_velocity_units_per_s"][target_key] = {
            "p95_abs": percentile_abs(vel, 95),
            "max_abs": round(float(np.nanmax(np.abs(vel))), 4) if np.isfinite(vel).any() else None,
        }

    for motor in range(1, 7):
        herr = [rows[i].get(f"herr{motor}", math.nan) for i in idx]
        summary["hold_error_deg_on_target_rows"][f"M{motor}"] = {
            **stats(herr),
            "p95_abs": percentile_abs(herr, 95),
            "max_abs": round(float(np.nanmax(np.abs(herr))), 4) if np.isfinite(herr).any() else None,
        }
        steps = np.diff(series(rows, f"a{motor}"))
        summary["motor_step_deg_per_sample"][f"M{motor}"] = {
            "p95_abs": percentile_abs(steps, 95),
            "max_abs": round(float(np.nanmax(np.abs(steps))), 4) if np.isfinite(steps).any() else None,
        }

    return summary


def setup_matplotlib():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def selected_time(rows: list[dict[str, float]], summary: dict) -> tuple[np.ndarray, float]:
    dt_s = summary["dt_ms_assumed"] / 1000
    return np.arange(len(rows)) * dt_s, dt_s


def add_target_span(ax, summary: dict, dt_s: float) -> None:
    span = summary.get("target_span_samples")
    if span:
        ax.axvline(span[0] * dt_s, color="r", alpha=0.14)
        ax.axvline(span[1] * dt_s, color="r", alpha=0.14)


def plot_motor_actual(ax, rows, t, motors):
    for motor in motors:
        color = MOTOR_COLORS[motor - 1]
        ax.plot(t, series(rows, f"a{motor}"), label=f"M{motor}", lw=1.3, color=color)
    ax.set_ylabel("Actual angle deg")
    ax.legend(ncol=min(6, len(motors)), fontsize=8)


def plot_motor_target(ax, rows, t, motors):
    for motor in motors:
        color = MOTOR_COLORS[motor - 1]
        ax.plot(t, series(rows, f"tgt{motor}"), label=f"T{motor}", lw=1.2, color=color)
    ax.set_ylabel("Target angle deg")
    ax.legend(ncol=min(6, len(motors)), fontsize=8)


def plot_motor_error(ax, rows, t, motors):
    for motor in motors:
        color = MOTOR_COLORS[motor - 1]
        ax.plot(t, series(rows, f"herr{motor}"), label=f"M{motor}", lw=1.2, color=color)
    ax.set_ylabel("Hold err deg")
    ax.legend(ncol=min(6, len(motors)), fontsize=8)


def plot_motor_step(ax, rows, t, motors):
    steps = []
    for motor in motors:
        a = series(rows, f"a{motor}")
        d = np.abs(np.diff(a, prepend=np.nan))
        steps.append(d)
        ax.plot(t, d, lw=0.8, alpha=0.45, color=MOTOR_COLORS[motor - 1])
    if steps:
        stack = np.vstack(steps)
        max_step = np.array([np.nanmax(col) if np.isfinite(col).any() else np.nan for col in stack.T])
    else:
        max_step = np.full(len(rows), np.nan)
    ax.plot(t, max_step, color="black", lw=1.5, label="max |motor step|")
    ax.set_ylabel("deg/sample")
    ax.legend(ncol=1, fontsize=8)


def plot_kinetic(ax, rows, t, summary):
    ax.plot(t, series(rows, "kinetic"), color="#1f77b4", lw=1.4, label="kinetic")
    for event in summary["top_kinetic"][:6]:
        ax.axvline(float(event["time_s"]), color="k", alpha=0.12)
    ax.set_ylabel("Kinetic")
    ax.legend(ncol=1, fontsize=8)


def plot_pose_zyaw(ax, rows, t):
    ax.plot(t, series(rows, "pose_z_off"), label="Z off mm", lw=1.6)
    ax.plot(t, series(rows, "pose_tgt_z_off"), "--", label="target Z off mm", lw=1.3)
    ax.plot(t, series(rows, "pose_yaw"), label="Yaw deg", lw=1.6)
    ax.plot(t, series(rows, "pose_tgt_yaw"), "--", label="target Yaw deg", lw=1.3)
    ax.set_ylabel("Z / Yaw")
    ax.legend(ncol=2, fontsize=8)


def plot_pose_cross(ax, rows, t):
    for key, label in [
        ("pose_x", "X mm"),
        ("pose_y", "Y mm"),
        ("pose_roll", "Roll deg"),
        ("pose_pitch", "Pitch deg"),
    ]:
        ax.plot(t, series(rows, key), label=label, lw=1.3)
    ax.set_ylabel("Cross axes")
    ax.legend(ncol=4, fontsize=8)


def plot_pose_error(ax, rows, t, pose_axes):
    for pose_key, target_key, label in pose_axes:
        ax.plot(t, series(rows, pose_key) - series(rows, target_key), label=label, lw=1.1)
    ax.set_ylabel("Pose error")
    ax.legend(ncol=min(6, len(pose_axes)), fontsize=8)


def panel_spec(panel_id: str, rows, t, summary, motors, pose_axes):
    if panel_id == "motor.actual":
        return "Motor actual angle", lambda ax: plot_motor_actual(ax, rows, t, motors)
    if panel_id == "motor.target":
        return "Motor target angle", lambda ax: plot_motor_target(ax, rows, t, motors)
    if panel_id == "motor.error":
        return "Motor hold error", lambda ax: plot_motor_error(ax, rows, t, motors)
    if panel_id == "motor.step":
        return "Motor step", lambda ax: plot_motor_step(ax, rows, t, motors)
    if panel_id == "motor.kinetic":
        return "Kinetic", lambda ax: plot_kinetic(ax, rows, t, summary)
    if panel_id == "pose.z-yaw":
        return "6DoF Z/Yaw target tracking", lambda ax: plot_pose_zyaw(ax, rows, t)
    if panel_id == "pose.cross":
        return "6DoF cross axes", lambda ax: plot_pose_cross(ax, rows, t)
    if panel_id == "pose.error":
        return "6DoF pose error", lambda ax: plot_pose_error(ax, rows, t, pose_axes)
    if panel_id == "pose.kinetic":
        return "Kinetic", lambda ax: plot_kinetic(ax, rows, t, summary)
    raise ValueError(f"unknown panel id: {panel_id}. valid: {', '.join(PANEL_IDS)}")


def draw_panels(
    path: Path,
    rows: list[dict[str, float]],
    summary: dict,
    out_png: Path,
    panel_ids: list[str],
    *,
    title: str,
    cols: int,
    figsize: tuple[float, float],
    motors: list[int],
    pose_axes: list[tuple[str, str, str]],
    show_target_span: bool,
) -> None:
    plt = setup_matplotlib()
    t, dt_s = selected_time(rows, summary)
    cols = max(1, min(cols, len(panel_ids)))
    rows_n = int(math.ceil(len(panel_ids) / cols))
    fig, axs = plt.subplots(rows_n, cols, figsize=figsize, sharex=True, squeeze=False)
    flat = list(axs.flat)
    for ax, panel_id in zip(flat, panel_ids):
        panel_title, draw = panel_spec(panel_id, rows, t, summary, motors, pose_axes)
        draw(ax)
        ax.set_title(panel_title, fontsize=10)
        ax.grid(True, alpha=0.25)
        if show_target_span:
            add_target_span(ax, summary, dt_s)
    for ax in flat[len(panel_ids):]:
        ax.axis("off")
    for ax in flat[-cols:]:
        if ax.has_data():
            ax.set_xlabel(f"Approx time (s, assuming {summary['dt_ms_assumed']:.0f} ms/sample)")
    fig.suptitle(title, fontsize=12)
    fig.tight_layout()
    fig.savefig(out_png, dpi=160)
    plt.close(fig)


def draw_motor_report(path: Path, rows: list[dict[str, float]], summary: dict, out_png: Path, args) -> None:
    worst_motor = max(
        summary["motor_step_deg_per_sample"].items(),
        key=lambda kv: kv[1]["max_abs"] if kv[1]["max_abs"] is not None else -math.inf,
    )
    subtitle = (
        f"samples={summary['samples']} targetRows={summary['target_rows']} "
        f"worstStep={worst_motor[0]}:{worst_motor[1]['max_abs']}deg/sample"
    )
    panel_ids = [f"motor.{p}" for p in csv_list(args.motor_panels, DEFAULT_MOTOR_PANELS)]
    draw_panels(
        path,
        rows,
        summary,
        out_png,
        panel_ids,
        title=f"{path.name}: motor scope\n{subtitle}",
        cols=args.cols,
        figsize=parse_figsize(args.figsize, (14, 10)),
        motors=parse_motors(args.motors),
        pose_axes=parse_pose_axes(args.pose_axes),
        show_target_span=not args.hide_target_span,
    )


def draw_pose_report(path: Path, rows: list[dict[str, float]], summary: dict, out_png: Path, args) -> None:
    z_err = summary["pose_error_on_target_rows"]["pose_z_off"]["p95_abs"]
    yaw_err = summary["pose_error_on_target_rows"]["pose_yaw"]["p95_abs"]
    subtitle = f"samples={summary['samples']} targetRows={summary['target_rows']} ZerrP95={z_err} YawErrP95={yaw_err}"
    panel_ids = [f"pose.{p}" for p in csv_list(args.pose_panels, DEFAULT_POSE_PANELS)]
    draw_panels(
        path,
        rows,
        summary,
        out_png,
        panel_ids,
        title=f"{path.name}: 6DoF scope\n{subtitle}",
        cols=args.cols,
        figsize=parse_figsize(args.figsize, (14, 10)),
        motors=parse_motors(args.motors),
        pose_axes=parse_pose_axes(args.pose_axes),
        show_target_span=not args.hide_target_span,
    )


def draw_custom_report(path: Path, rows: list[dict[str, float]], summary: dict, out_png: Path, args) -> None:
    panel_ids = csv_list(args.panels, [])
    if not panel_ids:
        raise ValueError("--figures custom requires --panels, for example pose.z-yaw,motor.error")
    draw_panels(
        path,
        rows,
        summary,
        out_png,
        panel_ids,
        title=f"{path.name}: custom scope",
        cols=args.cols,
        figsize=parse_figsize(args.figsize, (14, 10)),
        motors=parse_motors(args.motors),
        pose_axes=parse_pose_axes(args.pose_axes),
        show_target_span=not args.hide_target_span,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--dt-ms", type=float, default=30.0, help="scope CSV has samples only; default assumes 30 ms/sample")
    parser.add_argument("--out-dir", type=Path, default=Path("sysid/data/scope_reports"))
    parser.add_argument("--figures", default="motor,pose", help="comma list: motor,pose,custom")
    parser.add_argument("--panels", default="", help=f"custom panel ids: {', '.join(PANEL_IDS)}")
    parser.add_argument("--motor-panels", default=",".join(DEFAULT_MOTOR_PANELS), help="motor panels: actual,target,error,step,kinetic")
    parser.add_argument("--pose-panels", default=",".join(DEFAULT_POSE_PANELS), help="pose panels: z-yaw,cross,error,kinetic")
    parser.add_argument("--motors", default="all", help="motor list, for example all or 1,3,6")
    parser.add_argument("--pose-axes", default="x,y,z,roll,pitch,yaw", help="pose error axes, for example z,yaw or x,y,roll")
    parser.add_argument("--cols", type=int, default=1, help="panel columns for each figure")
    parser.add_argument("--figsize", default="", help="matplotlib figure size WIDTH,HEIGHT; default 14,10")
    parser.add_argument("--hide-target-span", action="store_true", help="do not draw red target-window markers")
    args = parser.parse_args()

    _headers, rows = load_csv(args.csv_path)
    if not rows:
        raise SystemExit("scope CSV has no rows")

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = args.csv_path.stem
    motor_png = out_dir / f"{stem}_motor_scope.png"
    pose_png = out_dir / f"{stem}_pose_scope.png"
    custom_png = out_dir / f"{stem}_custom_scope.png"
    out_json = out_dir / f"{stem}_scope.summary.json"

    summary = summarize(args.csv_path, rows, args.dt_ms / 1000.0)
    requested = csv_list(args.figures, ["motor", "pose"])
    unknown = [x for x in requested if x not in ("motor", "pose", "custom")]
    if unknown:
        raise SystemExit(f"unknown --figures value(s): {', '.join(unknown)}")
    summary["plot_options"] = {
        "figures": requested,
        "panels": csv_list(args.panels, []),
        "motor_panels": csv_list(args.motor_panels, DEFAULT_MOTOR_PANELS),
        "pose_panels": csv_list(args.pose_panels, DEFAULT_POSE_PANELS),
        "motors": parse_motors(args.motors),
        "pose_axes": [x[2] for x in parse_pose_axes(args.pose_axes)],
        "cols": args.cols,
        "figsize": parse_figsize(args.figsize, (14, 10)),
        "target_span": not args.hide_target_span,
    }
    summary["figures"] = {}
    if "motor" in requested:
        draw_motor_report(args.csv_path, rows, summary, motor_png, args)
        summary["figures"]["motor"] = str(motor_png)
    if "pose" in requested:
        draw_pose_report(args.csv_path, rows, summary, pose_png, args)
        summary["figures"]["pose"] = str(pose_png)
    if "custom" in requested:
        draw_custom_report(args.csv_path, rows, summary, custom_png, args)
        summary["figures"]["custom"] = str(custom_png)
    out_json.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")

    print(
        json.dumps(
            {"figures": summary["figures"], "summary": str(out_json), "samples": len(rows)},
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
