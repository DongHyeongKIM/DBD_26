#!/usr/bin/env python3
"""Analyse charger CSV logs and report charge-cycle and RPT trends."""

from __future__ import annotations

import argparse
import json
import math
import re
import unicodedata
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


CHARGE_FILE_RE = re.compile(
    r"(?P<date>\d{4}-\d{2}-\d{2})_(?P<pack>[A-Za-z]+)_(?P<rpt>\d+)_"
    r"CH_(?P<charge>\d+)\s*\.csv$",
    re.IGNORECASE,
)
CELL_COLUMNS = [f"cell_voltage_{number}_V" for number in range(1, 7)]
REQUIRED_COLUMNS = {
    "time_s", "voltage_V", "current_A", "capacity_mAh", "power_W", "energy_Wh",
    "temperature_C", "balance_mV", *CELL_COLUMNS,
}
TREND_METRICS = [
    ("duration_min", "Charge duration [min]"),
    ("charged_capacity_Ah", "Charged capacity [Ah]"),
    ("energy_Wh", "Charged energy [Wh]"),
    ("start_voltage_V", "Start voltage [V]"),
    ("mean_current_A", "Mean current [A]"),
    ("end_current_A", "End current [A]"),
    ("max_temperature_C", "Maximum temperature [°C]"),
    ("temperature_rise_C", "Temperature rise [°C]"),
    ("p95_cell_imbalance_mV", "95th percentile cell imbalance [mV]"),
    ("cv_duration_min", "Estimated CV duration [min]"),
]


@dataclass(frozen=True)
class ChargeInfo:
    date: str
    pack: str
    rpt: int
    charge: int
    path: Path

    @property
    def session_id(self) -> str:
        return f"{self.pack}_{self.rpt}_CH_{self.charge}"


def _normal(text: str) -> str:
    return unicodedata.normalize("NFC", text.strip())


def parse_charge_file(path: Path) -> ChargeInfo:
    match = CHARGE_FILE_RE.fullmatch(_normal(path.name))
    if not match:
        raise ValueError(f"Cannot parse charge log filename: {path.name}")
    fields = match.groupdict()
    return ChargeInfo(
        date=fields["date"],
        pack=fields["pack"].upper(),
        rpt=int(fields["rpt"]),
        charge=int(fields["charge"]),
        path=path.resolve(),
    )


def discover_charge_logs(dataset: Path) -> list[ChargeInfo]:
    logs = []
    for path in dataset.rglob("*.csv"):
        if CHARGE_FILE_RE.fullmatch(_normal(path.name)):
            logs.append(parse_charge_file(path))
    return sorted(logs, key=lambda item: (item.pack, item.rpt, item.charge, item.date))


def _column_name(raw: str) -> str | None:
    name = _normal(raw)
    direct = {
        "Time [hh:mm:ss.SSS]": "time_s",
        "Voltage [---]": "voltage_V",
        "Current [---]": "current_A",
        "Capacity [---]": "capacity_mAh",
        "Power [---]": "power_W",
        "Energy [---]": "energy_Wh",
        "Temperature Int [---]": "temperature_C",
        "BatteryRi [---]": "battery_resistance_mOhm",
        "Balance [---]": "balance_mV",
    }
    if name in direct:
        return direct[name]
    cell = re.fullmatch(r"CellVoltage\s+(\d+)\s+\[---\]", name, re.IGNORECASE)
    return f"cell_voltage_{cell.group(1)}_V" if cell else None


def _elapsed_seconds(values: pd.Series) -> pd.Series:
    text = values.astype(str).str.strip()
    parts = text.str.extract(
        r"^(?:(?P<hours>\d+):)?(?P<minutes>\d+):(?P<seconds>\d+(?:\.\d+)?)$"
    ).apply(pd.to_numeric, errors="coerce")
    return parts["hours"].fillna(0) * 3600.0 + parts["minutes"] * 60.0 + parts["seconds"]


def load_charge_log(path: Path) -> pd.DataFrame:
    if not path.is_file() or path.stat().st_size == 0:
        raise FileNotFoundError("Charge log is missing or empty")
    frame = pd.read_csv(path, sep=";", skiprows=1, skipinitialspace=True, low_memory=False)
    rename = {column: mapped for column in frame.columns if (mapped := _column_name(column))}
    frame = frame.rename(columns=rename)
    missing = sorted(REQUIRED_COLUMNS - set(frame.columns))
    if missing:
        raise ValueError(f"Missing columns: {', '.join(missing)}")
    frame = frame[list(REQUIRED_COLUMNS)].copy()
    frame["time_s"] = _elapsed_seconds(frame["time_s"])
    for column in frame.columns.difference(["time_s"]):
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    frame = frame.dropna(subset=["time_s"]).sort_values("time_s", kind="stable")
    frame = frame.drop_duplicates(subset=["time_s"], keep="last").reset_index(drop=True)
    if frame.empty:
        raise ValueError("Charge log has no valid samples")
    return frame


def _finite(values: pd.Series | np.ndarray) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    return array[np.isfinite(array)]


def _stat(values: pd.Series | np.ndarray, operation: str) -> float:
    finite = _finite(values)
    return float(getattr(np, operation)(finite)) if finite.size else math.nan


def _integral(y: np.ndarray, x: np.ndarray) -> float:
    valid = np.isfinite(x) & np.isfinite(y)
    x, y = x[valid], y[valid]
    if len(x) < 2:
        return math.nan
    increasing = np.r_[True, np.diff(x) > 0]
    x, y = x[increasing], y[increasing]
    if len(x) < 2:
        return math.nan
    if hasattr(np, "trapezoid"):
        return float(np.trapezoid(y, x))
    return float(np.trapz(y, x))


def _first_time_at_fraction(time: np.ndarray, values: np.ndarray, fraction: float) -> float:
    valid = np.isfinite(values)
    if not valid.any():
        return math.nan
    start = float(values[valid][0])
    finish = float(np.nanmax(values))
    target = start + fraction * (finish - start)
    matches = np.flatnonzero(valid & (values >= target))
    return float(time[matches[0]]) if matches.size else math.nan


def calculate_metrics(data: pd.DataFrame, info: ChargeInfo,
                      taper_current_A: float) -> dict[str, object]:
    time = data["time_s"].to_numpy(dtype=float)
    voltage = data["voltage_V"].to_numpy(dtype=float)
    current = data["current_A"].to_numpy(dtype=float)
    power = data["power_W"].to_numpy(dtype=float)
    capacity = data["capacity_mAh"].to_numpy(dtype=float)
    energy = data["energy_Wh"].to_numpy(dtype=float)
    temperature = data["temperature_C"].to_numpy(dtype=float)
    cell_values = data[CELL_COLUMNS].to_numpy(dtype=float)
    active_cell_mask = np.any(np.isfinite(cell_values) & (cell_values > 0), axis=0)
    cell_count = int(active_cell_mask.sum())
    active_cell_values = cell_values[:, active_cell_mask]
    active_cell_values[active_cell_values <= 0] = np.nan
    if cell_count:
        imbalance = np.nanmax(active_cell_values, axis=1) - np.nanmin(active_cell_values, axis=1)
        mean_cell_voltage = np.nanmean(active_cell_values, axis=1)
    else:
        imbalance = np.full(len(data), np.nan)
        mean_cell_voltage = np.full(len(data), np.nan)

    duration_s = float(time[-1] - time[0]) if len(time) > 1 else 0.0
    positive_dt = np.diff(time)
    positive_dt = positive_dt[np.isfinite(positive_dt) & (positive_dt > 0)]
    cv_matches = np.flatnonzero(mean_cell_voltage >= 4.18)
    cv_start_s = float(time[cv_matches[0]] - time[0]) if cv_matches.size else math.nan
    tail_size = max(1, min(30, len(data)))
    end_current = float(current[-1])
    completion_reached = bool(np.nanmin(current[-tail_size:]) <= taper_current_A)

    return {
        "pack": info.pack,
        "rpt": info.rpt,
        "charge": info.charge,
        "date": info.date,
        "session_id": info.session_id,
        "source_file": str(info.path),
        "status": "ok",
        "sample_count": len(data),
        "sample_interval_s": _stat(positive_dt, "median"),
        "duration_s": duration_s,
        "duration_min": duration_s / 60.0,
        "cell_count": cell_count,
        "start_voltage_V": float(voltage[0]),
        "end_voltage_V": float(voltage[-1]),
        "voltage_gain_V": float(voltage[-1] - voltage[0]),
        "max_voltage_V": _stat(voltage, "max"),
        "start_current_A": float(current[0]),
        "end_current_A": end_current,
        "mean_current_A": _stat(current, "mean"),
        "max_current_A": _stat(current, "max"),
        "mean_power_W": _stat(power, "mean"),
        "max_power_W": _stat(power, "max"),
        "charged_capacity_Ah": float((np.nanmax(capacity) - capacity[0]) / 1000.0),
        "integrated_capacity_Ah": _integral(np.maximum(current, 0), time) / 3600.0,
        "energy_Wh": float(np.nanmax(energy) - energy[0]),
        "integrated_energy_Wh": _integral(np.maximum(power, 0), time) / 3600.0,
        "time_to_80pct_min": _first_time_at_fraction(time - time[0], capacity, 0.80) / 60.0,
        "time_to_90pct_min": _first_time_at_fraction(time - time[0], capacity, 0.90) / 60.0,
        "time_to_95pct_min": _first_time_at_fraction(time - time[0], capacity, 0.95) / 60.0,
        "cv_start_min": cv_start_s / 60.0,
        "cv_duration_min": (duration_s - cv_start_s) / 60.0 if np.isfinite(cv_start_s) else math.nan,
        "start_temperature_C": float(temperature[0]),
        "end_temperature_C": float(temperature[-1]),
        "max_temperature_C": _stat(temperature, "max"),
        "temperature_rise_C": float(temperature[-1] - temperature[0]),
        "start_cell_imbalance_mV": float(imbalance[0] * 1000.0),
        "end_cell_imbalance_mV": float(imbalance[-1] * 1000.0),
        "mean_cell_imbalance_mV": _stat(imbalance * 1000.0, "mean"),
        "p95_cell_imbalance_mV": (float(np.nanpercentile(imbalance * 1000.0, 95))
                                    if np.isfinite(imbalance).any() else math.nan),
        "max_cell_imbalance_mV": _stat(imbalance * 1000.0, "max"),
        "imbalance_over_50mV_pct": float(np.nanmean(imbalance > 0.050) * 100.0),
        "charger_max_balance_mV": _stat(data["balance_mV"], "max"),
        "taper_current_threshold_A": taper_current_A,
        "taper_current_reached": completion_reached,
    }


def _downsample(frame: pd.DataFrame, max_points: int = 5000) -> pd.DataFrame:
    return frame.iloc[::max(1, math.ceil(len(frame) / max_points))]


def plot_charge_profile(data: pd.DataFrame, info: ChargeInfo, destination: Path) -> None:
    frame = _downsample(data)
    elapsed_min = (frame["time_s"] - frame["time_s"].iloc[0]) / 60.0
    cells = frame[CELL_COLUMNS].replace(0, np.nan)
    imbalance = (cells.max(axis=1) - cells.min(axis=1)) * 1000.0
    fig, axes = plt.subplots(4, 1, figsize=(13, 14), sharex=True, constrained_layout=True)
    axes[0].plot(elapsed_min, frame["voltage_V"], color="tab:blue", label="voltage")
    current_axis = axes[0].twinx()
    current_axis.plot(elapsed_min, frame["current_A"], color="tab:red", label="current", alpha=0.75)
    axes[0].set_ylabel("Pack voltage [V]")
    current_axis.set_ylabel("Current [A]")
    handles = axes[0].lines + current_axis.lines
    axes[0].legend(handles, [item.get_label() for item in handles], loc="best")
    for number, column in enumerate(CELL_COLUMNS, 1):
        if cells[column].notna().any():
            axes[1].plot(elapsed_min, cells[column], label=f"cell {number}", lw=0.8)
    axes[1].set_ylabel("Cell voltage [V]")
    axes[1].legend(ncol=4)
    axes[2].plot(elapsed_min, frame["temperature_C"], color="tab:red", label="temperature")
    imbalance_axis = axes[2].twinx()
    imbalance_axis.plot(elapsed_min, imbalance, color="tab:purple", alpha=0.7, label="imbalance")
    axes[2].set_ylabel("Temperature [°C]")
    imbalance_axis.set_ylabel("Cell imbalance [mV]")
    handles = axes[2].lines + imbalance_axis.lines
    axes[2].legend(handles, [item.get_label() for item in handles], loc="best")
    axes[3].plot(elapsed_min, frame["capacity_mAh"] / 1000.0, label="capacity [Ah]")
    axes[3].plot(elapsed_min, frame["energy_Wh"], label="energy [Wh]")
    axes[3].set_ylabel("Cumulative charge")
    axes[3].set_xlabel("Elapsed time [min]")
    axes[3].legend()
    for axis in axes:
        axis.grid(alpha=0.25)
    fig.suptitle(f"{info.session_id} charge profile")
    fig.savefig(destination, dpi=160)
    plt.close(fig)


def plot_trend_grid(frame: pd.DataFrame, destination: Path, title: str,
                    group: str | None = None) -> None:
    fig, axes = plt.subplots(4, 3, figsize=(18, 18), constrained_layout=True)
    for axis, (column, label) in zip(axes.flat, TREND_METRICS):
        parts = frame.groupby(group) if group else [(None, frame)]
        for name, part in parts:
            part = part.sort_values("charge")
            series_label = f"RPT {int(name)}" if name is not None else None
            axis.plot(part["charge"], part[column], marker="o", label=series_label)
        axis.set_xlabel("Charge number")
        axis.set_ylabel(label)
        axis.set_xticks(sorted(frame["charge"].unique()))
        axis.grid(alpha=0.25)
    for axis in list(axes.flat)[len(TREND_METRICS):]:
        axis.set_visible(False)
    if group:
        axes.flat[0].legend(title="RPT")
    fig.suptitle(title)
    fig.savefig(destination, dpi=160)
    plt.close(fig)


def plot_normalized_profiles(logs: Sequence[tuple[ChargeInfo, pd.DataFrame]],
                             destination: Path, title: str) -> None:
    fig, axes = plt.subplots(4, 1, figsize=(13, 15), sharex=True, constrained_layout=True)
    for info, raw in logs:
        frame = _downsample(raw, 1200)
        duration = frame["time_s"].iloc[-1] - frame["time_s"].iloc[0]
        progress = (frame["time_s"] - frame["time_s"].iloc[0]) / duration * 100.0 if duration else 0.0
        cells = frame[CELL_COLUMNS].replace(0, np.nan)
        label = f"CH {info.charge}"
        axes[0].plot(progress, frame["voltage_V"], label=label, lw=0.8)
        axes[1].plot(progress, frame["current_A"], label=label, lw=0.8)
        axes[2].plot(progress, frame["temperature_C"], label=label, lw=0.8)
        axes[3].plot(progress, (cells.max(axis=1) - cells.min(axis=1)) * 1000.0,
                     label=label, lw=0.8)
    labels = ("Pack voltage [V]", "Current [A]", "Temperature [°C]", "Cell imbalance [mV]")
    for axis, label in zip(axes, labels):
        axis.set_ylabel(label)
        axis.grid(alpha=0.25)
    axes[0].legend(ncol=5, fontsize=8)
    axes[-1].set_xlabel("Charge progress [% of logged duration]")
    fig.suptitle(title)
    fig.savefig(destination, dpi=160)
    plt.close(fig)


def _slope(values: pd.Series, x: pd.Series) -> float:
    valid = values.notna() & x.notna()
    return float(np.polyfit(x[valid], values[valid], 1)[0]) if valid.sum() >= 2 else math.nan


def write_summary(row: dict[str, object], path: Path) -> None:
    pd.DataFrame([row]).to_csv(path, index=False, encoding="utf-8-sig", float_format="%.8g")


def create_group_outputs(summaries: pd.DataFrame,
                         loaded: Sequence[tuple[ChargeInfo, pd.DataFrame]],
                         result_root: Path) -> pd.DataFrame:
    trend_rows = []
    for (pack, rpt), group in summaries.groupby(["pack", "rpt"], sort=True):
        output = result_root / "rpt" / str(pack) / f"RPT_{int(rpt)}"
        output.mkdir(parents=True, exist_ok=True)
        group = group.sort_values("charge")
        group.to_csv(output / "charge_trends.csv", index=False, encoding="utf-8-sig",
                     float_format="%.8g")
        plot_trend_grid(group, output / "charge_trends.png",
                        f"{pack} RPT {int(rpt)}: charging changes")
        group_logs = [(info, data) for info, data in loaded
                      if info.pack == pack and info.rpt == rpt]
        plot_normalized_profiles(group_logs, output / "normalized_charge_profiles.png",
                                 f"{pack} RPT {int(rpt)}: normalized charge profiles")
        trend_row: dict[str, object] = {
            "pack": pack,
            "rpt": int(rpt),
            "available_charges": len(group),
            "charge_numbers": ",".join(map(str, group["charge"].astype(int))),
        }
        for metric, _ in TREND_METRICS:
            trend_row[f"{metric}_mean"] = float(group[metric].mean())
            trend_row[f"{metric}_std"] = float(group[metric].std(ddof=1)) if len(group) > 1 else math.nan
            trend_row[f"{metric}_slope_per_charge"] = _slope(group[metric], group["charge"])
        trend_rows.append(trend_row)
        write_summary(trend_row, output / "rpt_charge_summary.csv")
    return pd.DataFrame(trend_rows)


def run_all(dataset: Path, result_root: Path, taper_current_A: float) -> None:
    logs = discover_charge_logs(dataset)
    rows: list[dict[str, object]] = []
    quality: list[dict[str, object]] = []
    loaded: list[tuple[ChargeInfo, pd.DataFrame]] = []
    for info in logs:
        base = {"pack": info.pack, "rpt": info.rpt, "charge": info.charge,
                "date": info.date, "session_id": info.session_id,
                "source_file": str(info.path)}
        try:
            print(f"ANALYSE {info.session_id}: {info.path.name}")
            data = load_charge_log(info.path)
            metrics = calculate_metrics(data, info, taper_current_A)
            output = (result_root / "sessions" / info.pack / f"RPT_{info.rpt}"
                      / f"CH_{info.charge}")
            output.mkdir(parents=True, exist_ok=True)
            write_summary(metrics, output / "charge_summary.csv")
            plot_charge_profile(data, info, output / "charge_profile.png")
            rows.append(metrics)
            loaded.append((info, data))
            quality.append({**base, "status": "ok", "detail": ""})
        except Exception as exc:
            quality.append({**base, "status": "error",
                            "detail": f"{type(exc).__name__}: {exc}"})
            print(f"ERROR {info.session_id}: {exc}")
    result_root.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(quality).to_csv(result_root / "charge_data_quality_report.csv", index=False,
                                 encoding="utf-8-sig")
    if not rows:
        raise RuntimeError("No charge logs could be analysed; see charge_data_quality_report.csv")
    summaries = pd.DataFrame(rows).sort_values(["pack", "rpt", "charge"])
    summaries.to_csv(result_root / "all_charge_summaries.csv", index=False,
                     encoding="utf-8-sig", float_format="%.8g")
    rpt_summary = create_group_outputs(summaries, loaded, result_root)
    rpt_summary.to_csv(result_root / "rpt_charge_summary.csv", index=False,
                       encoding="utf-8-sig", float_format="%.8g")
    for pack, frame in summaries.groupby("pack"):
        output = result_root / "rpt" / str(pack)
        plot_trend_grid(frame, output / "charge_comparison_by_rpt.png",
                        f"{pack}: charging comparison by RPT", group="rpt")
    metadata = {
        "dataset": str(dataset.resolve()),
        "taper_current_threshold_A": taper_current_A,
        "discovered_charge_logs": len(logs),
        "analysed_charge_logs": len(rows),
        "missing_or_failed_charge_logs": len(logs) - len(rows),
        "cv_start_definition": "first sample with mean active-cell voltage >= 4.18 V",
        "capacity_unit_assumption": "charger Capacity column is mAh",
        "energy_unit_assumption": "charger Energy column is Wh",
    }
    (result_root / "charge_analysis_metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"DONE: analysed {len(rows)}/{len(logs)} charge logs -> {result_root.resolve()}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=Path("Dataset"), help="Dataset root")
    parser.add_argument("--result", type=Path, default=Path("result/charging"),
                        help="Charging result root")
    parser.add_argument("--taper-current", type=float, default=1.0, metavar="A",
                        help="End-current threshold used to flag taper completion (default: 1 A)")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.taper_current < 0:
        raise ValueError("--taper-current must be non-negative")
    run_all(args.dataset, args.result, args.taper_current)


if __name__ == "__main__":
    main()
