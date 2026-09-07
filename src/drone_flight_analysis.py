#!/usr/bin/env python3
"""Drone flight performance analysis: flight, 10-flight cycle, and RPT trends."""

from __future__ import annotations

import argparse
import json
import math
import re
import unicodedata
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


FLIGHT_DIR_RE = re.compile(
    r"(?P<date>\d{4}-\d{2}-\d{2})\s+(?P<clock>\d{2}-\d{2}-\d{2})_"
    r"(?P<pack>[A-Za-z]+)_(?P<rpt>\d+)_(?P<motion>[A-Za-z]+)_(?P<flight>\d+)$"
)
DEFAULT_MODE_MAP = {"PB": 3, "PC": 2}
AXES = ("x", "y", "z")
SIGNALS = {
    "attitude": ("attitude", "targetAttitude", "rad"),
    "position": ("position", "targetPosition", "m"),
    "omega": ("omega", "targetOmega", "rad_s"),
    "velocity": ("positionDot", "targetPositionDot", "m_s"),
}
TRACKING_UNITS = {name: spec[2] for name, spec in SIGNALS.items()}
TRACKING_UNITS["acceleration"] = "m_s2"
BASE_COLUMNS = ["mode", "time", "batteryTimestamp", "totalV_V", "batteryCurrent_A",
                "batteryTemperature_K", "batteryConsumed_Ah", "cpuUsage"]
VECTOR_COLUMNS = [
    *(f"{p}[{i}]" for p in ("attitude", "position", "omega", "positionDot",
                              "targetAttitude", "targetPosition", "targetOmega",
                              "targetPositionDot", "accl") for i in range(3)),
    *(f"pwm[{i}]" for i in range(4)),
    *(f"cellV_V[{i}]" for i in range(6)),
]
REQUIRED_COLUMNS = set(BASE_COLUMNS + VECTOR_COLUMNS)
SUMMARY_ID_COLUMNS = ["pack", "rpt", "motion", "flight", "timestamp", "flight_id", "source_dir"]
PLOT_METRICS = [
    ("active_duration_s", "Active flight time [s]"),
    ("energy_Wh", "Energy [Wh]"),
    ("mean_power_W", "Mean power [W]"),
    ("min_voltage_V", "Minimum voltage [V]"),
    ("voltage_drop_V", "Voltage drop [V]"),
    ("max_cell_imbalance_mV", "Maximum cell imbalance [mV]"),
    ("max_battery_temp_C", "Maximum battery temperature [°C]"),
    ("position_rmse_norm", "Position RMSE norm [m]"),
    ("attitude_rmse_norm", "Attitude RMSE norm [rad]"),
]


def axis_rmse_column(signal: str, axis: str) -> str:
    return f"{signal}_rmse_{axis}_{TRACKING_UNITS[signal]}"


def axis_display_name(signal: str, axis: str) -> str:
    if signal in {"attitude", "omega"}:
        return {"x": "Roll (X)", "y": "Pitch (Y)", "z": "Yaw (Z)"}[axis]
    return "Vertical Z" if axis == "z" else f"Horizontal {axis.upper()}"


AXIS_RMSE_METRICS = [
    axis_rmse_column(signal, axis)
    for signal in TRACKING_UNITS
    for axis in AXES
]


@dataclass(frozen=True)
class FlightInfo:
    pack: str
    rpt: int
    motion: str
    flight: int
    timestamp: str
    directory: Path

    @property
    def flight_id(self) -> str:
        return f"{self.pack}_{self.rpt}_{self.motion}_{self.flight}"


def _normal(text: str) -> str:
    return unicodedata.normalize("NFC", text.strip())


def parse_flight_dir(directory: Path) -> FlightInfo:
    match = FLIGHT_DIR_RE.search(_normal(directory.name))
    if not match:
        raise ValueError(f"Cannot parse flight directory name: {directory.name}")
    g = match.groupdict()
    return FlightInfo(
        pack=g["pack"].upper(),
        rpt=int(g["rpt"]),
        motion=g["motion"].upper(),
        flight=int(g["flight"]),
        timestamp=f'{g["date"]} {g["clock"].replace("-", ":")}',
        directory=directory.resolve(),
    )


def discover_flights(dataset: Path) -> list[FlightInfo]:
    flights: list[FlightInfo] = []
    for path in dataset.rglob("*"):
        if path.is_dir() and FLIGHT_DIR_RE.search(_normal(path.name)):
            flights.append(parse_flight_dir(path))
    return sorted(flights, key=lambda x: (x.pack, x.rpt, x.flight, x.timestamp))


def flight_files(directory: Path) -> list[Path]:
    canonical = directory / "flightData.csv"
    if canonical.is_file() and canonical.stat().st_size > 0:
        return [canonical]
    numbered = []
    for path in directory.glob("flightData[0-9]*.csv"):
        match = re.fullmatch(r"flightData(\d+)\.csv", path.name)
        if match and path.stat().st_size > 0:
            numbered.append((int(match.group(1)), path))
    return [path for _, path in sorted(numbered)]


def load_flight(directory: Path) -> pd.DataFrame:
    files = flight_files(directory)
    if not files:
        raise FileNotFoundError("No non-empty flightData CSV found")
    frames = []
    for path in files:
        header = pd.read_csv(path, nrows=0, skipinitialspace=True).columns.tolist()
        missing = sorted(REQUIRED_COLUMNS - set(header))
        if missing:
            raise ValueError(f"{path.name}: missing columns: {', '.join(missing)}")
        frame = pd.read_csv(path, usecols=BASE_COLUMNS + VECTOR_COLUMNS,
                            skipinitialspace=True, low_memory=False)
        frames.append(frame)
    data = pd.concat(frames, ignore_index=True)
    for column in data.columns:
        data[column] = pd.to_numeric(data[column], errors="coerce")
    data = data.dropna(subset=["time"]).sort_values("time", kind="stable")
    data = data.drop_duplicates(subset=["time"], keep="last").reset_index(drop=True)
    if data.empty:
        raise ValueError("Flight data has no valid time samples")
    return data


def _segments(mask: np.ndarray) -> list[tuple[int, int]]:
    padded = np.r_[False, mask, False].astype(np.int8)
    edges = np.diff(padded)
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def analysis_mask(data: pd.DataFrame, active_mode: int, settle_seconds: float) -> np.ndarray:
    active = data["mode"].to_numpy() == active_mode
    keep = np.zeros(len(data), dtype=bool)
    times = data["time"].to_numpy(dtype=float)
    for start, stop in _segments(active):
        keep[start:stop] = times[start:stop] >= times[start] + settle_seconds
    return keep


def _safe(values: pd.Series | np.ndarray) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    return array[np.isfinite(array)]


def _stat(values: pd.Series | np.ndarray, operation: str) -> float:
    array = _safe(values)
    return float(getattr(np, operation)(array)) if array.size else math.nan


def _integral(y: np.ndarray, x: np.ndarray) -> float:
    valid = np.isfinite(x) & np.isfinite(y)
    if valid.sum() < 2:
        return math.nan
    x, y = x[valid], y[valid]
    positive_dt = np.r_[True, np.diff(x) > 0]
    x, y = x[positive_dt], y[positive_dt]
    if len(x) < 2:
        return math.nan
    if hasattr(np, "trapezoid"):
        return float(np.trapezoid(y, x))
    # NumPy < 2.0 compatibility.
    return float(np.trapz(y, x))


def kinematic_acceleration(active: pd.DataFrame, smoothing_seconds: float = 0.2) -> tuple[np.ndarray, np.ndarray]:
    """Return smoothed world-frame measured and target acceleration from velocity."""
    time = active["time"].to_numpy(dtype=float)
    if len(time) < 2:
        empty = np.full((len(active), 3), np.nan)
        return empty, empty.copy()
    dt = np.diff(time)
    median_dt = float(np.median(dt[np.isfinite(dt) & (dt > 0)]))
    window = max(3, int(round(smoothing_seconds / median_dt)))
    if window % 2 == 0:
        window += 1
    measured_velocity = active[[f"positionDot[{i}]" for i in range(3)]].rolling(
        window, center=True, min_periods=1).mean().to_numpy(dtype=float)
    target_velocity = active[[f"targetPositionDot[{i}]" for i in range(3)]].rolling(
        window, center=True, min_periods=1).mean().to_numpy(dtype=float)
    measured_acceleration = np.gradient(measured_velocity, time, axis=0)
    target_acceleration = np.gradient(target_velocity, time, axis=0)
    return measured_acceleration, target_acceleration


def calculate_metrics(data: pd.DataFrame, info: FlightInfo, active_mode: int,
                      settle_seconds: float) -> tuple[dict[str, object], pd.DataFrame]:
    mask = analysis_mask(data, active_mode, settle_seconds)
    active = data.loc[mask].copy()
    if active.empty:
        raise ValueError(f"No samples for mode {active_mode} after {settle_seconds:g} s settling")
    time = active["time"].to_numpy(dtype=float)
    duration = float(time[-1] - time[0]) if len(time) > 1 else 0.0
    dt = np.diff(time)
    positive_dt = dt[np.isfinite(dt) & (dt > 0)]
    metrics: dict[str, object] = {
        "pack": info.pack, "rpt": info.rpt, "motion": info.motion, "flight": info.flight,
        "timestamp": info.timestamp, "flight_id": info.flight_id,
        "source_dir": str(info.directory), "status": "ok",
        "active_mode": active_mode, "settle_seconds": settle_seconds,
        "raw_sample_count": len(data), "analysis_sample_count": len(active),
        "total_log_duration_s": float(data["time"].iloc[-1] - data["time"].iloc[0]),
        "active_duration_s": duration,
        "sample_rate_Hz": (1.0 / float(np.median(positive_dt))) if positive_dt.size else math.nan,
    }

    error_frame = pd.DataFrame({"time_s": time - time[0]})
    for label, (measured_prefix, target_prefix, unit) in SIGNALS.items():
        squared_sum = np.zeros(len(active), dtype=float)
        for i, axis in enumerate(AXES):
            error = (active[f"{measured_prefix}[{i}]"].to_numpy(dtype=float)
                     - active[f"{target_prefix}[{i}]"].to_numpy(dtype=float))
            error_frame[f"{label}_error_{axis}_{unit}"] = error
            squared_sum += np.nan_to_num(error, nan=0.0) ** 2
            valid = error[np.isfinite(error)]
            metrics[f"{label}_rmse_{axis}_{unit}"] = float(np.sqrt(np.mean(valid ** 2))) if valid.size else math.nan
            metrics[f"{label}_mae_{axis}_{unit}"] = float(np.mean(np.abs(valid))) if valid.size else math.nan
            metrics[f"{label}_max_abs_{axis}_{unit}"] = float(np.max(np.abs(valid))) if valid.size else math.nan
        metrics[f"{label}_rmse_norm"] = float(np.sqrt(np.nanmean(squared_sum)))

    measured_acceleration, target_acceleration = kinematic_acceleration(active)
    acceleration_error = measured_acceleration - target_acceleration
    squared_sum = np.zeros(len(active), dtype=float)
    for i, axis in enumerate(AXES):
        error = acceleration_error[:, i]
        error_frame[f"acceleration_error_{axis}_m_s2"] = error
        squared_sum += np.nan_to_num(error, nan=0.0) ** 2
        valid = error[np.isfinite(error)]
        metrics[f"acceleration_rmse_{axis}_m_s2"] = float(np.sqrt(np.mean(valid ** 2))) if valid.size else math.nan
        metrics[f"acceleration_mae_{axis}_m_s2"] = float(np.mean(np.abs(valid))) if valid.size else math.nan
        metrics[f"acceleration_max_abs_{axis}_m_s2"] = float(np.max(np.abs(valid))) if valid.size else math.nan
    metrics["acceleration_rmse_norm"] = float(np.sqrt(np.nanmean(squared_sum)))

    imu_acceleration = active[[f"accl[{i}]" for i in range(3)]].to_numpy(dtype=float)
    for i, axis in enumerate(AXES):
        values = imu_acceleration[:, i]
        metrics[f"imu_acc_mean_{axis}_m_s2"] = _stat(values, "mean")
        metrics[f"imu_acc_std_{axis}_m_s2"] = _stat(values, "std")
        metrics[f"imu_acc_rms_{axis}_m_s2"] = float(np.sqrt(np.nanmean(values ** 2)))
        metrics[f"imu_acc_max_abs_{axis}_m_s2"] = _stat(np.abs(values), "max")
    imu_magnitude = np.linalg.norm(imu_acceleration, axis=1)
    metrics["imu_acc_magnitude_mean_m_s2"] = _stat(imu_magnitude, "mean")
    metrics["imu_acc_magnitude_max_m_s2"] = _stat(imu_magnitude, "max")

    velocity = active[[f"positionDot[{i}]" for i in range(3)]].to_numpy(dtype=float)
    speed = np.linalg.norm(velocity, axis=1)
    metrics["mean_speed_m_s"] = _stat(speed, "mean")
    metrics["max_speed_m_s"] = _stat(speed, "max")
    metrics["distance_m"] = _integral(speed, time)
    metrics["mean_cpu_usage_pct"] = _stat(active["cpuUsage"], "mean")

    pwm = active[[f"pwm[{i}]" for i in range(4)]].to_numpy(dtype=float)
    metrics["mean_pwm"] = _stat(pwm, "mean")
    metrics["std_pwm"] = _stat(pwm, "std")
    metrics["max_pwm"] = _stat(pwm, "max")
    metrics["motor_imbalance_pwm"] = _stat(np.nanmax(pwm, axis=1) - np.nanmin(pwm, axis=1), "mean")

    battery = active.loc[(active["totalV_V"] > 0) & (active["batteryTimestamp"] > 0)].copy()
    battery = battery.drop_duplicates(subset=["batteryTimestamp"], keep="last")
    battery = battery.sort_values("batteryTimestamp")
    if not battery.empty:
        btime = battery["batteryTimestamp"].to_numpy(dtype=float) / 1000.0
        voltage = battery["totalV_V"].to_numpy(dtype=float)
        current = battery["batteryCurrent_A"].to_numpy(dtype=float)
        power = voltage * current
        cells = battery[[f"cellV_V[{i}]" for i in range(6)]].to_numpy(dtype=float, copy=True)
        cells[cells <= 0] = np.nan
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            imbalance = np.nanmax(cells, axis=1) - np.nanmin(cells, axis=1)
        temp_c = battery["batteryTemperature_K"].to_numpy(dtype=float) - 273.15
        consumed = battery["batteryConsumed_Ah"].to_numpy(dtype=float)
        metrics.update({
            "battery_sample_count": len(battery),
            "start_voltage_V": float(voltage[0]), "end_voltage_V": float(voltage[-1]),
            "min_voltage_V": _stat(voltage, "min"), "mean_voltage_V": _stat(voltage, "mean"),
            "voltage_drop_V": float(voltage[0] - voltage[-1]),
            "mean_current_A": _stat(current, "mean"), "max_current_A": _stat(current, "max"),
            "mean_power_W": _stat(power, "mean"), "max_power_W": _stat(power, "max"),
            "discharged_capacity_Ah": _integral(np.maximum(current, 0), btime) / 3600.0,
            "energy_Wh": _integral(np.maximum(power, 0), btime) / 3600.0,
            "consumed_Ah_sensor_delta": float(consumed[-1] - consumed[0]),
            "start_battery_temp_C": float(temp_c[0]), "end_battery_temp_C": float(temp_c[-1]),
            "max_battery_temp_C": _stat(temp_c, "max"),
            "battery_temp_rise_C": float(temp_c[-1] - temp_c[0]),
            "mean_cell_imbalance_mV": _stat(imbalance * 1000.0, "mean"),
            "max_cell_imbalance_mV": _stat(imbalance * 1000.0, "max"),
        })
    else:
        metrics["battery_sample_count"] = 0
        for key in ("start_voltage_V", "end_voltage_V", "min_voltage_V", "mean_voltage_V",
                    "voltage_drop_V", "mean_current_A", "max_current_A", "mean_power_W",
                    "max_power_W", "discharged_capacity_Ah", "energy_Wh",
                    "consumed_Ah_sensor_delta", "start_battery_temp_C", "end_battery_temp_C",
                    "max_battery_temp_C", "battery_temp_rise_C", "mean_cell_imbalance_mV",
                    "max_cell_imbalance_mV"):
            metrics[key] = math.nan
    return metrics, error_frame


def _downsample(frame: pd.DataFrame, max_points: int = 6000) -> pd.DataFrame:
    stride = max(1, math.ceil(len(frame) / max_points))
    return frame.iloc[::stride]


def plot_flight(data: pd.DataFrame, info: FlightInfo, output_dir: Path, active_mode: int,
                settle_seconds: float) -> None:
    mask = analysis_mask(data, active_mode, settle_seconds)
    active = data.loc[mask]
    stride = max(1, math.ceil(len(active) / 6000))
    frame = active.iloc[::stride]
    measured_acceleration, target_acceleration = kinematic_acceleration(active)
    measured_acceleration = measured_acceleration[::stride]
    target_acceleration = target_acceleration[::stride]
    t = frame["time"] - frame["time"].iloc[0]
    fig, axes = plt.subplots(4, 1, figsize=(13, 14), sharex=True, constrained_layout=True)
    for i, axis in enumerate(AXES):
        axes[0].plot(t, frame[f"position[{i}]"], label=f"{axis} measured", lw=0.8)
        axes[0].plot(t, frame[f"targetPosition[{i}]"], "--", label=f"{axis} target", lw=0.8)
    axes[0].set_ylabel("Position [m]"); axes[0].legend(ncol=3, fontsize=8)
    for i, axis in enumerate(AXES):
        error = frame[f"position[{i}]"] - frame[f"targetPosition[{i}]"]
        axes[1].plot(t, error, label=axis, lw=0.8)
    axes[1].set_ylabel("Position error [m]"); axes[1].legend(ncol=3)
    axes[2].plot(t, frame["totalV_V"], label="Voltage [V]", color="tab:blue")
    ax_current = axes[2].twinx()
    ax_current.plot(t, frame["batteryCurrent_A"], label="Current [A]", color="tab:red", alpha=0.7)
    axes[2].set_ylabel("Voltage [V]"); ax_current.set_ylabel("Current [A]")
    handles = axes[2].lines + ax_current.lines
    axes[2].legend(handles, [x.get_label() for x in handles], loc="best")
    for i in range(4): axes[3].plot(t, frame[f"pwm[{i}]"], label=f"motor {i + 1}", lw=0.7)
    axes[3].set_ylabel("PWM"); axes[3].set_xlabel("Analysis time [s]"); axes[3].legend(ncol=4)
    for ax in axes: ax.grid(alpha=0.25)
    fig.suptitle(f"{info.flight_id} performance overview")
    fig.savefig(output_dir / "performance_overview.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(5, 1, figsize=(13, 16), sharex=True, constrained_layout=True)
    for ax, (label, (measured, target, unit)) in zip(axes[:4], SIGNALS.items()):
        for i, name in enumerate(AXES):
            ax.plot(t, frame[f"{measured}[{i}]"] - frame[f"{target}[{i}]"], label=name, lw=0.7)
        ax.set_ylabel(f"{label} error\n[{unit.replace('_', '/') }]")
        ax.legend(ncol=3); ax.grid(alpha=0.25)
    for i, name in enumerate(AXES):
        axes[4].plot(t, measured_acceleration[:, i] - target_acceleration[:, i], label=name, lw=0.7)
    axes[4].set_ylabel("acceleration error\n[m/s²]")
    axes[4].legend(ncol=3); axes[4].grid(alpha=0.25)
    axes[-1].set_xlabel("Analysis time [s]")
    fig.suptitle(f"{info.flight_id} tracking errors")
    fig.savefig(output_dir / "tracking_errors.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True, constrained_layout=True)
    for i, name in enumerate(AXES):
        axes[0].plot(t, measured_acceleration[:, i], label=f"{name} measured", lw=0.7)
        axes[0].plot(t, target_acceleration[:, i], "--", label=f"{name} target", lw=0.7)
        axes[1].plot(t, measured_acceleration[:, i] - target_acceleration[:, i], label=name, lw=0.7)
        axes[2].plot(t, frame[f"accl[{i}]"], label=name, lw=0.7)
    axes[0].set_ylabel("Kinematic acceleration [m/s²]"); axes[0].legend(ncol=3, fontsize=8)
    axes[1].set_ylabel("Acceleration error [m/s²]"); axes[1].legend(ncol=3)
    axes[2].set_ylabel("Body IMU acceleration [m/s²]"); axes[2].legend(ncol=3)
    axes[2].set_xlabel("Analysis time [s]")
    for ax in axes: ax.grid(alpha=0.25)
    fig.suptitle(f"{info.flight_id} acceleration diagnostics")
    fig.savefig(output_dir / "acceleration_diagnostics.png", dpi=160)
    plt.close(fig)

    battery = frame.loc[frame["totalV_V"] > 0].copy()
    if not battery.empty:
        bt = battery["time"] - frame["time"].iloc[0]
        voltage = battery["totalV_V"]
        current = battery["batteryCurrent_A"]
        power = voltage * current
        cells = battery[[f"cellV_V[{i}]" for i in range(6)]].replace(0, np.nan)
        imbalance = cells.max(axis=1) - cells.min(axis=1)
        fig, axes = plt.subplots(4, 1, figsize=(13, 13), sharex=True, constrained_layout=True)
        axes[0].plot(bt, voltage, color="tab:blue", lw=0.8)
        axes[0].set_ylabel("Pack voltage [V]")
        axes[1].plot(bt, current, color="tab:red", lw=0.7, label="current")
        axes[1].set_ylabel("Current [A]")
        power_axis = axes[1].twinx()
        power_axis.plot(bt, power, color="tab:orange", lw=0.6, alpha=0.65, label="power")
        power_axis.set_ylabel("Power [W]")
        handles = axes[1].lines + power_axis.lines
        axes[1].legend(handles, [x.get_label() for x in handles], loc="best")
        for i, column in enumerate(cells):
            if cells[column].notna().any():
                axes[2].plot(bt, cells[column], label=f"cell {i + 1}", lw=0.7)
        axes[2].set_ylabel("Cell voltage [V]"); axes[2].legend(ncol=4)
        axes[3].plot(bt, battery["batteryTemperature_K"] - 273.15,
                     color="tab:red", label="temperature", lw=0.8)
        axes[3].set_ylabel("Temperature [°C]")
        imbalance_axis = axes[3].twinx()
        imbalance_axis.plot(bt, imbalance * 1000.0, color="tab:purple", alpha=0.7,
                            label="cell imbalance", lw=0.7)
        imbalance_axis.set_ylabel("Cell imbalance [mV]")
        handles = axes[3].lines + imbalance_axis.lines
        axes[3].legend(handles, [x.get_label() for x in handles], loc="best")
        axes[3].set_xlabel("Analysis time [s]")
        for ax in axes: ax.grid(alpha=0.25)
        fig.suptitle(f"{info.flight_id} battery diagnostics")
        fig.savefig(output_dir / "battery_diagnostics.png", dpi=160)
        plt.close(fig)


def write_row_csv(row: dict[str, object], path: Path) -> None:
    pd.DataFrame([row]).to_csv(path, index=False, encoding="utf-8-sig", float_format="%.8g")


def analyse_one(info: FlightInfo, result_root: Path, active_mode: int,
                settle_seconds: float, save_error_series: bool = False) -> dict[str, object]:
    data = load_flight(info.directory)
    metrics, errors = calculate_metrics(data, info, active_mode, settle_seconds)
    output = result_root / "flights" / info.pack / f"RPT_{info.rpt}" / f"{info.motion}_{info.flight}"
    output.mkdir(parents=True, exist_ok=True)
    write_row_csv(metrics, output / "flight_summary.csv")
    if save_error_series:
        errors.to_csv(output / "tracking_error_timeseries.csv", index=False,
                      encoding="utf-8-sig", float_format="%.8g")
    plot_flight(data, info, output, active_mode, settle_seconds)
    return metrics


def _slope(values: pd.Series, x: pd.Series) -> float:
    valid = values.notna() & x.notna()
    return float(np.polyfit(x[valid], values[valid], 1)[0]) if valid.sum() >= 2 else math.nan


def plot_metric_grid(frame: pd.DataFrame, x: str, group: str | None, metrics: Sequence[tuple[str, str]],
                     title: str, destination: Path) -> None:
    ncols = 3 if len(metrics) <= 9 else 4
    nrows = math.ceil(len(metrics) / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(6 * ncols, 4.5 * nrows),
                             constrained_layout=True, squeeze=False)
    for ax, (column, label) in zip(axes.flat, metrics):
        groups: Iterable[tuple[object, pd.DataFrame]] = frame.groupby(group) if group else [(None, frame)]
        for name, part in groups:
            part = part.sort_values(x)
            ax.plot(part[x], part[column], marker="o", ms=4, label=str(name) if name is not None else None)
        ax.set_xlabel(x.replace("_", " ").title()); ax.set_ylabel(label); ax.grid(alpha=0.25)
    for ax in list(axes.flat)[len(metrics):]:
        ax.set_visible(False)
    if group:
        axes.flat[0].legend(title=group.upper())
    fig.suptitle(title)
    fig.savefig(destination, dpi=160)
    plt.close(fig)


def plot_axis_rmse(frame: pd.DataFrame, x: str, signal: str, title: str, destination: Path,
                   group: str | None = None, aggregate: bool = False) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(12, 11), sharex=True, constrained_layout=True)
    unit = TRACKING_UNITS[signal].replace("_", "/")
    groups: Iterable[tuple[object, pd.DataFrame]] = frame.groupby(group) if group else [(None, frame)]
    for axis_plot, axis_name in zip(axes, AXES):
        base_column = axis_rmse_column(signal, axis_name)
        for name, part in groups:
            part = part.sort_values(x)
            column = f"{base_column}_mean" if aggregate else base_column
            label = f"RPT {name}" if name is not None else axis_name.upper()
            axis_plot.plot(part[x], part[column], marker="o", ms=4, label=label)
            if aggregate:
                std_column = f"{base_column}_std"
                low = part[column] - part[std_column]
                high = part[column] + part[std_column]
                axis_plot.fill_between(part[x], low, high, alpha=0.15)
        direction = axis_display_name(signal, axis_name)
        axis_plot.set_ylabel(f"{direction} RMSE [{unit}]")
        axis_plot.grid(alpha=0.25)
        if group:
            axis_plot.legend(title="RPT", ncol=4)
    axes[-1].set_xlabel(x.replace("_", " ").title())
    fig.suptitle(title)
    fig.savefig(destination, dpi=160)
    plt.close(fig)


def create_cycle_outputs(summaries: pd.DataFrame, result_root: Path) -> pd.DataFrame:
    rows = []
    for (pack, rpt), cycle in summaries.groupby(["pack", "rpt"], sort=True):
        cycle = cycle.sort_values("flight")
        output = result_root / "cycles" / str(pack) / f"RPT_{int(rpt)}"
        output.mkdir(parents=True, exist_ok=True)
        cycle.to_csv(output / "flight_performance.csv", index=False, encoding="utf-8-sig", float_format="%.8g")
        axis_columns = SUMMARY_ID_COLUMNS + ["total_log_duration_s", "active_duration_s"] + AXIS_RMSE_METRICS
        cycle[axis_columns].to_csv(output / "axis_control_performance.csv", index=False,
                                   encoding="utf-8-sig", float_format="%.8g")
        row: dict[str, object] = {
            "pack": pack, "rpt": int(rpt), "available_flights": len(cycle),
            "expected_flights": 10, "is_complete_cycle": len(cycle) == 10,
            "flight_numbers": ",".join(map(str, cycle["flight"].astype(int))),
        }
        trend_metrics = [metric for metric, _ in PLOT_METRICS] + AXIS_RMSE_METRICS
        for metric in trend_metrics:
            row[f"{metric}_mean"] = float(cycle[metric].mean())
            row[f"{metric}_std"] = float(cycle[metric].std(ddof=1)) if len(cycle) > 1 else math.nan
            row[f"{metric}_slope_per_flight"] = _slope(cycle[metric], cycle["flight"])
        rows.append(row)
        write_row_csv(row, output / "cycle_summary.csv")
        plot_metric_grid(cycle, "flight", None, PLOT_METRICS,
                         f"{pack} RPT {int(rpt)}: performance across flights",
                         output / "cycle_performance.png")
        for signal in TRACKING_UNITS:
            plot_axis_rmse(
                cycle, "flight", signal,
                f"{pack} RPT {int(rpt)}: {signal} RMSE by axis",
                output / f"{signal}_rmse_by_axis.png",
            )
    return pd.DataFrame(rows)


def create_rpt_outputs(summaries: pd.DataFrame, cycle_summary: pd.DataFrame, result_root: Path) -> None:
    output = result_root / "rpt"
    output.mkdir(parents=True, exist_ok=True)
    cycle_summary.to_csv(output / "rpt_summary.csv", index=False, encoding="utf-8-sig", float_format="%.8g")
    means = [f"{metric}_mean" for metric, _ in PLOT_METRICS]
    labels = [(column, label) for column, (_, label) in zip(means, PLOT_METRICS)]
    for pack, pack_frame in cycle_summary.groupby("pack"):
        pack_dir = output / str(pack); pack_dir.mkdir(exist_ok=True)
        pack_frame.to_csv(pack_dir / "rpt_performance.csv", index=False, encoding="utf-8-sig", float_format="%.8g")
        plot_metric_grid(pack_frame, "rpt", None, labels, f"{pack}: performance across RPT",
                         pack_dir / "rpt_performance.png")
        flight_comparison = summaries.loc[summaries["pack"] == pack].sort_values(["rpt", "flight"])
        comparison_columns = SUMMARY_ID_COLUMNS + ["total_log_duration_s"] + [
            metric for metric, _ in PLOT_METRICS
        ] + AXIS_RMSE_METRICS
        flight_comparison[comparison_columns].to_csv(
            pack_dir / "flight_performance_by_rpt.csv", index=False,
            encoding="utf-8-sig", float_format="%.8g")
        plot_metric_grid(
            flight_comparison, "flight", "rpt", PLOT_METRICS,
            f"{pack}: flight performance comparison by RPT",
            pack_dir / "flight_performance_by_rpt.png",
        )
        axis_rpt_columns = ["pack", "rpt", "available_flights"] + [
            f"{metric}_{suffix}"
            for metric in AXIS_RMSE_METRICS
            for suffix in ("mean", "std", "slope_per_flight")
        ]
        pack_frame[axis_rpt_columns].to_csv(
            pack_dir / "axis_control_performance_by_rpt.csv", index=False,
            encoding="utf-8-sig", float_format="%.8g")
        for signal in TRACKING_UNITS:
            plot_axis_rmse(
                flight_comparison, "flight", signal,
                f"{pack}: {signal} RMSE across flights and RPT",
                pack_dir / f"{signal}_rmse_by_axis_and_rpt.png", group="rpt",
            )
            plot_axis_rmse(
                pack_frame, "rpt", signal,
                f"{pack}: mean {signal} RMSE across RPT",
                pack_dir / f"{signal}_rmse_trend_by_rpt.png", aggregate=True,
            )
    if not cycle_summary.empty:
        plot_metric_grid(cycle_summary, "rpt", "pack", labels, "Performance across RPT",
                         output / "all_packs_rpt_performance.png")


def parse_mode_map(entries: Sequence[str]) -> dict[str, int]:
    mode_map = dict(DEFAULT_MODE_MAP)
    for entry in entries:
        try:
            pack, mode = entry.split("=", 1)
            mode_map[pack.strip().upper()] = int(mode)
        except (ValueError, TypeError) as exc:
            raise ValueError(f"Invalid --mode-map '{entry}'; use PACK=MODE, e.g. PC=2") from exc
    return mode_map


def selected_mode(info: FlightInfo, mode_map: dict[str, int], override: int | None) -> int:
    if override is not None:
        return override
    if info.pack not in mode_map:
        raise ValueError(f"No active mode configured for {info.pack}; add --mode-map {info.pack}=MODE")
    return mode_map[info.pack]


def run_all(dataset: Path, result_root: Path, mode_map: dict[str, int],
            active_mode_override: int | None, settle_seconds: float,
            save_error_series: bool) -> None:
    flights = discover_flights(dataset)
    rows, quality = [], []
    for info in flights:
        mode = selected_mode(info, mode_map, active_mode_override)
        files = flight_files(info.directory)
        if not files:
            quality.append({**{k: getattr(info, k) for k in ("pack", "rpt", "motion", "flight", "timestamp")},
                            "active_mode": mode,
                            "flight_id": info.flight_id, "status": "missing_data",
                            "detail": "No non-empty flightData CSV found", "source_dir": str(info.directory)})
            print(f"SKIP {info.flight_id}: no non-empty flightData CSV")
            continue
        try:
            print(f"ANALYSE {info.flight_id} (mode {mode}): {', '.join(p.name for p in files)}")
            rows.append(analyse_one(info, result_root, mode, settle_seconds, save_error_series))
            quality.append({**{k: getattr(info, k) for k in ("pack", "rpt", "motion", "flight", "timestamp")},
                            "active_mode": mode,
                            "flight_id": info.flight_id, "status": "ok", "detail": "",
                            "source_dir": str(info.directory)})
        except Exception as exc:  # continue the batch and preserve the exact failure
            quality.append({**{k: getattr(info, k) for k in ("pack", "rpt", "motion", "flight", "timestamp")},
                            "active_mode": mode,
                            "flight_id": info.flight_id, "status": "error",
                            "detail": f"{type(exc).__name__}: {exc}", "source_dir": str(info.directory)})
            print(f"ERROR {info.flight_id}: {exc}")
    result_root.mkdir(parents=True, exist_ok=True)
    quality_frame = pd.DataFrame(quality)
    quality_frame.to_csv(result_root / "data_quality_report.csv", index=False, encoding="utf-8-sig")
    if not rows:
        raise RuntimeError("No flights could be analysed; see data_quality_report.csv")
    summaries = pd.DataFrame(rows).sort_values(["pack", "rpt", "flight"])
    summaries.to_csv(result_root / "all_flight_summaries.csv", index=False,
                     encoding="utf-8-sig", float_format="%.8g")
    cycle_summary = create_cycle_outputs(summaries, result_root)
    create_rpt_outputs(summaries, cycle_summary, result_root)
    metadata = {
        "dataset": str(dataset.resolve()), "active_mode_map": mode_map,
        "active_mode_override": active_mode_override,
        "settle_seconds": settle_seconds, "discovered_flights": len(flights),
        "analysed_flights": len(rows), "missing_or_failed_flights": len(flights) - len(rows),
    }
    (result_root / "analysis_metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"DONE: analysed {len(rows)}/{len(flights)} flights -> {result_root.resolve()}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=Path("Dataset"), help="Dataset root")
    parser.add_argument("--result", type=Path, default=Path("result"), help="Result root")
    parser.add_argument("--active-mode", type=int,
                        help="Override the active mode for every pack")
    parser.add_argument("--mode-map", action="append", default=[], metavar="PACK=MODE",
                        help="Set a pack-specific mode; defaults: PB=3, PC=2 (repeatable)")
    parser.add_argument("--settle-seconds", type=float, default=3.0,
                        help="Seconds excluded after each active-mode transition")
    parser.add_argument("--save-error-series", action="store_true",
                        help="Save per-sample tracking errors (large CSV files)")
    parser.add_argument("--flight-dir", type=Path,
                        help="Analyse one flight directory instead of the complete dataset")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    mode_map = parse_mode_map(args.mode_map)
    if args.flight_dir:
        info = parse_flight_dir(args.flight_dir)
        mode = selected_mode(info, mode_map, args.active_mode)
        metrics = analyse_one(info, args.result, mode, args.settle_seconds,
                              args.save_error_series)
        args.result.mkdir(parents=True, exist_ok=True)
        write_row_csv(metrics, args.result / "single_flight_summary.csv")
        print(f"DONE: {info.flight_id} -> {args.result.resolve()}")
    else:
        run_all(args.dataset, args.result, mode_map, args.active_mode, args.settle_seconds,
                args.save_error_series)


if __name__ == "__main__":
    main()
