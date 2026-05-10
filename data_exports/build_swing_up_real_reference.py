#!/usr/bin/env python3
import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path


REAL_STYLE_PWM_DEADBAND = 3212.0
REAL_STYLE_PWM_PER_CMPS = 189.1


def fnum(row, key):
    value = row.get(key, "")
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def finite(values):
    return [value for value in values if math.isfinite(value)]


def fmt(value):
    if not math.isfinite(value):
        return ""
    return f"{value:.6f}"


def mean(values):
    values = finite(values)
    return sum(values) / len(values) if values else math.nan


def mean_abs(values):
    return mean([abs(value) for value in values])


def rms(values):
    values = finite(values)
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else math.nan


def percentile_abs(values, percent):
    values = sorted(abs(value) for value in values if math.isfinite(value))
    if not values:
        return math.nan
    if len(values) == 1:
        return values[0]
    rank = (len(values) - 1) * percent / 100.0
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return values[int(rank)]
    fraction = rank - lower
    return values[lower] * (1.0 - fraction) + values[upper] * fraction


def min_finite(values):
    values = finite(values)
    return min(values) if values else math.nan


def max_finite(values):
    values = finite(values)
    return max(values) if values else math.nan


def canonical_workspace(name):
    if name.startswith("ros2"):
        return "ros2_pendulum_ws"
    if name.startswith("real") or name.startswith("pendulum_real"):
        return "pendulum_real_ws"
    if name.startswith("pid") or name.startswith("pendulum_pid"):
        return "pendulum_pid_ws"
    return name


def is_real_style(name):
    canonical = canonical_workspace(name)
    return canonical in {"pendulum_real_ws", "pendulum_pid_ws"}


def pwm_equivalent(workspace, command_cmps):
    if not is_real_style(workspace) or not math.isfinite(command_cmps):
        return math.nan
    if abs(command_cmps) < 0.5:
        return 0.0
    return math.copysign(
        REAL_STYLE_PWM_DEADBAND + REAL_STYLE_PWM_PER_CMPS * abs(command_cmps),
        command_cmps,
    )


def sorted_series(rows, key):
    points = []
    for row in rows:
        elapsed = fnum(row, "elapsed_s")
        value = fnum(row, key)
        if math.isfinite(elapsed) and math.isfinite(value):
            points.append((elapsed, value))
    return sorted(points)


def integrate(points):
    if len(points) < 2:
        return math.nan
    total = 0.0
    for (t0, y0), (t1, y1) in zip(points, points[1:]):
        dt = t1 - t0
        if dt <= 0:
            continue
        total += 0.5 * (y0 + y1) * dt
    return total


def build_balance_energy_reference(rows):
    by_workspace = defaultdict(list)
    for row in rows:
        if row.get("phase") != "BALANCE":
            continue
        by_workspace[canonical_workspace(row.get("workspace", ""))].append(
            fnum(row, "energy_j")
        )
    return {workspace: mean(values) for workspace, values in by_workspace.items()}


def build_summary(rows, source_name):
    balance_energy = build_balance_energy_reference(rows)
    groups = defaultdict(list)
    for row in rows:
        if row.get("phase") != "SWING_UP":
            continue
        groups[canonical_workspace(row.get("workspace", ""))].append(row)

    summary_rows = []
    for workspace, group in sorted(groups.items()):
        group = sorted(group, key=lambda row: fnum(row, "elapsed_s"))
        elapsed = [fnum(row, "elapsed_s") for row in group]
        force = [fnum(row, "cart_force_cmd_n") for row in group]
        cart_velocity_cmd = [fnum(row, "cart_velocity_cmd_mps") for row in group]
        motor_command_cmps = [fnum(row, "motor_command_cm_s") for row in group]
        if not finite(motor_command_cmps):
            motor_command_cmps = [
                value * 100.0 if math.isfinite(value) else math.nan
                for value in cart_velocity_cmd
            ]
        energy = [fnum(row, "energy_j") for row in group]
        hinge_assist = [fnum(row, "hinge_assist_torque_nm") for row in group]
        cmx = [fnum(row, "cmX_cm") for row in group]
        theta_dot = [fnum(row, "theta_dot_rad_s") for row in group]
        pwm = [
            pwm_equivalent(workspace, command)
            for command in motor_command_cmps
        ]

        start_s = min_finite(elapsed)
        end_s = max_finite(elapsed)
        duration_s = end_s - start_s if math.isfinite(start_s + end_s) else math.nan

        abs_force_points = [
            (elapsed_value, abs(force_value))
            for elapsed_value, force_value in sorted_series(group, "cart_force_cmd_n")
        ]
        signed_force_points = sorted_series(group, "cart_force_cmd_n")
        cmd_power_points = []
        for row in group:
            elapsed_value = fnum(row, "elapsed_s")
            force_value = fnum(row, "cart_force_cmd_n")
            velocity_value = fnum(row, "cart_velocity_cmd_mps")
            if (
                math.isfinite(elapsed_value)
                and math.isfinite(force_value)
                and math.isfinite(velocity_value)
            ):
                cmd_power_points.append((elapsed_value, force_value * velocity_value))
        cmd_power_points.sort()
        positive_cmd_power_points = [
            (elapsed_value, max(0.0, power))
            for elapsed_value, power in cmd_power_points
        ]

        target_energy = balance_energy.get(workspace, math.nan)
        energy_peak = max_finite(energy)
        energy_start = group[0] and fnum(group[0], "energy_j")
        energy_end = group[-1] and fnum(group[-1], "energy_j")
        energy_gain_peak = (
            energy_peak - energy_start
            if math.isfinite(energy_peak) and math.isfinite(energy_start)
            else math.nan
        )
        energy_ready_ratio = (
            energy_peak / target_energy
            if math.isfinite(energy_peak) and math.isfinite(target_energy) and target_energy > 0
            else math.nan
        )

        reference_label = (
            "real_style_reference_envelope"
            if is_real_style(workspace)
            else "simulation_reference_envelope"
        )

        summary_rows.append(
            {
                "workspace": workspace,
                "source_csv": source_name,
                "swing_samples": str(len(group)),
                "swing_duration_s": fmt(duration_s),
                "force_min_n": fmt(min_finite(force)),
                "force_max_n": fmt(max_finite(force)),
                "force_abs_mean_n": fmt(mean_abs(force)),
                "force_rms_n": fmt(rms(force)),
                "force_abs_p95_n": fmt(percentile_abs(force, 95.0)),
                "force_abs_peak_n": fmt(max_finite([abs(value) for value in force])),
                "force_abs_impulse_n_s": fmt(integrate(abs_force_points)),
                "force_signed_impulse_n_s": fmt(integrate(signed_force_points)),
                "cmd_work_signed_estimate_j": fmt(integrate(cmd_power_points)),
                "cmd_work_positive_estimate_j": fmt(integrate(positive_cmd_power_points)),
                "motor_command_abs_mean_cm_s": fmt(mean_abs(motor_command_cmps)),
                "motor_command_abs_p95_cm_s": fmt(percentile_abs(motor_command_cmps, 95.0)),
                "motor_command_abs_peak_cm_s": fmt(
                    max_finite([abs(value) for value in motor_command_cmps])
                ),
                "real_style_pwm_abs_mean": fmt(mean_abs(pwm)),
                "real_style_pwm_abs_p95": fmt(percentile_abs(pwm, 95.0)),
                "energy_start_j": fmt(energy_start),
                "energy_end_j": fmt(energy_end),
                "energy_peak_j": fmt(energy_peak),
                "energy_gain_to_peak_j": fmt(energy_gain_peak),
                "balance_energy_reference_j": fmt(target_energy),
                "energy_ready_ratio": fmt(energy_ready_ratio),
                "theta_dot_abs_peak_rad_s": fmt(
                    max_finite([abs(value) for value in theta_dot])
                ),
                "cart_position_min_cm": fmt(min_finite(cmx)),
                "cart_position_max_cm": fmt(max_finite(cmx)),
                "hinge_assist_abs_mean_nm": fmt(mean_abs(hinge_assist)),
                "hinge_assist_abs_peak_nm": fmt(
                    max_finite([abs(value) for value in hinge_assist])
                ),
                "reference_label": reference_label,
                "hardware_validation_required": "yes",
                "interpretation": (
                    "patokan_awal_real_berbasis_model; "
                    "wajib dibandingkan dengan log alat asli sebelum diklaim gaya motor aktual"
                ),
            }
        )
    return summary_rows


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    samples_path = Path(args.samples)
    with samples_path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    summary_rows = build_summary(rows, samples_path.name)
    fieldnames = [
        "workspace",
        "source_csv",
        "swing_samples",
        "swing_duration_s",
        "force_min_n",
        "force_max_n",
        "force_abs_mean_n",
        "force_rms_n",
        "force_abs_p95_n",
        "force_abs_peak_n",
        "force_abs_impulse_n_s",
        "force_signed_impulse_n_s",
        "cmd_work_signed_estimate_j",
        "cmd_work_positive_estimate_j",
        "motor_command_abs_mean_cm_s",
        "motor_command_abs_p95_cm_s",
        "motor_command_abs_peak_cm_s",
        "real_style_pwm_abs_mean",
        "real_style_pwm_abs_p95",
        "energy_start_j",
        "energy_end_j",
        "energy_peak_j",
        "energy_gain_to_peak_j",
        "balance_energy_reference_j",
        "energy_ready_ratio",
        "theta_dot_abs_peak_rad_s",
        "cart_position_min_cm",
        "cart_position_max_cm",
        "hinge_assist_abs_mean_nm",
        "hinge_assist_abs_peak_nm",
        "reference_label",
        "hardware_validation_required",
        "interpretation",
    ]
    with output_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)

    print(f"wrote {output_path}")


if __name__ == "__main__":
    main()
