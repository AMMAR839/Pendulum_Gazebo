#!/usr/bin/env python3
import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path


REAL_WS_PWM_DEADBAND = 3212.0
REAL_WS_PWM_PER_CMPS = 189.1


def fnum(row, key):
    value = row.get(key, "")
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def mean_abs(values):
    finite = [abs(v) for v in values if math.isfinite(v)]
    if not finite:
        return math.nan
    return sum(finite) / len(finite)


def min_finite(values):
    finite = [v for v in values if math.isfinite(v)]
    return min(finite) if finite else math.nan


def max_finite(values):
    finite = [v for v in values if math.isfinite(v)]
    return max(finite) if finite else math.nan


def pwm_equivalent(workspace, command_cmps):
    if workspace != "pendulum_real_ws" or not math.isfinite(command_cmps):
        return math.nan
    if abs(command_cmps) < 0.5:
        return 0.0
    return math.copysign(
        REAL_WS_PWM_DEADBAND + REAL_WS_PWM_PER_CMPS * abs(command_cmps),
        command_cmps,
    )


def fmt(value):
    if not math.isfinite(value):
        return ""
    return f"{value:.6f}"


def build_summary(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[(row.get("workspace", ""), row.get("phase", ""))].append(row)

    summary_rows = []
    for (workspace, phase), group in sorted(groups.items()):
        elapsed = [fnum(row, "elapsed_s") for row in group]
        degree = [fnum(row, "degree_deg") for row in group]
        joint_effort = [fnum(row, "cart_force_cmd_n") for row in group]
        velocity_cmd_mps = [fnum(row, "cart_velocity_cmd_mps") for row in group]
        velocity_cmd_cmps = [
            value * 100.0 if math.isfinite(value) else math.nan
            for value in velocity_cmd_mps
        ]
        set_speed_cmps = [fnum(row, "setspeed_cm_s") for row in group]
        hinge_assist = [fnum(row, "hinge_assist_torque_nm") for row in group]
        pwm = [
            pwm_equivalent(workspace, command_cmps)
            for command_cmps in velocity_cmd_cmps
        ]

        start_s = min_finite(elapsed)
        end_s = max_finite(elapsed)
        duration_s = end_s - start_s if math.isfinite(start_s + end_s) else math.nan

        notes = sorted({row.get("note", "") for row in group if row.get("note", "")})
        summary_rows.append(
            {
                "workspace": workspace,
                "phase": phase,
                "samples": str(len(group)),
                "duration_s": fmt(duration_s),
                "degree_abs_mean_deg": fmt(mean_abs(degree)),
                "sim_joint_effort_min_n": fmt(min_finite(joint_effort)),
                "sim_joint_effort_max_n": fmt(max_finite(joint_effort)),
                "sim_joint_effort_abs_mean_n": fmt(mean_abs(joint_effort)),
                "motor_command_abs_mean_cm_s": fmt(mean_abs(velocity_cmd_cmps)),
                "setspeed_abs_mean_cm_s": fmt(mean_abs(set_speed_cmps)),
                "real_ws_pwm_equiv_abs_mean": fmt(mean_abs(pwm)),
                "hinge_assist_abs_mean_nm": fmt(mean_abs(hinge_assist)),
                "interpretation": (
                    "sim_joint_effort_only_not_real_motor_force"
                    if joint_effort
                    else ""
                ),
                "notes": ";".join(notes),
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

    fieldnames = [
        "workspace",
        "phase",
        "samples",
        "duration_s",
        "degree_abs_mean_deg",
        "sim_joint_effort_min_n",
        "sim_joint_effort_max_n",
        "sim_joint_effort_abs_mean_n",
        "motor_command_abs_mean_cm_s",
        "setspeed_abs_mean_cm_s",
        "real_ws_pwm_equiv_abs_mean",
        "hinge_assist_abs_mean_nm",
        "interpretation",
        "notes",
    ]
    with output_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(build_summary(rows))

    print(f"wrote {output_path}")


if __name__ == "__main__":
    main()
