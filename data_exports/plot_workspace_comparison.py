#!/usr/bin/env python3
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "data_exports" / "plots"

SMOOTH_RUNS = {
    "lqr": ROOT / "data_exports" / "lqr_smooth_noassist_v2_20260512.csv",
    "real": ROOT / "data_exports" / "real_smooth_noassist_v3_20260512.csv",
    "pid": ROOT / "data_exports" / "pid_smooth_noassist_20260512.csv",
}

IMPULSE_RUNS = {
    "lqr": (
        ROOT / "data_exports" / "lqr_impulse_video_20260512.csv",
        ROOT / "data_exports" / "lqr_impulse_events_20260512.csv",
    ),
    "real": (
        ROOT / "data_exports" / "real_impulse_video_20260512.csv",
        ROOT / "data_exports" / "real_impulse_events_20260512.csv",
    ),
    "pid": (
        ROOT / "data_exports" / "pid_impulse_video_20260512.csv",
        ROOT / "data_exports" / "pid_impulse_events_20260512.csv",
    ),
}

COLORS = {
    "lqr": "#1f77b4",
    "real": "#2ca02c",
    "pid": "#d62728",
}


def fnum(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return math.nan


def read_csv(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def read_events(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def rel_time(rows):
    if not rows:
        return []
    start = fnum(rows[0], "elapsed_s")
    return [fnum(row, "elapsed_s") - start for row in rows]


def phase_rows(rows, phase_name):
    if "phase" in rows[0]:
        return [row for row in rows if row.get("phase") == phase_name]
    mode = 6 if phase_name == "SWING_UP" else 7
    return [row for row in rows if int(round(fnum(row, "mode"))) == mode]


def tail_strict(rows, n=300):
    tail = rows[-n:]
    return sum(
        1
        for row in tail
        if int(round(fnum(row, "mode"))) == 7
        and abs(fnum(row, "degree_deg")) < 2.0
        and abs(fnum(row, "cmX_cm")) < 3.0
        and abs(fnum(row, "theta_dot_rad_s")) < 0.35
    )


def max_abs(rows, key):
    values = [abs(fnum(row, key)) for row in rows if math.isfinite(fnum(row, key))]
    return max(values) if values else math.nan


def mean_abs(rows, key):
    values = [abs(fnum(row, key)) for row in rows if math.isfinite(fnum(row, key))]
    return sum(values) / len(values) if values else math.nan


def duration(rows):
    if not rows:
        return math.nan
    return fnum(rows[-1], "elapsed_s") - fnum(rows[0], "elapsed_s")


def first_mode_time(rows, mode):
    for row in rows:
        if int(round(fnum(row, "mode"))) == mode:
            return fnum(row, "elapsed_s")
    return math.nan


def strict_upright(row):
    return (
        int(round(fnum(row, "mode"))) == 7
        and abs(fnum(row, "degree_deg")) < 2.0
        and abs(fnum(row, "cmX_cm")) < 3.0
        and abs(fnum(row, "theta_dot_rad_s")) < 0.35
    )


def smooth_metrics():
    metrics = {}
    for ws, path in SMOOTH_RUNS.items():
        rows = read_csv(path)
        swing = phase_rows(rows, "SWING_UP")
        balance = phase_rows(rows, "BALANCE")
        metrics[ws] = {
            "samples": len(rows),
            "swing_samples": len(swing),
            "balance_samples": len(balance),
            "swing_duration_s": duration(swing),
            "balance_duration_s": duration(balance),
            "swing_peak_force_n": max_abs(swing, "cart_force_cmd_n"),
            "swing_mean_force_n": mean_abs(swing, "cart_force_cmd_n"),
            "balance_peak_force_n": max_abs(balance, "cart_force_cmd_n"),
            "balance_mean_force_n": mean_abs(balance, "cart_force_cmd_n"),
            "tail_strict_300": tail_strict(rows, 300),
            "hinge_topic_peak_nm": max_abs(rows, "hinge_assist_torque_nm"),
            "last_degree_deg": fnum(rows[-1], "degree_deg"),
            "last_cart_cm": fnum(rows[-1], "cmX_cm"),
        }
    return metrics


def impulse_metrics():
    metrics = {}
    for ws, (csv_path, events_path) in IMPULSE_RUNS.items():
        rows = read_csv(csv_path)
        events = read_events(events_path)
        balance = [row for row in rows if int(round(fnum(row, "mode"))) == 7]
        metrics[ws] = {
            "samples": len(rows),
            "impulses_n": "; ".join(f"{fnum(event, 'force_n'):+.2f}" for event in events),
            "max_external_n": max_abs(rows, "external_force_n"),
            "max_balance_angle_deg": max_abs(balance, "degree_deg"),
            "max_cart_cm": max_abs(balance, "cmX_cm"),
            "tail_strict_120": tail_strict(rows, 120),
            "last_degree_deg": fnum(rows[-1], "degree_deg"),
            "last_cart_cm": fnum(rows[-1], "cmX_cm"),
        }
    return metrics


def save_metrics_csv(path, smooth, impulse):
    fieldnames = [
        "workspace",
        "smooth_swing_peak_force_n",
        "smooth_swing_mean_force_n",
        "smooth_balance_peak_force_n",
        "smooth_balance_mean_force_n",
        "smooth_swing_duration_s",
        "smooth_balance_duration_s",
        "smooth_tail_strict_300",
        "smooth_hinge_topic_peak_nm",
        "impulses_n",
        "impulse_max_external_n",
        "impulse_max_balance_angle_deg",
        "impulse_max_cart_cm",
        "impulse_tail_strict_120",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for ws in SMOOTH_RUNS:
            writer.writerow(
                {
                    "workspace": ws,
                    "smooth_swing_peak_force_n": f"{smooth[ws]['swing_peak_force_n']:.3f}",
                    "smooth_swing_mean_force_n": f"{smooth[ws]['swing_mean_force_n']:.3f}",
                    "smooth_balance_peak_force_n": f"{smooth[ws]['balance_peak_force_n']:.3f}",
                    "smooth_balance_mean_force_n": f"{smooth[ws]['balance_mean_force_n']:.3f}",
                    "smooth_swing_duration_s": f"{smooth[ws]['swing_duration_s']:.3f}",
                    "smooth_balance_duration_s": f"{smooth[ws]['balance_duration_s']:.3f}",
                    "smooth_tail_strict_300": smooth[ws]["tail_strict_300"],
                    "smooth_hinge_topic_peak_nm": f"{smooth[ws]['hinge_topic_peak_nm']:.6f}",
                    "impulses_n": impulse[ws]["impulses_n"],
                    "impulse_max_external_n": f"{impulse[ws]['max_external_n']:.3f}",
                    "impulse_max_balance_angle_deg": f"{impulse[ws]['max_balance_angle_deg']:.3f}",
                    "impulse_max_cart_cm": f"{impulse[ws]['max_cart_cm']:.3f}",
                    "impulse_tail_strict_120": impulse[ws]["tail_strict_120"],
                }
            )


def bar(ax, labels, values, title, ylabel):
    colors = [COLORS[label] for label in labels]
    bars = ax.bar(labels, values, color=colors, edgecolor="#222222", linewidth=0.8)
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(True, axis="y", alpha=0.25)
    for rect, value in zip(bars, values):
        ax.text(
            rect.get_x() + rect.get_width() / 2,
            rect.get_height(),
            f"{value:.2f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )


def plot_summary(smooth, impulse, path):
    labels = list(SMOOTH_RUNS)
    fig, axes = plt.subplots(2, 3, figsize=(15, 8), constrained_layout=True)
    fig.suptitle("Perbandingan Workspace Pendulum - Smooth No-Assist dan Impulse", fontsize=15)

    bar(
        axes[0, 0],
        labels,
        [smooth[ws]["swing_peak_force_n"] for ws in labels],
        "Peak Force Swing/Catch",
        "N",
    )
    bar(
        axes[0, 1],
        labels,
        [smooth[ws]["balance_peak_force_n"] for ws in labels],
        "Peak Force Saat Balance",
        "N",
    )
    bar(
        axes[0, 2],
        labels,
        [smooth[ws]["swing_duration_s"] for ws in labels],
        "Durasi Swing-Up",
        "s",
    )
    bar(
        axes[1, 0],
        labels,
        [smooth[ws]["balance_duration_s"] for ws in labels],
        "Durasi Data Balance",
        "s",
    )
    bar(
        axes[1, 1],
        labels,
        [impulse[ws]["max_balance_angle_deg"] for ws in labels],
        "Deviasi Sudut Maks Saat Impulse",
        "deg",
    )
    bar(
        axes[1, 2],
        labels,
        [impulse[ws]["max_external_n"] for ws in labels],
        "Impulse Eksternal Maks di Video",
        "N",
    )

    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_smooth_timeseries(path):
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True, constrained_layout=True)
    fig.suptitle("Perbandingan Time-Series Smooth No-Assist", fontsize=15)
    for ws, csv_path in SMOOTH_RUNS.items():
        rows = read_csv(csv_path)
        t = rel_time(rows)
        axes[0].plot(t, [fnum(r, "degree_deg") for r in rows], label=ws, color=COLORS[ws], linewidth=1.3)
        axes[1].plot(t, [fnum(r, "cmX_cm") for r in rows], label=ws, color=COLORS[ws], linewidth=1.3)
        axes[2].plot(t, [fnum(r, "cart_force_cmd_n") for r in rows], label=ws, color=COLORS[ws], linewidth=1.0)
        axes[3].plot(t, [fnum(r, "energy_j") for r in rows], label=ws, color=COLORS[ws], linewidth=1.3)

    axes[0].set_ylabel("Angle (deg)")
    axes[1].set_ylabel("Cart (cm)")
    axes[2].set_ylabel("Cart effort (N)")
    axes[3].set_ylabel("Energy (J)")
    axes[3].set_xlabel("Time from first sample (s)")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right")
    axes[0].axhline(0.0, color="#333333", linewidth=0.8)
    axes[1].axhline(0.0, color="#333333", linewidth=0.8)
    axes[3].axhline(0.7848, color="#555555", linestyle="--", linewidth=1.0, label="target")
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_start_to_balance(path):
    fig, axes = plt.subplots(3, 3, figsize=(16, 9), sharex="col", constrained_layout=True)
    fig.suptitle("Awal ke Swing-Up, Masuk BALANCE, lalu Menuju Tegak", fontsize=15)
    for col, (ws, csv_path) in enumerate(SMOOTH_RUNS.items()):
        rows = read_csv(csv_path)
        start_t = fnum(rows[0], "elapsed_s")
        balance_t = first_mode_time(rows, 7)
        end_t = min(fnum(rows[-1], "elapsed_s"), balance_t + 14.0)
        t = [fnum(row, "elapsed_s") - start_t for row in rows]
        balance_rel = balance_t - start_t
        end_rel = end_t - start_t
        color = COLORS[ws]

        axes[0, col].plot(t, [fnum(r, "degree_deg") for r in rows], color=color, linewidth=1.2)
        axes[1, col].plot(t, [fnum(r, "cmX_cm") for r in rows], color=color, linewidth=1.2)
        axes[2, col].plot(t, [fnum(r, "cart_force_cmd_n") for r in rows], color=color, linewidth=1.0)
        axes[0, col].set_title(f"{ws}: BALANCE @ {balance_rel:.2f}s")

        for row_axes in axes[:, col]:
            row_axes.axvspan(0.0, balance_rel, color="#ffcc66", alpha=0.18, label="SWING_UP")
            row_axes.axvspan(balance_rel, end_rel, color="#99dd99", alpha=0.16, label="BALANCE")
            row_axes.axvline(balance_rel, color="#1f5f1f", linestyle="--", linewidth=1.2)
            row_axes.set_xlim(0.0, end_rel)
            row_axes.grid(True, alpha=0.25)

        axes[0, col].axhline(0.0, color="#222222", linewidth=0.9)
        axes[0, col].axhline(2.0, color="#777777", linestyle=":", linewidth=0.9)
        axes[0, col].axhline(-2.0, color="#777777", linestyle=":", linewidth=0.9)
        axes[1, col].axhline(0.0, color="#222222", linewidth=0.9)
        axes[1, col].axhline(3.0, color="#777777", linestyle=":", linewidth=0.9)
        axes[1, col].axhline(-3.0, color="#777777", linestyle=":", linewidth=0.9)
        axes[2, col].axhline(0.0, color="#222222", linewidth=0.8)

    axes[0, 0].set_ylabel("Angle (deg)")
    axes[1, 0].set_ylabel("Cart (cm)")
    axes[2, 0].set_ylabel("Cart effort (N)")
    for ax in axes[2, :]:
        ax.set_xlabel("Time from first sample (s)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles[:2], labels[:2], loc="upper right")
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_final_upright_hold(path):
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True, constrained_layout=True)
    fig.suptitle("Zoom Akhir: Pendulum Tegak dan Cart di Tengah", fontsize=15)
    hold_seconds = 18.0
    for ws, csv_path in SMOOTH_RUNS.items():
        rows = read_csv(csv_path)
        end_t = fnum(rows[-1], "elapsed_s")
        start_t = max(fnum(rows[0], "elapsed_s"), end_t - hold_seconds)
        hold_rows = [row for row in rows if fnum(row, "elapsed_s") >= start_t]
        t = [fnum(row, "elapsed_s") - start_t for row in hold_rows]
        color = COLORS[ws]
        label = f"{ws} ({sum(1 for row in hold_rows if strict_upright(row))}/{len(hold_rows)} strict)"

        axes[0].plot(t, [fnum(r, "degree_deg") for r in hold_rows], label=label, color=color, linewidth=1.3)
        axes[1].plot(t, [fnum(r, "cmX_cm") for r in hold_rows], label=ws, color=color, linewidth=1.3)
        axes[2].plot(t, [fnum(r, "theta_dot_rad_s") for r in hold_rows], label=ws, color=color, linewidth=1.3)

    axes[0].axhspan(-2.0, 2.0, color="#99dd99", alpha=0.14)
    axes[1].axhspan(-3.0, 3.0, color="#99dd99", alpha=0.14)
    axes[2].axhspan(-0.35, 0.35, color="#99dd99", alpha=0.14)
    axes[0].set_ylim(-2.5, 2.5)
    axes[1].set_ylim(-4.0, 4.0)
    axes[2].set_ylim(-0.45, 0.45)
    axes[0].set_ylabel("Angle (deg)")
    axes[1].set_ylabel("Cart (cm)")
    axes[2].set_ylabel("Theta dot (rad/s)")
    axes[2].set_xlabel(f"Final {hold_seconds:.0f}s window")
    for ax in axes:
        ax.axhline(0.0, color="#222222", linewidth=0.8)
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right")
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_impulse_response(path):
    fig, axes = plt.subplots(3, 3, figsize=(16, 9), sharex="col", constrained_layout=True)
    fig.suptitle("Perbandingan Response Terhadap Impulse Eksternal", fontsize=15)
    for col, ws in enumerate(IMPULSE_RUNS):
        csv_path, events_path = IMPULSE_RUNS[ws]
        rows = read_csv(csv_path)
        events = read_events(events_path)
        first_event_t = fnum(events[0], "time_s") if events else fnum(rows[0], "elapsed_s")
        t = [fnum(r, "elapsed_s") - first_event_t for r in rows]
        color = COLORS[ws]
        axes[0, col].plot(t, [fnum(r, "degree_deg") for r in rows], color=color, linewidth=1.2)
        axes[1, col].plot(t, [fnum(r, "cmX_cm") for r in rows], color=color, linewidth=1.2)
        axes[2, col].plot(t, [fnum(r, "external_force_n") for r in rows], color=color, linewidth=1.2)
        event_label = ", ".join(f"{fnum(event, 'force_n'):+.2f}N" for event in events)
        axes[0, col].set_title(f"{ws}: {event_label}")
        for event in events:
            event_t = fnum(event, "time_s") - first_event_t
            for row_axes in axes[:, col]:
                row_axes.axvspan(
                    event_t,
                    event_t + fnum(event, "duration_s"),
                    color="#ffcc66",
                    alpha=0.25,
                )
                row_axes.axvline(event_t, color="#8a6d1d", linestyle="--", linewidth=0.8)
        for row_axes in axes[:, col]:
            row_axes.grid(True, alpha=0.25)
            row_axes.axvline(0.0, color="#333333", linewidth=0.8)

    axes[0, 0].set_ylabel("Angle (deg)")
    axes[1, 0].set_ylabel("Cart (cm)")
    axes[2, 0].set_ylabel("External force (N)")
    for ax in axes[2, :]:
        ax.set_xlabel("Time from first impulse (s)")
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    smooth = smooth_metrics()
    impulse = impulse_metrics()

    metrics_path = OUT_DIR / "workspace_comparison_metrics_20260512.csv"
    summary_path = OUT_DIR / "workspace_comparison_summary_20260512.png"
    smooth_path = OUT_DIR / "workspace_smooth_timeseries_20260512.png"
    impulse_path = OUT_DIR / "workspace_impulse_response_20260512.png"
    transition_path = OUT_DIR / "workspace_start_to_balance_20260512.png"
    upright_path = OUT_DIR / "workspace_final_upright_hold_20260512.png"

    save_metrics_csv(metrics_path, smooth, impulse)
    plot_summary(smooth, impulse, summary_path)
    plot_smooth_timeseries(smooth_path)
    plot_impulse_response(impulse_path)
    plot_start_to_balance(transition_path)
    plot_final_upright_hold(upright_path)

    print(metrics_path)
    print(summary_path)
    print(smooth_path)
    print(impulse_path)
    print(transition_path)
    print(upright_path)


if __name__ == "__main__":
    main()
