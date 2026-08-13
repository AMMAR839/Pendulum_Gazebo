#!/usr/bin/env python3
import argparse
import csv
import math
import os
import struct
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


BUTTON_A = 1 << 0
BUTTON_X = 1 << 2
BUTTON_Y = 1 << 3

MODE_READY = 3
MODE_SWING_UP = 6
MODE_BALANCE = 7


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def parse_force_list(value):
    forces = []
    for item in value.split(","):
        item = item.strip()
        if item:
            forces.append(float(item))
    if not forces:
        raise argparse.ArgumentTypeError("impulse list cannot be empty")
    return forces


def make_joystick_packet(seq, buttons):
    body = struct.pack("<BBhhhhH", 0x01, seq & 0xFF, 0, 0, 0, 0, buttons)
    checksum = sum(body) & 0xFFFF
    return b"\xAA\x55" + body + struct.pack("<H", checksum)


class ImpulseCapture(Node):
    def __init__(self, args):
        super().__init__("pendulum_impulse_video_capture")
        self.args = args
        self.latest_state = None
        self.latest_cart_velocity_cmd = math.nan
        self.latest_cart_force = math.nan
        self.latest_hinge_assist = math.nan
        self.external_force_n = 0.0
        self.rows = []
        self.events = []
        self.start_time = time.monotonic()
        self.last_record_time = 0.0
        self.seq = 0
        self.serial_fd = None
        self.note = "startup"

        self.create_subscription(
            Float64MultiArray,
            "/pendulum/sim_state",
            self._state_cb,
            50,
        )
        self.create_subscription(
            Float64,
            "/pendulum/cart_velocity_cmd",
            self._cart_velocity_cb,
            50,
        )
        self.create_subscription(Float64, "/pendulum/cart_force_cmd", self._force_cb, 50)
        self.create_subscription(
            Float64,
            "/pendulum/hinge_assist_force_cmd",
            self._hinge_cb,
            50,
        )
        self.external_pub = self.create_publisher(
            Float64,
            "/pendulum/external_disturbance_torque_cmd",
            10,
        )

    def _cart_velocity_cb(self, msg):
        self.latest_cart_velocity_cmd = float(msg.data)

    def _force_cb(self, msg):
        self.latest_cart_force = float(msg.data)

    def _hinge_cb(self, msg):
        self.latest_hinge_assist = float(msg.data)

    def _state_cb(self, msg):
        if len(msg.data) < 8:
            return
        now = time.monotonic()
        data = list(msg.data[:8])
        self.latest_state = data

        if now - self.last_record_time < self.args.sample_period:
            return
        self.last_record_time = now

        mode = int(round(data[7]))
        elapsed = now - self.start_time
        self.rows.append(
            {
                "workspace": self.args.workspace,
                "elapsed_s": f"{elapsed:.3f}",
                "degree_deg": f"{data[0]:.6f}",
                "cmX_cm": f"{data[1]:.6f}",
                "setspeed_cm_s": f"{data[2]:.6f}",
                "energy_j": f"{data[3]:.9f}",
                "theta_dot_rad_s": f"{data[4]:.9f}",
                "theta_rad": f"{data[5]:.9f}",
                "x_center_cm": f"{data[6]:.6f}",
                "mode": mode,
                "cart_velocity_cmd_mps": f"{self.latest_cart_velocity_cmd:.9f}",
                "cart_force_cmd_n": f"{self.latest_cart_force:.9f}",
                "hinge_assist_torque_nm": f"{self.latest_hinge_assist:.9f}",
                "external_force_n": f"{self.external_force_n:.6f}",
                "note": self.note,
            }
        )

    def spin_for(self, duration_s):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_state(self, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_state is not None:
                return True
        return False

    def current_mode(self):
        if self.latest_state is None:
            return None
        return int(round(self.latest_state[7]))

    def current_degree(self):
        if self.latest_state is None:
            return math.nan
        return float(self.latest_state[0])

    def current_theta_dot(self):
        if self.latest_state is None:
            return math.nan
        return float(self.latest_state[4])

    def current_cart_cm(self):
        if self.latest_state is None:
            return math.nan
        return float(self.latest_state[1])

    def open_serial(self):
        deadline = time.monotonic() + self.args.serial_timeout
        while time.monotonic() < deadline:
            if os.path.exists(self.args.serial):
                self.serial_fd = os.open(self.args.serial, os.O_RDWR | os.O_NOCTTY)
                return
            self.spin_for(0.1)
        raise RuntimeError(f"Serial symlink not found: {self.args.serial}")

    def send_button(self, button):
        os.write(self.serial_fd, make_joystick_packet(self.seq, button))
        self.seq += 1
        self.spin_for(0.12)
        os.write(self.serial_fd, make_joystick_packet(self.seq, 0))
        self.seq += 1
        self.spin_for(0.12)

    def stable_balance_now(self):
        return (
            self.current_mode() == MODE_BALANCE
            and abs(self.current_degree()) <= self.args.stable_angle_deg
            and abs(self.current_theta_dot()) <= self.args.stable_rate_rad_s
            and abs(self.current_cart_cm()) <= self.args.stable_cart_cm
        )

    def wait_for_stable_balance(self, timeout_s):
        deadline = time.monotonic() + timeout_s
        stable_since = None
        while time.monotonic() < deadline and rclpy.ok():
            self.spin_for(0.05)
            if self.stable_balance_now():
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= self.args.stable_time:
                    return True
            else:
                stable_since = None
        return False

    def publish_external_force(self, force_n):
        self.external_force_n = float(force_n)
        msg = Float64()
        msg.data = force_n * self.args.pendulum_length_m
        self.external_pub.publish(msg)

    def apply_impulse(self, force_n):
        start_elapsed = time.monotonic() - self.start_time
        self.note = f"impulse_{force_n:+.2f}N"
        self.events.append(
            {
                "time_s": start_elapsed,
                "force_n": force_n,
                "duration_s": self.args.impulse_duration,
            }
        )
        self.publish_external_force(force_n)
        self.spin_for(self.args.impulse_duration)
        self.publish_external_force(0.0)
        self.note = "recover"

    def run_sequence(self):
        if not self.wait_for_state(self.args.state_timeout):
            raise RuntimeError("No /pendulum/sim_state samples received")
        self.open_serial()

        self.note = "homing"
        self.send_button(BUTTON_Y)
        ready_deadline = time.monotonic() + self.args.homing_timeout
        while time.monotonic() < ready_deadline and self.current_mode() != MODE_READY:
            self.spin_for(0.1)

        self.note = "swing_up"
        self.send_button(BUTTON_X)
        swing_started = time.monotonic()
        balance_requested = False

        while time.monotonic() - swing_started < self.args.auto_timeout:
            self.spin_for(0.05)
            if self.current_mode() == MODE_BALANCE:
                break
            if (
                not balance_requested
                and time.monotonic() - swing_started >= self.args.min_swing_before_balance
                and abs(self.current_degree()) <= self.args.manual_balance_deg
            ):
                self.note = "manual_A_near_upright"
                self.send_button(BUTTON_A)
                balance_requested = True

        self.note = "wait_stable"
        if not self.wait_for_stable_balance(self.args.stable_timeout):
            raise RuntimeError("Pendulum did not reach stable BALANCE before impulses")

        for force_n in self.args.impulses:
            self.apply_impulse(force_n)
            if not self.wait_for_stable_balance(self.args.recovery_timeout):
                raise RuntimeError(
                    f"Pendulum did not recover after impulse {force_n:+.2f} N"
                )

        self.note = "final_hold"
        self.spin_for(self.args.final_hold_s)
        self.publish_external_force(0.0)
        if self.serial_fd is not None:
            os.close(self.serial_fd)

    def write_csv(self):
        output = Path(self.args.output_csv)
        output.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = [
            "workspace",
            "elapsed_s",
            "degree_deg",
            "cmX_cm",
            "setspeed_cm_s",
            "energy_j",
            "theta_dot_rad_s",
            "theta_rad",
            "x_center_cm",
            "mode",
            "cart_velocity_cmd_mps",
            "cart_force_cmd_n",
            "hinge_assist_torque_nm",
            "external_force_n",
            "note",
        ]
        with output.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)

    def write_events(self):
        output = Path(self.args.events_csv)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["time_s", "force_n", "duration_s"])
            writer.writeheader()
            writer.writerows(
                {
                    "time_s": f"{event['time_s']:.3f}",
                    "force_n": f"{event['force_n']:.6f}",
                    "duration_s": f"{event['duration_s']:.3f}",
                }
                for event in self.events
            )


def load_rows(path):
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            rows.append({key: row[key] for key in row})
    return rows


def read_events(path):
    if not Path(path).exists():
        return []
    with open(path, newline="") as f:
        return [
            {
                "time_s": float(row["time_s"]),
                "force_n": float(row["force_n"]),
                "duration_s": float(row["duration_s"]),
            }
            for row in csv.DictReader(f)
        ]


def interp_row(rows, t):
    if t <= float(rows[0]["elapsed_s"]):
        return rows[0]
    if t >= float(rows[-1]["elapsed_s"]):
        return rows[-1]
    lo = 0
    hi = len(rows) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if float(rows[mid]["elapsed_s"]) <= t:
            lo = mid
        else:
            hi = mid
    a = rows[lo]
    b = rows[hi]
    ta = float(a["elapsed_s"])
    tb = float(b["elapsed_s"])
    alpha = 0.0 if tb <= ta else (t - ta) / (tb - ta)
    out = dict(a)
    numeric = [
        "degree_deg",
        "cmX_cm",
        "setspeed_cm_s",
        "energy_j",
        "theta_dot_rad_s",
        "theta_rad",
        "x_center_cm",
        "cart_force_cmd_n",
        "hinge_assist_torque_nm",
        "external_force_n",
    ]
    for key in numeric:
        out[key] = str(float(a[key]) * (1.0 - alpha) + float(b[key]) * alpha)
    out["elapsed_s"] = str(t)
    return out


def active_event(events, t):
    for event in events:
        if event["time_s"] <= t <= event["time_s"] + event["duration_s"] + 0.35:
            return event
    return None


def draw_text(img, text, x, y, scale=0.58, color=(230, 235, 240), thickness=1):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def render_video(csv_path, events_path, video_path, title, fps=30.0, width=1280, height=720):
    rows = load_rows(csv_path)
    events = read_events(events_path)
    if not rows:
        raise RuntimeError(f"No rows in {csv_path}")

    start_t = float(rows[0]["elapsed_s"])
    end_t = float(rows[-1]["elapsed_s"])
    out_path = Path(video_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(out_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"Could not open video writer: {video_path}")

    rail_left = 120
    rail_right = width - 120
    rail_y = 500
    rail_limit_cm = 39.0
    px_per_cm = (rail_right - rail_left) / (2.0 * rail_limit_cm)
    rod_len_px = 300

    total_frames = int(math.ceil((end_t - start_t) * fps))
    for frame_idx in range(total_frames + 1):
        t = start_t + frame_idx / fps
        row = interp_row(rows, t)
        degree = float(row["degree_deg"])
        theta = math.radians(degree)
        cart_cm = float(row["cmX_cm"])
        energy = float(row["energy_j"])
        theta_dot = float(row["theta_dot_rad_s"])
        cart_force = float(row["cart_force_cmd_n"])
        external_force = float(row["external_force_n"])
        mode = int(round(float(row["mode"])))
        hinge = float(row["hinge_assist_torque_nm"])

        img = np.full((height, width, 3), (28, 31, 34), dtype=np.uint8)
        cv2.rectangle(img, (0, 0), (width, 92), (18, 22, 26), -1)
        draw_text(img, title, 34, 38, 0.88, (245, 248, 250), 2)
        draw_text(
            img,
            f"t={t-start_t:5.1f}s  mode={'BALANCE' if mode == MODE_BALANCE else 'SWING'}  no smart assist",
            34,
            72,
            0.58,
            (190, 205, 215),
            1,
        )

        # Bench, rail, and cart.
        cv2.rectangle(img, (rail_left - 50, rail_y + 34), (rail_right + 50, rail_y + 86), (45, 48, 48), -1)
        cv2.line(img, (rail_left, rail_y), (rail_right, rail_y), (180, 190, 188), 5)
        cv2.line(img, (rail_left, rail_y + 24), (rail_right, rail_y + 24), (120, 130, 128), 3)
        for tick_cm in range(-30, 31, 10):
            x = int(width / 2 + tick_cm * px_per_cm)
            cv2.line(img, (x, rail_y - 8), (x, rail_y + 36), (80, 88, 90), 1)

        cart_x = int(width / 2 + clamp(cart_cm, -rail_limit_cm, rail_limit_cm) * px_per_cm)
        cart_w = 120
        cart_h = 54
        cv2.rectangle(
            img,
            (cart_x - cart_w // 2, rail_y - cart_h),
            (cart_x + cart_w // 2, rail_y),
            (64, 70, 75),
            -1,
        )
        cv2.rectangle(
            img,
            (cart_x - cart_w // 2, rail_y - cart_h),
            (cart_x + cart_w // 2, rail_y),
            (140, 150, 155),
            2,
        )
        pivot = (cart_x, rail_y - cart_h - 8)
        cv2.circle(img, pivot, 18, (210, 214, 212), -1)
        cv2.circle(img, pivot, 10, (88, 92, 96), -1)

        tip = (
            int(pivot[0] + rod_len_px * math.sin(theta)),
            int(pivot[1] - rod_len_px * math.cos(theta)),
        )
        cv2.line(img, pivot, tip, (210, 210, 202), 8)
        cv2.line(img, pivot, tip, (95, 98, 100), 2)
        cv2.circle(img, tip, 7, (230, 230, 220), -1)

        # Upright target line.
        cv2.line(
            img,
            (pivot[0], pivot[1]),
            (pivot[0], pivot[1] - rod_len_px),
            (70, 100, 80),
            1,
        )

        # External impulse arrow.
        event = active_event(events, t)
        if event is not None or abs(external_force) > 1e-6:
            force = external_force if abs(external_force) > 1e-6 else event["force_n"]
            direction = 1 if force >= 0 else -1
            arrow_len = int(120 + 80 * min(abs(force), 1.5) / 1.5)
            start = (pivot[0] - direction * arrow_len, pivot[1] - 150)
            end = (pivot[0], pivot[1] - 150)
            color = (35, 145, 255) if force >= 0 else (80, 210, 120)
            cv2.arrowedLine(img, start, end, color, 6, tipLength=0.18)
            draw_text(
                img,
                f"External impulse {force:+.2f} N",
                min(start[0], end[0]) - 10,
                start[1] - 18,
                0.7,
                color,
                2,
            )

        panel_x = width - 390
        cv2.rectangle(img, (panel_x, 120), (width - 35, 315), (38, 42, 46), -1)
        cv2.rectangle(img, (panel_x, 120), (width - 35, 315), (85, 95, 100), 1)
        draw_text(img, f"angle        {degree:+7.3f} deg", panel_x + 20, 158)
        draw_text(img, f"theta_dot    {theta_dot:+7.3f} rad/s", panel_x + 20, 190)
        draw_text(img, f"cart         {cart_cm:+7.3f} cm", panel_x + 20, 222)
        draw_text(img, f"energy       {energy:7.4f} J", panel_x + 20, 254)
        draw_text(img, f"cart effort  {cart_force:+7.2f} N", panel_x + 20, 286)

        cv2.rectangle(img, (35, 620), (width - 35, 680), (18, 22, 26), -1)
        draw_text(
            img,
            f"hinge torque cmd = {hinge:+.3f} Nm     smart assist off; impulse torque appears here during external force",
            60,
            656,
            0.62,
            (205, 215, 220),
            1,
        )
        writer.write(img)

    writer.release()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--events-csv", required=True)
    parser.add_argument("--output-video", required=True)
    parser.add_argument("--title", required=True)
    parser.add_argument("--impulses", type=parse_force_list, default=parse_force_list("0.5,-0.5,0.8"))
    parser.add_argument("--impulse-duration", type=float, default=0.20)
    parser.add_argument("--pendulum-length-m", type=float, default=0.40)
    parser.add_argument("--sample-period", type=float, default=0.02)
    parser.add_argument("--state-timeout", type=float, default=25.0)
    parser.add_argument("--serial-timeout", type=float, default=25.0)
    parser.add_argument("--homing-timeout", type=float, default=8.0)
    parser.add_argument("--auto-timeout", type=float, default=45.0)
    parser.add_argument("--min-swing-before-balance", type=float, default=8.0)
    parser.add_argument("--manual-balance-deg", type=float, default=8.0)
    parser.add_argument("--stable-time", type=float, default=0.8)
    parser.add_argument("--stable-timeout", type=float, default=20.0)
    parser.add_argument("--stable-angle-deg", type=float, default=2.0)
    parser.add_argument("--stable-rate-rad-s", type=float, default=0.35)
    parser.add_argument("--stable-cart-cm", type=float, default=3.0)
    parser.add_argument("--recovery-timeout", type=float, default=14.0)
    parser.add_argument("--final-hold-s", type=float, default=3.0)
    parser.add_argument("--render-only", action="store_true")
    parser.add_argument("--fps", type=float, default=30.0)
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.render_only:
        rclpy.init()
        node = ImpulseCapture(args)
        try:
            node.run_sequence()
            node.write_csv()
            node.write_events()
            print(
                f"wrote {len(node.rows)} rows to {args.output_csv}; "
                f"{len(node.events)} impulses to {args.events_csv}"
            )
        finally:
            try:
                node.publish_external_force(0.0)
            except Exception:
                pass
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    render_video(
        args.output_csv,
        args.events_csv,
        args.output_video,
        args.title,
        fps=args.fps,
    )
    print(f"wrote video {args.output_video}")


if __name__ == "__main__":
    main()
