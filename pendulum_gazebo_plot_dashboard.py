#!/usr/bin/env python3
import argparse
import math
import os
import re
import signal
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
os.environ.setdefault("GDK_BACKEND", "x11")

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from lib_com import make_external_force_packet, make_packet, open_serial


ROOT = Path(__file__).resolve().parent

BUTTON_A = 1 << 0
BUTTON_B = 1 << 1
BUTTON_X = 1 << 2
BUTTON_Y = 1 << 3

MODE_WAITING = 1
MODE_HOMING = 2
MODE_READY = 3
MODE_SINE = 4
MODE_FINISH = 5
MODE_SWING_UP = 6
MODE_BALANCE = 7

MODE_LABELS = {
    MODE_WAITING: "WAIT",
    MODE_HOMING: "HOME",
    MODE_READY: "READY",
    MODE_SINE: "SINE",
    MODE_FINISH: "FINISH",
    MODE_SWING_UP: "SWING",
    MODE_BALANCE: "BALANCE",
}

CLOSE_CAMERA_POSE = "-0.70 -0.95 0.72 0 0.25 0.64"


@dataclass(frozen=True)
class WorkspaceConfig:
    key: str
    label: str
    workspace: Path
    package: str
    launch_file: str
    serial_link: str


LQR_WORKSPACE = WorkspaceConfig(
    key="lqr",
    label="Pendulum LQR",
    workspace=ROOT / "lqr-pendulum",
    package="linear_inverted_pendulum_sim",
    launch_file="sim.launch.py",
    serial_link="/tmp/pendulum_lqr_dashboard_serial",
)

WORKSPACES = {
    "lqr": LQR_WORKSPACE,
    "ros2": LQR_WORKSPACE,
    "real": WorkspaceConfig(
        key="real",
        label="Pendulum real/manual-book",
        workspace=ROOT / "pendulum_real_ws",
        package="linear_inverted_pendulum_real_sim",
        launch_file="real_sim.launch.py",
        serial_link="/tmp/pendulum_real_dashboard_serial",
    ),
    "pid": WorkspaceConfig(
        key="pid",
        label="Pendulum PID",
        workspace=ROOT / "pendulum_pid_ws",
        package="linear_inverted_pendulum_pid_sim",
        launch_file="pid_sim.launch.py",
        serial_link="/tmp/pendulum_pid_dashboard_serial",
    ),
}


def resolve_ros_domain_id(value):
    if str(value).lower() != "auto":
        return str(value)
    return str(100 + (os.getpid() % 80))


class PlotMonitor(Node):
    def __init__(self, sample_period_s, max_points):
        super().__init__("pendulum_plot_dashboard")
        self.sample_period_s = sample_period_s
        self.rows = deque(maxlen=max_points)
        self.lock = threading.Lock()
        self.start_time = time.monotonic()
        self.last_sample_time = 0.0
        self.latest_state = None
        self.latest_cart_force = math.nan
        self.latest_hinge_torque = math.nan
        self.latest_external_force = 0.0

        self.create_subscription(Float64MultiArray, "/pendulum/sim_state", self._state_cb, 50)
        self.create_subscription(Float64, "/pendulum/cart_force_cmd", self._cart_force_cb, 50)
        self.create_subscription(
            Float64,
            "/pendulum/hinge_assist_force_cmd",
            self._hinge_torque_cb,
            50,
        )

    def _cart_force_cb(self, msg):
        with self.lock:
            self.latest_cart_force = float(msg.data)

    def _hinge_torque_cb(self, msg):
        with self.lock:
            self.latest_hinge_torque = float(msg.data)

    def _state_cb(self, msg):
        if len(msg.data) < 8:
            return
        now = time.monotonic()
        if now - self.last_sample_time < self.sample_period_s:
            return
        self.last_sample_time = now

        data = list(msg.data[:8])
        with self.lock:
            self.latest_state = data
            self.rows.append(
                {
                    "t": now - self.start_time,
                    "degree": float(data[0]),
                    "cart_cm": float(data[1]),
                    "energy": float(data[3]),
                    "theta_dot": float(data[4]),
                    "mode": int(round(data[7])),
                    "cart_force": self.latest_cart_force,
                    "hinge_torque": self.latest_hinge_torque,
                    "external_force": self.latest_external_force,
                }
            )

    def set_external_force(self, force_n):
        with self.lock:
            self.latest_external_force = float(force_n)

    def snapshot(self):
        with self.lock:
            return list(self.rows)

    def state(self):
        with self.lock:
            return None if self.latest_state is None else list(self.latest_state)

    def mode(self):
        state = self.state()
        if state is None:
            return None
        return int(round(state[7]))

    def degree(self):
        state = self.state()
        if state is None:
            return math.nan
        return float(state[0])

    def theta_dot(self):
        state = self.state()
        if state is None:
            return math.nan
        return float(state[4])

    def cart_cm(self):
        state = self.state()
        if state is None:
            return math.nan
        return float(state[1])


class DashboardRunner:
    def __init__(self, args, monitor, ui_pump=None):
        self.args = args
        self.monitor = monitor
        self.cfg = WORKSPACES[args.workspace]
        self.ui_pump = ui_pump
        self.launch_proc = None
        self.launch_pgid = None
        self.launch_log = None
        self.serial = None
        self.seq = 0
        self.auto_thread = None
        self.external_thread = None
        self.stop_event = threading.Event()
        self.external_done = threading.Event()
        self.manual_external_lock = threading.Lock()
        self.main_thread_id = threading.get_ident()
        self.screen_w, self.screen_h = screen_size()

    def start(self):
        self._launch_gazebo()
        self._wait_for_serial(self.cfg.serial_link, self.args.serial_timeout)
        self._wait_for_state(self.args.state_timeout)
        self.serial = open_serial(self.cfg.serial_link, 115200, timeout=0.1)
        self.set_external_force(0.0, quiet=True)
        print(f"[OK] {self.cfg.label} siap tanpa main.py GUI.")

        if self.args.auto:
            self.auto_thread = threading.Thread(target=self.auto_swing_balance, daemon=True)
            self.auto_thread.start()
        if self.external_test_requested():
            self.external_thread = threading.Thread(target=self.external_worker, daemon=True)
            self.external_thread.start()

    def close(self):
        self.stop_event.set()
        if self.external_thread is not None and self.external_thread.is_alive():
            self.external_thread.join(timeout=2.0)
        if self.auto_thread is not None and self.auto_thread.is_alive():
            self.auto_thread.join(timeout=2.0)
        if self.serial is not None:
            try:
                self.finish()
                self.set_external_force(0.0, quiet=True)
                self.serial.close()
            except Exception:
                pass
            self.serial = None

        if self.launch_proc is not None and self.launch_proc.poll() is None:
            print("[STOP] Menghentikan Gazebo / ROS 2 ...")
            try:
                os.killpg(self.launch_pgid, signal.SIGTERM)
                self.launch_proc.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.launch_pgid, signal.SIGKILL)
                self.launch_proc.wait(timeout=5.0)
            except ProcessLookupError:
                pass

        if self.launch_pgid is not None:
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                try:
                    os.killpg(self.launch_pgid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.1)
            else:
                try:
                    os.killpg(self.launch_pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

        self.launch_proc = None
        self.launch_pgid = None
        if self.launch_log is not None:
            try:
                self.launch_log.close()
            except Exception:
                pass
            self.launch_log = None

    def _launch_gazebo(self):
        log_dir = ROOT / "logs"
        log_dir.mkdir(exist_ok=True)
        self.launch_log = (log_dir / f"pendulum_dashboard_{self.cfg.key}.log").open("w")

        if self.args.headless:
            gz_args = "-r -s empty.sdf"
        else:
            world = (
                self.cfg.workspace
                / "install"
                / self.cfg.package
                / "share"
                / self.cfg.package
                / "worlds"
                / "pendulum_close_camera.sdf"
            )
            left_x, left_y, left_w, _, _, _, usable_h = split_window_geometry(
                self.screen_w,
                self.screen_h,
            )
            gui_config = write_gazebo_gui_config(
                log_dir / "pendulum_dashboard_gazebo_gui.config",
                left_x,
                left_y,
                left_w,
                usable_h,
            )
            gz_args = f"-r --gui-config {gui_config} {world}"

        cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {self.cfg.workspace}/install/setup.bash && "
            f"ros2 launch {self.cfg.package} {self.cfg.launch_file} "
            f"gz_args:='{gz_args}' "
            f"serial_link:={self.cfg.serial_link} "
            f"external_force_visual_hold_s:={self.args.external_visual_hold}"
        )
        print(f"[RUN] Menjalankan {self.cfg.label}: Gazebo + grafik live")
        self.launch_proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            cwd=str(self.cfg.workspace),
            stdin=subprocess.DEVNULL,
            stdout=self.launch_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.launch_pgid = os.getpgid(self.launch_proc.pid)

    def _wait_for_serial(self, serial_link, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._pump_ui()
            if self.launch_proc and self.launch_proc.poll() is not None:
                raise RuntimeError("Launch berhenti sebelum pseudo-serial siap.")
            if os.path.exists(serial_link):
                return
            time.sleep(0.1)
        raise RuntimeError(f"Pseudo-serial belum muncul: {serial_link}")

    def _wait_for_state(self, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._pump_ui()
            if self.launch_proc and self.launch_proc.poll() is not None:
                raise RuntimeError("Launch berhenti sebelum /pendulum/sim_state siap.")
            if self.monitor.state() is not None:
                return
            time.sleep(0.1)
        raise RuntimeError("Belum ada data /pendulum/sim_state.")

    def _pump_ui(self):
        if self.ui_pump is None:
            return
        try:
            self.ui_pump()
        except Exception:
            pass

    def send_button(self, button, label):
        if self.serial is None:
            raise RuntimeError("Serial belum terbuka.")
        self.serial.write(make_packet(self.seq & 0xFF, 0, 0, 0, 0, button))
        self.serial.flush()
        self.seq += 1
        time.sleep(0.12)
        self.serial.write(make_packet(self.seq & 0xFF, 0, 0, 0, 0, 0))
        self.serial.flush()
        self.seq += 1
        time.sleep(0.12)
        print(f"[TX] {label}")

    def set_external_force(self, force_n, quiet=False):
        if self.serial is None:
            raise RuntimeError("Serial belum terbuka.")
        self.serial.write(make_external_force_packet(self.seq & 0xFF, force_n))
        self.serial.flush()
        self.seq += 1
        self.monitor.set_external_force(force_n)
        if not quiet:
            print(f"[TX] External force {force_n:+.3f} N")

    def homing(self):
        self.send_button(BUTTON_Y, "W / HOMING")
        deadline = time.monotonic() + self.args.homing_timeout
        while time.monotonic() < deadline:
            if self.monitor.mode() == MODE_READY:
                print("[OK] READY")
                return
            time.sleep(0.1)
        print("[WARN] Homing timeout, lanjut mencoba swing-up.")

    def swing_up(self):
        self.send_button(BUTTON_X, "A / SWING-UP")

    def balance(self):
        self.send_button(BUTTON_A, "S / BALANCE")

    def external_impulse_now(self):
        self.trigger_external_impulse("X / EXTERNAL FORCE")

    def finish(self):
        if self.serial is not None:
            self.send_button(BUTTON_B, "D / FINISH")

    def auto_swing_balance(self):
        try:
            if self.monitor.mode() != MODE_READY:
                self.homing()
            self.swing_up()

            start = time.monotonic()
            balance_requested = False
            print("[AUTO] Swing-up berjalan; grafik akan menunjukkan mode 6 -> mode 7.")

            while time.monotonic() - start < self.args.auto_timeout:
                mode = self.monitor.mode()
                degree = abs(self.monitor.degree())
                elapsed = time.monotonic() - start

                if mode == MODE_BALANCE:
                    print("[OK] BALANCE aktif.")
                    return

                if (
                    not balance_requested
                    and elapsed >= self.args.min_swing_before_balance
                    and degree <= self.args.manual_balance_deg
                ):
                    self.balance()
                    balance_requested = True

                if (
                    not balance_requested
                    and self.args.force_balance_after > 0.0
                    and elapsed >= self.args.force_balance_after
                ):
                    self.balance()
                    balance_requested = True

                time.sleep(0.05)

            print("[WARN] Auto swing/balance timeout.")
        except Exception as exc:
            print(f"[AUTO] Gagal: {exc}")

    def external_test_requested(self):
        return self.args.find_max_external or (
            self.args.external_impulse_force is not None
            and self.args.external_trigger == "balance"
        )

    def trigger_external_impulse(self, reason):
        if self.args.external_impulse_force is None:
            return
        if not self.manual_external_lock.acquire(blocking=False):
            print("[IMPULSE] Gaya eksternal masih aktif; trigger X diabaikan dulu.")
            return

        try:
            force_n = float(self.args.external_impulse_force)
            duration_s = max(0.01, float(self.args.external_impulse_duration))
            print(
                f"[IMPULSE] {reason} memicu gaya eksternal {force_n:+.3f} N "
                f"selama {duration_s:.3f} s tanpa menunggu BALANCE."
            )
            self.set_external_force(force_n)
            self._sleep_with_ui(duration_s)
        finally:
            try:
                self.set_external_force(0.0, quiet=True)
            finally:
                self.manual_external_lock.release()

    def external_worker(self):
        try:
            if self.auto_thread is not None:
                while self.auto_thread.is_alive() and not self.stop_event.is_set():
                    time.sleep(0.1)

            if not self.ensure_stable_balance("awal uji gaya eksternal"):
                return
            if self.args.external_impulse_delay > 0.0:
                self._sleep_with_ui(self.args.external_impulse_delay)

            if self.args.external_impulse_force is not None:
                force_n = float(self.args.external_impulse_force)
                survived, reason = self.run_external_impulse(force_n)
                result = "SURVIVED" if survived else f"FALL/{reason}"
                print(f"[IMPULSE] force={force_n:+.3f} N result={result}")

            if self.args.find_max_external:
                self.find_max_external_impulse()
        except Exception as exc:
            print(f"[IMPULSE] Gagal menjalankan uji gaya eksternal: {exc}")
        finally:
            self.external_done.set()
            if self.serial is not None:
                try:
                    self.set_external_force(0.0, quiet=True)
                except Exception:
                    pass

    def ensure_stable_balance(self, context):
        if self.wait_for_stable_balance(self.args.external_stable_timeout):
            return True

        print(f"[IMPULSE] Balance belum stabil untuk {context}; mencoba homing + swing-up ulang.")
        try:
            self.set_external_force(0.0, quiet=True)
            self.finish()
            if self.monitor.mode() != MODE_READY:
                self.homing()
            self.swing_up()
            if self.wait_for_balance_mode(self.args.auto_timeout):
                return self.wait_for_stable_balance(self.args.external_stable_timeout)
        except Exception as exc:
            print(f"[IMPULSE] Gagal menyiapkan BALANCE ulang: {exc}")
        print("[IMPULSE] Uji gaya eksternal dibatalkan karena BALANCE belum stabil.")
        return False

    def wait_for_balance_mode(self, timeout_s):
        start = time.monotonic()
        balance_requested = False
        while time.monotonic() - start < timeout_s and not self.stop_event.is_set():
            mode = self.monitor.mode()
            degree = abs(self.monitor.degree())
            elapsed = time.monotonic() - start

            if mode == MODE_BALANCE:
                return True

            if (
                not balance_requested
                and elapsed >= self.args.min_swing_before_balance
                and degree <= self.args.manual_balance_deg
            ):
                self.balance()
                balance_requested = True

            if (
                not balance_requested
                and self.args.force_balance_after > 0.0
                and elapsed >= self.args.force_balance_after
            ):
                self.balance()
                balance_requested = True

            self._sleep_with_ui(0.05)
        return False

    def wait_for_stable_balance(self, timeout_s):
        stable_since = None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not self.stop_event.is_set():
            stable = (
                self.monitor.mode() == MODE_BALANCE
                and abs(self.monitor.degree()) <= self.args.external_stable_angle_deg
                and abs(self.monitor.theta_dot()) <= self.args.external_stable_rate_rad_s
                and abs(self.monitor.cart_cm()) <= self.args.external_stable_cart_cm
            )
            if stable:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= self.args.external_stable_time:
                    return True
            else:
                stable_since = None
            self._sleep_with_ui(0.05)
        return False

    def run_external_impulse(self, force_n):
        print(
            f"[IMPULSE] Memberi gaya sekali: {force_n:+.3f} N "
            f"selama {self.args.external_impulse_duration:.3f} s"
        )
        reason = ""
        start = time.monotonic()
        self.set_external_force(force_n)
        try:
            while (
                time.monotonic() - start < self.args.external_impulse_duration
                and not self.stop_event.is_set()
            ):
                reason = self.external_failure_reason()
                if reason:
                    return False, reason
                self._sleep_with_ui(0.02)
        finally:
            self.set_external_force(0.0, quiet=True)

        recovery_deadline = time.monotonic() + self.args.external_recovery_timeout
        stable_since = None
        while time.monotonic() < recovery_deadline and not self.stop_event.is_set():
            reason = self.external_failure_reason()
            if reason:
                return False, "recovery_" + reason

            stable = (
                self.monitor.mode() == MODE_BALANCE
                and abs(self.monitor.degree()) <= self.args.external_stable_angle_deg
                and abs(self.monitor.theta_dot()) <= self.args.external_stable_rate_rad_s
                and abs(self.monitor.cart_cm()) <= self.args.external_stable_cart_cm
            )
            if stable:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= self.args.external_stable_time:
                    return True, ""
            else:
                stable_since = None
            self._sleep_with_ui(0.05)
        return False, "did_not_recover_to_stable_balance"

    def external_failure_reason(self):
        mode = self.monitor.mode()
        degree = abs(self.monitor.degree())
        cart_cm = abs(self.monitor.cart_cm())
        if mode != MODE_BALANCE:
            return f"mode_exited_{mode}"
        if math.isfinite(degree) and degree >= self.args.external_fail_angle_deg:
            return "angle_limit"
        if math.isfinite(cart_cm) and cart_cm >= self.args.external_rail_fail_cm:
            return "rail_limit"
        return ""

    def find_max_external_impulse(self):
        directions = parse_direction_list(self.args.external_directions)
        forces = sorted({abs(force) for force in self.args.external_max_forces if force > 0.0})
        if not forces:
            print("[MAX] Daftar gaya kosong; uji maksimum dilewati.")
            return

        print(
            "[MAX] Mencari gaya impulse maksimum yang masih bisa ditahan "
            f"(durasi {self.args.external_impulse_duration:.3f} s)."
        )
        for direction in directions:
            sign = 1.0 if direction == "positive" else -1.0
            last_survived = math.nan
            critical = math.nan
            reason = ""

            if not self.ensure_stable_balance(f"arah {direction}"):
                break

            for force in forces:
                if not self.wait_for_stable_balance(self.args.external_stable_timeout):
                    reason = "not_stable_before_trial"
                    critical = force
                    break
                survived, fail_reason = self.run_external_impulse(sign * force)
                if survived:
                    last_survived = force
                    print(f"[MAX] {direction} {force:.3f} N: survived")
                    continue
                critical = force
                reason = fail_reason
                print(f"[MAX] {direction} {force:.3f} N: failed ({reason})")
                break

            if math.isfinite(last_survived):
                print(
                    f"[MAX] {self.cfg.key}/{direction}: gaya impulse aman terakhir "
                    f"{last_survived:.3f} N"
                )
            else:
                print(f"[MAX] {self.cfg.key}/{direction}: belum ada gaya yang survive.")

            if math.isfinite(critical):
                print(
                    f"[MAX] {self.cfg.key}/{direction}: gaya gagal pertama "
                    f"{critical:.3f} N, reason={reason}"
                )
            else:
                print(
                    f"[MAX] {self.cfg.key}/{direction}: belum gagal sampai "
                    f"{forces[-1]:.3f} N"
                )

    def _sleep_with_ui(self, duration_s):
        deadline = time.monotonic() + max(0.0, duration_s)
        while time.monotonic() < deadline and not self.stop_event.is_set():
            if threading.get_ident() == self.main_thread_id:
                self._pump_ui()
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))


def screen_size():
    try:
        out = subprocess.check_output(["xrandr", "--current"], text=True)
    except Exception:
        return 1920, 1080

    for line in out.splitlines():
        if " connected primary " in line or " connected " in line:
            match = re.search(r"(\d+)x(\d+)\+", line)
            if match:
                return int(match.group(1)), int(match.group(2))
    return 1920, 1080


def parse_float_list_arg(text):
    values = []
    for item in text.split(","):
        item = item.strip()
        if item:
            values.append(float(item))
    if not values:
        raise argparse.ArgumentTypeError("daftar gaya tidak boleh kosong")
    return values


def parse_direction_list(text):
    directions = []
    for item in text.split(","):
        direction = item.strip().lower()
        if not direction:
            continue
        if direction not in ("positive", "negative"):
            raise ValueError("external direction harus positive atau negative")
        directions.append(direction)
    return directions or ["positive"]


def set_plot_geometry(fig, x, y, width, height):
    manager = plt.get_current_fig_manager()
    try:
        manager.set_window_title("Pendulum swing-up / balance graph")
    except Exception:
        pass

    window = getattr(manager, "window", None)
    if window is None:
        return

    try:
        window.setGeometry(int(x), int(y), int(width), int(height))
        return
    except Exception:
        pass

    try:
        window.wm_geometry(f"{int(width)}x{int(height)}+{int(x)}+{int(y)}")
    except Exception:
        pass


def desktop_work_area(screen_w, screen_h):
    try:
        out = subprocess.check_output(
            ["xprop", "-root", "_NET_WORKAREA"],
            text=True,
            stderr=subprocess.DEVNULL,
        )
        values = [int(value) for value in re.findall(r"-?\d+", out)]
        if len(values) >= 4 and values[2] > 0 and values[3] > 0:
            return values[0], values[1], values[2], values[3]
    except Exception:
        pass
    return 0, 0, screen_w, max(500, screen_h - 80)


def split_window_geometry(screen_w, screen_h):
    area_x, area_y, area_w, area_h = desktop_work_area(screen_w, screen_h)
    usable_h = max(500, area_h)
    left_w = max(640, area_w // 2)
    right_x = area_x + left_w
    right_w = max(640, area_w - left_w)
    if area_w >= 1280 and left_w + right_w > area_w:
        right_w = area_w - left_w
    return area_x, area_y, left_w, right_x, area_y, right_w, usable_h


def tile_window_by_title(title_part, x, y, width, height):
    title_part = title_part.lower()
    for window_id in x11_window_ids():
        try:
            props = subprocess.check_output(
                ["xprop", "-id", window_id, "_NET_WM_NAME", "WM_NAME", "WM_CLASS"],
                text=True,
                stderr=subprocess.DEVNULL,
            ).lower()
        except Exception:
            continue
        if title_part not in props:
            continue
        return x11_move_resize_window(window_id, x, y, width, height)
    return False


def x11_window_ids():
    for prop in ("_NET_CLIENT_LIST_STACKING", "_NET_CLIENT_LIST"):
        try:
            out = subprocess.check_output(
                ["xprop", "-root", prop],
                text=True,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            continue
        ids = re.findall(r"0x[0-9a-fA-F]+", out)
        if ids:
            return ids
    return []


def x11_move_resize_window(window_id, x, y, width, height):
    try:
        import ctypes
        from ctypes.util import find_library

        x11_clear_maximized_state(window_id)
        lib = ctypes.cdll.LoadLibrary(find_library("X11") or "libX11.so.6")
        lib.XOpenDisplay.argtypes = [ctypes.c_char_p]
        lib.XOpenDisplay.restype = ctypes.c_void_p
        display = lib.XOpenDisplay(None)
        if not display:
            return False

        try:
            window = ctypes.c_ulong(int(window_id, 16))
            lib.XMoveResizeWindow(
                display,
                window,
                int(x),
                int(y),
                max(1, int(width)),
                max(1, int(height)),
            )
            lib.XRaiseWindow(display, window)
            lib.XFlush(display)
            return True
        finally:
            lib.XCloseDisplay(display)
    except Exception:
        return False


def x11_clear_maximized_state(window_id):
    try:
        subprocess.run(
            ["xprop", "-id", window_id, "-remove", "_NET_WM_STATE"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except Exception:
        pass


def tile_dashboard_windows(screen_w, screen_h):
    left_x, left_y, left_w, right_x, right_y, right_w, usable_h = split_window_geometry(
        screen_w,
        screen_h,
    )
    gazebo_ok = tile_window_by_title("Gazebo Sim", left_x, left_y, left_w, usable_h)
    plot_ok = tile_window_by_title(
        "Pendulum swing-up / balance graph",
        right_x,
        right_y,
        right_w,
        usable_h,
    )
    return gazebo_ok or plot_ok


def write_gazebo_gui_config(path, x, y, width, height):
    path.parent.mkdir(exist_ok=True)
    path.write_text(
        f"""<?xml version="1.0"?>
<window>
  <position_x>{int(x)}</position_x>
  <position_y>{int(y)}</position_y>
  <width>{int(width)}</width>
  <height>{int(height)}</height>
  <style
    material_theme="Light"
    material_primary="DeepOrange"
    material_accent="LightBlue"
    toolbar_color_light="#f3f3f3"
    toolbar_text_color_light="#111111"
    toolbar_color_dark="#414141"
    toolbar_text_color_dark="#f3f3f3"
  />
</window>

<plugin filename="MinimalScene" name="3D View">
  <gz-gui>
    <title>3D View</title>
    <property type="bool" key="showTitleBar">false</property>
    <property type="string" key="state">docked</property>
  </gz-gui>
  <engine>ogre2</engine>
  <scene>scene</scene>
  <ambient_light>0.55 0.55 0.55</ambient_light>
  <background_color>0.78 0.80 0.83</background_color>
  <camera_pose>{CLOSE_CAMERA_POSE}</camera_pose>
</plugin>
<plugin filename="GzSceneManager" name="Scene Manager">
  <gz-gui>
    <property key="resizable" type="bool">false</property>
    <property key="width" type="double">5</property>
    <property key="height" type="double">5</property>
    <property key="state" type="string">floating</property>
    <property key="showTitleBar" type="bool">false</property>
  </gz-gui>
</plugin>
<plugin filename="InteractiveViewControl" name="Interactive view control">
  <gz-gui>
    <property key="resizable" type="bool">false</property>
    <property key="width" type="double">5</property>
    <property key="height" type="double">5</property>
    <property key="state" type="string">floating</property>
    <property key="showTitleBar" type="bool">false</property>
  </gz-gui>
</plugin>
<plugin filename="CameraTracking" name="Camera Tracking">
  <gz-gui>
    <property key="resizable" type="bool">false</property>
    <property key="width" type="double">5</property>
    <property key="height" type="double">5</property>
    <property key="state" type="string">floating</property>
    <property key="showTitleBar" type="bool">false</property>
  </gz-gui>
</plugin>
<plugin filename="MarkerManager" name="Marker manager">
  <gz-gui>
    <property key="resizable" type="bool">false</property>
    <property key="width" type="double">5</property>
    <property key="height" type="double">5</property>
    <property key="state" type="string">floating</property>
    <property key="showTitleBar" type="bool">false</property>
  </gz-gui>
</plugin>
<plugin filename="WorldControl" name="World control">
  <gz-gui>
    <title>World control</title>
    <property type="bool" key="showTitleBar">false</property>
    <property type="bool" key="resizable">false</property>
    <property type="double" key="height">72</property>
    <property type="double" key="z">1</property>
    <property type="string" key="state">floating</property>
    <anchors target="3D View">
      <line own="left" target="left"/>
      <line own="bottom" target="bottom"/>
    </anchors>
  </gz-gui>
  <play_pause>true</play_pause>
  <step>true</step>
  <start_paused>false</start_paused>
  <use_event>true</use_event>
</plugin>
<plugin filename="WorldStats" name="World stats">
  <gz-gui>
    <title>World stats</title>
    <property type="bool" key="showTitleBar">false</property>
    <property type="bool" key="resizable">false</property>
    <property type="double" key="height">110</property>
    <property type="double" key="width">290</property>
    <property type="double" key="z">1</property>
    <property type="string" key="state">floating</property>
    <anchors target="3D View">
      <line own="right" target="right"/>
      <line own="bottom" target="bottom"/>
    </anchors>
  </gz-gui>
  <sim_time>true</sim_time>
  <real_time>true</real_time>
  <real_time_factor>true</real_time_factor>
  <iterations>true</iterations>
</plugin>
""",
        encoding="utf-8",
    )
    return path


def create_plot(screen_w, screen_h):
    plt.ion()
    fig, axes = plt.subplots(5, 1, figsize=(8, 8), sharex=True)
    fig.subplots_adjust(left=0.10, right=0.97, top=0.91, bottom=0.08, hspace=0.22)

    lines = {}
    labels = [
        ("degree", "Pendulum angle (deg)", "#1f77b4"),
        ("cart_cm", "Cart position (cm)", "#2ca02c"),
        ("cart_force", "Cart force command (N)", "#d62728"),
        ("external_force", "External impulse (N)", "#ff7f0e"),
        ("mode", "Mode", "#9467bd"),
    ]
    for ax, (key, label, color) in zip(axes, labels):
        (line,) = ax.plot([], [], color=color, linewidth=1.8)
        lines[key] = line
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.35)

    axes[-1].set_yticks(list(MODE_LABELS.keys()))
    axes[-1].set_yticklabels([MODE_LABELS[key] for key in MODE_LABELS])
    axes[-1].set_xlabel("time (s)")

    status_text = fig.text(
        0.01,
        0.965,
        "waiting for /pendulum/sim_state",
        ha="left",
        va="top",
        fontsize=10,
        family="monospace",
    )

    _, _, _, right_x, right_y, right_w, usable_h = split_window_geometry(screen_w, screen_h)
    set_plot_geometry(fig, right_x, right_y, right_w, usable_h)
    plt.show(block=False)
    set_plot_geometry(fig, right_x, right_y, right_w, usable_h)
    tile_dashboard_windows(screen_w, screen_h)
    fig.canvas.draw_idle()
    fig.canvas.flush_events()
    return fig, axes, lines, status_text


def smooth_values(values, window):
    if window <= 1:
        return values

    smoothed = []
    recent = deque(maxlen=window)
    for value in values:
        if math.isfinite(value):
            recent.append(value)
            smoothed.append(sum(recent) / len(recent))
        else:
            smoothed.append(value)
    return smoothed


def update_plot(fig, axes, lines, status_text, rows, history_s, smoothing_samples):
    if not rows:
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        return

    t = [row["t"] for row in rows]
    t_max = max(t[-1], history_s)
    x_min = max(0.0, t[-1] - history_s)
    x_max = max(history_s, t[-1] + 0.5)

    for key, line in lines.items():
        y = [row[key] for row in rows]
        if key in ("degree", "cart_cm", "cart_force"):
            y = smooth_values(y, smoothing_samples)
        line.set_data(t, y)

    for ax in axes:
        ax.set_xlim(x_min, x_max)
        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)

    axes[-1].set_ylim(0.5, 7.5)

    latest = rows[-1]
    mode = int(latest["mode"])
    status_text.set_text(
        f"mode={mode} {MODE_LABELS.get(mode, '?'):<7} "
        f"angle={latest['degree']:+7.2f} deg  "
        f"cart={latest['cart_cm']:+7.2f} cm  "
        f"force={latest['cart_force']:+8.2f} N  "
        f"external={latest['external_force']:+6.2f} N"
    )
    fig.canvas.draw_idle()
    fig.canvas.flush_events()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Gazebo kiri + grafik live kanan tanpa memakai main.py GUI."
    )
    parser.add_argument(
        "--workspace",
        choices=sorted(WORKSPACES.keys()),
        default="lqr",
        help="Workspace yang dijalankan.",
    )
    parser.add_argument(
        "--no-auto",
        dest="auto",
        action="store_false",
        help="Jangan otomatis homing, swing-up, dan balance.",
    )
    parser.set_defaults(auto=True)
    parser.add_argument("--headless", action="store_true", help="Uji tanpa UI Gazebo.")
    parser.add_argument("--duration", type=float, default=0.0, help="Stop otomatis setelah N detik.")
    parser.add_argument("--history", type=float, default=35.0, help="Rentang waktu grafik.")
    parser.add_argument("--sample-period", type=float, default=0.02)
    parser.add_argument("--plot-hz", type=float, default=30.0)
    parser.add_argument(
        "--plot-smoothing-samples",
        type=int,
        default=3,
        help="Moving-average ringan untuk grafik saja; status dan data ROS tetap raw.",
    )
    parser.add_argument("--serial-timeout", type=float, default=45.0)
    parser.add_argument("--state-timeout", type=float, default=45.0)
    parser.add_argument(
        "--ros-domain-id",
        default="auto",
        help="ROS_DOMAIN_ID untuk run ini. Default auto agar tidak bercampur dengan launch lama.",
    )
    parser.add_argument("--homing-timeout", type=float, default=8.0)
    parser.add_argument("--auto-timeout", type=float, default=40.0)
    parser.add_argument("--min-swing-before-balance", type=float, default=4.0)
    parser.add_argument("--manual-balance-deg", type=float, default=10.0)
    parser.add_argument(
        "--force-balance-after",
        type=float,
        default=0.0,
        help="Kirim perintah BALANCE paksa setelah N detik; 0 berarti nonaktif dan bridge menunggu capture yang siap.",
    )
    parser.add_argument(
        "--external-impulse-force",
        type=float,
        default=None,
        help="Besar gaya eksternal impulse. Default dipakai saat tombol X ditekan; nilai negatif mendorong arah sebaliknya.",
    )
    parser.add_argument(
        "--external-impulse-duration",
        type=float,
        default=0.20,
        help="Lama gaya impulse aktif sebelum otomatis dikembalikan ke 0 N.",
    )
    parser.add_argument(
        "--external-trigger",
        choices=("x", "balance"),
        default="x",
        help=(
            "Kapan --external-impulse-force dikirim: 'x' hanya saat tombol "
            "X ditekan, atau 'balance' setelah BALANCE stabil."
        ),
    )
    parser.add_argument(
        "--external-visual-hold",
        type=float,
        default=10.0,
        help="Opsi kompatibilitas lama; visual gaya eksternal di Gazebo dinonaktifkan.",
    )
    parser.add_argument(
        "--external-impulse-delay",
        type=float,
        default=1.0,
        help="Jeda setelah BALANCE stabil sebelum impulse pertama dikirim.",
    )
    parser.add_argument(
        "--external-recovery-timeout",
        type=float,
        default=5.0,
        help="Waktu tunggu pendulum kembali stabil setelah impulse.",
    )
    parser.add_argument("--external-stable-time", type=float, default=1.0)
    parser.add_argument("--external-stable-timeout", type=float, default=20.0)
    parser.add_argument("--external-stable-angle-deg", type=float, default=2.0)
    parser.add_argument("--external-stable-rate-rad-s", type=float, default=0.35)
    parser.add_argument(
        "--external-stable-cart-cm",
        type=float,
        default=3.0,
        help="Cart harus dekat tengah sebelum balance dianggap stabil untuk uji impulse.",
    )
    parser.add_argument("--external-fail-angle-deg", type=float, default=18.0)
    parser.add_argument("--external-rail-fail-cm", type=float, default=38.0)
    parser.add_argument(
        "--find-max-external",
        action="store_true",
        help="Cari gaya impulse maksimum yang masih bisa dipertahankan saat BALANCE.",
    )
    parser.add_argument(
        "--external-max-forces",
        type=parse_float_list_arg,
        default=parse_float_list_arg(
            "0.5,1.0,1.5,2.0,2.5,3.0,4.0,5.0,6.0,8.0,10.0,12.0,15.0,20.0,25.0,30.0"
        ),
        help="Daftar magnitudo gaya untuk --find-max-external, dipisah koma.",
    )
    parser.add_argument(
        "--external-directions",
        default="positive,negative",
        help="Arah uji maksimum: positive, negative, atau keduanya dipisah koma.",
    )
    parser.add_argument(
        "--exit-after-external",
        action="store_true",
        help="Keluar otomatis setelah impulse atau sweep gaya eksternal selesai.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    args.ros_domain_id = resolve_ros_domain_id(args.ros_domain_id)
    os.environ["ROS_DOMAIN_ID"] = args.ros_domain_id
    print(f"[ROS] ROS_DOMAIN_ID={args.ros_domain_id}")
    max_points = max(200, int(args.history / max(args.sample_period, 1e-3)) + 200)

    rclpy.init(args=None)
    monitor = PlotMonitor(args.sample_period, max_points)
    spin_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    spin_thread.start()

    runner = DashboardRunner(args, monitor)
    fig = None
    try:
        fig, axes, lines, status_text = create_plot(runner.screen_w, runner.screen_h)
        tile_until = time.monotonic() + 20.0

        def run_key_action(action):
            try:
                action()
            except Exception as exc:
                print(f"[KEY] Gagal menjalankan tombol manual: {exc}")

        def on_key_press(event):
            key = (event.key or "").lower()
            actions = {
                "w": runner.homing,
                "a": runner.swing_up,
                "s": runner.balance,
                "d": runner.finish,
                "x": runner.external_impulse_now,
                "z": runner.auto_swing_balance,
            }
            action = actions.get(key)
            if action is None:
                return
            threading.Thread(target=run_key_action, args=(action,), daemon=True).start()

        def pump_plot():
            update_plot(
                fig,
                axes,
                lines,
                status_text,
                monitor.snapshot(),
                args.history,
                args.plot_smoothing_samples,
            )
            if time.monotonic() < tile_until:
                tile_dashboard_windows(runner.screen_w, runner.screen_h)

        runner.ui_pump = pump_plot
        fig.canvas.mpl_connect("key_press_event", on_key_press)
        pump_plot()
        runner.start()
        start = time.monotonic()
        period = 1.0 / max(1.0, args.plot_hz)

        while plt.fignum_exists(fig.number):
            update_plot(
                fig,
                axes,
                lines,
                status_text,
                monitor.snapshot(),
                args.history,
                args.plot_smoothing_samples,
            )
            if (
                args.exit_after_external
                and runner.external_test_requested()
                and runner.external_done.is_set()
            ):
                break
            if args.duration > 0.0 and time.monotonic() - start >= args.duration:
                break
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[STOP] Dihentikan oleh user.")
    finally:
        runner.close()
        try:
            monitor.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        if fig is not None:
            try:
                plt.close(fig)
            except Exception:
                pass


if __name__ == "__main__":
    main()
