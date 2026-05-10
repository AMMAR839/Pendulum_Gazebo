#!/usr/bin/env python3
import argparse
import math
import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from lib_com import make_packet, open_serial, send_external_force, send_reset


ROOT = Path(__file__).resolve().parent

BUTTON_A = 1 << 0
BUTTON_B = 1 << 1
BUTTON_X = 1 << 2
BUTTON_Y = 1 << 3

MODE_READY = 3
MODE_SWING_UP = 6
MODE_BALANCE = 7


@dataclass(frozen=True)
class WorkspaceConfig:
    key: str
    label: str
    workspace: Path
    package: str
    launch_file: str
    serial_link: str
    aliases: tuple[str, ...]


WORKSPACES = {
    "ros2": WorkspaceConfig(
        key="ros2",
        label="Pendulum ROS2 demo",
        workspace=ROOT / "ros2_pendulum_ws",
        package="linear_inverted_pendulum_sim",
        launch_file="sim.launch.py",
        serial_link="/tmp/pendulum_sim_command_serial",
        aliases=("ros2", "demo", "sim", "easier", "biasa"),
    ),
    "real": WorkspaceConfig(
        key="real",
        label="Pendulum real/manual-book",
        workspace=ROOT / "pendulum_real_ws",
        package="linear_inverted_pendulum_real_sim",
        launch_file="real_sim.launch.py",
        serial_link="/tmp/pendulum_real_command_serial",
        aliases=("real", "manual", "manual book", "pemdulum real", "pendulum real"),
    ),
    "pid": WorkspaceConfig(
        key="pid",
        label="Pendulum PID",
        workspace=ROOT / "pendulum_pid_ws",
        package="linear_inverted_pendulum_pid_sim",
        launch_file="pid_sim.launch.py",
        serial_link="/tmp/pendulum_pid_command_serial",
        aliases=("pid", "pendulum pid", "pemdulum pid"),
    ),
}


class StateMonitor(Node):
    def __init__(self):
        super().__init__("pendulum_command_assistant")
        self._lock = threading.Lock()
        self._latest_state = None
        self.create_subscription(Float64MultiArray, "/pendulum/sim_state", self._state_cb, 50)

    def _state_cb(self, msg):
        if len(msg.data) < 8:
            return
        with self._lock:
            self._latest_state = list(msg.data[:8])

    def clear(self):
        with self._lock:
            self._latest_state = None

    def state(self):
        with self._lock:
            if self._latest_state is None:
                return None
            return list(self._latest_state)

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


class PendulumCommandAssistant:
    def __init__(self, args):
        self.args = args
        self.monitor = StateMonitor()
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.monitor,), daemon=True)
        self.spin_thread.start()

        self.current_ws = None
        self.launch_proc = None
        self.launch_pgid = None
        self.launch_log = None
        self.serial = None
        self.seq = 0

    def close(self):
        self.stop_workspace()
        try:
            self.monitor.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        self.spin_thread.join(timeout=2.0)

    def launch_workspace(self, key):
        cfg = WORKSPACES[key]
        if self.current_ws == cfg and self.launch_proc and self.launch_proc.poll() is None:
            print(f"[OK] {cfg.label} sudah jalan.")
            return

        self.stop_workspace()
        self.monitor.clear()
        self.seq = 0

        gz_args = "-r -s empty.sdf" if self.args.headless else None
        log_dir = ROOT / "logs"
        log_dir.mkdir(exist_ok=True)
        log_path = log_dir / f"pendulum_command_assistant_{cfg.key}.log"
        self.launch_log = log_path.open("w")

        cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {cfg.workspace}/install/setup.bash && "
            f"ros2 launch {cfg.package} {cfg.launch_file} "
            + (f"gz_args:='{gz_args}' " if gz_args is not None else "")
            + f"serial_link:={cfg.serial_link}"
        )
        print(f"[RUN] Menjalankan {cfg.label} ...")
        self.launch_proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            cwd=str(cfg.workspace),
            stdin=subprocess.DEVNULL,
            stdout=self.launch_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.launch_pgid = os.getpgid(self.launch_proc.pid)
        self.current_ws = cfg
        self._wait_for_serial(cfg.serial_link, self.args.serial_timeout)
        self._wait_for_state(self.args.state_timeout)
        self.serial = open_serial(cfg.serial_link, 115200, timeout=0.1)
        self.set_external_force(0.0, quiet=True)
        print(f"[OK] {cfg.label} siap. Log: {log_path}")

    def stop_workspace(self):
        if self.serial is not None:
            try:
                self.set_external_force(0.0, quiet=True)
                self.serial.close()
            except Exception:
                pass
            self.serial = None

        if self.launch_proc is not None and self.launch_proc.poll() is None:
            print("[STOP] Menghentikan simulasi yang sedang jalan ...")
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
        self.current_ws = None
        if self.launch_log is not None:
            try:
                self.launch_log.close()
            except Exception:
                pass
            self.launch_log = None

    def _wait_for_serial(self, serial_link, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.launch_proc and self.launch_proc.poll() is not None:
                raise RuntimeError("Launch ROS/Gazebo berhenti sebelum serial siap.")
            if os.path.exists(serial_link):
                return
            time.sleep(0.1)
        raise RuntimeError(f"Serial simulasi belum muncul: {serial_link}")

    def _wait_for_state(self, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.launch_proc and self.launch_proc.poll() is not None:
                raise RuntimeError("Launch ROS/Gazebo berhenti sebelum /pendulum/sim_state siap.")
            if self.monitor.state() is not None:
                return
            time.sleep(0.1)
        raise RuntimeError("Belum ada data /pendulum/sim_state dari simulasi.")

    def _require_serial(self):
        if self.serial is None or self.current_ws is None:
            raise RuntimeError("Jalankan workspace dulu, misalnya: pendulum real")

    def send_button(self, button, name):
        self._require_serial()
        self.serial.write(make_packet(self.seq & 0xFF, 0, 0, 0, 0, button))
        self.seq += 1
        time.sleep(0.12)
        self.serial.write(make_packet(self.seq & 0xFF, 0, 0, 0, 0, 0))
        self.seq += 1
        time.sleep(0.12)
        print(f"[TX] Tombol {name}")

    def homing(self):
        self.send_button(BUTTON_Y, "Y / HOMING")
        deadline = time.monotonic() + self.args.homing_timeout
        while time.monotonic() < deadline:
            if self.monitor.mode() == MODE_READY:
                print("[OK] Pendulum siap swing-up.")
                return
            time.sleep(0.1)
        print("[WARN] Homing dikirim, tapi mode READY belum terlihat.")

    def swing_up(self):
        self.send_button(BUTTON_X, "X / SWING-UP")

    def balance_now(self):
        self.send_button(BUTTON_A, "A / BALANCE")

    def finish(self):
        self.send_button(BUTTON_B, "B / FINISH")

    def reset(self):
        self._require_serial()
        send_reset(self.serial, self.seq & 0xFF)
        self.seq += 1
        self.monitor.clear()
        print("[TX] Reset simulasi")

    def set_external_force(self, force_n, quiet=False):
        self._require_serial()
        send_external_force(self.serial, float(force_n), self.seq & 0xFF)
        self.serial.flush()
        self.seq += 1
        if not quiet:
            if abs(force_n) < 1e-9:
                print("[OK] Gaya eksternal dimatikan.")
            else:
                print(f"[OK] Gaya eksternal aktif: {force_n:+.3f} N")

    def impulse_disturbance(self):
        self._require_serial()
        force_n = (
            self.args.impulse_force
            if self.args.impulse_force is not None
            else self.args.external_force
        )
        duration_s = max(0.01, self.args.impulse_duration)
        print(f"[IMPULSE] Gaya {force_n:+.3f} N selama {duration_s:.3f} s")
        self.set_external_force(force_n, quiet=True)
        time.sleep(duration_s)
        self.set_external_force(0.0, quiet=True)
        time.sleep(0.08)
        print("[OK] Impulse selesai; gaya eksternal kembali 0 N.")

    def auto_swing_balance(self):
        self._require_serial()
        if self.monitor.state() is None:
            self._wait_for_state(self.args.state_timeout)

        if self.monitor.mode() != MODE_READY:
            self.homing()

        self.swing_up()
        start = time.monotonic()
        balance_requested = False

        print("[AUTO] Menunggu pendulum dekat tegak, lalu kirim A / BALANCE ...")
        while time.monotonic() - start < self.args.auto_timeout:
            mode = self.monitor.mode()
            degree = abs(self.monitor.degree())
            elapsed = time.monotonic() - start

            if mode == MODE_BALANCE:
                print("[OK] Mode BALANCE sudah aktif.")
                return

            if (
                not balance_requested
                and elapsed >= self.args.min_swing_before_balance
                and degree <= self.args.manual_balance_deg
            ):
                self.balance_now()
                balance_requested = True

            if not balance_requested and elapsed >= self.args.force_balance_after:
                self.balance_now()
                balance_requested = True

            time.sleep(0.05)

        print("[WARN] Auto swing-balance selesai timeout; cek visual dan log simulasi.")

    def print_status(self):
        state = self.monitor.state()
        if self.current_ws is None:
            print("[STATUS] Belum ada workspace yang jalan.")
            return
        if state is None:
            print(f"[STATUS] {self.current_ws.label}: belum ada data state.")
            return
        mode = int(round(state[7]))
        print(
            f"[STATUS] {self.current_ws.label}: mode={mode} "
            f"degree={state[0]:+.2f} deg cart={state[1]:+.2f} cm"
        )


def workspace_from_text(text):
    for key, cfg in WORKSPACES.items():
        if any(alias in text for alias in cfg.aliases):
            return key
    return None


def is_external_force_off(text):
    return any(word in text for word in ("mati", "matikan", "off", "hapus", "nol", "stop gaya"))


def is_impulse_command(text):
    return (
        "impulse" in text
        or "impuls" in text
        or "dorong sekali" in text
        or "gangguan sesaat" in text
        or "gaya sesaat" in text
        or ("sekali" in text and any(word in text for word in ("gaya", "dorong", "gangguan")))
    )


def handle_command(assistant, raw_text):
    text = raw_text.strip().lower()
    if not text:
        return True

    if any(word in text for word in ("keluar", "exit", "quit")):
        return False

    workspace_key = workspace_from_text(text)
    if workspace_key is not None:
        assistant.launch_workspace(workspace_key)

    if "status" in text:
        assistant.print_status()

    if "reset" in text:
        assistant.reset()

    if any(word in text for word in ("homing", "home", "siapkan")):
        assistant.homing()

    wants_external = "gaya eksternal" in text or "external" in text or "gangguan" in text
    wants_impulse = is_impulse_command(text)
    if wants_impulse:
        assistant.impulse_disturbance()
    elif wants_external:
        force = 0.0 if is_external_force_off(text) else assistant.args.external_force
        assistant.set_external_force(force)

    wants_auto = (
        ("swing" in text and any(word in text for word in ("tegak", "balance", "seimbang")))
        or "auto balance" in text
        or "otomatis tegak" in text
    )
    if wants_auto:
        assistant.auto_swing_balance()
    elif "swing" in text or "ayun" in text:
        assistant.swing_up()
    elif any(word in text for word in ("tegak", "balance", "seimbang")):
        assistant.balance_now()

    if any(word in text for word in ("finish", "selesai", "stop pendulum")):
        assistant.finish()

    if workspace_key is None and not any(
        token in text
        for token in (
            "status",
            "reset",
            "homing",
            "home",
            "siapkan",
            "gaya eksternal",
            "external",
            "gangguan",
            "impulse",
            "impuls",
            "dorong sekali",
            "swing",
            "ayun",
            "tegak",
            "balance",
            "seimbang",
            "finish",
            "selesai",
            "stop pendulum",
        )
    ):
        print("[INFO] Perintah belum dikenali.")

    return True


def print_help():
    print(
        "Perintah contoh:\n"
        "  pendulum real\n"
        "  pendulum pid\n"
        "  pendulum ros2\n"
        "  homing\n"
        "  swing up\n"
        "  swing up sampai tegak\n"
        "  balance / tegak\n"
        "  impulse disturbance / dorong sekali\n"
        "  berikan gaya eksternal\n"
        "  matikan gaya eksternal\n"
        "  status\n"
        "  keluar"
    )


def text_loop(assistant):
    print_help()
    while True:
        try:
            raw = input("\npendulum> ")
        except EOFError:
            break
        if not handle_command(assistant, raw):
            break


def voice_loop(assistant, args):
    try:
        import speech_recognition as sr
    except ImportError as exc:
        raise RuntimeError(
            "Mode suara butuh library tambahan: pip install SpeechRecognition pyaudio"
        ) from exc

    recognizer = sr.Recognizer()
    print_help()
    print(f"[VOICE] Mendengarkan perintah suara bahasa {args.language}. Ucapkan 'keluar' untuk stop.")

    with sr.Microphone() as source:
        recognizer.adjust_for_ambient_noise(source, duration=0.8)
        while True:
            try:
                audio = recognizer.listen(source, phrase_time_limit=args.phrase_time_limit)
                raw = recognizer.recognize_google(audio, language=args.language)
            except sr.UnknownValueError:
                print("[VOICE] Tidak terdengar jelas.")
                continue
            except sr.RequestError as exc:
                print(f"[VOICE] STT gagal: {exc}")
                continue

            print(f"[VOICE] {raw}")
            if not handle_command(assistant, raw):
                break


def parse_args():
    parser = argparse.ArgumentParser(
        description="Command assistant untuk menjalankan dan mengendalikan tiga workspace pendulum."
    )
    parser.add_argument(
        "--external-force",
        type=float,
        default=2.05,
        help="Gaya eksternal default pada ujung pendulum dalam Newton.",
    )
    parser.add_argument(
        "--impulse-force",
        type=float,
        default=None,
        help="Gaya impulse dalam Newton. Jika kosong, pakai --external-force.",
    )
    parser.add_argument(
        "--impulse-duration",
        type=float,
        default=0.25,
        help="Durasi impulse dalam detik sebelum gaya otomatis menjadi 0.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Jalankan Gazebo tanpa GUI untuk uji otomatis.",
    )
    parser.add_argument(
        "--voice",
        action="store_true",
        help="Pakai input suara. Membutuhkan SpeechRecognition dan PyAudio.",
    )
    parser.add_argument(
        "--language",
        default="id-ID",
        help="Kode bahasa untuk speech-to-text, default id-ID.",
    )
    parser.add_argument("--phrase-time-limit", type=float, default=4.0)
    parser.add_argument("--serial-timeout", type=float, default=35.0)
    parser.add_argument("--state-timeout", type=float, default=35.0)
    parser.add_argument("--homing-timeout", type=float, default=8.0)
    parser.add_argument("--auto-timeout", type=float, default=35.0)
    parser.add_argument("--min-swing-before-balance", type=float, default=1.8)
    parser.add_argument("--manual-balance-deg", type=float, default=18.0)
    parser.add_argument("--force-balance-after", type=float, default=10.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init(args=None)
    assistant = PendulumCommandAssistant(args)

    try:
        if args.voice:
            voice_loop(assistant, args)
        else:
            text_loop(assistant)
    except KeyboardInterrupt:
        print("\n[STOP] Dihentikan oleh user.")
    finally:
        assistant.close()


if __name__ == "__main__":
    main()
