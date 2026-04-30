# Linear Inverted Pendulum PID Sim

Workspace ini adalah turunan dari `pendulum_real_ws`, tetapi balance controller
dibuat khusus memakai PID. Model fisik, motor, swing-up, capture, dan assist
simulasi tetap diambil dari workspace real karena baseline itu sudah bisa
berdiri tegak lebih stabil.

Perbedaan utama:

- Workspace baru: `/home/ammar/Documents/Pendulum/pendulum_pid_ws`
- Package baru: `linear_inverted_pendulum_pid_sim`
- Launch baru: `pid_sim.launch.py`
- Serial baru: `/tmp/pendulum_pid_serial`
- Balance tidak memakai LQR dan tidak memakai full-state feedback Manual Book.
  Jalur balance hanya memakai PID sudut pendulum ditambah PD kecil untuk
  menjaga posisi cart.

## Build

```bash
cd /home/ammar/Documents/Pendulum/pendulum_pid_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Jalankan Gazebo

Dengan GUI Gazebo:

```bash
ros2 launch linear_inverted_pendulum_pid_sim pid_sim.launch.py
```

Headless:

```bash
ros2 launch linear_inverted_pendulum_pid_sim pid_sim.launch.py gz_args:="-r -s empty.sdf"
```

Launch ini membuat pseudo serial:

```bash
/tmp/pendulum_pid_serial
```

## Hubungkan ke GUI Python lama

Terminal baru:

```bash
cd /home/ammar/Documents/Pendulum
PENDULUM_PORT=/tmp/pendulum_pid_serial PENDULUM_SIM=1 PENDULUM_NO_JOYSTICK=1 python3 main.py
```

`PENDULUM_SIM=1` tetap wajib supaya GUI memakai konfigurasi simulasi, bukan
asumsi port hardware. `main.py` juga sudah mengenali `/tmp/pendulum_pid_serial`
sebagai port real-style, sehingga default GUI memakai gain PID baseline:
`K_TH=10.0`, `K_TH_D=3.0`, `K_X=2.4`, `K_X_D=3.4`, `K_X_INT=0.08`.

Alur pemakaian:

1. Klik `Apply Gains`.
2. Klik `START`.
3. Klik `Y` untuk homing ke tengah.
4. Klik `X` untuk swing-up.
5. Mode akan masuk balance otomatis kalau pendulum sudah dekat tegak.
6. Tombol `A` hanya untuk memaksa balance saat pendulum sudah dekat tegak.
7. Klik `B` untuk finish/stop.

## Topic ROS 2 penting

- `/joint_states`: posisi dan kecepatan `cart_slider` serta `pendulum_hinge`.
- `/pendulum/cart_velocity_cmd`: command kecepatan cart dari bridge PID.
- `/pendulum/cart_force_cmd`: gaya cart yang dikirim ke Gazebo.
- `/pendulum/sim_state`: `[degree, cmX, setspeed, energy, theta_dot_rad, theta_rad, x_center_cm, mode]`.
- `/pendulum/hinge_assist_force_cmd`: assist kecil khusus simulasi saat catch/balance.

Assist engsel default tetap aktif agar demo simulasi bisa berdiri tegak seperti
baseline `pendulum_real_ws`. Kalau ingin uji cart-only, jalankan:

```bash
ros2 launch linear_inverted_pendulum_pid_sim pid_sim.launch.py balance_assist_enabled:=false
```

## Metode kontrol

### 1. Homing

Tombol `Y` memakai PD sederhana untuk membawa cart ke tengah rail:

```python
error = self.x_center_m - self.cart_x_m
command = 2.8 * error - 0.35 * self.cart_v_mps
```

### 2. Swing-up energi

Tombol `X` masih memakai metode energi dari baseline `pendulum_real_ws`.
Controller menghitung energi pendulum, lalu memberi gaya cart untuk menaikkan
energi sampai pendulum masuk daerah atas:

```python
energy = self._energy(theta_top)
target_energy = 2.0 * self.pendulum_mass * 9.81 * self.pendulum_com
phase = self.pendulum_vel_radps * math.cos(theta_top)
energy_deficit = target_energy - energy

force = 52.0 * self.swing_gain * energy_deficit * phase
force -= 34.0 * self.swing_centering_gain * self.cart_x_m
force -= 14.0 * self.swing_damping_gain * self.cart_v_mps
```

Swing-up tidak langsung masuk balance. Capture baru aktif kalau sudut, kecepatan
sudut, posisi cart, dan kecepatan cart sudah masuk jendela aman:

```python
return (
    abs(theta_top) < self.balance_capture
    and abs(self.pendulum_vel_radps) < self.balance_capture_rate
    and abs(self.cart_x_m) < capture_cart_pos
    and abs(self.cart_v_mps) < self.balance_capture_cart_vel
)
```

### 3. Balance PID

Mode `BALANCE` memakai PID pada error sudut pendulum terhadap posisi tegak:

- `P`: error sudut `theta_top`.
- `I`: integral error sudut `theta_integral_rad_s`.
- `D`: kecepatan sudut `pendulum_vel_radps`.

Rumus inti di bridge:

```python
self.theta_integral_rad_s = clamp(
    self.theta_integral_rad_s + theta_top * dt,
    -0.18,
    0.18,
)

force = (
    -theta_force_gain * theta_top
    -theta_rate_force_gain * self.pendulum_vel_radps
    -theta_integral_force_gain * self.theta_integral_rad_s
)
```

Karena cart tetap harus berada di rail, PID sudut ditambah PD cart-centering:

```python
force += (
    -center_scale * centering_force_gain * x_error_m
    -center_scale * cart_damping_force_gain * self.cart_v_mps
)
```

Mapping gain dari GUI:

```text
K_TH     -> P sudut pendulum
K_TH_D   -> D sudut pendulum
K_X_INT  -> I sudut pendulum
K_X      -> P posisi cart
K_X_D    -> D kecepatan cart
```

Default gain awal:

```text
K_TH    = 10.0
K_TH_D  = 3.0
K_X     = 2.4
K_X_D   = 3.4
K_X_INT = 0.08
```

Nilai tersebut sengaja diambil dari tuning `pendulum_real_ws`, lalu makna
`K_X_INT` dipindahkan menjadi integral sudut supaya metode balance benar-benar
PID. Integral diberi anti-windup:

```python
theta_integral_rad_s = clamp(theta_integral_rad_s, -0.18, 0.18)
```

Jika pendulum keluar dari jendela balance, mode otomatis kembali ke swing-up:

```python
return (
    abs(theta_top) > self.balance_fallback
    or abs(self.cart_x_m) > (self.rail_limit + 0.02)
)
```

### 4. Model motor

Model motor tetap mengikuti baseline real-style:

```python
self.motor_pwm = self.motor_pwm_deadband + self.motor_pwm_per_cmps * abs(command_cmps)
alpha = 1.0 - math.exp(-dt / max(self.motor_time_constant_s, 1e-3))
self.motor_velocity_mps += alpha * (target_mps - self.motor_velocity_mps)
```

Parameter utama:

```text
motor_pwm_deadband      = 3212.0
motor_pwm_per_cmps      = 189.1
motor_time_constant_s   = 0.40
swing_force_limit_n     = 145.0
catch_force_limit_n     = 95.0
balance_force_limit_n   = 45.0
effort_limit_n          = 150.0
```

## Catatan metode

Workspace ini memang sengaja PID untuk balance. Homing masih PD, swing-up masih
energy-based, dan motor model tetap real-style. Assist engsel adalah bantuan
simulasi agar pendulum bisa berdiri tegak di Gazebo; itu bukan aktuator tambahan
pada alat asli.
