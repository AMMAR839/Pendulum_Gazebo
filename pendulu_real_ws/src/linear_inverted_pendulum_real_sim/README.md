# Linear Inverted Pendulum Real Manual Sim

Workspace ini dibuat sebagai versi baru yang lebih dekat ke Manual Book LIP01.
Bedanya dengan `ros2_pendulum_ws` lama: model ini tidak memakai bantuan torsi
langsung pada engsel pendulum. Aktuator cart dibuat lebih mirip sistem asli
dengan deadband PWM, konstanta waktu motor, batas travel 78 cm, dan batas gaya
yang jauh lebih realistis.

Data utama dari Manual Book yang dipakai:

- Base: 1000 mm x 350 mm.
- Travel rail: 78 cm.
- Pendulum shaft: diameter 8 mm, panjang 400 mm, massa 200 g.
- Motor PG45 24 V dengan mapping sweep `p1 = 189.1` dan deadband `p2 = 3212`.
- Konstanta waktu motor sekitar 0.4 s.
- Balance aktif hanya saat pendulum dekat tegak, sekitar `-10 deg` sampai `10 deg`.

## Build

```bash
cd /home/ammar/Documents/Pendulum/pendulu_real_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Jalankan Gazebo

Dengan GUI Gazebo:

```bash
ros2 launch linear_inverted_pendulum_real_sim real_sim.launch.py
```

Headless:

```bash
ros2 launch linear_inverted_pendulum_real_sim real_sim.launch.py gz_args:="-r -s empty.sdf"
```

Launch ini membuat pseudo serial baru:

```bash
/tmp/pendulum_real_serial
```

## Hubungkan ke GUI Python lama

Terminal baru:

```bash
cd /home/ammar/Documents/Pendulum
PENDULUM_PORT=/tmp/pendulum_real_serial PENDULUM_SIM=1 PENDULUM_NO_JOYSTICK=1 python3 main.py
```

`PENDULUM_SIM=1` penting karena port baru ini bukan
`/tmp/pendulum_sim_serial`. Tanpa flag itu, `main.py` akan memakai gain default
untuk hardware asli yang tandanya berbeda dan terlalu besar untuk simulasi.

Alur pemakaian:

1. Klik `Apply Gains`.
2. Klik `START`.
3. Klik `Y` untuk homing ke tengah.
4. Klik `X` untuk swing-up.
5. Mode akan masuk balance otomatis kalau sudut, theta-dot, posisi cart, dan kecepatan cart sudah masuk jendela capture.
6. Tombol `A` hanya untuk memaksa balance saat pendulum sudah dekat tegak.
7. Klik `B` untuk finish/stop.

## Topic ROS 2 penting

- `/joint_states`: posisi dan kecepatan `cart_slider` serta `pendulum_hinge`.
- `/pendulum/cart_velocity_cmd`: command kecepatan cart dari controller internal.
- `/pendulum/cart_force_cmd`: gaya motor cart yang dikirim ke Gazebo.
- `/pendulum/sim_state`: `[degree, cmX, setspeed, energy, theta_dot_rad, theta_rad, x_center_cm, mode]`.

Tidak ada `/pendulum/hinge_assist_force_cmd` di workspace ini, karena versi ini
memang dibuat tanpa bantuan torsi langsung di pendulum.

## Metode kontrol

### Homing

Tombol `Y` memakai PD sederhana agar cart kembali ke tengah:

```python
error = self.x_center_m - self.cart_x_m
command = 2.8 * error - 0.35 * self.cart_v_mps
```

### Swing-up energi

Tombol `X` menghitung energi pendulum dan memberi command cart untuk menambah
energi sampai mendekati posisi atas:

```python
energy = self._energy(theta_top)
target_energy = 2.0 * self.pendulum_mass * 9.81 * self.pendulum_com
phase = self.pendulum_vel_radps * math.cos(theta_top)
energy_deficit = target_energy - energy

force = -52.0 * self.swing_gain * energy_deficit * phase
force -= 34.0 * self.swing_centering_gain * self.cart_x_m
force -= 14.0 * self.swing_damping_gain * self.cart_v_mps
```

### Capture balance

Swing-up pindah otomatis ke balance hanya kalau state sudah cukup aman:

```python
return (
    abs(theta_top) < self.balance_capture
    and abs(self.pendulum_vel_radps) < self.balance_capture_rate
    and abs(self.cart_x_m) < capture_cart_pos
    and abs(self.cart_v_mps) < self.balance_capture_cart_vel
)
```

Default workspace ini mengikuti manual:

```python
self.declare_parameter("balance_capture_deg", 10.0)
self.declare_parameter("balance_capture_rate_rad_s", 2.2)
self.declare_parameter("balance_capture_cart_pos_m", 0.22)
self.declare_parameter("balance_capture_cart_vel_mps", 0.80)
```

### Balance full-state feedback

Saat mode `BALANCE`, controller memakai feedback sudut, kecepatan sudut, posisi
cart, kecepatan cart, dan integral posisi cart. Ini bentuk praktis dari metode
full-state feedback di Manual Book:

```python
command = (
    self.gains["K_TH"] * theta_top
    + self.gains["K_TH_D"] * self.pendulum_vel_radps
    - self.gains["K_X"] * x_error_m
    - self.gains["K_X_D"] * self.cart_v_mps
    - self.gains["K_X_INT"] * self.x_integral_cm_s
)
```

### Model motor manual

Command controller tidak langsung menjadi gaya besar. Bridge lebih dulu
membuat model motor dengan deadband PWM dan konstanta waktu 0.4 s:

```python
self.motor_pwm = self.motor_pwm_deadband + self.motor_pwm_per_cmps * abs(command_cmps)
alpha = 1.0 - math.exp(-dt / max(self.motor_time_constant_s, 1e-3))
self.motor_velocity_mps += alpha * (target_mps - self.motor_velocity_mps)
```

Parameter default:

```python
self.declare_parameter("motor_pwm_deadband", 3212.0)
self.declare_parameter("motor_pwm_per_cmps", 189.1)
self.declare_parameter("motor_time_constant_s", 0.40)
self.declare_parameter("effort_limit_n", 60.0)
```

## Catatan

Versi ini memang bisa lebih susah tegak daripada workspace lama yang sudah
dibantu. Itu normal, karena sistem real/manual tidak mendapat torsi langsung di
engsel pendulum. Kalau ingin dibuat lebih mudah untuk demo visual, gunakan
workspace lama `ros2_pendulum_ws`; kalau ingin mendekati karakter manual,
gunakan `pendulu_real_ws` ini.
