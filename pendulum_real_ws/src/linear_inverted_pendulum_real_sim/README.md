# Linear Inverted Pendulum Real Manual Sim

Workspace ini dibuat sebagai versi baru yang lebih dekat ke Manual Book LIP01.
Aktuator cart dibuat lebih mirip sistem asli dengan deadband PWM, konstanta waktu
motor, batas travel 78 cm, dan batas gaya yang tidak dibiarkan sampai ratusan N.
Untuk demo simulasi supaya bisa benar-benar tegak, bridge juga memakai assist
torsi kecil pada engsel saat fase catch/balance.

Data utama dari Manual Book yang dipakai:

- Base: 1000 mm x 350 mm.
- Travel rail: 78 cm.
- Pendulum shaft: diameter 8 mm, panjang 400 mm, massa 200 g.
- Motor PG45 24 V dengan mapping sweep `p1 = 189.1` dan deadband `p2 = 3212`.
- Konstanta waktu motor sekitar 0.4 s.
- Balance aktif hanya saat pendulum dekat tegak, sekitar `-10 deg` sampai `10 deg`.

## Build

```bash
cd /home/ammar/Documents/Pendulum/pendulum_real_ws
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
- `/pendulum/hinge_assist_force_cmd`: assist kecil khusus simulasi saat catch/balance.

Assist engsel bisa dimatikan dengan parameter `balance_assist_enabled:=false`
kalau ingin menguji mode cart-only murni.

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

force = 52.0 * self.swing_gain * energy_deficit * phase
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

Default workspace ini dibuat lebih ketat agar swing-up memakai gaya lebih kecil,
mengayun lebih banyak, dan balance baru aktif saat pendulum sudah lebih pelan di
dekat posisi tegak:

```python
self.declare_parameter("swing_gain", 2.70)
self.declare_parameter("swing_kick_mps", 0.82)
self.declare_parameter("swing_force_limit_n", 145.0)
self.declare_parameter("swing_min_top_passes_before_catch", 3)
self.declare_parameter("swing_min_energy_build_time_s", 5.0)
self.declare_parameter("swing_energy_ready_ratio", 0.84)
self.declare_parameter("swing_top_pass_angle_deg", 80.0)
self.declare_parameter("balance_capture_deg", 9.0)
self.declare_parameter("balance_capture_rate_rad_s", 1.0)
self.declare_parameter("balance_capture_cart_pos_m", 0.30)
self.declare_parameter("balance_capture_cart_vel_mps", 1.4)
self.declare_parameter("catch_region_deg", 95.0)
self.declare_parameter("catch_region_rate_rad_s", 14.0)
```

Dengan nilai ini, workspace tidak langsung menangkap pendulum pada ayunan
pertama. `SWING_UP` harus melewati area atas beberapa kali dan energinya harus
mencapai rasio minimum sebelum `catch` atau `BALANCE` boleh aktif. Jika gerak
pendulum sudah berada dekat atas dan tidak kembali melewati area bawah, gate
waktu minimal `5.0 s` tetap mencegah balance terlalu dini tanpa mengunci swing-up
selamanya. Jika tombol `A` ditekan saat swing-up belum siap, request balance
disimpan dulu dan baru dijalankan saat syarat energi/capture terpenuhi.

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

Default gain yang direkomendasikan untuk GUI pada workspace ini:

```text
K_TH    = 10.0
K_TH_D  = 3.0
K_X     = 2.4
K_X_D   = 3.4
K_X_INT = 0.08
```

Nilai ini memberi damping sudut dan damping cart lebih kuat dibanding default
awal, sementara centering dan integral posisi cart dibuat lebih kecil agar cart
tidak terlalu agresif mengejar titik tengah saat proses catch/balance.

Gaya cart sekarang dibatasi bertingkat supaya masih bisa dipertanggungjawabkan
sebagai simulasi aktuator real:

```text
swing_force_limit_n  = 145.0
catch_force_limit_n  = 95.0
balance_force_limit_n = 45.0
effort_limit_n       = 150.0
```

Kalau balance terlihat butuh gaya besar, penyebabnya biasanya balance masuk
terlalu awal: sudut masih jauh dari tegak, theta-dot masih tinggi, cart sudah
terlalu cepat, atau rail guard sedang menarik cart kembali dari ujung rel. Karena
itu capture dibuat lebih ketat, bukan menaikkan force lagi.

Untuk demo simulasi yang lebih stabil, assist engsel aktif saat pendulum sudah
mulai naik ke area atas. Ini bantuan khusus simulasi, bukan aktuator tambahan
pada alat asli:

```text
balance_assist_angle_deg        = 115.0
balance_assist_kp_nm_per_rad    = 3.4
balance_assist_kd_nm_per_rad_s  = 2.4
balance_assist_torque_limit_nm  = 4.5
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
self.declare_parameter("motor_velocity_servo_p", 55.0)
self.declare_parameter("motor_velocity_servo_d", 1.2)
self.declare_parameter("effort_limit_n", 150.0)
```

## Catatan

Versi ini tetap memakai dimensi manual book, tetapi assist engsel default aktif
agar simulasi bisa berdiri tegak dan cocok untuk demonstrasi. Untuk eksperimen
yang lebih ketat terhadap cart-only, jalankan dengan `balance_assist_enabled:=false`.
