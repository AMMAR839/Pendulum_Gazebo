# Linear Inverted Pendulum Gazebo Sim

Paket ini dibuat untuk ROS 2 Jazzy dan Gazebo Harmonic 8. Model mengikuti data dari manual LIP01:

- Base 1000 mm x 350 mm.
- Rail linear sekitar 900 mm dengan travel kontrol 78 cm.
- Cart bergerak pada joint prismatic `cart_slider`.
- Pendulum shaft diameter 8 mm, panjang 400 mm, massa 200 g.
- Encoder pendulum dimodelkan dari joint `pendulum_hinge`.

## Build

```bash
cd /home/ammar/Documents/Pendulum/ros2_pendulum_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Jalankan Gazebo

```bash
ros2 launch linear_inverted_pendulum_sim sim.launch.py
```

Setiap launch otomatis memakai `GZ_PARTITION` unik, supaya data `/clock`
dan joint-state tidak tercampur dengan proses Gazebo lama.

Untuk headless:

```bash
ros2 launch linear_inverted_pendulum_sim sim.launch.py gz_args:="-r -s empty.sdf"
```

Untuk test force langsung tanpa controller serial:

```bash
ros2 launch linear_inverted_pendulum_sim sim.launch.py enable_serial_bridge:=false
```

Launch ini membuat pseudo serial di:

```bash
/tmp/pendulum_sim_serial
```

## Hubungkan ke GUI Python lama

Terminal baru:

```bash
cd /home/ammar/Documents/Pendulum
PENDULUM_PORT=/tmp/pendulum_sim_serial PENDULUM_NO_JOYSTICK=1 python3 main.py
```

Saat `PENDULUM_PORT=/tmp/pendulum_sim_serial`, `main.py` otomatis memakai gain default yang cocok untuk simulasi balance.

Alur pemakaian:

1. Klik `Apply Gains`; bridge akan mengirim ACK seperti STM32.
2. Klik `START` pada GUI.
3. Klik tombol `Y` untuk homing ke tengah.
4. Klik `X` untuk swing up, yaitu membangun energi ayunan sampai batang dekat atas.
5. Klik `A` hanya untuk masuk/paksa mode balance saat batang sudah dekat tegak.
6. Klik `B` untuk finish/stop.

Jika memakai joystick fisik, hilangkan `PENDULUM_NO_JOYSTICK=1`.

Jika Gazebo pernah ditutup paksa dan simulasi terlihat aneh, bersihkan proses
headless lama sebelum launch ulang:

```bash
pkill -f '^gz sim -r -s empty\.sdf$'
```

## Topic ROS 2 penting

- `/joint_states`: posisi dan kecepatan `cart_slider` serta `pendulum_hinge`.
- `/pendulum/cart_velocity_cmd`: setpoint kecepatan cart dari controller internal dalam m/s, bukan input langsung dari Gazebo.
- `/pendulum/cart_force_cmd`: gaya yang dikirim ke joint cart di Gazebo.
- `/pendulum/hinge_assist_force_cmd`: torsi bantuan simulasi ke engsel pendulum saat dekat tegak.
- `/pendulum/sim_state`: data ringkas `[degree, cmX, setspeed, energy, theta_dot_rad, theta_rad, x_center_cm, mode]`.

Mode mengikuti label GUI lama: `1=WAITING`, `2=HOMING`, `3=READY`, `4=SINUS`, `5=FINISH`, `6=SWING UP`, `7=BALANCING`.

Catatan sudut untuk simulasi ini: `degree` adalah sudut terhadap posisi atas,
jadi `0 deg` berarti pendulum tegak di atas, sedangkan sekitar `+/-180 deg`
berarti pendulum berada di bawah.

Mode `SINUS` adalah mode test osilasi cart. Mode `SWING UP` memakai kontrol
energi, bukan sinus fixed, meskipun hasil gerak cart tetap terlihat berayun.

## Kenapa pendulum belum bisa tegak stabil

Masalah utamanya bukan hanya visual Gazebo, tetapi transisi kontrol dari
`SWING UP` ke `BALANCING` masih sensitif. Swing-up sudah bisa memberi
energi ke pendulum, tetapi balance hanya boleh mengambil alih kalau kondisi
capture terpenuhi:

- Sudut harus dekat atas: `abs(theta_top) < 26 deg`.
- Kecepatan sudut tidak boleh terlalu besar: `abs(theta_dot) < 3.5 rad/s`.
- Cart harus masih cukup dekat tengah: `abs(x) < 0.24 m`.
- Kecepatan cart harus tidak terlalu besar: `abs(x_dot) < 1.30 m/s`.

Kalau batang lewat dekat atas tetapi masih terlalu cepat, mode tidak masuk
capture balance. Kalau mode balance dipaksa terlalu awal dengan tombol `A`,
controller langsung bekerja pada kondisi yang belum bisa ditahan, lalu jatuh
dan otomatis kembali ke `SWING UP` saat sudut sudah melewati batas fallback.

Untuk membuat simulasi lebih mudah tegak, default sekarang dibuat lebih
permisif: area capture diperlebar, batas gaya diperbesar, LQR catch masuk lebih
awal, mode `BALANCE` default memakai LQR force controller, dan ada torsi kecil
`balance_assist` di engsel saat pendulum sudah dekat tegak. Gain GUI tetap
diterima dari `main.py`, tetapi bukan controller utama selama
`balance_use_lqr=True`.

Memperbesar radius visual batang tidak banyak membantu di Gazebo karena tidak
ada model aerodinamika. Kalau ingin membuat simulasi lebih mudah secara fisik,
parameter yang lebih berpengaruh adalah panjang pendulum, massa, damping
engsel, batas gaya cart, tuning controller, atau bantuan torsi engsel khusus
simulasi. Kalau ingin kembali ke model pendulum pasif, set
`balance_assist_enabled:=false`.

Parameter tuning utama ada di `sim_serial_bridge.py`:

```python
self.declare_parameter("balance_capture_deg", 26.0)
self.declare_parameter("balance_capture_rate_rad_s", 3.5)
self.declare_parameter("balance_capture_cart_pos_m", 0.24)
self.declare_parameter("balance_capture_cart_vel_mps", 1.30)
self.declare_parameter("lqr_catch_deg", 130.0)
self.declare_parameter("lqr_catch_rate_rad_s", 25.0)
self.declare_parameter("catch_force_limit_n", 280.0)
self.declare_parameter("balance_force_limit_n", 340.0)
self.declare_parameter("balance_use_lqr", True)
self.declare_parameter("balance_assist_enabled", True)
self.declare_parameter("balance_assist_angle_deg", 55.0)
self.declare_parameter("balance_assist_kp_nm_per_rad", 1.8)
self.declare_parameter("balance_assist_kd_nm_per_rad_s", 0.38)
self.declare_parameter("balance_assist_torque_limit_nm", 2.4)
self.declare_parameter("effort_limit_n", 360.0)
```

## Metode kontrol yang digunakan

### 1. Homing

Tombol `Y` membawa cart ke tengah sebelum swing-up. Kontrolnya PD sederhana:

```python
if self.mode == MODE_HOMING:
    error = self.x_center_m - self.cart_x_m
    command = 2.8 * error - 0.35 * self.cart_v_mps
    if abs(error) < 0.006 and abs(self.cart_v_mps) < 0.03:
        self._set_mode_locked(MODE_READY)
        return 0.0, None
    return command, None
```

### 2. Swing-up berbasis energi

Tombol `X` masuk ke mode `MODE_SWING_UP`. Metode ini menghitung energi aktual
pendulum lalu memberi gaya ke cart berdasarkan selisih energi menuju posisi
atas:

```python
energy = self._energy(theta_top)
target_energy = 2.0 * self.pendulum_mass * 9.81 * self.pendulum_com
phase = self.pendulum_vel_radps * math.cos(theta_top)
energy_deficit = target_energy - energy

force = -52.0 * self.swing_gain * energy_deficit * phase
force -= 34.0 * self.swing_centering_gain * self.cart_x_m
force -= 14.0 * self.swing_damping_gain * self.cart_v_mps
```

Target energinya dihitung dari model pendulum:

```python
def _energy(self, theta_top):
    potential = self.pendulum_mass * 9.81 * self.pendulum_com * (
        1.0 + math.cos(theta_top)
    )
    kinetic = (
        0.5
        * self.pendulum_mass
        * self.pendulum_com
        * self.pendulum_com
        * self.pendulum_vel_radps
        * self.pendulum_vel_radps
    )
    return potential + kinetic
```

Saat pendulum masih tepat di bawah dan belum bergerak, hukum energi belum
punya arah yang jelas. Karena itu ada kick awal kecil:

```python
if abs(theta_deg) > 160.0 and abs(self.pendulum_vel_radps) < 0.60:
    if abs(self.cart_x_m) > 0.04:
        kick_direction = -math.copysign(1.0, self.cart_x_m)
    else:
        elapsed = time.monotonic() - self.swing_state_started
        kick_direction = 1.0 if int(elapsed / 0.45) % 2 == 0 else -1.0
    force += kick_direction * max(35.0, 70.0 * self.swing_kick_mps)
```

### 3. LQR catch dekat posisi atas

Sebelum masuk balance, controller memakai LQR catch untuk mengurangi kecepatan
ayunan dan membawa state masuk ke capture window:

```python
if self._in_lqr_catch_region_locked(theta_top):
    force = self._state_feedback_force_locked(
        self.catch_lqr_gain,
        theta_top,
        dt,
        self.catch_force_limit_n,
    )
    return self._force_to_command_hint(force), force
```

Gain LQR dihitung dari model linear inverted pendulum:

```python
q_matrix = np.diag([q_x, q_x_dot, q_theta, q_theta_dot])
r_matrix = np.array([[max(r_value, 1e-4)]], dtype=float)
p_matrix = solve_continuous_are(a_matrix, b_matrix, q_matrix, r_matrix)
k_matrix = np.linalg.inv(r_matrix) @ b_matrix.T @ p_matrix
```

Force LQR menggunakan state `[x, x_dot, theta, theta_dot]`:

```python
state = (
    x_error_m,
    self.cart_v_mps,
    theta_top,
    self.pendulum_vel_radps,
)
force = -sum(
    gain_value * state_value
    for gain_value, state_value in zip(gain, state)
)
```

### 4. Capture condition ke balance

Mode `BALANCING` hanya diambil otomatis kalau pendulum sudah cukup dekat atas,
bergerak pelan, dan cart belum keluar jauh dari tengah:

```python
def _ready_for_balance_locked(self, theta_top):
    capture_cart_pos = min(
        self.balance_capture_cart_pos,
        max(0.05, self.rail_limit - 0.004),
    )
    return (
        abs(theta_top) < self.balance_capture
        and abs(self.pendulum_vel_radps) < self.balance_capture_rate
        and abs(self.cart_x_m) < capture_cart_pos
        and abs(self.cart_v_mps) < self.balance_capture_cart_vel
    )
```

### 5. Balance LQR

Saat mode `BALANCE`, controller default sekarang memakai LQR yang sama dengan
state feedback `[x, x_dot, theta, theta_dot]`. Saat mode balance dimasuki,
`x_center_m` diset ke posisi cart saat itu dulu, lalu perlahan dikembalikan ke
tengah agar cart tidak langsung ditarik kasar.

```python
if self.balance_use_lqr:
    force = self._state_feedback_force_locked(
        self.balance_lqr_gain,
        theta_top,
        dt,
        self.balance_force_limit_n,
    )
else:
    force = self._gui_force_feedback_locked(
        theta_top,
        dt,
        self.balance_force_limit_n,
    )
```

Feedback gain GUI masih ada sebagai fallback bila `balance_use_lqr` dibuat
`False`. Bentuknya mirip PD untuk sudut dan PD+I untuk posisi cart:

```python
force = (
    14.0 * self.gains["K_TH"] * theta_top
    + 35.0 * self.gains["K_TH_D"] * self.pendulum_vel_radps
    - 20.0 * self.gains["K_X"] * x_error_m
    - 8.0 * self.gains["K_X_D"] * self.cart_v_mps
    - 12.0 * self.gains["K_X_INT"] * self.x_integral_cm_s
)
```

### 6. PID velocity servo untuk aktuasi cart

Untuk mode yang menghasilkan command kecepatan, bridge mengubah command
kecepatan menjadi gaya joint dengan PID velocity servo:

```python
velocity_error = command - self.cart_v_mps
self.velocity_error_integral = clamp(
    self.velocity_error_integral + velocity_error * dt,
    -2.0,
    2.0,
)
velocity_error_dot = (
    velocity_error - self.prev_velocity_error
) / max(dt, 1e-3)

effort = (
    self.velocity_servo_p * velocity_error
    + self.velocity_servo_i * self.velocity_error_integral
    + self.velocity_servo_d * velocity_error_dot
)
```

Untuk `SWING_UP`, `LQR catch`, dan `BALANCE`, kode biasanya memberi
`effort_override`, sehingga gaya langsung dikirim ke Gazebo melalui
`/pendulum/cart_force_cmd`.

### 7. Balance assist khusus simulasi

Supaya simulasi lebih mudah berdiri, bridge juga memberi torsi kecil langsung
ke joint `pendulum_hinge` saat pendulum sudah dekat tegak. Ini bukan bagian
dari hardware pendulum pasif; ini hanya bantuan simulasi agar proses capture
lebih mudah dilihat dan dituning.

```python
if self.mode in (MODE_SWING_UP, MODE_BALANCE) and abs(theta_top) <= self.balance_assist_angle:
    torque = (
        -self.balance_assist_kp * theta_top
        - self.balance_assist_kd * self.pendulum_vel_radps
    )
```

## Cara cek penyebab gagal saat running

Monitor state saat menekan `X` dan saat mencoba `A`:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ammar/Documents/Pendulum/ros2_pendulum_ws/install/setup.bash
ros2 topic echo /pendulum/sim_state
```

Interpretasi cepat:

- Jika `mode` tidak berubah ke `6`, tombol `X`/serial packet belum masuk.
- Jika `mode=6` tetapi `degree` tetap sekitar `+/-180`, cart belum memberi energi cukup.
- Jika `degree` pernah dekat `0` tetapi `theta_dot_rad` masih besar, capture gagal karena batang lewat terlalu cepat.
- Jika `mode=7` sebentar lalu kembali ke `6`, balance gagal menahan dan fallback aktif.
- Target balance yang benar adalah `degree` dekat `0`, `theta_dot_rad` dekat `0`, `cmX` tidak dekat batas rel, dan `mode=7`.
