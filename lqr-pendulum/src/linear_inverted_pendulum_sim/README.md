# Linear Inverted Pendulum LQR Gazebo Sim

Paket ini dibuat untuk ROS 2 Jazzy dan Gazebo Harmonic 8. Model mengikuti data dari manual LIP01:

- Base 1000 mm x 350 mm.
- Rail linear sekitar 900 mm dengan travel kontrol 78 cm.
- Cart bergerak pada joint prismatic `cart_slider`.
- Pendulum shaft diameter 8 mm, panjang 400 mm, massa 200 g.
- Encoder pendulum dimodelkan dari joint `pendulum_hinge`.

## Build

```bash
cd /home/ammar/Documents/Pendulum/lqr-pendulum
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
/tmp/pendulum_lqr_serial
```

## Model visual lebih detail

Model Gazebo sekarang dibuat lebih mirip trainer linear inverted pendulum pada
gambar referensi. Perubahan ini hanya visual/cosmetic; nama joint, limit rail,
massa utama, dan topic kontrol tetap sama.

Detail yang ditambahkan:

- Base plate hitam dengan side extrusion dan baut.
- Dua linear rail dengan profile gelap dan shaft metal.
- Chain/belt track dengan link berulang, pulley kiri/kanan, dan motor drive.
- Carriage lebih detail dengan bearing sleeve, belt clamp, encoder disk, side plate, dan sensor kecil.
- Pendulum rod dibuat metal seperti batang asli, bukan batang merah tebal.

File visual yang sama dipakai di `lqr-pendulum`, `pendulum_real_ws`, dan
`pendulum_pid_ws`, sehingga tampilan ketiga workspace konsisten.

## Jalankan dashboard split otomatis

Cara ini menjalankan Gazebo di kiri dan grafik live di kanan seperti screenshot,
tanpa perlu membuka `main.py` terpisah:

```bash
cd /home/ammar/Documents/Pendulum
python3 pendulum_gazebo_plot_dashboard.py --workspace lqr
```

Workspace lain memakai skrip yang sama:

```bash
python3 pendulum_gazebo_plot_dashboard.py --workspace real
python3 pendulum_gazebo_plot_dashboard.py --workspace pid
```

Dashboard sekarang memakai refresh grafik lebih rapat: `--sample-period 0.02`,
`--plot-hz 30`, dan smoothing visual ringan `--plot-smoothing-samples 3`.
Smoothing ini hanya untuk tampilan grafik; data ROS dan CSV tetap raw. Jika
ingin melihat garis raw penuh, jalankan dengan `--plot-smoothing-samples 1`.

## Gaya eksternal setelah balance

Dashboard memberi gaya eksternal hanya setelah alur otomatis mencapai
`BALANCE` dan state sudah stabil. Setelah impulse selesai, gaya otomatis
dikembalikan ke `0 N`.

Contoh memberi gaya eksternal sekali saja:

```bash
cd /home/ammar/Documents/Pendulum
python3 pendulum_gazebo_plot_dashboard.py --workspace lqr --external-impulse-force 2.0 --external-impulse-duration 0.20
```

Untuk arah sebaliknya, pakai nilai negatif:

```bash
python3 pendulum_gazebo_plot_dashboard.py --workspace lqr --external-impulse-force -2.0 --external-impulse-duration 0.20
```

Cari gaya impulse maksimal yang masih bisa ditahan:

```bash
python3 pendulum_gazebo_plot_dashboard.py --workspace lqr --find-max-external
```

Hasil uji dashboard 2026-05-10 setelah smoothing effort cart, dengan impulse
`0.20 s` setelah `BALANCE`:

| Workspace | Arah positif | Gagal positif | Arah negatif | Gagal negatif |
| --- | ---: | ---: | ---: | ---: |
| `lqr-pendulum` | `6 N` survive | `8 N` gagal | `6 N` survive | `8 N` gagal |
| `pendulum_real_ws` | `6 N` survive | `8 N` gagal | `6 N` survive | `8 N` gagal |
| `pendulum_pid_ws` | `4 N` survive | `6 N` gagal | `6 N` survive | `8 N` gagal |

Ringkasan CSV: `data_exports/external_impulse_threshold_summary_20260510_dashboard.csv`.

Angka di atas adalah gaya ujung pendulum simulasi. Bridge mengubahnya menjadi
torsi engsel (`force * pendulum_length`) selama durasi impulse, lalu otomatis
mengembalikan gaya ke `0 N`. Metode balance dinilai berhasil jika setelah
impulse pendulum tetap di mode `BALANCE`, sudut kembali kecil, dan cart tidak
menyentuh batas rel dalam window recovery.

## Tuning balance lebih dominan

Pada 2026-05-10, balance gate disetel supaya syarat sudut tetap
`-10 deg < theta < 10 deg`, tetapi swing-up tidak terlalu dominan setelah
pendulum sudah tegak. Caranya, balance diberi kesempatan setelah top pass
pertama, waktu tunggu swing-up dipersingkat, lalu ditambah auto-lock saat
pendulum sudah dekat posisi atas:

- `balance_capture_deg`: `9 deg` -> `10 deg`.
- `balance_capture_rate_rad_s`: `1.0` -> `2.4`.
- `balance_fallback_deg`: `45 deg` -> `70 deg`.
- `balance_give_up_deg`: `135 deg`, supaya mode tidak cepat balik ke `SWING_UP`.
- `balance_force_limit_n`: tetap dibatasi `45 N` setelah swing-up/catch dikembalikan kuat.
- `swing_min_top_passes_before_catch`: `3`.
- `swing_min_energy_build_time_s`: `8.0 s`.
- `swing_energy_ready_ratio`: `0.88`.
- `balance_immediate_capture_deg`: `1 deg`, supaya angle yang sudah hampir `0 deg` langsung masuk `BALANCE`.
- `balance_immediate_capture_cart_pos_m`: `0.34 m`.
- `balance_immediate_capture_cart_vel_mps`: `2.2 m/s`.
- `balance_auto_lock_angle_deg`: `3 deg`.
- `balance_auto_lock_rate_rad_s`: `2.4`.
- `balance_auto_lock_cart_pos_m`: `0.34 m`.
- `balance_auto_lock_cart_vel_mps`: `2.2 m/s`.
- `balance_auto_lock_time_s`: `0.08 s`, supaya auto-lock tidak dipicu oleh satu sampel sesaat.

Validasi headless 28 detik setelah tuning:

| Workspace | Sampel SWING_UP | Sampel BALANCE | Mode switch | Swing setelah balance | Phase terakhir |
| --- | ---: | ---: | ---: | ---: | --- |
| `lqr-pendulum` | `210` | `342` | `1` | `0` | `BALANCE` |
| `pendulum_real_ws` | `116` | `435` | `1` | `0` | `BALANCE` |
| `pendulum_pid_ws` | `158` | `389` | `1` | `0` | `BALANCE` |

Ringkasan CSV: `data_exports/balance_immediate_capture_validation_summary_20260510.csv`.

Validasi ulang setelah model visual detail ditambahkan tetap menunjukkan tidak
ada `SWING_UP` lagi setelah pertama kali masuk `BALANCE`:

| Workspace | First BALANCE | Swing setelah balance | Swing force min/max | Swing force abs mean | Balance force abs mean |
| --- | ---: | ---: | ---: | ---: | ---: |
| `lqr-pendulum` | `11.212 s` | `0` | `-90.000..92.084 N` | `56.617 N` | `0.485 N` |
| `pendulum_real_ws` | `11.134 s` | `0` | `-145.000..145.000 N` | `83.011 N` | `0.303 N` |
| `pendulum_pid_ws` | `11.535 s` | `0` | `-145.000..145.000 N` | `84.991 N` | `0.361 N` |

Ringkasan CSV: `data_exports/post_visual_balance_validation_summary_20260510.csv`.

Catatan laporan: nilai `cart_force_cmd_n` / `cart_joint_effort_cmd_n` adalah
perintah effort joint cart di Gazebo. Nilai ini boleh dipakai untuk membandingkan
workspace simulasi, tetapi jangan ditulis sebagai gaya motor fisik alat asli
tanpa kalibrasi motor/driver/sensor gaya.

## Swing-up lebih halus

Proses swing-up memang memakai teori energy-based swing-up: controller membaca
energi pendulum, membandingkannya dengan energi target posisi tegak, lalu
memberi dorongan cart searah fase ayunan. Bagian ini benar secara teori.

Yang sebelumnya membuat gerak terlihat kurang smooth adalah implementasi
simulasi, bukan rumus energinya:

- ada minimum pump force yang langsung memberi dorongan besar,
- kick awal dari posisi bawah terlalu keras,
- force swing-up bisa melompat langsung ke batas `90 N` atau `145 N`,
- effort langsung dikirim ke joint Gazebo, sehingga tidak selalu terlihat
  seperti respons motor real yang bertahap.

Perbaikan yang dipakai sekarang:

- `swing_min_pump_force_n`: minimum pump force dasar dibuat lebih kecil.
- `swing_min_pump_force_extra_n`: tambahan minimum pump force dibuat gradual
  mengikuti energy deficit.
- `swing_kick_force_n`: kick awal dibuat lebih rendah.
- `swing_force_rate_limit_nps`: force swing-up diberi rate limit agar command
  naik/turun bertahap, bukan loncat mendadak.
- `swing_min_top_passes_before_catch`: capture ditunda sampai beberapa pass
  area atas atau waktu build energi minimum terpenuhi.
- `swing_min_energy_build_time_s`: dinaikkan menjadi `8.0 s` agar pendulum
  terlihat mengumpulkan energi lewat beberapa ayunan sebelum balance.

Validasi headless bersih setelah tuning multi-swing:

| Workspace | First BALANCE | Phase terakhir | Ayunan besar | Swing force min/max | Swing force abs mean | Balance force abs mean |
| --- | ---: | --- | ---: | ---: | ---: | ---: |
| `lqr-pendulum` | `8.283 s` | `BALANCE` | `10` reversal | `-33.530..29.045 N` | `5.290 N` | `0.009 N` |
| `pendulum_real_ws` | `8.299 s` | `BALANCE` | `7` reversal | `-30.956..28.253 N` | `4.958 N` | `0.027 N` |
| `pendulum_pid_ws` | `8.278 s` | `BALANCE` | `9` reversal | `-26.203..28.169 N` | `5.685 N` | `0.050 N` |

Ringkasan CSV:

- `data_exports/multi_swing_validation_summary_20260510.csv`
- `data_exports/multi_swing_pass_summary_20260510.csv`
- `data_exports/multi_swing_real_reference_summary_20260510.csv`

Jika ada Gazebo lama yang masih aktif dan window menjadi bentrok, hentikan dulu:

```bash
pkill -f "ros2 launch linear_inverted_pendulum"
pkill -f "gz sim"
```

## Hubungkan ke GUI Python lama

Terminal baru:

```bash
cd /home/ammar/Documents/Pendulum
PENDULUM_PORT=/tmp/pendulum_lqr_serial PENDULUM_NO_JOYSTICK=1 python3 main.py
```

Saat `PENDULUM_PORT=/tmp/pendulum_lqr_serial`, `main.py` otomatis memakai gain default yang cocok untuk simulasi balance.

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

## Kenapa balance bisa berganti-ganti

Kalau pendulum secara visual sudah tegak tetapi mode masih `SWING UP`, masalah
utamanya ada di transisi kontrol. Swing-up sudah berhasil memberi energi, tetapi
bridge hanya boleh memberi alih ke `BALANCING` kalau kondisi capture terpenuhi:

- Sudut harus dekat atas: `abs(theta_top) < 10 deg`.
- Kecepatan sudut tidak boleh terlalu besar: `abs(theta_dot) < 2.4 rad/s`.
- Cart harus masih cukup dekat tengah: `abs(x) < 0.34 m`.
- Kecepatan cart harus tidak terlalu besar: `abs(x_dot) < 2.2 m/s`.

Sekarang ada tambahan immediate capture dan auto-lock: bila mode masih
`SWING UP`, tetapi pendulum sudah sangat dekat tegak (`1 deg`), bridge langsung
mengalihkan mode ke `BALANCE`. Kalau belum sedekat itu, auto-lock tetap akan
mengambil alih pada area `3 deg` selama cart masih aman. Jadi kondisi seperti
screenshot `angle=0.00 deg` tidak dibiarkan tetap lama di `SWING`.

Kalau batang lewat dekat atas tetapi masih terlalu cepat, mode belum masuk
capture balance. Setelah sudah masuk `BALANCE`, bridge sekarang tetap memberi
kesempatan ke controller balance untuk bekerja sampai sudut benar-benar jauh
(`135 deg`) atau cart keluar batas rail. Jadi `SWING UP` hanya kembali dipakai
kalau pendulum sudah dianggap jatuh, bukan saat baru sedikit keluar dari area
tegak.

Pada checkout aktif sekarang, workspace ini adalah varian `lqr-pendulum`.
Mode `BALANCE` default memakai LQR karena `balance_use_lqr=True`. Gain LQR
dihitung dari model linear inverted pendulum dengan state:

```text
x = [posisi_cart, kecepatan_cart, sudut_pendulum_atas, kecepatan_sudut]
u = -Kx
```

Bobot `Q/R` yang dipakai ada pada parameter `lqr_q_x`, `lqr_q_x_dot`,
`lqr_q_theta`, `lqr_q_theta_dot`, dan `lqr_r`. Gain GUI dari `main.py` tetap
diterima agar protokol serial lama kompatibel, tetapi balance LQR aktif tidak
bergantung pada gain GUI tersebut.

Memperbesar radius visual batang tidak banyak membantu di Gazebo karena tidak
ada model aerodinamika. Kalau ingin membuat simulasi lebih mudah secara fisik,
parameter yang lebih berpengaruh adalah panjang pendulum, massa, damping
engsel, batas gaya cart, tuning controller, atau bantuan torsi engsel khusus
simulasi. Kalau ingin kembali ke model pendulum pasif, set
`balance_assist_enabled:=false`.

Parameter tuning utama ada di `sim_serial_bridge.py`:

```python
self.declare_parameter("balance_capture_deg", 10.0)
self.declare_parameter("balance_capture_rate_rad_s", 2.4)
self.declare_parameter("balance_fallback_deg", 70.0)
self.declare_parameter("balance_give_up_deg", 135.0)
self.declare_parameter("balance_capture_cart_pos_m", 0.08)
self.declare_parameter("balance_capture_cart_vel_mps", 2.2)
self.declare_parameter("balance_immediate_capture_deg", 1.0)
self.declare_parameter("balance_immediate_capture_cart_pos_m", 0.08)
self.declare_parameter("balance_immediate_capture_cart_vel_mps", 2.2)
self.declare_parameter("balance_auto_lock_enabled", True)
self.declare_parameter("balance_auto_lock_angle_deg", 3.0)
self.declare_parameter("balance_auto_lock_rate_rad_s", 2.4)
self.declare_parameter("balance_auto_lock_cart_pos_m", 0.08)
self.declare_parameter("balance_auto_lock_cart_vel_mps", 2.2)
self.declare_parameter("balance_auto_lock_time_s", 0.08)
self.declare_parameter("catch_region_deg", 95.0)
self.declare_parameter("catch_region_rate_rad_s", 14.0)
self.declare_parameter("catch_force_limit_n", 95.0)
self.declare_parameter("balance_force_limit_n", 45.0)
self.declare_parameter("balance_use_lqr", True)
self.declare_parameter("balance_assist_enabled", True)
self.declare_parameter("balance_assist_angle_deg", 115.0)
self.declare_parameter("balance_assist_kp_nm_per_rad", 3.4)
self.declare_parameter("balance_assist_kd_nm_per_rad_s", 2.4)
self.declare_parameter("balance_assist_torque_limit_nm", 4.5)
self.declare_parameter("effort_limit_n", 150.0)
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

### 3. Catch region dekat posisi atas

Sebelum masuk balance, controller memakai catch region untuk mengurangi
kecepatan ayunan dan membawa state masuk ke capture window. Pada default aktif
sekarang, catch memakai upright force feedback yang sama dengan balance, tetapi
dengan limit `catch_force_limit_n`.

```python
if self._in_catch_region_locked(theta_top):
    force = self._catch_force_locked(theta_top, dt)
    return self._force_to_command_hint(force), force
```

Fungsi catch region mengecek sudut, kecepatan sudut, dan posisi cart:

```python
return (
    abs(theta_top) < self.catch_region_angle
    and abs(self.pendulum_vel_radps) < self.catch_region_rate
    and abs(self.cart_x_m) < (self.rail_limit - 0.004)
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

### 5. Balance LQR aktif

Saat mode `BALANCE`, controller default memakai LQR. Target regulasi adalah
state tengah `[x, x_dot, theta, theta_dot] = [0, 0, 0, 0]`, sehingga
`x_center_m` diset ke `0.0 m`, bukan ke posisi capture. Error posisi cart juga
dipakai sebagai bias sudut kecil agar cart kembali ke tengah lewat state
feedback.

```python
force = self._state_feedback_force_locked(
    self.balance_lqr_gain,
    theta_top,
    dt,
    self.balance_force_limit_n,
)
return self._force_to_command_hint(force), force
```

Bentuk LQR memakai state feedback:

```python
state = (x_error_m, self.cart_v_mps, theta_top, self.pendulum_vel_radps)
force = -sum(gain_value * state_value for gain_value, state_value in zip(gain, state))
```

Fallback PID-like masih ada hanya untuk debugging jika `balance_use_lqr` sengaja
diubah menjadi `False`.

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

Untuk `SWING_UP`, catch, dan `BALANCE`, kode biasanya memberi
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
source /home/ammar/Documents/Pendulum/lqr-pendulum/install/setup.bash
ros2 topic echo /pendulum/sim_state
```

Interpretasi cepat:

- Jika `mode` tidak berubah ke `6`, tombol `X`/serial packet belum masuk.
- Jika `mode=6` tetapi `degree` tetap sekitar `+/-180`, cart belum memberi energi cukup.
- Jika `degree` pernah dekat `0` tetapi `theta_dot_rad` masih besar, capture gagal karena batang lewat terlalu cepat.
- Jika `mode=7` sebentar lalu kembali ke `6`, balance gagal menahan dan fallback aktif.
- Target balance yang benar adalah `degree` dekat `0`, `theta_dot_rad` dekat `0`, `cmX` tidak dekat batas rel, dan `mode=7`.
