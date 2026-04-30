# Pendulum ROS 2 + Gazebo Workspaces

Repository ini berisi GUI Python lama untuk trainer linear inverted pendulum dan
beberapa workspace ROS 2 Jazzy + Gazebo Harmonic. Semua workspace dibuat supaya
tetap bisa dihubungkan ke `main.py` lewat pseudo serial, jadi GUI Python lama
tetap dipakai sebagai panel tombol, tuning gain, grafik, dan pembaca status.

## Ringkasan workspace

| Workspace | Package | Serial GUI | Launch | Tujuan utama |
| --- | --- | --- | --- | --- |
| `ros2_pendulum_ws` | `linear_inverted_pendulum_sim` | `/tmp/pendulum_sim_serial` | `sim.launch.py` | Simulasi demo yang lebih mudah bergerak dan cocok untuk eksperimen awal. |
| `pendulum_real_ws` | `linear_inverted_pendulum_real_sim` | `/tmp/pendulum_real_serial` | `real_sim.launch.py` | Simulasi yang lebih dekat ke Manual Book dan baseline yang sudah bisa tegak stabil. |
| `pendulum_pid_ws` | `linear_inverted_pendulum_pid_sim` | `/tmp/pendulum_pid_serial` | `pid_sim.launch.py` | Turunan `pendulum_real_ws`, tetapi balance controller dibuat PID. |

## Arsitektur umum

Alur sistemnya sama di semua workspace:

```text
main.py GUI
  -> pseudo serial /tmp/pendulum_*_serial
  -> ROS 2 serial bridge
  -> /pendulum/cart_force_cmd dan /pendulum/hinge_assist_force_cmd
  -> ros_gz_bridge
  -> Gazebo Harmonic model
  -> /joint_states
  -> serial bridge
  -> /pendulum/sim_state dan status packet balik ke GUI
```

File penting di root:

- `main.py`: GUI lama, tombol A/B/X/Y, input gain, pembaca status serial.
- `lib_com.py`: format packet serial.
- `lib_stick.py`: joystick sender jika joystick fisik dipakai.
- `lib_gui.py`: tampilan GUI.
- `data_exports/`: data CSV dan helper capture runtime.

Pseudo serial dibuat otomatis oleh node bridge. Dari sisi `main.py`, simulasi
terlihat seperti hardware STM32 karena format packet tetap sama.

## Mode dan tombol

Mode yang dikirim lewat `/pendulum/sim_state`:

| Mode | Nama | Fungsi |
| --- | --- | --- |
| `1` | `WAITING` | Bridge hidup, belum mulai kontrol. |
| `2` | `HOMING` | Cart balik ke tengah rail. |
| `3` | `READY` | Siap menerima perintah manual/swing. |
| `4` | `SINE` | Gerak sinusoidal untuk test. |
| `5` | `FINISH` | Stop. |
| `6` | `SWING_UP` | Membangun energi pendulum agar naik ke atas. |
| `7` | `BALANCE` | Menahan pendulum agar tegak. |

Urutan operasi yang disarankan:

1. Jalankan Gazebo dari workspace yang dipilih.
2. Jalankan `main.py` dengan `PENDULUM_PORT` sesuai serial workspace.
3. Klik `Apply Gains`.
4. Klik `START`.
5. Klik `Y` untuk homing.
6. Klik `X` untuk swing-up.
7. Klik `A` hanya saat pendulum sudah dekat tegak jika ingin memaksa balance.
8. Klik `B` untuk stop.

Catatan tombol `A`: pada `pendulum_real_ws` dan `pendulum_pid_ws`, jika `A`
ditekan saat masih `SWING_UP`, request balance disimpan dulu dan baru dipakai
saat syarat capture sudah aman. Pada `ros2_pendulum_ws`, tombol `A` lebih
langsung masuk balance.

## Topic utama

Semua workspace memakai topic yang sama agar mudah dibandingkan:

| Topic | Isi |
| --- | --- |
| `/joint_states` | Posisi dan kecepatan `cart_slider` serta `pendulum_hinge`. |
| `/pendulum/cart_velocity_cmd` | Command internal bridge dalam m/s. |
| `/pendulum/cart_force_cmd` | Gaya cart yang benar-benar dikirim ke Gazebo. |
| `/pendulum/hinge_assist_force_cmd` | Assist torsi kecil pada engsel khusus simulasi. |
| `/pendulum/sim_state` | Data ringkas untuk GUI dan debug. |

Format `/pendulum/sim_state`:

```text
[degree, cmX, setspeed, energy, theta_dot_rad, theta_rad, x_center_cm, mode]
```

Interpretasi cepat:

- `degree = 0` berarti pendulum tegak di atas.
- `degree = +/-180` berarti pendulum di bawah.
- `mode = 6` berarti swing-up.
- `mode = 7` berarti balance.
- `cmX` besar berarti cart mendekati ujung rail.

## Perbedaan metode kontrol

### 1. `ros2_pendulum_ws`

Path:

```bash
/home/ammar/Documents/Pendulum/ros2_pendulum_ws
```

Jalankan:

```bash
cd /home/ammar/Documents/Pendulum/ros2_pendulum_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch linear_inverted_pendulum_sim sim.launch.py
```

GUI:

```bash
cd /home/ammar/Documents/Pendulum
PENDULUM_PORT=/tmp/pendulum_sim_serial PENDULUM_NO_JOYSTICK=1 python3 main.py
```

Sistem kontrol:

- Homing: PD sederhana untuk membawa cart ke tengah.
- Swing-up: energy-based swing-up.
- Balance: default `balance_use_lqr=True`, jadi jalur balance utamanya LQR /
  state feedback.
- Ada assist engsel kecil untuk membantu simulasi.
- Force limit lebih besar (`balance_force_limit_n` dan `effort_limit_n` tinggi)
  supaya demo lebih mudah bergerak.

Kapan dipakai:

- Untuk demo awal.
- Untuk melihat alur serial, topic, dan GUI tanpa terlalu ketat mengikuti
  karakter motor real.
- Untuk eksperimen LQR atau controller yang lebih bebas.

### 2. `pendulum_real_ws`

Path:

```bash
/home/ammar/Documents/Pendulum/pendulum_real_ws
```

Jalankan:

```bash
cd /home/ammar/Documents/Pendulum/pendulum_real_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch linear_inverted_pendulum_real_sim real_sim.launch.py
```

GUI:

```bash
cd /home/ammar/Documents/Pendulum
PENDULUM_PORT=/tmp/pendulum_real_serial PENDULUM_SIM=1 PENDULUM_NO_JOYSTICK=1 python3 main.py
```

Sistem kontrol:

- Homing: PD.
- Swing-up: energy-based, dengan gate tambahan agar pendulum diayun dulu dan
  tidak langsung dipaksa balance.
- Balance: `balance_use_lqr=False`. Bentuk balance mengikuti state Manual Book:
  sudut pendulum, kecepatan sudut, posisi cart, kecepatan cart, dan integral
  kecil posisi cart. Penamaan yang paling tepat untuk workspace ini adalah
  full-state feedback, bukan PID murni.
- Motor model lebih real-style: deadband PWM `3212`, slope `189.1`, time
  constant sekitar `0.40 s`.
- Force limit lebih rendah daripada workspace demo.
- Assist engsel aktif default sebagai bantuan khusus simulasi agar bisa tegak
  stabil di Gazebo.

Kapan dipakai:

- Untuk baseline yang paling dekat dengan Manual Book.
- Untuk membandingkan simulasi dengan trainer fisik.
- Untuk tuning yang ingin mempertahankan karakter real-style.

Catatan penting: hasil tegak stabil di simulasi ini masih dibantu assist engsel.
Jadi jangan dianggap sebagai clone hardware murni tanpa bantuan simulasi.

### 3. `pendulum_pid_ws`

Path:

```bash
/home/ammar/Documents/Pendulum/pendulum_pid_ws
```

Jalankan:

```bash
cd /home/ammar/Documents/Pendulum/pendulum_pid_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch linear_inverted_pendulum_pid_sim pid_sim.launch.py
```

GUI:

```bash
cd /home/ammar/Documents/Pendulum
PENDULUM_PORT=/tmp/pendulum_pid_serial PENDULUM_SIM=1 PENDULUM_NO_JOYSTICK=1 python3 main.py
```

Sistem kontrol:

- Workspace ini diturunkan dari `pendulum_real_ws`.
- Model fisik, motor model, swing-up, capture, dan assist simulasi tetap mirip
  `pendulum_real_ws`.
- Balance sengaja diganti menjadi PID sudut pendulum:
  - `K_TH` menjadi P sudut.
  - `K_TH_D` menjadi D sudut.
  - `K_X_INT` menjadi I sudut.
- Agar cart tidak lari ke ujung rail, PID sudut ditambah PD cart-centering:
  - `K_X` untuk posisi cart.
  - `K_X_D` untuk damping kecepatan cart.
- Tidak ada jalur LQR dan tidak ada jalur full-state feedback untuk balance.

Kapan dipakai:

- Untuk eksperimen yang memang harus memakai PID.
- Untuk membandingkan PID melawan full-state feedback di `pendulum_real_ws`.
- Untuk laporan yang ingin memisahkan metode PID dari metode Manual Book.

## Perbandingan cepat

| Aspek | `ros2_pendulum_ws` | `pendulum_real_ws` | `pendulum_pid_ws` |
| --- | --- | --- | --- |
| Fokus | Demo/eksperimen awal | Manual Book + real-style | PID balance |
| Serial | `/tmp/pendulum_sim_serial` | `/tmp/pendulum_real_serial` | `/tmp/pendulum_pid_serial` |
| Launch | `sim.launch.py` | `real_sim.launch.py` | `pid_sim.launch.py` |
| Homing | PD | PD | PD |
| Swing-up | Energy-based | Energy-based dengan readiness gate | Energy-based dengan readiness gate |
| Balance | LQR/state feedback default | Full-state feedback Manual-style | PID sudut + PD cart |
| Motor model | Lebih demo-oriented | Deadband PWM + time constant | Deadband PWM + time constant |
| Force limit | Lebih besar | Lebih rendah/real-style | Lebih rendah/real-style |
| Assist engsel | Ada | Ada, untuk stabilitas simulasi | Ada, untuk stabilitas simulasi |

## Build semua workspace

```bash
source /opt/ros/jazzy/setup.bash

cd /home/ammar/Documents/Pendulum/ros2_pendulum_ws
colcon build --symlink-install

cd /home/ammar/Documents/Pendulum/pendulum_real_ws
colcon build --symlink-install

cd /home/ammar/Documents/Pendulum/pendulum_pid_ws
colcon build --symlink-install
```

## Verifikasi cepat

Gunakan ini sebelum tuning:

```bash
source /opt/ros/jazzy/setup.bash

python3 -m py_compile main.py
python3 -m py_compile ros2_pendulum_ws/src/linear_inverted_pendulum_sim/linear_inverted_pendulum_sim/sim_serial_bridge.py
python3 -m py_compile pendulum_real_ws/src/linear_inverted_pendulum_real_sim/linear_inverted_pendulum_real_sim/real_serial_bridge.py
python3 -m py_compile pendulum_pid_ws/src/linear_inverted_pendulum_pid_sim/linear_inverted_pendulum_pid_sim/pid_serial_bridge.py

check_urdf <(xacro ros2_pendulum_ws/src/linear_inverted_pendulum_sim/urdf/linear_inverted_pendulum.urdf.xacro)
check_urdf <(xacro pendulum_real_ws/src/linear_inverted_pendulum_real_sim/urdf/linear_inverted_pendulum_real.urdf.xacro)
check_urdf <(xacro pendulum_pid_ws/src/linear_inverted_pendulum_pid_sim/urdf/linear_inverted_pendulum_pid.urdf.xacro)
```

## Cara memilih workspace

Pilih `ros2_pendulum_ws` kalau tujuan utamanya melihat simulasi bergerak,
menguji plumbing ROS 2/Gazebo, atau eksperimen controller dengan toleransi gaya
lebih besar.

Pilih `pendulum_real_ws` kalau tujuan utamanya mempertahankan bentuk Manual
Book, motor model, dan baseline yang sudah dibuat agar pendulum bisa tegak
lebih stabil di Gazebo.

Pilih `pendulum_pid_ws` kalau laporan atau eksperimen harus memakai PID sebagai
metode balance. Workspace ini sengaja dibuat terpisah supaya metode PID tidak
tercampur dengan full-state feedback di `pendulum_real_ws`.

## Catatan debugging

- Jika GUI tidak connect, cek apakah serial symlink sudah dibuat:
  `ls -l /tmp/pendulum_*_serial`.
- Jika mode tidak berubah, cek `/pendulum/sim_state`.
- Jika balance masuk terlalu awal lalu jatuh, cek `degree`, `theta_dot_rad`,
  `cmX`, dan `mode`.
- Jika cart menabrak rail, cek `cmX`, `cart_force_cmd`, dan force limit.
- Jika memakai `pendulum_real_ws` atau `pendulum_pid_ws`, tetap pakai
  `PENDULUM_SIM=1` saat menjalankan `main.py`.
