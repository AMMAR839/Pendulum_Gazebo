# Pendulum ROS 2 + Gazebo Workspaces

Repository ini berisi GUI Python lama untuk trainer linear inverted pendulum dan
beberapa workspace ROS 2 Jazzy + Gazebo Harmonic. Semua workspace dibuat supaya
tetap bisa dihubungkan ke `main.py` lewat pseudo serial, jadi GUI Python lama
tetap dipakai sebagai panel tombol, tuning gain, grafik, dan pembaca status.

## Ringkasan workspace

| Workspace | Package | Serial GUI | Launch | Tujuan utama |
| --- | --- | --- | --- | --- |
| `lqr-pendulum` | `linear_inverted_pendulum_sim` | `/tmp/pendulum_lqr_serial` | `sim.launch.py` | Workspace LQR untuk balance berbasis model linear. |
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
- `DOKUMENTASI_METODE_DAN_DASAR_TEORI.md`: dokumen laporan yang menjelaskan
  dasar teori, metode kontrol, dan perbandingan semua workspace.

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

Catatan tombol `A`: pada semua workspace, jika `A` ditekan saat state belum
masuk jendela capture, request balance disimpan dulu dan baru dipakai saat
sudut, theta-dot, posisi cart, dan kecepatan cart sudah aman. Ini mencegah
mode `BALANCE` aktif ketika cart masih jauh dari tengah.

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

### 1. `lqr-pendulum`

Path:

```bash
/home/ammar/Documents/Pendulum/lqr-pendulum
```

Jalankan:

```bash
cd /home/ammar/Documents/Pendulum/lqr-pendulum
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch linear_inverted_pendulum_sim sim.launch.py
```

GUI:

```bash
cd /home/ammar/Documents/Pendulum
PENDULUM_PORT=/tmp/pendulum_lqr_serial PENDULUM_NO_JOYSTICK=1 python3 main.py
```

Sistem kontrol:

- Homing: PD sederhana untuk membawa cart ke tengah.
- Swing-up: energy-based swing-up.
- Balance memakai LQR aktif dengan default `balance_use_lqr=True`.
- Gain LQR dihitung dari model linear inverted pendulum memakai bobot `Q/R`
  (`lqr_q_x`, `lqr_q_x_dot`, `lqr_q_theta`, `lqr_q_theta_dot`, `lqr_r`).
- Target regulasi balance adalah state tengah:
  `[x, x_dot, theta, theta_dot] = [0, 0, 0, 0]`.
- Error posisi cart dipakai sebagai bias sudut kecil pada LQR supaya cart
  kembali ke tengah lewat state feedback, bukan dorongan kasar.
- Jika `balance_use_lqr` dimatikan manual, bridge masih punya fallback
  PID-like upright controller untuk debugging.
- Ada assist engsel kecil untuk membantu simulasi.
- Tuning memakai force limit konservatif agar hasil simulasi tidak bergantung
  pada effort Gazebo yang terlalu bebas.

Kapan dipakai:

- Untuk eksperimen dan pembuktian simulasi balance berbasis LQR.
- Untuk membandingkan LQR dengan full-state feedback Manual-style di
  `pendulum_real_ws` dan PID di `pendulum_pid_ws`.
- Untuk laporan yang ingin memisahkan metode LQR dari metode Manual Book dan
  PID.

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

| Aspek | `lqr-pendulum` | `pendulum_real_ws` | `pendulum_pid_ws` |
| --- | --- | --- | --- |
| Fokus | LQR balance | Manual Book + real-style | PID balance |
| Serial | `/tmp/pendulum_lqr_serial` | `/tmp/pendulum_real_serial` | `/tmp/pendulum_pid_serial` |
| Launch | `sim.launch.py` | `real_sim.launch.py` | `pid_sim.launch.py` |
| Homing | PD | PD | PD |
| Swing-up | Energy-based | Energy-based dengan readiness gate | Energy-based dengan readiness gate |
| Balance | LQR aktif | Full-state feedback Manual-style | PID sudut + PD cart |
| Motor model | Deadband PWM + time constant | Deadband PWM + time constant | Deadband PWM + time constant |
| Force limit | LQR balance dibatasi | Lebih rendah/real-style | Lebih rendah/real-style |
| Assist engsel | Ada | Ada, untuk stabilitas simulasi | Ada, untuk stabilitas simulasi |

## Patokan gaya swing-up untuk real

Gaya swing-up dari simulasi bisa dipakai sebagai patokan awal alat real, tetapi
posisinya harus jelas: angka `cart_force_cmd_n` adalah effort cart pada model
Gazebo, bukan hasil ukur gaya motor fisik. Agar tetap bisa
dipertanggungjawabkan sebagai patokan real, repo ini menyediakan ringkasan
envelope swing-up yang tidak hanya melihat rata-rata force, tetapi juga durasi,
RMS/P95/peak effort, impulse, estimasi kerja dari command, kenaikan energi
pendulum, command kecepatan cart, dan PWM ekuivalen untuk workspace real-style.

File utama:

- `data_exports/SWING_UP_REAL_REFERENCE.md`: narasi laporan dan batas klaim.
- `data_exports/multi_swing_real_reference_summary_20260510.csv`: ringkasan
  terbaru setelah force dibuat konservatif dan swing-up ditahan lebih lama
  sebelum balance.
- `data_exports/build_swing_up_real_reference.py`: generator ringkasan.

Klaim yang aman untuk laporan:

```text
Simulasi digunakan sebagai patokan awal kebutuhan energi, durasi swing-up, dan
envelope command actuator. Nilai force Gazebo belum diklaim sebagai gaya motor
fisik sampai dibandingkan dengan log alat asli.
```

## Catatan swing-up multi-swing

Swing-up tetap memakai teori energy-based swing-up. Perbaikan terbaru tidak
mengganti teorinya, tetapi mengembalikan cap swing/catch ke baseline yang bisa
mencapai posisi tegak: `swing_force_limit_n = 145 N` dan
`catch_force_limit_n = 95 N`. Balance tetap dibatasi lebih rendah dengan
`balance_force_limit_n = 45 N`, sehingga gaya besar dipakai untuk membangun
energi swing-up, bukan untuk menahan mode balance secara kasar. Gate terbaru
tetap menahan capture sampai swing-up membangun energi lebih lama:
`swing_min_top_passes_before_catch = 3`,
`swing_min_energy_build_time_s = 8.0`, dan
`swing_energy_ready_ratio = 0.88`.

Validasi bersih menunjukkan ketiga workspace tetap masuk `BALANCE`. Fase
swing-up berlangsung sekitar `8 s`, rata-rata effort turun menjadi sekitar
`4.9 N` sampai `5.7 N`, dan peak hanya sekitar `28 N` sampai `34 N`. Ini lebih
mudah dipertanggungjawabkan karena pendulum mengumpulkan energi lewat beberapa
ayunan besar sebelum ditangkap ke balance.

### Kenapa peak force masih masuk akal

Peak force pada tabel bukan total energi ayunan dan bukan gaya rata-rata
motor. Peak force adalah gaya cart terbesar sesaat pada satu sampel swing-up.
Secara teori swing-up, controller perlu menaikkan energi pendulum dari posisi
bawah ke dekat tegak:

```text
E_target = m_pendulum * g * (2 * l_com)
E_target = 0.20 * 9.81 * (2 * 0.20)
E_target ~= 0.785 J
```

Energi ini masuk dari kerja aktuator cart:

```text
W_cart ~= integral(F_cart * v_cart dt) = integral(F_cart dx)
```

Karena rail terbatas, arah gaya tidak selalu ideal, ada damping, dan sebagian
dorongan hanya mengatur timing ayunan, kerja positif yang diperintah actuator
lebih besar dari energi ideal pendulum. Pada validasi terbaru, estimasi kerja
positif swing-up sekitar `6.2 J` sampai `7.6 J`, sedangkan kenaikan energi
pendulum yang dibutuhkan sekitar `0.8 J`. Ini wajar untuk swing-up simulasi:
aktuator memberi beberapa dorongan bolak-balik, tidak satu dorongan ideal.

Dengan pulsa sesaat `28 N` sampai `34 N`, gaya tersebut masih logis untuk alat
pendulum kecil jika dibaca sebagai peak aktuator cart. Jika memakai pulley
radius `1 cm` sampai `2 cm`, torsi motor ekuivalennya kira-kira:

```text
tau = F * r / eta
tau(34 N, r=0.01 m, eta=0.7) ~= 0.49 Nm
tau(34 N, r=0.02 m, eta=0.7) ~= 0.97 Nm
```

Jadi peak force `28 N` sampai `34 N` dapat dipertanggungjawabkan sebagai peak
effort simulasi yang muncul sesaat selama energy-based swing-up. Klaim ini
tidak sama dengan klaim gaya motor fisik; untuk hardware harus tetap
divalidasi dengan radius pulley, efisiensi transmisi, PWM/arus motor, dan log
gerak cart.

Ringkasan validasi:

```text
data_exports/multi_swing_validation_summary_20260510.csv
data_exports/multi_swing_pass_summary_20260510.csv
data_exports/multi_swing_real_reference_summary_20260510.csv
```

## Build semua workspace

```bash
source /opt/ros/jazzy/setup.bash

cd /home/ammar/Documents/Pendulum/lqr-pendulum
colcon build --symlink-install

cd /home/ammar/Documents/Pendulum/pendulum_real_ws
colcon build --symlink-install

cd /home/ammar/Documents/Pendulum/pendulum_pid_ws
colcon build --symlink-install
```

## Dashboard dengan gaya eksternal impulse

Dashboard `pendulum_gazebo_plot_dashboard.py` bisa menjalankan Gazebo, grafik
live, auto homing/swing/balance, lalu memberi gaya eksternal sekali saja
sebagai impulse. Gaya eksternal ini adalah gaya ujung pendulum di simulasi; di
bridge dikonversi menjadi torsi engsel, bukan gaya motor asli.

Grafik dashboard sekarang lebih halus secara visual dengan default
`--sample-period 0.02`, `--plot-hz 30`, dan `--plot-smoothing-samples 3`.
Smoothing ini hanya memengaruhi tampilan grafik, bukan topic ROS atau CSV.

Contoh memberi impulse 2 N selama 0.20 s:

```bash
cd /home/ammar/Documents/Pendulum
python3 pendulum_gazebo_plot_dashboard.py --workspace lqr --external-impulse-force 2.0 --external-impulse-duration 0.20
python3 pendulum_gazebo_plot_dashboard.py --workspace real --external-impulse-force 2.0 --external-impulse-duration 0.20
python3 pendulum_gazebo_plot_dashboard.py --workspace pid --external-impulse-force 2.0 --external-impulse-duration 0.20
```

Gunakan nilai negatif untuk dorongan arah sebaliknya, misalnya
`--external-impulse-force -2.0`.

Untuk mencari gaya impulse maksimum yang masih bisa dipertahankan:

```bash
python3 pendulum_gazebo_plot_dashboard.py \
  --workspace lqr \
  --find-max-external \
  --external-impulse-duration 0.20 \
  --external-max-forces 0.5,1.0,1.5,2.0,2.5,3.0,4.0,5.0,6.0,8.0,10.0
```

Output pentingnya:

- `gaya impulse aman terakhir`: gaya terbesar yang masih kembali stabil.
- `gaya gagal pertama`: gaya terkecil di daftar uji yang membuat keluar balance,
  sudut melewati batas, atau cart menyentuh batas rail.

Hasil uji terbaru dengan impulse `0.20 s` dan effort cart di-slew-limit
`320 N/s`:

| Workspace | Positif aman/gagal | Negatif aman/gagal |
| --- | ---: | ---: |
| `lqr-pendulum` | `6 N / 8 N` | `6 N / 8 N` |
| `pendulum_real_ws` | `6 N / 8 N` | `6 N / 8 N` |
| `pendulum_pid_ws` | `4 N / 6 N` | `6 N / 8 N` |

Dashboard sekarang menunggu balance yang benar-benar dekat tengah sebelum uji
impulse: mode harus `BALANCE`, sudut dan theta-dot kecil, dan cart harus berada
dalam batas `--external-stable-cart-cm` dari tengah. Kalau uji impulse
dibatalkan, itu berarti state belum cukup stabil untuk diklaim sebagai uji
gangguan yang bisa dipertanggungjawabkan.

## Verifikasi cepat

Gunakan ini sebelum tuning:

```bash
source /opt/ros/jazzy/setup.bash

python3 -m py_compile main.py
python3 -m py_compile lqr-pendulum/src/linear_inverted_pendulum_sim/linear_inverted_pendulum_sim/sim_serial_bridge.py
python3 -m py_compile pendulum_real_ws/src/linear_inverted_pendulum_real_sim/linear_inverted_pendulum_real_sim/real_serial_bridge.py
python3 -m py_compile pendulum_pid_ws/src/linear_inverted_pendulum_pid_sim/linear_inverted_pendulum_pid_sim/pid_serial_bridge.py

check_urdf <(xacro lqr-pendulum/src/linear_inverted_pendulum_sim/urdf/linear_inverted_pendulum.urdf.xacro)
check_urdf <(xacro pendulum_real_ws/src/linear_inverted_pendulum_real_sim/urdf/linear_inverted_pendulum_real.urdf.xacro)
check_urdf <(xacro pendulum_pid_ws/src/linear_inverted_pendulum_pid_sim/urdf/linear_inverted_pendulum_pid.urdf.xacro)
```

## Cara memilih workspace

Pilih `lqr-pendulum` kalau tujuan utamanya menguji balance berbasis LQR.
Klaim yang bisa dipertanggungjawabkan adalah simulasi model-based: state
feedback LQR dihitung dari model linear, gaya tetap dibatasi, dan hasilnya
dibaca sebagai effort joint Gazebo. Jangan klaim sebagai gaya motor fisik
sebelum dibandingkan dengan log alat asli.

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
