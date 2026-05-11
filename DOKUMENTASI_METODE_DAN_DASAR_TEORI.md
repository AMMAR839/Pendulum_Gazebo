# Dokumentasi Metode dan Dasar Teori Linear Inverted Pendulum

Dokumen ini menjelaskan dasar teori, metode kontrol, dan perbedaan implementasi
pada semua workspace linear inverted pendulum di repository:

```text
/home/ammar/Documents/Pendulum
```

Workspace yang dibahas:

| Workspace | Package | Fokus |
| --- | --- | --- |
| `lqr-pendulum` | `linear_inverted_pendulum_sim` | Simulasi awal/eksperimen ROS 2 + Gazebo. |
| `pendulum_real_ws` | `linear_inverted_pendulum_real_sim` | Baseline Manual Book dengan model motor lebih realistis. |
| `pendulum_pid_ws` | `linear_inverted_pendulum_pid_sim` | Turunan real-style dengan balance controller PID. |

Catatan sumber: dokumen ini mengikuti implementasi aktif pada file bridge di
masing-masing workspace. Jika ada README lama yang berbeda, gunakan file bridge
sebagai acuan perilaku runtime.

## 1. Tujuan Sistem

Sistem ini dibuat untuk mensimulasikan trainer linear inverted pendulum
menggunakan ROS 2 Jazzy dan Gazebo Harmonic, tetapi tetap mempertahankan GUI
Python lama sebagai antarmuka pengguna. Dengan begitu, tombol, gain, grafik, dan
status yang sebelumnya dipakai untuk hardware tetap bisa dipakai pada simulasi.

Tujuan utamanya:

1. Memodelkan cart-pendulum pada Gazebo.
2. Menjaga alur GUI lama melalui pseudo serial.
3. Menguji beberapa metode kontrol: PD, energy-based swing-up, LQR,
   full-state feedback Manual-style, dan PID.
4. Membandingkan simulasi LQR, simulasi Manual Book, dan simulasi PID.

## 2. Arsitektur Sistem

Semua workspace memakai arsitektur umum yang sama:

```text
main.py GUI
  -> pseudo serial /tmp/pendulum_*_serial
  -> ROS 2 serial bridge
  -> /pendulum/cart_force_cmd
  -> /pendulum/hinge_assist_force_cmd
  -> ros_gz_bridge
  -> Gazebo Harmonic model
  -> /joint_states
  -> serial bridge
  -> /pendulum/sim_state
  -> status packet balik ke GUI
```

Komponen utama:

| Komponen | Fungsi |
| --- | --- |
| `main.py` | GUI lama, tombol A/B/X/Y, input gain, grafik, status. |
| `lib_com.py` | Format packet serial antara GUI dan bridge. |
| `*_serial_bridge.py` | Controller, parser serial, publisher ROS 2, pembentuk status GUI. |
| `ros_gz_bridge` | Penghubung topic ROS 2 dengan Gazebo Transport. |
| URDF/Xacro | Model fisik cart, rail, dan pendulum. |

Pseudo serial membuat simulasi terlihat seperti perangkat serial STM32 dari
sisi GUI. Karena itu GUI tetap dijalankan dengan `PENDULUM_PORT` sesuai
workspace.

## 3. Model Fisik Linear Inverted Pendulum

Linear inverted pendulum terdiri dari cart yang bergerak translasi pada rail dan
batang pendulum yang berputar pada engsel di atas cart.

Data fisik yang dipakai mengikuti Manual Book LIP01:

| Parameter | Nilai |
| --- | --- |
| Panjang base | 1000 mm |
| Lebar base | 350 mm |
| Travel rail kontrol | 78 cm |
| Panjang pendulum | 400 mm |
| Diameter batang pendulum | 8 mm |
| Massa pendulum | 200 g |
| Center of mass pendulum | sekitar 0.20 m dari engsel |

Koordinat penting:

| Simbol | Makna |
| --- | --- |
| `x` | Posisi cart terhadap tengah rail. |
| `x_dot` | Kecepatan cart. |
| `theta_top` | Sudut pendulum terhadap posisi tegak atas. |
| `theta_dot` | Kecepatan sudut pendulum. |

Konvensi sudut pada simulasi:

```text
theta_top = 0 deg      -> pendulum tegak di atas
theta_top = +/-180 deg -> pendulum menggantung di bawah
```

## 4. Dasar Teori Inverted Pendulum

Inverted pendulum adalah sistem tidak stabil secara alami. Jika pendulum berada
di atas, sedikit gangguan akan membuatnya jatuh jika tidak dikoreksi oleh gerak
cart. Karena itu kontrol dibagi menjadi dua masalah:

1. Swing-up: menaikkan pendulum dari bawah ke dekat posisi atas.
2. Balance: menahan pendulum agar tetap tegak setelah berada dekat atas.

Pembagian ini penting karena satu controller tidak cocok untuk semua kondisi.
Saat pendulum masih di bawah, controller balance tidak efektif karena posisi
pendulum jauh dari titik linear. Saat pendulum sudah dekat atas, controller
swing-up harus berhenti dan balance harus mengambil alih.

## 5. Mode Operasi

Mode yang dikirim ke GUI melalui `/pendulum/sim_state`:

| Mode | Nama | Fungsi |
| --- | --- | --- |
| `1` | `WAITING` | Bridge aktif, sistem belum mulai kontrol. |
| `2` | `HOMING` | Cart bergerak ke tengah rail. |
| `3` | `READY` | Cart siap menerima perintah manual atau swing-up. |
| `4` | `SINE` | Gerak sinusoidal untuk pengujian cart. |
| `5` | `FINISH` | Stop. |
| `6` | `SWING_UP` | Membangun energi pendulum agar naik ke atas. |
| `7` | `BALANCE` | Menahan pendulum agar tetap tegak. |

Urutan operasi normal:

1. Jalankan Gazebo dari workspace yang dipilih.
2. Jalankan GUI dengan `PENDULUM_PORT` sesuai workspace.
3. Klik `Apply Gains`.
4. Klik `START`.
5. Klik `Y` untuk homing.
6. Klik `X` untuk swing-up.
7. Biarkan sistem masuk balance otomatis saat syarat capture terpenuhi.
8. Klik `A` hanya jika ingin meminta balance saat pendulum sudah dekat tegak.
9. Klik `B` untuk stop.

Pada `pendulum_real_ws` dan `pendulum_pid_ws`, tombol `A` saat masih
`SWING_UP` tidak langsung memaksa balance. Request disimpan sebagai
`balance_request_pending`, lalu baru dijalankan saat syarat energi dan capture
sudah aman.

## 6. Metode Homing: PD Control

Homing dipakai untuk membawa cart ke tengah rail sebelum swing-up. Metode yang
dipakai adalah PD sederhana:

```text
u = Kp * error - Kd * x_dot
```

Implementasi umum:

```python
error = x_center - cart_x
command = 2.8 * error - 0.35 * cart_v
```

Makna:

| Bagian | Fungsi |
| --- | --- |
| `2.8 * error` | Menarik cart menuju titik tengah. |
| `-0.35 * cart_v` | Meredam gerakan agar tidak overshoot. |

Homing bukan controller utama untuk menegakkan pendulum. Fungsinya hanya
menyiapkan posisi awal agar swing-up tidak langsung dimulai dari ujung rail.

## 7. Metode Swing-Up Berbasis Energi

Swing-up bertujuan menaikkan pendulum dari posisi bawah ke posisi atas dengan
menambah energi sistem. Dasar teorinya adalah energi mekanik pendulum:

```text
E = E_potensial + E_kinetik
```

Energi potensial:

```text
E_p = m * g * l * (1 + cos(theta_top))
```

Energi kinetik:

```text
E_k = 0.5 * m * l^2 * theta_dot^2
```

Target energi untuk mencapai posisi atas:

```text
E_target = 2 * m * g * l
```

Controller menghitung selisih energi:

```text
energy_deficit = E_target - E
```

Lalu gaya cart dihitung dari arah gerak pendulum:

```text
phase = theta_dot * cos(theta_top)
force = K_swing * energy_deficit * phase
```

Pada implementasi real-style dan PID:

```python
force = 52.0 * swing_gain * energy_deficit * phase
force -= 34.0 * swing_centering_gain * cart_x
force -= 14.0 * swing_damping_gain * cart_v
```

Makna tiap komponen:

| Komponen | Fungsi |
| --- | --- |
| `energy_deficit * phase` | Menentukan kapan cart menambah atau mengurangi energi. |
| `cart_x` term | Menjaga cart agar tidak menjauh dari tengah rail. |
| `cart_v` term | Meredam kecepatan cart. |
| `swing_force_limit_n` | Membatasi gaya cart swing-up per langkah kontrol. |

`swing_force_limit_n` bukan total akumulasi gaya. Nilai ini adalah clamp gaya
sesaat, misalnya `145 N` berarti force swing-up dibatasi di antara `-145 N`
sampai `+145 N` pada tiap update controller.

Saat pendulum masih diam di bawah, hukum energi belum punya arah yang jelas.
Karena itu bridge memberi kick awal kecil agar pendulum mulai bergerak.

## 8. Transisi Swing-Up ke Balance

Balance tidak boleh aktif terlalu awal. Jika balance mengambil alih saat
pendulum masih terlalu jauh dari tegak atau masih terlalu cepat, controller akan
meminta gaya besar dan pendulum mudah jatuh.

Syarat capture umum:

```text
abs(theta_top) < balance_capture_deg
abs(theta_dot) < balance_capture_rate
abs(cart_x) < balance_capture_cart_pos
abs(cart_v) < balance_capture_cart_vel
```

Pada workspace real-style dan PID, syarat tambahan dipakai agar pendulum
diayun dulu sebelum masuk balance:

```text
swing_top_passes >= swing_min_top_passes_before_catch
atau waktu SWING_UP >= swing_min_energy_build_time_s
energy / target_energy >= swing_energy_ready_ratio
```

Tujuannya:

1. Pendulum tidak ditangkap pada ayunan pertama yang masih kasar.
2. Energi pendulum cukup untuk mendekati posisi atas.
3. Kecepatan sudut sudah cukup rendah agar balance bisa menahan.
4. Cart masih berada di area rail yang aman.

### 8.1 Cara Sistem Memutuskan Sudah Bisa Balance

Pada saat mode `SWING_UP`, bridge terus membaca state dari Gazebo dan
menghitung apakah pendulum sudah layak ditangkap oleh controller balance.
Keputusan ini tidak hanya melihat sudut pendulum, tetapi juga melihat kecepatan
pendulum dan posisi cart.

Urutan logikanya:

1. Gazebo mengirim posisi dan kecepatan joint melalui `/joint_states`.
2. Bridge mengambil posisi cart dari joint `cart_slider`.
3. Bridge mengambil sudut pendulum dari joint `pendulum_hinge`.
4. Sudut pendulum diubah menjadi `theta_top`, yaitu sudut terhadap posisi tegak
   atas.
5. Jika mode masih `SWING_UP`, bridge menghitung energi dan memberi gaya ayunan.
6. Jika pendulum sudah dekat atas, bridge mengecek syarat capture.
7. Jika semua syarat aman, mode berpindah ke `BALANCE`.
8. Jika belum aman, sistem tetap di `SWING_UP` dan melanjutkan ayunan.

Logika ini penting karena pendulum bisa saja lewat dekat posisi tegak, tetapi
kecepatannya masih terlalu besar. Pada kondisi seperti itu, balance belum boleh
aktif karena gaya cart yang dibutuhkan akan sangat besar dan pendulum mudah
jatuh lagi.

### 8.2 Apa yang Terjadi Saat Mode BALANCE

Saat mode `BALANCE`, target controller bukan lagi menambah energi. Targetnya
berubah menjadi menjaga state tetap kecil:

```text
theta_top  -> mendekati 0
theta_dot  -> mendekati 0
x          -> tetap di area rail aman
x_dot      -> tidak terlalu cepat
```

Jika pendulum mulai miring ke kanan atau kiri, controller menggerakkan cart agar
titik tumpu pendulum ikut bergeser. Gerakan cart ini menghasilkan percepatan
yang membantu mengembalikan pendulum ke posisi tegak.

Prinsip sederhananya:

```text
pendulum miring -> cart digerakkan untuk mengejar arah jatuh
pendulum terlalu cepat -> cart memberi damping agar gerak melambat
cart terlalu jauh dari tengah -> controller menarik cart kembali ke area aman
```

Dengan kata lain, balance bukan membuat cart diam. Cart justru harus bergerak
kecil dan cepat untuk menjaga pendulum tetap berdiri.

### 8.3 Kenapa Balance Bisa Gagal

Balance dianggap gagal jika state keluar dari batas aman. Contoh kondisi gagal:

1. `abs(theta_top)` terlalu besar, artinya pendulum sudah terlalu miring.
2. Cart terlalu dekat ujung rail.
3. Kecepatan pendulum terlalu besar saat masuk balance.
4. Gaya cart dibatasi oleh `balance_force_limit_n` atau `effort_limit_n`.
5. Model motor terlalu lambat merespons karena time constant.

Jika kondisi gagal terdeteksi, bridge mengembalikan mode ke `SWING_UP`. Karena
itu pada runtime bisa terlihat pola:

```text
SWING_UP -> BALANCE -> SWING_UP
```

Pola tersebut berarti pendulum sudah bisa mencapai atas, tetapi belum bisa
ditahan stabil.

## 9. Metode Balance Full-State Feedback

Full-state feedback memakai beberapa state sistem sekaligus:

```text
state = [x, x_dot, theta, theta_dot]
```

Dasar teorinya:

```text
u = -(Kx*x + Kxd*x_dot + Ktheta*theta + Kthetad*theta_dot)
```

Pada Manual Book, bentuknya ditulis sebagai gabungan kontribusi sudut,
kecepatan sudut, error posisi cart, dan kecepatan cart. Implementasi
`pendulum_real_ws` mengikuti bentuk ini, dengan tambahan integral kecil posisi
cart untuk membantu mengurangi offset.

Implementasi praktis:

```python
command = (
    K_TH * theta_top
    + K_TH_D * theta_dot
    - K_X * x_error
    - K_X_D * cart_v
    - K_X_INT * x_integral
)
```

Makna gain:

| Gain GUI | Fungsi pada full-state feedback |
| --- | --- |
| `K_TH` | Koreksi sudut pendulum. |
| `K_TH_D` | Damping kecepatan sudut. |
| `K_X` | Koreksi posisi cart. |
| `K_X_D` | Damping kecepatan cart. |
| `K_X_INT` | Integral kecil posisi cart. |

Istilah yang paling tepat untuk `pendulum_real_ws` adalah full-state feedback
Manual-style, bukan PID murni. Alasannya, controller memakai beberapa state
sistem sekaligus, bukan hanya error sudut dengan P, I, dan D.

### 9.1 Cara Full-State Feedback Menyeimbangkan Pendulum

Pada `pendulum_real_ws`, controller balance membaca empat state utama:

```text
theta_top, theta_dot, x_error, cart_v
```

Setiap state memberi kontribusi koreksi:

| State | Jika nilainya besar | Aksi controller |
| --- | --- | --- |
| `theta_top` | Pendulum miring dari posisi tegak | Cart diberi gaya untuk mengejar arah jatuh pendulum. |
| `theta_dot` | Pendulum bergerak terlalu cepat | Gaya cart dipakai sebagai damping. |
| `x_error` | Cart jauh dari titik balance | Cart ditarik kembali ke area aman. |
| `cart_v` | Cart terlalu cepat | Gerak cart diredam. |
| `x_integral` | Cart punya offset lama | Offset posisi dikurangi perlahan. |

Implementasi di bridge mengubah koreksi state tersebut menjadi gaya cart.
Walaupun nama fungsi di kode dapat memakai istilah `upright_pid`, pada
`pendulum_real_ws` makna kontrolnya tetap Manual-style karena state yang
dipakai bukan hanya error sudut, tetapi juga posisi dan kecepatan cart.

Alur balance full-state feedback:

```text
joint_states
  -> hitung theta_top, theta_dot, x_error, cart_v
  -> hitung force berdasarkan gain K_TH, K_TH_D, K_X, K_X_D, K_X_INT
  -> batasi force dengan balance_force_limit_n
  -> kirim force ke /pendulum/cart_force_cmd
  -> Gazebo memberi gaya ke joint cart_slider
```

## 10. Metode LQR

LQR adalah metode state feedback yang menghitung gain dari model linear sistem.
Dasar teorinya:

```text
u = -Kx
```

Gain `K` dihitung dengan memilih bobot:

```text
Q = bobot state
R = bobot usaha kontrol
```

Secara konsep:

1. `Q` besar pada sudut pendulum membuat controller lebih serius menjaga sudut.
2. `Q` besar pada posisi cart membuat controller lebih serius menjaga cart di tengah.
3. `R` besar membuat controller lebih hemat gaya.
4. `R` kecil membuat controller lebih agresif.

Pada checkout aktif sekarang, workspace `lqr-pendulum` memakai LQR sebagai
controller balance utama. Parameter `balance_use_lqr` bernilai `True` secara
default, gain `K` dihitung dari model linear inverted pendulum dengan
`solve_continuous_are`, dan fallback PID-like hanya dipakai jika LQR sengaja
dimatikan untuk debugging.

## 11. Metode Balance PID

PID memakai tiga komponen:

```text
u = Kp*e + Ki*integral(e) + Kd*de/dt
```

Pada `pendulum_pid_ws`, error utama adalah sudut pendulum terhadap posisi tegak:

```text
e = theta_top
```

Rumus inti:

```python
theta_integral = clamp(theta_integral + theta_top * dt, -0.18, 0.18)

force_pid = (
    -Kp_theta * theta_top
    -Kd_theta * theta_dot
    -Ki_theta * theta_integral
)
```

Karena cart tetap harus berada di rail, PID sudut ditambah PD cart-centering:

```python
force_cart = -Kp_cart * x_error - Kd_cart * cart_v
force = force_pid + center_scale * force_cart
```

Mapping gain GUI pada `pendulum_pid_ws`:

| Gain GUI | Makna pada PID workspace |
| --- | --- |
| `K_TH` | P sudut pendulum. |
| `K_TH_D` | D sudut pendulum. |
| `K_X_INT` | I sudut pendulum. |
| `K_X` | P posisi cart. |
| `K_X_D` | D kecepatan cart. |

PID workspace sengaja dibuat terpisah agar perbandingan metode tidak tercampur
dengan full-state feedback pada `pendulum_real_ws`.

### 11.1 Cara PID Menyeimbangkan Pendulum

Pada `pendulum_pid_ws`, yang dijadikan error utama adalah sudut pendulum:

```text
error sudut = theta_top
```

Controller PID bekerja seperti ini:

| Komponen | Sumber | Fungsi |
| --- | --- | --- |
| P | `theta_top` | Mengoreksi besar kemiringan pendulum. |
| I | integral `theta_top` | Mengurangi error sudut yang tersisa lama. |
| D | `theta_dot` | Meredam gerakan pendulum agar tidak overshoot. |

Namun PID sudut saja tidak cukup karena cart bisa lari ke ujung rail. Karena
itu implementasi menambahkan PD cart-centering:

| Komponen tambahan | Fungsi |
| --- | --- |
| `K_X * x_error` | Menarik cart kembali ke area tengah. |
| `K_X_D * cart_v` | Meredam kecepatan cart. |

Alur balance PID:

```text
joint_states
  -> hitung theta_top dan theta_dot
  -> update theta_integral
  -> hitung force_pid dari P, I, D sudut
  -> tambah force_cart agar cart tidak keluar rail
  -> batasi force dengan balance_force_limit_n
  -> kirim force ke /pendulum/cart_force_cmd
```

Perbedaan paling jelas:

```text
pendulum_real_ws -> integral dipakai untuk posisi cart
pendulum_pid_ws  -> integral dipakai untuk sudut pendulum
```

## 12. Model Motor Real-Style

Pada `pendulum_real_ws` dan `pendulum_pid_ws`, command controller tidak langsung
dianggap sebagai gaya ideal. Bridge menambahkan model motor yang lebih dekat ke
Manual Book:

```text
motor_pwm = motor_pwm_deadband + motor_pwm_per_cmps * abs(command_cmps)
```

Respons motor dibuat seperti sistem orde satu:

```text
alpha = 1 - exp(-dt / motor_time_constant_s)
motor_velocity = motor_velocity + alpha * (target_velocity - motor_velocity)
```

Parameter utama:

| Parameter | Nilai |
| --- | --- |
| `motor_pwm_deadband` | `3212` |
| `motor_pwm_per_cmps` | `189.1` |
| `motor_time_constant_s` | `0.40 s` |
| `effort_limit_n` | `150 N` pada real-style/PID |

Makna:

1. Deadband membuat motor tidak langsung bergerak pada command kecil.
2. Time constant membuat respons motor tidak instan.
3. Effort limit membatasi gaya agar tidak menjadi simulasi ideal yang terlalu kuat.

## 13. Implementasi Metode Balance di Gazebo

Bagian ini menjelaskan bagaimana hasil perhitungan controller benar-benar
masuk ke simulasi Gazebo.

### 13.1 Joint yang Dipakai

Model Gazebo memakai dua joint utama:

| Joint | Tipe | Fungsi |
| --- | --- | --- |
| `cart_slider` | prismatic | Cart bergerak maju-mundur sepanjang rail. |
| `pendulum_hinge` | continuous | Pendulum berputar pada engsel di atas cart. |

`cart_slider` adalah aktuator utama. Semua metode balance pada dasarnya
menghasilkan gaya untuk joint ini.

`pendulum_hinge` dipakai untuk membaca sudut pendulum. Pada konfigurasi
simulasi sekarang, joint ini juga dapat menerima torsi assist kecil khusus
Gazebo.

### 13.2 Plugin Gazebo yang Dipakai

URDF/Xacro memasang dua jenis plugin Gazebo:

| Plugin | Fungsi |
| --- | --- |
| `JointStatePublisher` | Mengirim state `cart_slider` dan `pendulum_hinge` dari Gazebo. |
| `ApplyJointForce` pada `cart_slider` | Menerima gaya cart dari controller. |
| `ApplyJointForce` pada `pendulum_hinge` | Menerima torsi assist simulasi. |

Dengan plugin ini, Gazebo dapat menjadi plant fisik: controller memberi gaya,
lalu Gazebo menghitung gerak cart dan pendulum berdasarkan dinamika model.

### 13.3 Bridge Topic ROS 2 ke Gazebo

Launch file membuat bridge dari ROS 2 ke Gazebo:

| ROS 2 topic | Gazebo topic | Fungsi |
| --- | --- | --- |
| `/joint_states` | `/world/empty/model/linear_inverted_pendulum/joint_state` | Membaca posisi dan kecepatan joint. |
| `/pendulum/cart_force_cmd` | `/model/linear_inverted_pendulum/joint/cart_slider/cmd_force` | Mengirim gaya cart. |
| `/pendulum/hinge_assist_force_cmd` | `/model/linear_inverted_pendulum/joint/pendulum_hinge/cmd_force` | Mengirim torsi assist engsel. |

Jalur utama balancing adalah:

```text
controller force
  -> /pendulum/cart_force_cmd
  -> ros_gz_bridge
  -> /model/linear_inverted_pendulum/joint/cart_slider/cmd_force
  -> ApplyJointForce
  -> cart bergerak di Gazebo
```

### 13.4 Siklus Kontrol Runtime

Saat simulasi berjalan, bridge melakukan siklus berikut berulang-ulang:

1. Menerima `/joint_states` dari Gazebo.
2. Mengambil posisi cart `x` dan kecepatan cart `x_dot`.
3. Mengambil sudut pendulum dan kecepatan sudut.
4. Menghitung `theta_top = wrap_pi(pendulum_down_rad - pi)`.
5. Menentukan mode: `HOMING`, `SWING_UP`, `BALANCE`, atau mode lain.
6. Menghitung command atau force berdasarkan mode.
7. Membatasi command agar tidak melewati rail dan force limit.
8. Mengirim gaya cart ke `/pendulum/cart_force_cmd`.
9. Mengirim torsi assist ke `/pendulum/hinge_assist_force_cmd` jika aktif.
10. Membentuk `/pendulum/sim_state` dan status packet untuk GUI.

Ringkasnya:

```text
Gazebo joint state -> bridge hitung controller -> bridge kirim force -> Gazebo bergerak
```

### 13.5 Kenapa Memakai Force, Bukan Langsung Posisi

Controller tidak langsung mengatur posisi cart. Controller mengirim gaya ke
joint `cart_slider`, lalu Gazebo menghitung respons fisiknya. Ini lebih sesuai
dengan sistem nyata karena motor tidak bisa memindahkan cart langsung ke posisi
tertentu tanpa dinamika.

Pada mode tertentu, bridge menghitung command kecepatan terlebih dahulu lalu
mengubahnya menjadi gaya melalui velocity servo. Pada mode swing-up, catch, dan
balance, bridge biasanya langsung memakai `effort_override`, yaitu force yang
dikirim ke Gazebo.

### 13.6 Rail Guard dan Force Limit

Karena rail terbatas, controller tidak boleh terus mendorong cart keluar dari
lintasan. Karena itu bridge memakai dua pelindung:

| Pelindung | Fungsi |
| --- | --- |
| rail guard | Mengurangi atau membalik command jika cart mendekati ujung rail. |
| force limit | Membatasi gaya maksimum agar tidak terlalu ideal. |

Contoh batas gaya real-style terbaru setelah tuning hardware-logical:

```text
swing_force_limit_n   = 70 N
catch_force_limit_n   = 65 N
balance_force_limit_n = 60 N
effort_limit_n        = 150 N
```

Maknanya:

1. Swing-up boleh memakai gaya lebih besar karena perlu membangun energi.
2. Catch memakai gaya lebih kecil agar transisi tidak kasar.
3. Balance memakai gaya lebih kecil karena seharusnya hanya menjaga pendulum
   dekat tegak.
4. Effort limit adalah batas akhir sebelum gaya dikirim ke Gazebo.

### 13.7 Hubungan GUI dengan Implementasi Gazebo

GUI tidak langsung berbicara dengan Gazebo. GUI hanya mengirim packet serial
berisi tombol dan gain. Bridge menerjemahkan packet tersebut menjadi mode dan
parameter controller.

Contoh hubungan tombol:

| Tombol GUI | Efek di bridge | Efek di Gazebo |
| --- | --- | --- |
| `Y` | Mode `HOMING` | Cart diberi gaya/command menuju tengah rail. |
| `X` | Mode `SWING_UP` | Cart bergerak bolak-balik untuk menambah energi pendulum. |
| `A` | Request/Mode `BALANCE` | Cart diberi gaya kecil cepat untuk menahan pendulum tegak. |
| `B` | Mode `FINISH` | Force dihentikan. |

Jadi yang terlihat di Gazebo adalah hasil akhir dari mode dan gain yang dikirim
oleh GUI.

## 14. Assist Engsel Khusus Simulasi

Beberapa workspace memakai `/pendulum/hinge_assist_force_cmd`, yaitu torsi
kecil langsung pada engsel pendulum. Ini bukan aktuator tambahan pada alat asli.
Fungsinya hanya membantu simulasi Gazebo agar pendulum bisa ditahan stabil saat
fase catch atau balance.

Rumus assist:

```text
torque = -Kp_assist * theta_top - Kd_assist * theta_dot
```

Assist hanya aktif jika:

1. `balance_assist_enabled=True`.
2. Mode berada pada `SWING_UP` atau `BALANCE`.
3. Sudut pendulum sudah masuk area assist.

Parameter real-style dan PID:

| Parameter | Nilai |
| --- | --- |
| `balance_assist_angle_deg` | `115.0` |
| `balance_assist_kp_nm_per_rad` | `3.4` |
| `balance_assist_kd_nm_per_rad_s` | `2.4` |
| `balance_assist_torque_limit_nm` | `4.5` |

Untuk laporan, tulis secara jelas bahwa hasil tegak stabil di simulasi masih
memakai bantuan simulasi. Jangan ditulis seolah-olah hardware asli memiliki
aktuator engsel tambahan.

## 15. Perbedaan Tiap Workspace

### 15.1 `lqr-pendulum`

Path:

```text
lqr-pendulum/src/linear_inverted_pendulum_sim
```

Serial GUI:

```text
/tmp/pendulum_lqr_serial
```

Peran:

1. Workspace awal untuk integrasi ROS 2 + Gazebo.
2. Dipakai untuk menguji pseudo serial, topic, status GUI, dan model Gazebo.
3. Memakai parameter LQR dan state-feedback sebagai metode balance aktif.

Kondisi aktif pada checkout sekarang:

| Bagian | Metode |
| --- | --- |
| Homing | PD |
| Swing-up | Energy-based |
| Capture | readiness gate |
| Balance aktif | LQR state feedback, karena `balance_use_lqr=True` |
| LQR | controller utama saat mode `BALANCE` |
| Assist | aktif default |

Catatan laporan: `lqr-pendulum` boleh disebut sebagai workspace LQR karena
jalur balance aktif memanggil state feedback `u = -Kx`. Batas klaimnya tetap
simulasi Gazebo: force yang dilaporkan adalah effort joint, bukan gaya motor
fisik aktual.

Tuning terbaru untuk LQR menekankan balance di tengah rail, bukan hanya sudut
pendulum tegak. Capture dibuat lebih ketat supaya `BALANCE` hanya aktif saat
cart sudah dekat tengah (`balance_capture_cart_pos_m = 0.035` dan
`balance_auto_lock_cart_pos_m = 0.03`) serta kecepatan sudut sudah rendah
(`balance_capture_rate_rad_s = 0.35`). Bobot LQR tetap model-based dan
konservatif (`lqr_q_x = 160`, `lqr_q_x_dot = 30`, `lqr_q_theta = 1000`,
`lqr_q_theta_dot = 600`) supaya cart kembali ke tengah tanpa force berosilasi
kasar. Force LQR pada mode `BALANCE` juga diskalakan dengan
`balance_lqr_force_scale = 0.25` agar koreksi cart lebih halus. Dengan demikian
klaim yang aman adalah "LQR menjaga pendulum tegak sambil meregulasi cart ke
tengah", bukan "cart diam total".
Saat mode `BALANCE` baru aktif, referensi LQR diambil dari posisi cart saat
tertangkap lalu digeser pelan-pelan ke `0 cm`, sehingga tidak ada hentakan
besar yang membuat cart berosilasi.

Validasi headless terbaru disimpan pada
`data_exports/lqr_center_balance_validation_scaled_20260511.csv`. Pada run ini
mode berpindah `SWING_UP -> BALANCE` sekitar `8.36 s`, fase `BALANCE` bertahan
sekitar `28.10 s`, rata-rata absolut sudut balance `0.0256 deg`, dan rata-rata
absolut posisi cart balance `0.1534 cm`. Setelah 50 sampel awal transien,
rata-rata absolut posisi cart turun menjadi `0.0810 cm` dengan maksimum
`0.6399 cm`. Angka tersebut boleh dipakai sebagai bukti simulasi bahwa LQR
tidak hanya menahan sudut, tetapi juga mengembalikan cart ke tengah rail.

### 15.2 `pendulum_real_ws`

Path:

```text
pendulum_real_ws/src/linear_inverted_pendulum_real_sim
```

Serial GUI:

```text
/tmp/pendulum_real_serial
```

Peran:

1. Workspace baseline yang paling dekat ke Manual Book.
2. Memakai dimensi, batas rail, dan model motor real-style.
3. Swing-up dibuat lebih hati-hati dengan multi-swing readiness gate.
4. Balance memakai full-state feedback Manual-style.

Metode:

| Bagian | Metode |
| --- | --- |
| Homing | PD |
| Swing-up | Energy-based |
| Capture | sudut, theta-dot, posisi cart, kecepatan cart, dan readiness gate |
| Balance | full-state feedback Manual-style |
| Motor | deadband PWM + time constant |
| Assist | aktif default sebagai bantuan simulasi |

Parameter penting:

| Parameter | Nilai |
| --- | --- |
| `swing_force_limit_n` | `145.0 N` |
| `catch_force_limit_n` | `95.0 N` |
| `balance_force_limit_n` | `45.0 N` |
| `effort_limit_n` | `150.0 N` |
| `balance_capture_deg` | `9.0 deg` |
| `balance_capture_rate_rad_s` | `1.0 rad/s` |

### 15.3 `pendulum_pid_ws`

Path:

```text
pendulum_pid_ws/src/linear_inverted_pendulum_pid_sim
```

Serial GUI:

```text
/tmp/pendulum_pid_serial
```

Peran:

1. Workspace pembanding yang dibuat dari baseline `pendulum_real_ws`.
2. Model fisik, motor model, swing-up, capture, dan assist tetap real-style.
3. Balance controller diganti menjadi PID sudut pendulum.

Metode:

| Bagian | Metode |
| --- | --- |
| Homing | PD |
| Swing-up | Energy-based |
| Capture | sama seperti real-style |
| Balance | PID sudut + PD cart-centering |
| Motor | deadband PWM + time constant |
| Assist | aktif default sebagai bantuan simulasi |

Perbedaan utama dari `pendulum_real_ws`:

| Aspek | `pendulum_real_ws` | `pendulum_pid_ws` |
| --- | --- | --- |
| Balance | Full-state feedback | PID sudut |
| Integral | Integral posisi cart | Integral sudut pendulum |
| Tujuan | Mengikuti Manual Book | Membandingkan metode PID |
| File bridge | `real_serial_bridge.py` | `pid_serial_bridge.py` |

## 16. Perbandingan Ringkas Metode

| Metode | Dipakai di | Tujuan | Kelebihan | Kekurangan |
| --- | --- | --- | --- | --- |
| PD homing | Semua workspace | Cart ke tengah rail | Sederhana dan stabil untuk posisi awal | Tidak untuk balance pendulum |
| Energy swing-up | Semua workspace | Menaikkan pendulum ke atas | Cocok untuk kondisi jauh dari tegak | Perlu transisi/capture yang tepat |
| Capture gate | Semua workspace, paling ketat di real-style/PID | Mencegah balance terlalu awal | Mengurangi gaya besar saat balance | Jika terlalu ketat bisa sulit masuk balance |
| Full-state feedback | `pendulum_real_ws` | Menahan pendulum dengan beberapa state | Sesuai Manual-style | Perlu tanda dan gain yang tepat |
| LQR | Opsional/eksperimen terutama `lqr-pendulum` | State feedback berbasis optimisasi | Sistematis lewat Q/R | Butuh model linear yang cocok |
| PID sudut | `pendulum_pid_ws` | Pembanding metode PID | Mudah dijelaskan dan dituning | Sudut saja tidak cukup, perlu PD cart |
| Motor real-style | `pendulum_real_ws`, `pendulum_pid_ws` | Mendekati respons motor asli | Lebih realistis | Membuat balance lebih sulit |
| Hinge assist | Simulasi | Membantu stabilitas Gazebo | Demo lebih stabil | Bukan aktuator hardware asli |

## 17. Data Runtime Swing-Up dan Balance

Data berikut diambil dari tiga workspace dengan menjalankan Gazebo headless,
mengirim tombol `Y` untuk homing, `X` untuk swing-up, lalu request `A` saat
pendulum sudah dekat tegak. Sampling dilakukan dari `/pendulum/sim_state`,
`/pendulum/cart_velocity_cmd`, `/pendulum/cart_force_cmd`, dan
`/pendulum/hinge_assist_force_cmd`.

File data yang dibuat:

| File | Isi |
| --- | --- |
| `data_exports/runtime_swing_balance_samples_20260509_three_ws.csv` | Semua sampel runtime gabungan tiga workspace. |
| `data_exports/runtime_swing_balance_summary_20260509_three_ws.csv` | Ringkasan lengkap per workspace dan fase. |
| `data_exports/runtime_swing_balance_interpreted_summary_20260509_three_ws.csv` | Ringkasan aman untuk laporan dengan label bahwa force adalah effort joint Gazebo. |
| `data_exports/ros2_pendulum_ws_swing_balance_samples_20260509.csv` | Sampel historis workspace ini sebelum rename menjadi `lqr-pendulum`. |
| `data_exports/pendulum_real_ws_swing_balance_samples_20260509.csv` | Sampel khusus `pendulum_real_ws`. |
| `data_exports/pendulum_pid_ws_swing_balance_samples_20260509.csv` | Sampel khusus `pendulum_pid_ws`. |

Catatan penting: `cart_force_cmd_n` adalah effort joint Gazebo, bukan gaya motor
fisik aktual pada alat asli.

### 17.1 Ringkasan Data Swing-Up

| Workspace | Sampel | Durasi swing-up | Rata-rata abs sudut | Rata-rata abs theta-dot | Rata-rata energi | Rata-rata abs force cart |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `lqr-pendulum` | 133 | 6.749 s | 123.366 deg | 1.572 rad/s | 0.261 J | 49.044 N |
| `pendulum_real_ws` | 132 | 6.740 s | 127.752 deg | 1.634 rad/s | 0.245 J | 73.181 N |
| `pendulum_pid_ws` | 130 | 6.622 s | 114.801 deg | 1.550 rad/s | 0.299 J | 64.360 N |

Interpretasi:

1. Semua workspace berhasil masuk fase `SWING_UP` dan mencapai area atas.
2. `lqr-pendulum` memakai batas swing-up lebih rendah, yaitu sekitar
   `+/-90 N`, sehingga rata-rata effort swing-up lebih kecil.
3. `pendulum_real_ws` dan `pendulum_pid_ws` memakai batas swing-up real-style
   `+/-145 N`, sehingga rata-rata effort swing-up lebih besar.
4. Durasi swing-up ketiganya mirip, sekitar `6.6 s` sampai `6.8 s`, karena
   request balance diberikan setelah pendulum cukup dekat tegak.

### 17.2 Ringkasan Data Balance

| Workspace | Sampel | Durasi balance | Rata-rata abs sudut | Rata-rata abs theta-dot | Rata-rata energi | Rata-rata abs force cart |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `lqr-pendulum` | 456 | 23.434 s | 0.322 deg | 0.008 rad/s | 0.785 J | 0.302 N |
| `pendulum_real_ws` | 458 | 23.447 s | 0.324 deg | 0.007 rad/s | 0.785 J | 0.051 N |
| `pendulum_pid_ws` | 459 | 23.547 s | 0.195 deg | 0.006 rad/s | 0.785 J | 0.072 N |

Interpretasi:

1. Ketiga workspace berhasil masuk `BALANCE` dan menahan pendulum selama sekitar
   `23.4 s` pada run ini.
2. Saat balance, rata-rata sudut sangat kecil, kurang dari `0.4 deg`, artinya
   pendulum berada sangat dekat posisi tegak.
3. `pendulum_pid_ws` punya rata-rata error sudut paling kecil pada run ini,
   yaitu sekitar `0.195 deg`.
4. Effort balance rata-rata jauh lebih kecil daripada swing-up karena balance
   hanya menjaga pendulum dekat posisi atas, bukan lagi membangun energi besar.
5. Nilai effort balance kecil pada ringkasan ini terjadi setelah pendulum sudah
   stabil dekat tegak; effort maksimum sesaat tetap bisa lebih besar saat
   transisi.

### 17.3 Data Assist Engsel

| Workspace | Rata-rata abs assist saat swing-up | Rata-rata abs assist saat balance |
| --- | ---: | ---: |
| `lqr-pendulum` | 0.190 Nm | 0.012 Nm |
| `pendulum_real_ws` | 0.316 Nm | 0.005 Nm |
| `pendulum_pid_ws` | 0.256 Nm | 0.006 Nm |

Interpretasi:

1. Assist lebih terlihat saat swing-up/catch karena pendulum masih bergerak
   menuju area atas.
2. Saat balance sudah stabil, assist rata-rata menjadi sangat kecil.
3. Assist tetap harus disebut sebagai bantuan simulasi, bukan aktuator hardware
   asli.

### 17.4 Kesimpulan Data Runtime

Berdasarkan capture ini, semua workspace berhasil menjalankan dua fase utama:

```text
SWING_UP -> BALANCE
```

Untuk laporan, data ini bisa dipakai untuk menunjukkan bahwa:

1. Swing-up membutuhkan gaya lebih besar karena tujuannya menaikkan energi.
2. Balance membutuhkan gaya lebih kecil saat pendulum sudah dekat tegak.
3. `pendulum_real_ws` dan `pendulum_pid_ws` memakai model real-style yang sama,
   tetapi metode balance-nya berbeda.
4. `pendulum_pid_ws` pada run ini menghasilkan rata-rata error sudut balance
   paling kecil, tetapi kesimpulan akhir tetap perlu diuji dengan beberapa run
   jika ingin menjadi klaim performa final.

### 17.5 Patokan Awal Swing-Up untuk Alat Real

Data gaya swing-up dapat dijadikan patokan awal untuk alat real jika klaimnya
ditulis sebagai envelope berbasis model dan energi, bukan sebagai hasil ukur
gaya motor fisik. Dasarnya adalah:

1. Metode swing-up menghitung selisih energi pendulum terhadap energi target.
2. Runtime menunjukkan energi pendulum naik hingga mendekati energi posisi
   tegak sebelum balance mengambil alih.
3. Force limit dan command motor memberi batas kebutuhan actuator yang bisa
   dibandingkan dengan log alat real.
4. Validasi real tetap harus memakai data alat asli seperti sudut pendulum,
   posisi cart, kecepatan cart, PWM atau arus motor, dan keberhasilan masuk
   balance.

Setelah tuning multi-swing hardware-logical, ringkasan patokan swing-up dibuat
di:

```text
data_exports/multi_swing_real_reference_summary_20260510.csv
```

Ringkasan utama:

| Workspace | Durasi swing-up | Mean abs effort | P95/peak effort | Abs impulse | Positive cmd-work | Energy ready ratio | Patokan command real |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `lqr-pendulum` | 7.961 s | 5.290 N | 18.326 / 33.530 N | 42.248 N.s | 7.590 J | 1.022 | 7.394 / 25.508 / 47.057 cm/s |
| `pendulum_real_ws` | 7.957 s | 4.958 N | 16.516 / 30.956 N | 39.610 N.s | 6.215 J | 1.014 | 6.941 / 23.122 / 43.339 cm/s; PWM mean/P95 4272 / 7584 |
| `pendulum_pid_ws` | 7.950 s | 5.685 N | 17.246 / 28.169 N | 45.058 N.s | 7.260 J | 1.107 | 7.978 / 24.144 / 39.902 cm/s; PWM mean/P95 4549 / 7778 |

Interpretasi:

1. `energy_ready_ratio` sekitar `0.99` menunjukkan swing-up berhasil
   mengumpulkan energi sampai hampir sama dengan energi posisi tegak.
2. `pendulum_real_ws` dan `pendulum_pid_ws` menjadi patokan real-style utama
   karena memakai model deadband PWM dan time constant motor.
3. `lqr-pendulum` tetap berguna sebagai pembanding simulasi, tetapi patokan
   real yang lebih kuat adalah workspace real-style.
4. Capture sekarang ditahan lebih lama dengan gate multi-swing:
   `swing_min_top_passes_before_catch = 3`,
   `swing_min_energy_build_time_s = 8.0`, dan
   `swing_energy_ready_ratio = 0.88`.
5. Limit swing-up/catch tetap konservatif, yaitu sekitar `+/-70 N` untuk
   swing-up dan `+/-65 N` untuk catch, tetapi peak aktual pada validasi terbaru
   hanya sekitar `28 N` sampai `33 N`.
6. Angka N tetap disebut effort cart model Gazebo. Untuk mengubahnya menjadi
   gaya motor aktual, perlu kalibrasi actuator atau sensor gaya/arus pada alat
   asli.

Kalimat laporan yang aman:

```text
Gaya swing-up digunakan sebagai patokan awal actuator real melalui envelope
hasil simulasi. Controller berbasis energi menaikkan energi pendulum hingga
sekitar energi referensi posisi tegak. Setelah tuning multi-swing
hardware-logical, capture ditahan sampai swing-up membangun energi sekitar 8 s.
Run headless terbaru memiliki rata-rata effort swing-up sekitar 4.9 N sampai
5.7 N dengan peak sekitar 28 N sampai 34 N, sehingga proses ayunan lebih
bertahap dan lebih mudah dibandingkan dengan kemampuan actuator kecil. Karena
belum digunakan sensor gaya motor fisik, nilai force Gazebo tidak diklaim
sebagai gaya motor aktual, tetapi digunakan sebagai referensi command envelope
yang harus divalidasi terhadap log alat asli.
```

## 18. Hal yang Harus Dijelaskan di Laporan

Untuk laporan, struktur yang disarankan:

1. Pendahuluan:
   jelaskan linear inverted pendulum sebagai sistem tidak stabil dan alasan
   memakai ROS 2 + Gazebo.

2. Model sistem:
   jelaskan cart, rail, pendulum, joint `cart_slider`, joint `pendulum_hinge`,
   dimensi Manual Book, serta konvensi sudut.

3. Arsitektur perangkat lunak:
   jelaskan GUI Python lama, pseudo serial, ROS 2 bridge, Gazebo, dan topic
   utama.

4. Metode kontrol:
   jelaskan PD homing, energy swing-up, capture gate, full-state feedback, LQR,
   PID, motor model, implementasi Gazebo, dan assist simulasi.

5. Implementasi workspace:
   jelaskan perbedaan `lqr-pendulum`, `pendulum_real_ws`, dan
   `pendulum_pid_ws` dengan tabel.

6. Parameter dan tuning:
   masukkan nilai `swing_force_limit_n`, `catch_force_limit_n`,
   `balance_force_limit_n`, `balance_capture_deg`, `motor_pwm_deadband`,
   `motor_time_constant_s`, dan parameter assist.

7. Pembahasan:
   jelaskan kenapa swing-up dan balance dipisah, kenapa balance tidak boleh
   aktif terlalu awal, kenapa PID dibuat workspace terpisah, dan kenapa assist
   engsel harus disebut sebagai bantuan simulasi. Jelaskan juga bahwa gaya
   swing-up menjadi patokan awal real pada level energi dan command envelope,
   bukan klaim gaya motor fisik tanpa kalibrasi.

8. Kesimpulan:
   jelaskan workspace mana yang menjadi demo awal, baseline Manual Book, dan
   pembanding PID.

## 19. Contoh Narasi Laporan

Berikut contoh paragraf yang bisa langsung dipakai:

```text
Sistem linear inverted pendulum pada penelitian ini disimulasikan menggunakan
ROS 2 Jazzy dan Gazebo Harmonic. Simulasi tetap dihubungkan dengan GUI Python
lama melalui pseudo serial, sehingga GUI dapat mengirim tombol kontrol dan gain
seolah-olah terhubung ke hardware asli. Alur komunikasi dimulai dari main.py,
dilanjutkan ke serial bridge ROS 2, kemudian bridge mengirim gaya cart ke Gazebo
melalui topic /pendulum/cart_force_cmd. Status sistem dibaca kembali dari
/joint_states dan dikirim ke GUI melalui /pendulum/sim_state.
```

```text
Metode kontrol dibagi menjadi beberapa fase. Fase homing menggunakan kontrol PD
untuk membawa cart ke tengah rail. Fase swing-up menggunakan metode berbasis
energi, yaitu menghitung selisih energi pendulum terhadap energi target agar
pendulum dapat naik menuju posisi tegak. Setelah pendulum mendekati posisi
atas, sistem hanya berpindah ke balance jika sudut, kecepatan sudut, posisi
cart, dan kecepatan cart berada pada batas aman. Fase balance berbeda pada tiap
workspace: pendulum_real_ws memakai full-state feedback Manual-style, sedangkan
pendulum_pid_ws memakai PID sudut pendulum yang ditambah PD cart-centering.
```

```text
Tiga workspace dibuat agar metode dapat dibandingkan secara jelas.
lqr-pendulum digunakan sebagai workspace LQR untuk balance berbasis model,
pendulum_real_ws digunakan sebagai baseline yang lebih dekat dengan Manual Book
dan model motor real-style, sedangkan pendulum_pid_ws dibuat sebagai pembanding
yang mengganti metode balance menjadi PID. Dengan pemisahan ini, perubahan
metode PID tidak mengganggu baseline Manual Book.
```

```text
Pada simulasi Gazebo, assist torsi kecil pada engsel pendulum digunakan untuk
membantu stabilitas saat fase catch dan balance. Assist ini hanya bantuan
simulasi dan bukan aktuator tambahan pada trainer asli. Oleh karena itu hasil
tegak stabil pada simulasi harus dijelaskan sebagai hasil kontrol cart yang
dibantu oleh stabilisasi simulasi, bukan replika hardware murni tanpa bantuan.
```

## 20. Kesimpulan Metode

Kesimpulan utama:

1. Semua workspace memakai arsitektur komunikasi yang sama agar GUI lama tetap
   bisa digunakan.
2. Semua workspace memakai PD untuk homing dan energy-based controller untuk
   swing-up.
3. Perbedaan utama ada pada metode balance.
4. `pendulum_real_ws` adalah baseline Manual-style dengan full-state feedback.
5. `pendulum_pid_ws` adalah pembanding yang memakai PID sudut pendulum.
6. `lqr-pendulum` adalah workspace LQR dengan `balance_use_lqr=True` sebagai
   default dan tuning terbaru menahan capture sampai cart dekat tengah.
7. Motor model dan force limit real-style membuat simulasi lebih realistis
   tetapi lebih sulit dibanding model ideal.
8. Assist engsel perlu ditulis sebagai bantuan simulasi, bukan komponen hardware
   asli.
