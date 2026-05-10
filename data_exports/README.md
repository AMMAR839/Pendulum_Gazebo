# Data Export Notes

File di folder ini dipakai untuk membandingkan runtime tiga workspace:

- `ros2_pendulum_ws`: workspace demo yang lebih mudah dibuat tegak.
- `pendulum_real_ws`: workspace manual-book-style yang lebih dekat ke model alat, tetapi tetap memakai bantuan simulasi saat demo tegak.
- `pendulum_pid_ws`: turunan real-style untuk membandingkan balance PID.

## Kolom gaya

Kolom `cart_force_cmd_n` pada `runtime_swing_balance_samples.csv` adalah
**perintah effort langsung ke joint Gazebo**, bukan gaya motor aktual alat asli.
Nilai ini sengaja bisa besar karena dipakai untuk membuat simulasi mampu catch
dan balance:

- `ros2_pendulum_ws`: sampai sekitar `340 N` saat balance.
- `pendulum_real_ws`: sampai sekitar `150 N` saat balance.

Jadi angka tersebut valid untuk debug Gazebo, tetapi tidak boleh langsung
dimasukkan ke laporan sebagai gaya motor fisik.

## Kenapa tidak langsung dikecilkan?

Jika limit gaya langsung diturunkan tanpa mengubah model actuator, damping,
capture threshold, dan controller, pendulum memang tidak bisa tegak. Saat ini
tegak stabil dicapai dengan kombinasi:

- force command ke cart joint Gazebo,
- capture threshold yang cukup longgar/ketat sesuai workspace,
- `balance_assist` yaitu torsi kecil langsung ke engsel pendulum untuk bantuan
  simulasi.

Untuk laporan, gunakan `cart_force_cmd_n` atau `cart_joint_effort_cmd_n` sebagai
**sim joint effort**, lalu gunakan `setspeed_cm_s`, `cart_velocity_cmd_mps`,
`motor_command_cm_s`, dan ringkasan dari `interpret_runtime_exports.py` sebagai
data command motor yang lebih masuk akal.

## Membuat ringkasan yang lebih aman untuk laporan

Jalankan:

```bash
cd /home/ammar/Documents/Pendulum
python3 data_exports/interpret_runtime_exports.py \
  --samples data_exports/runtime_swing_balance_samples.csv \
  --output data_exports/runtime_swing_balance_interpreted_summary.csv
```

Output ini tetap menampilkan effort Gazebo, tetapi memberi label eksplisit
bahwa nilai itu adalah `sim_joint_effort`, bukan `real_motor_force`.

## Membuat patokan awal untuk alat real

Untuk menjadikan data swing-up sebagai patokan awal real, jangan hanya memakai
rata-rata force. Pakai envelope lengkap:

- durasi swing-up,
- mean/RMS/P95/peak effort cart model,
- impulse effort selama swing-up,
- estimasi kerja positif dari `force * cart_velocity_cmd_mps`,
- kenaikan energi pendulum sampai mendekati energi posisi tegak,
- command kecepatan cart dan PWM ekuivalen pada workspace real-style.

Jalankan:

```bash
cd /home/ammar/Documents/Pendulum
python3 data_exports/build_swing_up_real_reference.py \
  --samples data_exports/post_visual_balance_validation_20260510.csv \
  --output data_exports/swing_up_real_reference_summary_20260510.csv
```

Dokumen ringkas untuk laporan ada di:

```text
data_exports/SWING_UP_REAL_REFERENCE.md
```

Klaim yang aman: simulasi adalah patokan awal kebutuhan energi, durasi swing-up,
dan envelope command actuator. Klaim gaya motor fisik tetap harus divalidasi
dengan log alat asli seperti sudut pendulum, posisi/kecepatan cart, PWM atau
arus motor, dan keberhasilan masuk balance.
