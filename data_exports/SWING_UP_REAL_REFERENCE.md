# Patokan Real Swing-Up

File ini menjelaskan cara menjadikan gaya swing-up dari simulasi sebagai
patokan awal untuk alat real tanpa mengubahnya menjadi klaim gaya motor yang
belum diukur langsung.

## Prinsip klaim

Yang bisa dipertanggungjawabkan:

1. Controller swing-up di semua workspace memakai metode berbasis energi.
2. Runtime Gazebo menunjukkan energi pendulum naik sampai hampir sama dengan
   energi saat posisi tegak.
3. Batas dan envelope command dapat dipakai sebagai target awal untuk alat real:
   durasi swing-up, effort cart model, command kecepatan cart, dan PWM ekuivalen
   pada workspace real-style.
4. Untuk klaim hardware final, angka simulasi harus dibandingkan dengan log alat
   asli: posisi cart, kecepatan cart, sudut pendulum, PWM/current motor, dan
   keberhasilan mencapai area capture.

Yang tidak boleh diklaim tanpa sensor atau kalibrasi tambahan:

- `cart_force_cmd_n` bukan hasil ukur gaya motor fisik.
- Hinge assist adalah bantuan simulasi, bukan aktuator tambahan pada alat real.
- Nilai N dari Gazebo tidak boleh ditulis sebagai gaya motor aktual sebelum ada
  pemetaan actuator yang divalidasi.

## Ringkasan terbaru setelah model visual

Sumber data:
`data_exports/post_visual_balance_validation_20260510.csv`

Ringkasan lengkap:
`data_exports/swing_up_real_reference_summary_20260510.csv`

| Workspace | Durasi swing-up | Mean abs effort | RMS effort | P95/peak effort | Abs impulse | Positive cmd-work | Energy ready ratio | Motor command mean / P95 / peak | PWM mean / P95 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `ros2_pendulum_ws` | 8.194 s | 56.617 N | 65.631 N | 90.000 / 92.084 N | 466.727 N.s | 102.424 J | 0.992 | 25.481 / 86.748 / 126.000 cm/s | - |
| `pendulum_real_ws` | 8.126 s | 83.011 N | 100.846 N | 145.000 / 145.000 N | 692.949 N.s | 124.536 J | 0.993 | 23.774 / 59.950 / 203.000 cm/s | 7686 / 14549 |
| `pendulum_pid_ws` | 8.465 s | 84.991 N | 103.772 N | 145.000 / 145.000 N | 722.098 N.s | 136.866 J | 0.994 | 25.032 / 62.467 / 203.000 cm/s | 7906 / 15024 |

Makna tabel:

- `Mean abs effort`, `RMS`, `P95`, dan `peak` menjelaskan envelope effort cart
  pada model Gazebo.
- `Abs impulse` menjelaskan total kebutuhan dorongan selama swing-up, bukan
  sekadar spike sesaat.
- `Positive cmd-work` adalah estimasi kerja positif dari `force * command
  velocity`; ini dipakai sebagai indikator energi actuator, bukan pengukuran
  kerja mekanik real.
- `Energy ready ratio` mendekati `1.0`, artinya swing-up sudah mengumpulkan
  energi hampir setara kondisi tegak sebelum balance mengambil alih.
- PWM ekuivalen hanya dipakai untuk `pendulum_real_ws` dan `pendulum_pid_ws`
  karena keduanya memakai model motor real-style.

## Cara menulis di laporan

Kalimat yang aman:

```text
Gaya swing-up pada penelitian ini digunakan sebagai patokan awal actuator real
melalui envelope hasil simulasi. Controller berbasis energi menaikkan energi
pendulum hingga sekitar 99% energi referensi posisi tegak, dengan batas effort
cart model sebesar +/-90 N pada ros2_pendulum_ws dan +/-145 N pada
pendulum_real_ws serta pendulum_pid_ws. Karena belum digunakan sensor gaya motor
fisik, nilai force Gazebo tidak diklaim sebagai gaya motor aktual, tetapi
digunakan sebagai referensi command envelope yang kemudian harus divalidasi
terhadap log alat asli berupa sudut pendulum, posisi/kecepatan cart, PWM atau
arus motor, durasi swing-up, dan keberhasilan masuk fase balance.
```

Kalimat singkat untuk kesimpulan:

```text
Simulasi dapat dijadikan patokan awal alat real pada level kebutuhan energi,
durasi swing-up, dan envelope command actuator. Validasi hardware tetap
diperlukan sebelum nilai N dari Gazebo disebut sebagai gaya motor fisik.
```

## Cara membuat ulang ringkasan

```bash
cd /home/ammar/Documents/Pendulum
python3 data_exports/build_swing_up_real_reference.py \
  --samples data_exports/post_visual_balance_validation_20260510.csv \
  --output data_exports/swing_up_real_reference_summary_20260510.csv
```

Untuk data capture lama:

```bash
python3 data_exports/build_swing_up_real_reference.py \
  --samples data_exports/runtime_swing_balance_samples_20260509_three_ws.csv \
  --output data_exports/swing_up_real_reference_summary_from_20260509.csv
```
