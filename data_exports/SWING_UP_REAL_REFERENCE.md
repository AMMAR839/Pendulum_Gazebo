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

## Ringkasan terbaru setelah tuning multi-swing hardware-logical

Sumber data:
`data_exports/multi_swing_validation_20260510.csv`

Ringkasan lengkap:
`data_exports/multi_swing_real_reference_summary_20260510.csv`

| Workspace | Durasi swing-up | Mean abs effort | RMS effort | P95/peak effort | Abs impulse | Positive cmd-work | Energy ready ratio | Motor command mean / P95 / peak | PWM mean / P95 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `lqr-pendulum` (CSV lama berlabel `ros2_pendulum_ws`) | 7.961 s | 5.290 N | 8.281 N | 18.326 / 33.530 N | 42.248 N.s | 7.590 J | 1.022 | 7.394 / 25.508 / 47.057 cm/s | - |
| `pendulum_real_ws` | 7.957 s | 4.958 N | 7.446 N | 16.516 / 30.956 N | 39.610 N.s | 6.215 J | 1.014 | 6.941 / 23.122 / 43.339 cm/s | 4272 / 7584 |
| `pendulum_pid_ws` | 7.950 s | 5.685 N | 8.100 N | 17.246 / 28.169 N | 45.058 N.s | 7.260 J | 1.107 | 7.978 / 24.144 / 39.902 cm/s | 4549 / 7778 |

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
pendulum hingga sekitar energi referensi posisi tegak. Setelah tuning
multi-swing hardware-logical, capture ditahan sampai ayunan membangun energi
lebih lama, dengan gate `3` top-pass atau minimal sekitar `8 s` build energy.
Run headless terbaru tetap masuk fase balance, dengan mean effort swing-up
sekitar 4.9 N sampai 5.7 N dan peak sekitar 28 N sampai 34 N. Karena belum
digunakan sensor gaya motor fisik, nilai force Gazebo tidak diklaim sebagai gaya
motor aktual, tetapi digunakan sebagai referensi command envelope yang kemudian
harus divalidasi terhadap log alat asli berupa sudut pendulum, posisi/kecepatan
cart, PWM atau arus motor, durasi swing-up, dan keberhasilan masuk fase balance.
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
  --samples data_exports/multi_swing_validation_20260510.csv \
  --output data_exports/multi_swing_real_reference_summary_20260510.csv
```

Untuk data capture lama:

```bash
python3 data_exports/build_swing_up_real_reference.py \
  --samples data_exports/runtime_swing_balance_samples_20260509_three_ws.csv \
  --output data_exports/swing_up_real_reference_summary_from_20260509.csv
```
