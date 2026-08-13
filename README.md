# Linear Inverted Pendulum Simulation

Selamat datang di repository simulasi **Linear Inverted Pendulum** berbasis ROS 2 (Jazzy) dan Gazebo Harmonic! 

Repository ini dirancang untuk mensimulasikan sistem inverted pendulum guna menguji dan membandingkan performa berbagai algoritma kontrol dalam menjaga keseimbangan pendulum (kereta dan tuas). Terdapat tiga metode kontrol utama yang dieksperimenkan, di mana masing-masing mampu menahan gangguan (impulse) agar pendulum tetap tegak (balance).

Berikut adalah demonstrasi simulasi dari 3 metode yang diuji:

## 1. Metode LQR (Linear Quadratic Regulator)
Metode ini menggunakan algoritma kontrol optimal LQR yang didasarkan pada model state-space linear dari sistem pendulum. Algoritma ini menghitung nilai gain yang meminimalkan *cost function* untuk menyeimbangkan pendulum dengan pergerakan cart dan respons energi yang paling efisien.

<video src="./videos/lqr_impulse_balance_20260512.mp4" controls="controls" style="max-width: 100%;"></video>

*(Jika video di atas tidak dapat diputar, Anda bisa melihatnya secara langsung di `videos/lqr_impulse_balance_20260512.mp4`)*

## 2. Metode PID (Proportional-Integral-Derivative)
Metode klasik PID diterapkan pada sistem ini. Meskipun tidak membutuhkan model matematika sistem yang mendetail seperti LQR, *tuning* (penalaan) nilai Proportional, Integral, dan Derivative yang tepat terbukti cukup stabil dalam merespons gangguan eksternal (impulse).

<video src="./videos/pid_impulse_balance_20260512.mp4" controls="controls" style="max-width: 100%;"></video>

*(Jika video di atas tidak dapat diputar, Anda bisa melihatnya secara langsung di `videos/pid_impulse_balance_20260512.mp4`)*

## 3. Metode Full-State Feedback (Real-Style)
Metode ini menggunakan controller bertipe full-state feedback yang arsitekturnya dibuat sedekat mungkin dengan panduan *Manual Book* alat aslinya (real system). Metode ini secara khusus berfokus meniru perilaku dinamika *controller hardware* aslinya.

<video src="./videos/real_impulse_balance_20260512.mp4" controls="controls" style="max-width: 100%;"></video>

*(Jika video di atas tidak dapat diputar, Anda bisa melihatnya secara langsung di `videos/real_impulse_balance_20260512.mp4`)*

---

## 🛠 Panduan Teknis & Cara Menjalankan

Supaya pengunjung halaman utama ini tidak kebingungan dengan hal yang terlalu teknis, penjelasan tentang cara *running* code, struktur *workspace* ROS 2 + Gazebo, *GUI serial bridge*, hingga *state machine* pendulum telah dipisahkan. 

Untuk membaca **Panduan Menjalankan Simulasi** dan detail teknis lainnya, silakan klik tautan di bawah ini:

👉 **[TEKNIS_GAZEBO.md](./TEKNIS_GAZEBO.md)**

*(Jika Anda butuh penjelasan algoritma, dokumen dasar teori tersedia di [DOKUMENTASI_METODE_DAN_DASAR_TEORI.md](./DOKUMENTASI_METODE_DAN_DASAR_TEORI.md))*
