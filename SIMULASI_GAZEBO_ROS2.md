# Simulasi Gazebo Harmonic + ROS Jazzy

Saya tambahkan workspace ROS 2 di:

```bash
/home/ammar/Documents/Pendulum/ros2_pendulum_ws
```

Paket utamanya:

```bash
ros2_pendulum_ws/src/linear_inverted_pendulum_sim
```

Model dibuat dari manual LIP01: travel translasi 78 cm, base 1000 mm x 350 mm, rail 900 mm, pendulum D8 x 400 mm dengan massa 200 g. Simulasi memakai Gazebo Harmonic 8 dan `ros_gz_bridge`.

## Build pertama kali

```bash
cd /home/ammar/Documents/Pendulum/ros2_pendulum_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Jalankan simulasi

```bash
ros2 launch linear_inverted_pendulum_sim sim.launch.py
```

Launch ini memakai `GZ_PARTITION` unik per proses, jadi `/clock` dan
`joint_state` dari Gazebo lama tidak ikut terbaca.

Kalau ingin tanpa GUI Gazebo:

```bash
ros2 launch linear_inverted_pendulum_sim sim.launch.py gz_args:="-r -s empty.sdf"
```

Kalau ingin menjalankan `test_cart_movement.py` atau `test_manual_swing.py`,
matikan bridge controller supaya script menjadi satu-satunya publisher force:

```bash
ros2 launch linear_inverted_pendulum_sim sim.launch.py enable_serial_bridge:=false
```

## Jalankan GUI Python kamu ke simulasi

Terminal baru:

```bash
cd /home/ammar/Documents/Pendulum
PENDULUM_PORT=/tmp/pendulum_sim_serial PENDULUM_NO_JOYSTICK=1 python3 main.py
```

`/tmp/pendulum_sim_serial` dibuat otomatis oleh node ROS 2. Dari sisi `main.py`, port ini terlihat seperti serial STM32. Paket gain, reset, tombol A/B/X/Y, dan packet status memakai format yang sama dengan kode Python lama.

## Urutan tes

1. Jalankan Gazebo dengan launch ROS 2.
2. Jalankan `main.py` dengan `PENDULUM_PORT=/tmp/pendulum_sim_serial`.
3. Klik `Apply Gains`.
4. Klik `START`.
5. Klik `Y` untuk homing.
6. Klik `X` untuk swing-up, atau `A` untuk balance jika pendulum sudah dekat posisi atas.
7. Klik `B` untuk stop.

Jika kamu memakai joystick fisik, jalankan tanpa `PENDULUM_NO_JOYSTICK=1`.

## Topic penting

```bash
ros2 topic echo /joint_states
ros2 topic echo /pendulum/sim_state
ros2 topic echo /pendulum/cart_force_cmd
```

`/pendulum/cart_velocity_cmd` adalah setpoint internal bridge dalam m/s.
Input yang benar-benar masuk ke Gazebo adalah `/pendulum/cart_force_cmd`,
yang dipublish oleh `sim_serial_bridge`.

Jika simulasi menampilkan waktu/RTF aneh atau cart bergerak sendiri, cek proses
Gazebo lama:

```bash
pgrep -fa 'gz sim -r -s empty.sdf'
pkill -f '^gz sim -r -s empty\.sdf$'
```
