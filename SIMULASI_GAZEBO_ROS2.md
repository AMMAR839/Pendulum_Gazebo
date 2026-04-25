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

Kalau ingin tanpa GUI Gazebo:

```bash
ros2 launch linear_inverted_pendulum_sim sim.launch.py gz_args:="-r -s empty.sdf"
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
ros2 topic pub /pendulum/cart_velocity_cmd std_msgs/msg/Float64 "{data: 0.2}"
```

`/pendulum/cart_velocity_cmd` adalah command kecepatan cart dalam m/s.
