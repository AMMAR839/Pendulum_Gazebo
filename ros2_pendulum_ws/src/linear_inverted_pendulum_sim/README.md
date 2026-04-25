# Linear Inverted Pendulum Gazebo Sim

Paket ini dibuat untuk ROS 2 Jazzy dan Gazebo Harmonic 8. Model mengikuti data dari manual LIP01:

- Base 1000 mm x 350 mm.
- Rail linear sekitar 900 mm dengan travel kontrol 78 cm.
- Cart bergerak pada joint prismatic `cart_slider`.
- Pendulum shaft diameter 8 mm, panjang 400 mm, massa 200 g.
- Encoder pendulum dimodelkan dari joint `pendulum_hinge`.

## Build

```bash
cd /home/ammar/Documents/Pendulum/ros2_pendulum_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Jalankan Gazebo

```bash
ros2 launch linear_inverted_pendulum_sim sim.launch.py
```

Untuk headless:

```bash
ros2 launch linear_inverted_pendulum_sim sim.launch.py gz_args:="-r -s empty.sdf"
```

Launch ini membuat pseudo serial di:

```bash
/tmp/pendulum_sim_serial
```

## Hubungkan ke GUI Python lama

Terminal baru:

```bash
cd /home/ammar/Documents/Pendulum
PENDULUM_PORT=/tmp/pendulum_sim_serial PENDULUM_NO_JOYSTICK=1 python3 main.py
```

Saat `PENDULUM_PORT=/tmp/pendulum_sim_serial`, `main.py` otomatis memakai gain default yang cocok untuk simulasi balance.

Alur pemakaian:

1. Klik `Apply Gains`; bridge akan mengirim ACK seperti STM32.
2. Klik `START` pada GUI.
3. Klik tombol `Y` untuk homing ke tengah.
4. Klik `X` untuk swing up dan auto-balance.
5. Klik `A` hanya untuk paksa mode balance saat batang sudah dekat tegak.
6. Klik `B` untuk finish/stop.

Jika memakai joystick fisik, hilangkan `PENDULUM_NO_JOYSTICK=1`.

## Topic ROS 2 penting

- `/joint_states`: posisi dan kecepatan `cart_slider` serta `pendulum_hinge`.
- `/pendulum/cart_velocity_cmd`: setpoint kecepatan cart dari controller internal dalam m/s.
- `/pendulum/cart_force_cmd`: gaya yang dikirim ke joint cart di Gazebo.
- `/pendulum/sim_state`: data ringkas `[degree, cmX, setspeed, energy, theta_dot_rad, theta_rad, x_center_cm, mode]`.

Mode mengikuti label GUI lama: `1=WAITING`, `2=HOMING`, `3=READY`, `4=SINUS`, `5=FINISH`, `6=SWING UP`, `7=BALANCING`.
