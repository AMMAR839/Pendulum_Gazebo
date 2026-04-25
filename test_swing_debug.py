#!/usr/bin/env python3
"""Debug script untuk check swing-up performance"""
import subprocess
import time
import threading

def run_monitoring():
    """Monitor pendulum state dari ROS topic"""
    cmd = "source /opt/ros/jazzy/setup.bash && ros2 topic echo /pendulum/sim_state -n 100 --csv"
    try:
        subprocess.run(cmd, shell=True, cwd="/home/ammar/Documents/Pendulum/ros2_pendulum_ws")
    except KeyboardInterrupt:
        pass

def main():
    print("""
    ========================================
    Swing-Up Debug Monitor
    ========================================
    
    Jalankan ini di terminal terpisah untuk monitor pendulum state.
    Format output: [degree, cmX, setspeed, energy, theta_dot_rad, theta_rad, x_center_cm, mode]
    
    Mode: 1=WAITING, 2=HOMING, 3=READY, 4=SINE, 5=FINISH, 6=SWING_UP, 7=BALANCE
    
    Lihat apakah:
    1. degree berubah (ada gerakan swing)
    2. energy terus naik saat swing-up
    3. setspeed berubah (ada command ke motor)
    """)
    
    try:
        run_monitoring()
    except KeyboardInterrupt:
        print("\nDibatalkan")

if __name__ == "__main__":
    main()
