#!/usr/bin/env python3
"""
Test manual cart movement untuk melihat apakah pendulum dapat bergerak.

Jalankan Gazebo tanpa serial bridge supaya force dari script ini tidak
ditimpa controller:
ros2 launch linear_inverted_pendulum_sim sim.launch.py enable_serial_bridge:=false
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time
import math

class ManualSwingTester(Node):
    def __init__(self):
        super().__init__('manual_swing_tester')
        self.force_pub = self.create_publisher(Float64, '/pendulum/cart_force_cmd', 10)
        self.sub = self.create_subscription(JointState, 'joint_states', self._joint_cb, 10)
        
        self.cart_pos = 0.0
        self.cart_vel = 0.0
        self.pendulum_angle = 0.0
        
    def _joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            joint = name.split('/')[-1].split('::')[-1]
            if joint == 'cart_slider':
                self.cart_pos = msg.position[i]
                if i < len(msg.velocity):
                    self.cart_vel = msg.velocity[i]
            elif joint == 'pendulum_hinge':
                self.pendulum_angle = msg.position[i]
    
    def test_aggressive_swing(self):
        """Test with aggressive oscillation"""
        print("\n=== Testing Aggressive Cart Swing ===")
        print("Moving cart in sine wave with high amplitude...")
        
        start = time.time()
        amplitude = 0.35  # Large amplitude
        freq = 0.8  # Hz
        
        for _ in range(1000):  # Run for ~10 seconds at 100Hz
            elapsed = time.time() - start
            
            # Target position for oscillation
            target_x = amplitude * math.sin(2.0 * math.pi * freq * elapsed)
            
            # Simple PD force control to reach target
            error = target_x - self.cart_pos
            force = 80.0 * error - 8.0 * self.cart_vel
            
            # Limit force to the URDF joint effort range
            force = max(-45.0, min(45.0, force))
            
            msg = Float64()
            msg.data = force
            self.force_pub.publish(msg)
            
            theta_deg = math.degrees(self.pendulum_angle)
            print(f"t={elapsed:5.2f}s  θ={theta_deg:7.2f}°  cart={self.cart_pos:6.3f}m  force={force:6.2f}N")
            
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    tester = ManualSwingTester()
    
    try:
        tester.test_aggressive_swing()
    except KeyboardInterrupt:
        print("\n\nAborted")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
