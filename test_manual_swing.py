#!/usr/bin/env python3
"""
Test manual cart movement untuk see if pendulum dapat bergerak
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
        self.cmd_pub = self.create_publisher(Float64, '/pendulum/cart_velocity_cmd', 10)
        self.sub = self.create_subscription(JointState, 'joint_states', self._joint_cb, 10)
        
        self.cart_pos = 0.0
        self.pendulum_angle = 0.0
        
    def _joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            joint = name.split('/')[-1].split('::')[-1]
            if joint == 'cart_slider':
                self.cart_pos = msg.position[i]
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
            
            # Simple P control to reach target
            error = target_x - self.cart_pos
            command = 2.0 * error  # High proportional gain
            
            # Limit command
            command = max(-1.5, min(1.5, command))
            
            msg = Float64()
            msg.data = command
            self.cmd_pub.publish(msg)
            
            theta_deg = math.degrees(self.pendulum_angle)
            print(f"t={elapsed:5.2f}s  θ={theta_deg:7.2f}°  cart={self.cart_pos:6.3f}m  cmd={command:6.3f}m/s")
            
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
