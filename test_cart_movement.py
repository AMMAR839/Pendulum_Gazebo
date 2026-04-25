#!/usr/bin/env python3
"""
Simple test: check if cart moves when we publish velocity command
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time

class SimpleCartTest(Node):
    def __init__(self):
        super().__init__('simple_cart_test')
        self.cmd_pub = self.create_publisher(Float64, '/pendulum/cart_velocity_cmd', 10)
        self.sub = self.create_subscription(JointState, 'joint_states', self._joint_cb, 10)
        
        self.positions = {}
        
    def _joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            joint = name.split('/')[-1].split('::')[-1]
            self.positions[joint] = msg.position[i]
    
    def test1_constant_velocity(self):
        """Test 1: Push cart with constant velocity"""
        print("\n=== Test 1: Constant Velocity (0.5 m/s for 5 sec) ===")
        start = time.time()
        
        while time.time() - start < 5.0:
            msg = Float64()
            msg.data = 0.5  # 0.5 m/s right
            self.cmd_pub.publish(msg)
            
            cart_x = self.positions.get('cart_slider', 0.0)
            pend_theta = self.positions.get('pendulum_hinge', 0.0)
            
            print(f"cart_x = {cart_x:7.4f} m,  pend_θ = {pend_theta:7.4f} rad")
            time.sleep(0.1)
        
        print("\nTest 1 complete. Did cart move? Check values above.")
    
    def test2_oscillation(self):
        """Test 2: Oscillate between +0.3 and -0.3"""
        print("\n=== Test 2: Oscillation ±0.3 m/s ===")
        
        for cycle in range(5):
            for direction in [0.3, -0.3]:
                start = time.time()
                while time.time() - start < 1.0:
                    msg = Float64()
                    msg.data = float(direction)
                    self.cmd_pub.publish(msg)
                    
                    cart_x = self.positions.get('cart_slider', 0.0)
                    pend_theta = self.positions.get('pendulum_hinge', 0.0)
                    
                    elapsed = time.time() - start
                    print(f"cycle {cycle} dir={direction:+.1f} t={elapsed:.2f}s  "
                          f"cart={cart_x:+.4f}m  pend={pend_theta:+.4f}rad")
                    time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)
    tester = SimpleCartTest()
    
    print("Waiting for joint_states...")
    time.sleep(1)
    
    try:
        tester.test1_constant_velocity()
        time.sleep(2)
        tester.test2_oscillation()
    except KeyboardInterrupt:
        print("\nAborted")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
