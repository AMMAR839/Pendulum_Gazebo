#!/usr/bin/env python3
"""
Simple test: check if cart moves when we publish Gazebo force command.

Run Gazebo without the serial bridge so this script is the only force source:
ros2 launch linear_inverted_pendulum_sim sim.launch.py enable_serial_bridge:=false
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time

class SimpleCartTest(Node):
    def __init__(self):
        super().__init__('simple_cart_test')
        self.force_pub = self.create_publisher(Float64, '/pendulum/cart_force_cmd', 10)
        self.sub = self.create_subscription(JointState, 'joint_states', self._joint_cb, 10)
        
        self.positions = {}
        
    def _joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            joint = name.split('/')[-1].split('::')[-1]
            self.positions[joint] = msg.position[i]
    
    def test1_constant_velocity(self):
        """Test 1: Push cart with constant force."""
        print("\n=== Test 1: Constant Force (+15 N for 5 sec) ===")
        start = time.time()
        
        while time.time() - start < 5.0:
            msg = Float64()
            msg.data = 15.0
            self.force_pub.publish(msg)
            
            cart_x = self.positions.get('cart_slider', 0.0)
            pend_theta = self.positions.get('pendulum_hinge', 0.0)
            
            print(f"cart_x = {cart_x:7.4f} m,  pend_θ = {pend_theta:7.4f} rad")
            time.sleep(0.1)
        
        print("\nTest 1 complete. Did cart move? Check values above.")
    
    def test2_oscillation(self):
        """Test 2: Oscillate with alternating force."""
        print("\n=== Test 2: Oscillation ±15 N ===")
        
        for cycle in range(5):
            for direction in [15.0, -15.0]:
                start = time.time()
                while time.time() - start < 1.0:
                    msg = Float64()
                    msg.data = float(direction)
                    self.force_pub.publish(msg)
                    
                    cart_x = self.positions.get('cart_slider', 0.0)
                    pend_theta = self.positions.get('pendulum_hinge', 0.0)
                    
                    elapsed = time.time() - start
                    print(f"cycle {cycle} force={direction:+.1f}N t={elapsed:.2f}s  "
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
