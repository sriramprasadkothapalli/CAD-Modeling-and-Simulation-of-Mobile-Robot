#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class IMUSubscriber(Node):

    def __init__(self):
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu, 'imu_plugin/out', self.imu_callback, qos_profile)
        
        self.subscription  # prevent unused variable warning

    def imu_callback(self, msg):
        # Extract yaw angle from the Imu message
        orientation = msg.orientation
        # Assuming the orientation is represented as a quaternion
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w)
        
        # Now, 'yaw' contains the yaw angle in radians
        self.get_logger().info(f'Yaw Angle: {yaw} radians')

    def quaternion_to_euler(self, x, y, z, w):
        # Convert a quaternion to Euler angles (roll, pitch, yaw)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = IMUSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
