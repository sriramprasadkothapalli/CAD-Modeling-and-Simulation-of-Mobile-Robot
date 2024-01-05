#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from matplotlib import pyplot as plt
from geometry_msgs.msg import Pose2D
import time  # Importing the time module

Time = []  # List for time data
control = []  # List for control data
error = []  # List for error data
posx = []  # List for X positions
posy = []  # List for Y positions
desired_position = Pose2D()
desired_position.x = 10.0
desired_position.y = 10.0

class ProportionalController(Node):
    def __init__(self):
        super().__init__('proportional_control')
        self.get_logger().info(f'Proportional controller activated...') 
        self.position_x = 0
        self.position_y = 0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.start_time = time.time()
 
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.imu_sub = self.create_subscription(Imu, 'imu_plugin/out', self.imu_callback, qos_profile)
        self.imu_sub

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def imu_callback(self, msg):
        self.time_starts = self.get_clock().now()
        imu_timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec) / 1000000000
        current_heading = msg.orientation.z
        self.timestamp = imu_timestamp
        self.current_heading = current_heading
        time_rightnow = self.get_clock().now()
        t_elapsed = (time_rightnow - self.time_starts).nanoseconds / 1e9
        self.vel_x += msg.linear_acceleration.x * 0.01
        self.vel_y += msg.linear_acceleration.y * 0.01
        self.position_x += self.vel_x * 0.01
        self.position_y += self.vel_y * 0.01

    def timer_callback(self):
        K = 5
        des_phi = -0.375
        des_x = 10
        des_y = 10
        linear_vel = 8.0
        steer_angle = 0.0

        phi_error = des_phi - self.current_heading
        steer_angle = -K * phi_error
        wheel_velocities = Float64MultiArray()
        joint_positions = Float64MultiArray()
        
        if steer_angle > 1.0:
            steer_angle = 1.0

        if steer_angle < -1.0:
            steer_angle = -1.0

        curr = int(self.position_x)
        print(curr)
        if curr > 8:
            wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
            joint_positions.data = [0.0, 0.0]
        else:
            wheel_velocities.data = [-linear_vel, linear_vel, -linear_vel, linear_vel]
            joint_positions.data = [steer_angle, steer_angle]

        elapsed_time = time.time() - self.start_time
        if elapsed_time >= 30.0:
            wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]

        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)
        self.get_logger().info(f'Current Heading: {self.current_heading}, Steer Control Input: {steer_angle}')

        Time.append(self.timestamp)
        control.append(steer_angle)
        error.append(phi_error)
        posx.append(self.position_x)
        posy.append(self.position_y)

        self.counter += 1

def main(args = None):
    print('Starting the controller: Moving from (0,0) to (10,10)') 
    rclpy.init(args=args)
    controller = ProportionalController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()
    print('Graphs ready!')

    plt.title('Error vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (rad)')
    plt.plot(Time, error, 'b-', label='Error')
    plt.legend()
    plt.show()

    plt.title('Steer Angle vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Steer Angle (rad)')
    plt.plot(Time, control, 'b-', label='Steer Angle')
    plt.legend()
    plt.show()
    
    plt.title('Position vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.plot(posx, posy, 'b-', label='Position')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
