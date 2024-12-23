#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Pose, Twist, Vector3
from simple_pid import PID
import time


class DiffDriveCtrl(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')

        # Initialize parameters
        self.cpr = 125  # Encoder counts per revolution
        self.wheel_dia = 0.125  # meters
        self.wheel_separation = 0.43  # meters
        self.ticks_per_meter = self.cpr / (self.wheel_dia * 3.14)

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0

        self.t_vl = 0
        self.t_vr = 0
        self.l_rpm = 0
        self.r_rpm = 0

        self.tick_count = Vector3()

        # Set up subscriptions and publishers
        self.left_wheel_sub = self.create_subscription(Float32, "left_tickrate", self.left_wheel_callback, 1)
        self.right_wheel_sub = self.create_subscription(Float32, "right_tickrate", self.right_wheel_callback, 1)
        self.tick_count_sub = self.create_subscription(Vector3, "tick_count", self.tick_count_callback, 1)

        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.recv_cmd_vel, 1)

        self.left_pub = self.create_publisher(Float32, "left_wheel/control_effort", 1)
        self.right_pub = self.create_publisher(Float32, "right_wheel/control_effort", 1)

        self.cvl_pub = self.create_publisher(Float32, "left_measured_vel", 1)
        self.cvr_pub = self.create_publisher(Float32, "right_measured_vel", 1)

        self.tvl_pub = self.create_publisher(Float32, "left_setpoint", 1)
        self.tvr_pub = self.create_publisher(Float32, "right_setpoint", 1)

        self.pid_sub = self.create_subscription(Vector3, "pid_tune", self.pid_callback, 1)

        # Initialize PID controllers
        self.pid_left = PID(self.kp, self.ki, self.kd, setpoint=0, output_limits=(-100, 100))
        self.pid_right = PID(self.kp, self.ki, self.kd, setpoint=0, output_limits=(-100, 100))

        self.prev_l_tick_count = 0
        self.prev_r_tick_count = 0
        self.prev_time = time.time_ns()

        # Set up a timer for PID control loop
        self.timer = self.create_timer(0.1, self.control_loop_callback)  # Adjust the timer period as needed

    def recv_cmd_vel(self, msg: Twist):
        self.bot_vel = msg
        v = self.bot_vel.linear.x
        w = self.bot_vel.angular.z

        self.t_vl = v - (w * self.wheel_separation / 2)
        self.t_vr = v + (w * self.wheel_separation / 2)
        self.l_rpm = -(self.t_vl * 60) / (3.14 * self.wheel_dia)
        self.r_rpm = -(self.t_vr * 60) / (3.14 * self.wheel_dia)

        self.left_pub.publish(Float32(data=self.l_rpm))
        self.right_pub.publish(Float32(data=self.r_rpm))

        self.tvl_pub.publish(Float32(data=self.t_vl))
        self.tvr_pub.publish(Float32(data=self.t_vr))

    def left_wheel_callback(self, msg: Float32):
        self.c_vl = msg.data / self.cpr * self.wheel_dia * 3.14

    def right_wheel_callback(self, msg: Float32):
        self.c_vr = msg.data / self.cpr * self.wheel_dia * 3.14

    def tick_count_callback(self, msg: Vector3):
        self.tick_count.x = msg.x
        self.tick_count.y = msg.y
        self.tick_count.z = msg.z

    def pid_callback(self, msg: Vector3):
        self.kp = msg.x
        self.ki = msg.y
        self.kd = msg.z

        # Update PID controller gains
        self.pid_left.tunings = (self.kp, self.ki, self.kd)
        self.pid_right.tunings = (self.kp, self.ki, self.kd)

    def control_loop_callback(self):
        current_time = time.time_ns()
        dt = (current_time - self.prev_time) * 1e-9  # Convert to seconds

        current_l_tick_count = self.tick_count.x
        current_r_tick_count = self.tick_count.y

        # Calculate current velocities
        c_vl = (current_l_tick_count - self.prev_l_tick_count) / dt
        c_vr = (current_r_tick_count - self.prev_r_tick_count) / dt

        # Convert to meters per second
        c_vl = c_vl / self.cpr * self.wheel_dia * 3.14
        c_vr = c_vr / self.cpr * self.wheel_dia * 3.14

        # Publish measured velocities
        self.cvl_pub.publish(Float32(data=c_vl *1.0))
        self.cvr_pub.publish(Float32(data=c_vr *1.0))

        # Calculate control efforts
        control_effort_left = self.pid_left(c_vl)
        control_effort_right = self.pid_right(c_vr)

        # Publish control efforts
        self.left_pub.publish(Float32(data=control_effort_left *1.0))
        self.right_pub.publish(Float32(data=control_effort_right *1.0))

        # Update previous values
        self.prev_l_tick_count = current_l_tick_count
        self.prev_r_tick_count = current_r_tick_count
        self.prev_time = current_time


def main(args=None):
    rclpy.init(args=args)
    ctrlr = DiffDriveCtrl()
    rclpy.spin(ctrlr)
    ctrlr.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
