#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3
import time
import math

class DiffDriveCtrl(Node):

    def __init__(self):
        super().__init__('diff_drive_controller')

        # Robot parameters
        self.cpr = 125  # encoder counts per revolution
        self.wheel_dia = 0.125  # meters
        self.wheel_separation = 0.43  # meters
        self.ticksPerMeter = self.cpr / (self.wheel_dia * 3.14)

        # PID parameters (if PID control is implemented)
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0

        # Target velocities
        self.t_vl = 0.0
        self.t_vr = 0.0
        self.l_rpm = 0.0
        self.r_rpm = 0.0

        # Robot position tracking
        self.x1 = 0.0
        self.y1 = 0.0
        self.x2 = 0.0
        self.y2 = 0.0

        # Calculated velocities
        self.Vrobo = 0.0

        self.tick_count = Vector3()

        # Subscriptions
        self.left_wheel_sub = self.create_subscription(Float32, "left_tickrate", self.left_wheel_callback, 1)
        self.right_wheel_sub = self.create_subscription(Float32, "right_tickrate", self.right_wheel_callback, 1)
        self.tick_count_sub = self.create_subscription(Vector3, "tick_count", self.tick_count_callback, 1)

        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.recv_cmd_vel, 1)
        self.x_sub = self.create_subscription(Float32, "x_sub", self.distance_x, 1)
        self.y_sub = self.create_subscription(Float32, "y_sub", self.distance_y, 1)

        # Publishers
        self.left_pub = self.create_publisher(Float32, "left_wheel/control_effort", 1)
        self.right_pub = self.create_publisher(Float32, "right_wheel/control_effort", 1)

        self.cvl_pub = self.create_publisher(Float32, "left_measured_vel", 1)
        self.cvr_pub = self.create_publisher(Float32, "right_measured_vel", 1)

        self.tvl_pub = self.create_publisher(Float32, "left_setpoint", 1)
        self.tvr_pub = self.create_publisher(Float32, "right_setpoint", 1)

        # Velocity calculations
        self.c_vl = 0
        self.c_vr = 0

        self.bot_vel = Twist()
        self.rate = self.create_rate(8, self.get_clock())

        # Previous tick count and time tracking
        self.prev_l_tick_count = 0
        self.prev_r_tick_count = 0
        self.prev_time = time.time_ns()

        # Control loop timer
        self.timer = self.create_timer(0.125, self.control_loop)

        # Timing for checking distance and stopping the robot
        self.start_time = None
        self.time_req = 0.0

    def recv_cmd_vel(self, msg: Twist):
        self.bot_vel = msg

        # Calculate target left and right wheel velocities
        v = self.bot_vel.linear.x
        w = self.bot_vel.angular.z

        self.t_vl = v - (w * self.wheel_separation / 2)
        self.t_vr = v + (w * self.wheel_separation / 2)
        self.l_rpm = -(self.t_vl * 60) / (3.14 * self.wheel_dia)
        self.r_rpm = -(self.t_vr * 60) / (3.14 * self.wheel_dia)

        self.tvl_pub.publish(Float32(data=self.t_vl))
        self.tvr_pub.publish(Float32(data=self.t_vr))

    def left_wheel_callback(self, msg: Float32):
        self.c_vl = (msg.data / self.cpr) * self.wheel_dia * 3.14

    def right_wheel_callback(self, msg: Float32):
        self.c_vr = (msg.data / self.cpr) * self.wheel_dia * 3.14

    def tick_count_callback(self, msg: Vector3):
        self.tick_count = msg

    def distance_x(self, msg: Float32):
        self.x2 = msg.data

    def distance_y(self, msg: Float32):
        self.y2 = msg.data

    def control_loop(self):
        # Update current coordinates
        x1 = self.x1
        y1 = self.y1
        x2 = self.x2
        y2 = self.y2

        # Calculate distance to the target
        self.distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        
        # Calculate required time based on distance and target velocity
        self.Vrobo_need = (self.t_vl + self.t_vr) / 2
     
        self.time_req = self.distance / self.Vrobo_need if self.Vrobo_need != 0 else 0
        

        # Start tracking time if not already done
        if self.start_time is None:
            self.start_time = time.time()
            

        # Calculate elapsed time
        elapsed_time = time.time() - self.start_time
        print(elapsed_time) 

        if (elapsed_time>self.time_req):
            self.left_pub.publish(Float32(data=0.0))
            self.right_pub.publish(Float32(data=0.0))
            
            actual_distance = self.Vrobo_need*self.time_req
            
            calculated_distance = self.Vrobo*elapsed_time
            self.left_pub.publish(Float32(data=float(self.l_rpm)))
            self.right_pub.publish(Float32(data=float(self.r_rpm)))
            


        # If the required time has elapsed, stop the robot
        # if elapsed_time >= self.time_req:
        #     # Stop the robot by setting PWM values to zero
        #     self.left_pub.publish(Float32(data=0.0))
        #     self.right_pub.publish(Float32(data=0.0))

        #     # Check if the robot has reached the target
        #     current_l_tick_count = self.tick_count.x
        #     current_r_tick_count = self.tick_count.y

        #     # Calculate the actual distance moved based on encoder data
        #     actual_distance_l = (current_l_tick_count / self.cpr) * self.wheel_dia * 3.14
        #     actual_distance_r = (current_r_tick_count / self.cpr) * self.wheel_dia * 3.14
        #     actual_distance = (actual_distance_l + actual_distance_r) / 2
            

        #     # If the robot hasn't moved the specified distance, apply corrections
        #     if actual_distance < self.distance:
        #         # Reset the start time to continue moving
        #         self.start_time = time.time()

        #         # Continue with the previous measured velocities
        #         self.left_pub.publish(Float32(data=float(self.l_rpm)))
        #         self.right_pub.publish(Float32(data=float(self.r_rpm)))
        # else:
        #     # Publish the calculated velocities to continue movings
        #     self.left_pub.publish(Float32(data=float(self.l_rpm)))
        #     self.right_pub.publish(Float32(data=float(self.r_rpm)))

        # Update previous position and time
        self.prev_l_tick_count = self.tick_count.x
        self.prev_r_tick_count = self.tick_count.y
        self.prev_time = time.time_ns()

def main(args=None):
    rclpy.init(args=args)
    ctrlr = DiffDriveCtrl()
    rclpy.spin(ctrlr)
    ctrlr.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
