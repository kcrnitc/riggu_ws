#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3
from simple_pid import PID

import time
import tkinter as tk

class diff_drive_ctrl(Node):

    def __init__(self):
        super().__init__('diff_drive_controller')

        self.cpr = 125 # encoder counts per revolution
        self.wheel_dia = .125 # meters
        self.wheel_separation = 0.43 # meters
        self.ticksPerMeter = self.cpr / (self.wheel_dia * 3.14)

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0

        self.t_vl = 0
        self.t_vr = 0
        self.l_rpm = 0
        self.r_rpm = 0

        self.tick_count = Vector3()

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

        self.c_vl = 0
        self.c_vr = 0

        self.bot_vel = Twist()
        self.rate = self.create_rate(8, self.get_clock())
    
    def recv_cmd_vel(self, msg: Twist):
        self.bot_vel = msg
        
        v = self.bot_vel.linear.x
        w = self.bot_vel.angular.z
        
        self.t_vl = v - (w * self.wheel_separation / 2)
        self.t_vr = v + (w * self.wheel_separation / 2)
        self.l_rpm = -(self.t_vl * 60) / (3.14 * self.wheel_dia)
        self.r_rpm = -(self.t_vr * 60) / (3.14 * self.wheel_dia)

        self.tvl_pub.publish(Float32(data=self.t_vl * 1.0))
        self.tvr_pub.publish(Float32(data=self.t_vr * 1.0))

    def left_wheel_callback(self, msg: Float32):
        self.c_vl = msg.data / self.cpr * self.wheel_dia * 3.14
        self.cvl_pub.publish(Float32(data=float(self.c_vl)))

    def right_wheel_callback(self, msg: Float32):
        self.c_vr = msg.data / self.cpr * self.wheel_dia * 3.14
        self.cvr_pub.publish(Float32(data=float(self.c_vr)))

    def tick_count_callback(self, msg: Vector3):
        self.tick_count.x = msg.x
        self.tick_count.y = msg.y
        self.tick_count.z = msg.z

def main(args=None):
    rclpy.init(args=args)

    ctrlr = diff_drive_ctrl()
    
    pidl = PID(0, 0, 0, setpoint=0, output_limits=(-100, 100))
    pidr = PID(0, 0, 0, setpoint=0, output_limits=(-100, 100))

    pwm_r = Float32()
    pwm_l = Float32()

    prev_l_tick_count = 0
    prev_r_tick_count = 0
    prev_time = time.time_ns()

    def update_pid_values():
        ctrlr.kp = float(kp_slider.get())
        ctrlr.ki = float(ki_slider.get())
        ctrlr.kd = float(kd_slider.get())

    root = tk.Tk()
    root.title("PID Tuning")

    tk.Label(root, text="Kp").pack()
    kp_slider = tk.Scale(root, from_=0, to_=10, resolution=0.1, orient=tk.HORIZONTAL)
    kp_slider.pack()

    tk.Label(root, text="Ki").pack()
    ki_slider = tk.Scale(root, from_=0, to_=10, resolution=0.1, orient=tk.HORIZONTAL)
    ki_slider.pack()

    tk.Label(root, text="Kd").pack()
    kd_slider = tk.Scale(root, from_=0, to_=10, resolution=0.1, orient=tk.HORIZONTAL)
    kd_slider.pack()

    tk.Button(root, text="Update PID", command=update_pid_values).pack()

    def spin_once():
        rclpy.spin_once(ctrlr, timeout_sec=0.1)
        root.after(100, spin_once)

    root.after(100, spin_once)

    while rclpy.ok():
        c_time = time.time_ns()
        dt = (c_time - prev_time) * 1e-9

        t_vr = ctrlr.t_vr
        t_vl = ctrlr.t_vl

        current_l_tick_count = ctrlr.tick_count.x
        current_r_tick_count = ctrlr.tick_count.y

        c_vl = (current_l_tick_count - prev_l_tick_count) / dt
        c_vr = (current_r_tick_count - prev_r_tick_count) / dt

        c_vl = c_vl / ctrlr.cpr * ctrlr.wheel_dia * 3.14
        c_vr = c_vr / ctrlr.cpr * ctrlr.wheel_dia * 3.14

        el = c_vl - t_vl
        er = c_vr - t_vr

        pidl.Kd = ctrlr.kd
        pidl.Ki = ctrlr.ki
        pidl.Kp = ctrlr.kp

        pidr.Kd = ctrlr.kd
        pidr.Ki = ctrlr.ki
        pidr.Kp = ctrlr.kp

        pidl.setpoint = ctrlr.t_vl
        pidr.setpoint = ctrlr.t_vr

        ctrl_effort_left = pidl(el)
        ctrl_effort_right = pidr(er)

        pwm_l.data = float(ctrl_effort_left)
        pwm_r.data = float(ctrl_effort_right)
        
        ctrlr.left_pub.publish(pwm_l)
        ctrlr.right_pub.publish(pwm_r)

        prev_l_tick_count = current_l_tick_count
        prev_r_tick_count = current_r_tick_count
        prev_time = c_time

        time.sleep(0.1)

    ctrlr.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()