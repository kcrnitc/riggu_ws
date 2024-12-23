#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String,Float32
from geometry_msgs.msg import Pose,Twist,Vector3
from simple_pid import PID

import time

class diff_drive_ctrl(Node):

    def __init__(self):
        super().__init__('diff_drive_controller')

        self.cpr = 125 #encoder counts per revolution
        self.wheel_dia = .125 #meters
        self.wheel_separation = 0.43 #meters
        self.ticksPerMeter = self.cpr/(self.wheel_dia*3.14)

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
        self.right_wheel_sub = self.create_subscription(Vector3, "tick_count", self.tick_count_callback, 1)

        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.recv_cmd_vel, 1)

        self.left_pub = self.create_publisher(Float32, "left_wheel/control_effort",1)
        self.right_pub = self.create_publisher(Float32, "right_wheel/control_effort",1)

        self.cvl_pub = self.create_publisher(Float32, "left_measured_vel",1)
        self.cvr_pub = self.create_publisher(Float32, "right_measured_vel",1)
    
        self.tvl_pub = self.create_publisher(Float32, "left_setpoint",1)
        self.tvr_pub = self.create_publisher(Float32, "right_setpoint",1)

        self.pid_sub = self.create_subscription(Vector3, "pid_tune", self.pid_callback, 1)

        self.c_vl = 0
        self.c_vr = 0

        self.bot_vel = Twist()
        self.rate = self.create_rate(8,self.get_clock())
    
    def recv_cmd_vel(self,msg:Twist):
        self.bot_vel = msg
        
        v = self.bot_vel.linear.x
        w =self.bot_vel.angular.z
        
        self.t_vl = v - (w*self.wheel_separation/2)
        self.t_vr = v + (w*self.wheel_separation/2)
        self.l_rpm = -(self.t_vl*60)/(3.14*self.wheel_dia)
        self.r_rpm = -(self.t_vr*60)/(3.14*self.wheel_dia)

        # self.left_pub.publish(Float32(data=self.l_rpm *1.0))
        # self.right_pub.publish(Float32(data=self.r_rpm *1.0))

        self.tvl_pub.publish(Float32(data=self.t_vl *1.0))
        self.tvr_pub.publish(Float32(data=self.t_vr *1.0))
        pass

    def left_wheel_callback(self,msg:Float32):
        self.c_vl=msg.data/self.cpr*self.wheel_dia*3.14
        self.cvl_pub.publish(Float32(data=float(self.c_vl)))    
        pass
    def right_wheel_callback(self,msg:Float32):
        self.c_vr=msg.data/self.cpr*self.wheel_dia*3.14
        self.cvr_pub.publish(Float32(data=float(self.c_vr)))
        pass

    def tick_count_callback(self,msg:Vector3):
        self.tick_count.x = msg.x
        self.tick_count.y = msg.y
        self.tick_count.z = msg.z
        pass

    def pid_callback(self,msg:Vector3):
        self.kp = msg.x
        self.ki = msg.y
        self.kd = msg.z
        pass
    def add_pid_loop(self,fn):
        fn
        pass



def main(args=None):
    rclpy.init(args=args)

    ctrlr = diff_drive_ctrl()
    rclpy.spin(ctrlr)
   
    pidl = PID(0,0,0,setpoint=0,output_limits=(-100,100))
    pidr = PID(0,0,0,setpoint=0,output_limits=(-100,100))

    pwm_r=Float32()
    pwm_l=Float32()

    # pidl.output_limits = (-5,5)
    # pidr.output_limits = (-5,5)
    prev_l_tick_count = 0
    prev_r_tick_count = 0
    c_time = time.time_ns()
    prev_time = 0
    dt = 0.0
    tc = Vector3()

    #while tc != ctrlr.tick_count:

    tc = ctrlr.tick_count
    c_time = time.time_ns()
    dt = (c_time-prev_time)*10e-9

    t_vr = ctrlr.t_vr
    t_vl = ctrlr.t_vl

    current_l_tick_count = ctrlr.tick_count.x
    current_r_tick_count = ctrlr.tick_count.y

    c_vl = (current_l_tick_count - prev_l_tick_count)/dt
    c_vr = (current_r_tick_count - prev_r_tick_count)/dt
    print('dt: ',dt)
    print("Left_vel : ",c_vl)
    print("Right_vel : ",c_vr)
    c_vl = c_vl/ctrlr.cpr*ctrlr.wheel_dia*3.14
    c_vr = c_vr/ctrlr.cpr*ctrlr.wheel_dia*3.14
    # ctrlr.cvl_pub.publish(Float32(data=float(c_vl)))
    # ctrlr.tvl_pub.publish(Float32(data=float(t_vl)))
    ctrlr.tvr_pub.publish(Float32(data=float(t_vr)))
    ctrlr.cvr_pub.publish(Float32(data=float(c_vr)))
    # ctrlr.left_pub.publish(Float32(data=ctrlr.l_rpm *1.0))
    # ctrlr.right_pub.publish(Float32(data=ctrlr.r_rpm *1.0))


    el = c_vl-t_vl
    er = c_vr-t_vr

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
    
    print("params :",ctrlr.kp,ctrlr.ki,ctrlr.kd)

    print("Left : ",ctrl_effort_left)
    print("Right : ",ctrl_effort_right)

    ctrlr.left_pub.publish(data=pwm_l)
    ctrlr.right_pub.publish(data=pwm_r *-1.0)
    prev_l_tick_count = current_l_tick_count
    prev_r_tick_count = current_r_tick_count
    prev_time = c_time
    time.sleep(0.5)
       

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ctrlr.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
