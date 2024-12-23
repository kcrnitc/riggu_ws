import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

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

        self.integral_left = 0.0
        self.integral_right = 0.0
        self.prev_error_left = 0.0
        self.prev_error_right = 0.0
        self.prev_time = self.get_clock().now()

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

    def left_wheel_callback(self, msg):
        self.l_rpm = msg.data
        self.calculate_control_effort()

    def right_wheel_callback(self, msg):
        self.r_rpm = msg.data
        self.calculate_control_effort()

    def tick_count_callback(self, msg):
        self.tick_count = msg

    def recv_cmd_vel(self, msg):
        self.t_vl = msg.linear.x - (msg.angular.z * self.wheel_separation / 2)
        self.t_vr = msg.linear.x + (msg.angular.z * self.wheel_separation / 2)

    def pid_callback(self, msg):
        self.kp = msg.x
        self.ki = msg.y
        self.kd = msg.z

    def calculate_control_effort(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt == 0:
            return

        error_left = self.t_vl - self.l_rpm
        error_right = self.t_vr - self.r_rpm

        self.integral_left += error_left * dt
        self.integral_right += error_right * dt

        derivative_left = (error_left - self.prev_error_left) / dt
        derivative_right = (error_right - self.prev_error_right) / dt

        control_effort_left = (self.kp * error_left) + (self.ki * self.integral_left) + (self.kd * derivative_left)
        control_effort_right = (self.kp * error_right) + (self.ki * self.integral_right) + (self.kd * derivative_right)

        self.left_pub.publish(Float32(data=control_effort_left))
        self.right_pub.publish(Float32(data=control_effort_right))

        self.prev_error_left = error_left
        self.prev_error_right = error_right
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()