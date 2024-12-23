import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_p_i_controller')

        # Initialize gains with default values
        self.kp = 1.0  # Default proportional gain
        self.ki = 0.1  # Default integral gain

        # Declare parameters for Kp and Ki (optional, as we're using subscribers here)
        self.declare_parameter('Kp', self.kp)
        self.declare_parameter('Ki', self.ki)

        # Initialize other parameters
        self.cpr = 125  # Encoder counts per revolution
        self.wheel_dia = 0.125  # Wheel diameter in meters
        self.wheel_separation = 0.43  # Wheel separation in meters
        self.t_vl = 0.0
        self.t_vr = 0.0
        self.l_rpm = 0.0
        self.r_rpm = 0.0

        # Maximum allowed velocities
        self.max_linear_velocity = 1.5  # meters per second
        self.max_angular_velocity = 2.08  # radians per second

        # Desired reference pose (setpoint)
        self.refPose = np.array([5, 5, np.pi/2])  # Example reference pose [x, y, theta]

        # Robot's current pose (from odometry or sensors)
        self.odom_data = np.array([0, 0, 0])  # [x, y, theta]

        # Subscribers and Publishers
        self.setpoint_sub = self.create_subscription(Twist, '/cmd_vel', self.setpoint_callback, 10)
        self.left_wheel_pub = self.create_publisher(Float32, '/left_wheel/control_effort', 10)
        self.right_wheel_pub = self.create_publisher(Float32, '/right_wheel/control_effort', 10)

        # Measured velocities
        self.left_measured_velocity = 0.0
        self.right_measured_velocity = 0.0

        # Subscribers for encoder tick rates
        self.left_encoder_sub = self.create_subscription(Float32, '/left_tickrate', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(Float32, '/right_tickrate', self.right_encoder_callback, 10)

        # Subscriber for Kp and Ki values (to dynamically change the gains)
        self.kp_sub = self.create_subscription(Float32, '/kp_value', self.kp_callback, 10)
        self.ki_sub = self.create_subscription(Float32, '/ki_value', self.ki_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # Control loop runs every 50ms

        # Initialize integral error term
        self.integral_error = np.array([0.0, 0.0, 0.0])  # Integral error for [x, y, theta]
        self.prev_error = np.array([0.0, 0.0, 0.0])  # Previous error for integration

    def setpoint_callback(self, msg):
        # Extract velocity commands from the received Twist message
        v = msg.linear.x
        w = msg.angular.z

        # Calculate the target velocities for the left and right wheels
        self.t_vl = v - (w * self.wheel_separation / 2)
        self.t_vr = v + (w * self.wheel_separation / 2)

        # Convert target velocities to RPM for each wheel
        self.l_rpm = -(self.t_vl * 60) / (np.pi * self.wheel_dia)
        self.r_rpm = (-(self.t_vr * 60) / (np.pi * self.wheel_dia))

        # Publish the target RPM values to the motor controllers
        self.left_wheel_pub.publish(Float32(data=self.l_rpm))
        self.right_wheel_pub.publish(Float32(data=self.r_rpm))

    def left_encoder_callback(self, msg):
        # Calculate the left wheel's measured velocity from encoder ticks
        self.left_measured_velocity = (msg.data / self.cpr) * (self.wheel_dia * np.pi)

    def right_encoder_callback(self, msg):
        # Calculate the right wheel's measured velocity from encoder ticks
        self.right_measured_velocity = (msg.data / self.cpr) * (self.wheel_dia * np.pi)

    def kp_callback(self, msg):
        # Update Kp whenever a new value is received
        self.kp = msg.data
        self.get_logger().info(f"Updated Kp value: {self.kp}")

    def ki_callback(self, msg):
        # Update Ki whenever a new value is received
        self.ki = msg.data
        self.get_logger().info(f"Updated Ki value: {self.ki}")

    def control_loop(self):
        # Calculate the error in position (x, y) and heading (theta)
        x_error = self.refPose[0] - self.odom_data[0]
        y_error = self.refPose[1] - self.odom_data[1]
        theta_error = self.refPose[2] - self.odom_data[2]

        # Proportional control for linear velocity and angular velocity
        v_error = np.sqrt(x_error**2 + y_error**2)  # Error in position
        w_error = theta_error  # Error in orientation

        # Compute the integral of the errors
        self.integral_error += np.array([x_error, y_error, theta_error]) * 0.05  # Integrate over time (assuming 50ms control loop)

        # Compute control efforts for linear and angular velocities
        v_control = self.kp * v_error + self.ki * self.integral_error[0]
        w_control = self.kp * w_error + self.ki * self.integral_error[2]

        # Clamp the control efforts to maximum velocities
        v_control = np.clip(v_control, -self.max_linear_velocity, self.max_linear_velocity)
        w_control = np.clip(w_control, -self.max_angular_velocity, self.max_angular_velocity)

        # Publish the velocities as commands to the wheels
        self.send_vel(v_control, w_control)

    def send_vel(self, v, w):
        # Send control commands to the wheels
        msg_left = Float32()
        msg_right = Float32()

        # Convert desired linear and angular velocities to RPM for each wheel
        t_vl = v - (w * self.wheel_separation / 2)
        t_vr = v + (w * self.wheel_separation / 2)

        msg_left.data = -(t_vl * 60) / (np.pi * self.wheel_dia)
        msg_right.data = (-(t_vr * 60) / (np.pi * self.wheel_dia))

        # Publish the calculated RPMs
        self.left_wheel_pub.publish(msg_left)
        self.right_wheel_pub.publish(msg_right)

def main(args=None):
    rclpy.init(args=args)
    controller_node = DifferentialDriveController()
    rclpy.spin(controller_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
