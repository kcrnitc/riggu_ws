import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_pid_controller')
        self.cpr = 125  # Encoder counts per revolution
        self.wheel_dia = 0.125  # Wheel diameter in meters
        self.wheel_separation = 0.43  # Wheel separation in meters
        self.t_vl = 0.0
        self.t_vr = 0.0
        self.Kp = 0.5  # Proportional gain (initial value)
        self.Ki = 0.0  # Integral gain
        self.Kd = 0.05  # Derivative gain

        # PID-specific variables
        self.integral_error_left = 0.0
        self.integral_error_right = 0.0
        self.previous_error_left = 0.0
        self.previous_error_right = 0.0

        # Velocity ramping
        self.max_acceleration = 0.02  # Maximum change in velocity per control loop
        self.current_left_velocity = 0.0
        self.current_right_velocity = 0.0

        # Low-pass filter parameters
        self.alpha = 0.7  # Smoothing factor for low-pass filter
        self.previous_left_effort = 0.0
        self.previous_right_effort = 0.0

        # Setpoint velocities (published from another node)
        self.left_setpoint_sub = self.create_subscription(Float32, '/left_wheel/setpoint', self.left_setpoint_callback, 10)
        self.right_setpoint_sub = self.create_subscription(Float32, '/right_wheel/setpoint', self.right_setpoint_callback, 10)

        # Subscribers for PID gains (published from another node)
        self.kp_sub = self.create_subscription(Float32, '/kp_value', self.kp_callback, 10)
        self.ki_sub = self.create_subscription(Float32, '/ki_value', self.ki_callback, 10)
        self.kd_sub = self.create_subscription(Float32, '/kd_value', self.kd_callback, 10)

        # Publishers for control effort (RPM)
        self.left_wheel_pub = self.create_publisher(Float32, '/left_wheel/control_effort', 10)
        self.right_wheel_pub = self.create_publisher(Float32, '/right_wheel/control_effort', 10)

        # Measured velocities
        self.left_measured_velocity = 0.0  # From left wheel encoder
        self.right_measured_velocity = 0.0  # From right wheel encoder

        # Subscribers for encoder tick rates
        self.left_encoder_sub = self.create_subscription(Float32, '/left_tickrate', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(Float32, '/right_tickrate', self.right_encoder_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # Control loop runs every 50ms

    def left_setpoint_callback(self, msg):
        self.t_vl = msg.data

    def right_setpoint_callback(self, msg):
        self.t_vr = msg.data

    def kp_callback(self, msg):
        self.Kp = msg.data

    def ki_callback(self, msg):
        self.Ki = msg.data

    def kd_callback(self, msg):
        self.Kd = msg.data

    def left_encoder_callback(self, msg):
        self.left_measured_velocity = (msg.data / self.cpr) * (self.wheel_dia * 3.14)

    def right_encoder_callback(self, msg):
        self.right_measured_velocity = (msg.data / self.cpr) * (self.wheel_dia * 3.14)

    def pid_control(self, setpoint, measured, integral_error, previous_error, Kp, Ki, Kd):
        error = setpoint - measured
        integral_error += error  # Accumulate integral error
        derivative_error = error - previous_error  # Rate of change of error
        control_effort = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error)
        return control_effort, integral_error, error

    def ramp_velocity(self, current_velocity, target_velocity):
        if current_velocity < target_velocity:
            current_velocity += min(self.max_acceleration, target_velocity - current_velocity)
        elif current_velocity > target_velocity:
            current_velocity -= min(self.max_acceleration, current_velocity - target_velocity)
        return current_velocity

    def low_pass_filter(self, new_value, old_value, alpha):
        return alpha * new_value + (1 - alpha) * old_value

    def control_loop(self):
        # Read setpoint velocities
        left_setpoint = self.t_vl
        right_setpoint = self.t_vr

        # Gradually ramp the velocity
        self.current_left_velocity = self.ramp_velocity(self.current_left_velocity, left_setpoint)
        self.current_right_velocity = self.ramp_velocity(self.current_right_velocity, right_setpoint)

        # PID control for left and right wheels
        left_control_effort, self.integral_error_left, self.previous_error_left = self.pid_control(
            self.current_left_velocity, self.left_measured_velocity,
            self.integral_error_left, self.previous_error_left,
            self.Kp, self.Ki, self.Kd
        )

        right_control_effort, self.integral_error_right, self.previous_error_right = self.pid_control(
            self.current_right_velocity, self.right_measured_velocity,
            self.integral_error_right, self.previous_error_right,
            self.Kp, self.Ki, self.Kd
        )

        # Apply low-pass filter to smooth out the control effort
        filtered_left_effort = self.low_pass_filter(left_control_effort, self.previous_left_effort, self.alpha)
        filtered_right_effort = self.low_pass_filter(right_control_effort, self.previous_right_effort, self.alpha)

        # Publish control efforts (filtered)
        self.left_wheel_pub.publish(Float32(data=filtered_left_effort))
        self.right_wheel_pub.publish(Float32(data=filtered_right_effort))

        # Update previous control effort for the next iteration
        self.previous_left_effort = filtered_left_effort
        self.previous_right_effort = filtered_right_effort

def main(args=None):
    rclpy.init(args=args)
    controller_node = DifferentialDriveController()
    rclpy.spin(controller_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()