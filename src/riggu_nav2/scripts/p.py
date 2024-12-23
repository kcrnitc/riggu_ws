import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterType

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_p_controller')
        self.cpr = 125 # encoder counts per revolution
        self.wheel_dia = .125 # meters
        self.wheel_separation = 0.43 # meters
        self.t_vl=0.0
        self.t_vr=0.0
        self.Kp=0

        # Setpoint velocities (published from another node)
        self.setpoint_sub = self.create_subscription(Twist, '/cmd_vel', self.setpoint_callback, 10)
        


        self.left_stpt_pub = self.create_publisher(Float32, '/left_wheel/setpoint',  10)
        self.right_stpt_pub = self.create_publisher(Float32, '/right_wheel/setpoint', 10)
        
        # Subscriber for Kp value (published from another node)
        self.kp_sub = self.create_subscription(Float32, '/kp_value', self.kp_callback, 10)
        
        # Current velocity for ramping
        self.current_left_velocity = 0.0
        self.current_right_velocity = 0.0

        # Measured velocities
        self.left_measured_velocity = 0.0  # From left wheel encoder
        self.right_measured_velocity = 0.0  # From right wheel encoder

        # Publishers for control effort (RPM)
        self.left_wheel_pub = self.create_publisher(Float32, '/left_wheel/control_effort', 10)
        self.right_wheel_pub = self.create_publisher(Float32, '/right_wheel/control_effort', 10)

        self.left_measured_pub = self.create_publisher(Float32, '/left_measured', 10)
        self.right_measured_pub = self.create_publisher(Float32, '/right_measured', 10)

        # Subscribers for encoder tick rates
        self.left_encoder_sub = self.create_subscription(Float32, '/left_tickrate', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(Float32, '/right_tickrate', self.right_encoder_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # Control loop runs every 100ms

        # Ramping parameters
        self.max_acceleration = 0.05  # Max velocity change per control loop (in RPM)

    def setpoint_callback(self, msg):
        self.bot_vel = msg
        
        v = self.bot_vel.linear.x
        w = self.bot_vel.angular.z
        
        self.t_vl = v - (w * self.wheel_separation / 2)
        self.t_vr = v + (w * self.wheel_separation / 2)
        self.l_rpm = -(self.t_vl * 60) / (3.14 * self.wheel_dia)
        self.r_rpm = -(self.t_vr * 60) / (3.14 * self.wheel_dia)

        
        self.left_stpt_pub.publish(Float32(data=self.t_vl * 1.0))
        self.right_stpt_pub.publish(Float32(data=self.t_vr * 1.0))


          

    def kp_callback(self, msg):
        self.Kp = msg.data

    def left_encoder_callback(self, msg):
        # Assuming the encoder tick rate is published as tick per second
       self.left_measured_velocity = msg.data / self.cpr * self.wheel_dia * 3.14
       self.left_measured_pub.publish(Float32(data=float(self.left_measured_velocity)))

    def right_encoder_callback(self, msg):
        # Assuming the encoder tick rate is published as tick per second
        self.right_measured_velocity = msg.data / self.cpr * self.wheel_dia * 3.14
        self.right_measured_pub.publish(Float32(data=float(self.right_measured_velocity)))

    

    def p_control(self, setpoint, measured, Kp):
        # Proportional control effort
        error = setpoint - measured
        control_effort = Kp * error
        if error > 0.0:
               self.rpm = -(control_effort * 60) / (3.14 * self.wheel_dia)
        elif(error < 0.0):
               self.rpm = (control_effort * 60) / (3.14 * self.wheel_dia)
        else:
               self.rpm =setpoint
                       
        return self.rpm

    def ramp_velocity(self, current_velocity, target_velocity):
        # Gradually increase or decrease current velocity towards target_velocity
        if current_velocity < target_velocity:
            current_velocity += min(self.max_acceleration, target_velocity - current_velocity)
        elif current_velocity > target_velocity:
            current_velocity -= min(self.max_acceleration, current_velocity - target_velocity)
        return current_velocity

    def control_loop(self):
        # Read setpoint velocities (from rqt or manually set)
        left_setpoint = self.t_vl
        right_setpoint = self.t_vr

        # Gradually ramp the velocity
        self.current_left_velocity = self.ramp_velocity(self.current_left_velocity, left_setpoint)
        self.current_right_velocity = self.ramp_velocity(self.current_right_velocity, right_setpoint)
        # print (self.current_left_velocity)

        # Proportional control for left and right wheels
        left_control_effort = self.p_control(self.current_left_velocity, self.left_measured_velocity, self.Kp)
        right_control_effort = self.p_control(self.current_right_velocity, self.right_measured_velocity, self.Kp)

        # left_control_effort = self.p_control(left_setpoint, self.left_measured_velocity, self.Kp)
        # right_control_effort = self.p_control(right_setpoint, self.right_measured_velocity, self.Kp)

        # Publish control effort (RPM)
        self.left_wheel_pub.publish(Float32(data=left_control_effort))
        self.right_wheel_pub.publish(Float32(data=right_control_effort))

def main(args=None):
    rclpy.init(args=args)
    controller_node = DifferentialDriveController()
    rclpy.spin(controller_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()