import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3

import numpy as np

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_lqr_controller')
        self.cpr = 1820  # Encoder counts per revolution
        self.wheel_dia = 0.125  # Wheel diameter in meters
        self.wheel_separation = 0.43  # Wheel separation in meters
        self.t_vl = 0.0
        self.t_vr = 0.0
        self.l_rpm = 0.0
        self.r_rpm = 0.0

        # LQR-specific variables
        self.Ts = 0.05  # Sampling time
        self.A = np.eye(3)
        self.R = np.array([[0.01, 0],  # Penalty for linear velocity effort
                           [0, 0.01]])  # Penalty for angular velocity effort
        self.Q = np.array([[0.05, 0, 0],  # Penalize X position error
                           [0, 0.5, 0],  # Penalize Y position error 
                           [0, 0, 0.04]])  # Penalize YAW ANGLE heading error 
        self.max_linear_velocity = 1.5  # meters per second
        self.max_angular_velocity = 2.08  # radians per second
        self.dTol = 0.08
        self.refPose = np.array([5, 5, np.pi/2])
        self.odom_data = np.array([0, 0, 0])

        # Setpoint velocities (published from another node)
        self.setpoint_sub = self.create_subscription(Twist, '/cmd_vel', self.setpoint_callback, 10)

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
        self.timer = self.create_timer(self.Ts, self.control_loop)  # Control loop runs every 50ms

        # Additional publishers and subscribers from the provided excerpt
        self.bot_vel = Twist()
        self.tvl_pub = self.create_publisher(Float32, '/left_wheel/setpoint', 10)
        self.tvr_pub = self.create_publisher(Float32, '/right_wheel/setpoint', 10)
        self.cvl_pub = self.create_publisher(Float32, '/left_wheel/measured', 10)
        self.cvr_pub = self.create_publisher(Float32, '/right_wheel/measured', 10)
        self.tick_count = Vector3()
        self.tick_count_sub = self.create_subscription(Vector3, '/tick_count', self.tick_count_callback, 10)
        self.pid_sub = self.create_subscription(Vector3, '/pid_params', self.pid_callback, 10)

    def setpoint_callback(self, msg):
        self.bot_vel = msg

        v = self.bot_vel.linear.x
        w = self.bot_vel.angular.z

        self.t_vl = v - (w * self.wheel_separation / 2)  # Target velocity for the left wheel
        self.t_vr = v + (w * self.wheel_separation / 2)  # Target velocity for the right wheel

        self.l_rpm = -(self.t_vl * 60) / (np.pi * self.wheel_dia)
        self.r_rpm = (-(self.t_vr * 60) / (np.pi * self.wheel_dia))

        # Publishing the set RPM values
        self.left_wheel_pub.publish(Float32(data=self.l_rpm))
        self.right_wheel_pub.publish(Float32(data=self.r_rpm))

        self.tvl_pub.publish(Float32(data=self.t_vl))
        self.tvr_pub.publish(Float32(data=self.t_vr))

    def left_encoder_callback(self, msg):
        self.left_measured_velocity = (msg.data / self.cpr) * (self.wheel_dia * np.pi)
        self.cvl_pub.publish(Float32(data=float(self.left_measured_velocity)))

    def right_encoder_callback(self, msg):
        self.right_measured_velocity = (msg.data / self.cpr) * (self.wheel_dia * np.pi)
        self.cvr_pub.publish(Float32(data=float(self.right_measured_velocity)))

    def tick_count_callback(self, msg):
        self.tick_count.x = msg.x
        self.tick_count.y = msg.y
        self.tick_count.z = msg.z

    def pid_callback(self, msg):
        self.kp = msg.x
        self.ki = msg.y
        self.kd = msg.z

    def getB(self):
        B = np.array([[np.cos(self.odom_data[2]) * self.Ts, 0],
                      [np.sin(self.odom_data[2]) * self.Ts, 0],
                      [0, self.Ts]])
        return B

    def lqr(self, B):
        x_error = self.odom_data - self.refPose
        N = 50
        P = [None] * (N + 1)
        Qf = self.Q
        P[N] = Qf

        for i in range(N, 0, -1):
            P[i - 1] = self.Q + self.A.T @ P[i] @ self.A - (self.A.T @ P[i] @ B) @ np.linalg.pinv(
                self.R + B.T @ P[i] @ B) @ (B.T @ P[i] @ self.A)

        K = [None] * N
        u = [None] * N

        for i in range(N):
            K[i] = -np.linalg.pinv(self.R + B.T @ P[i + 1] @ B) @ B.T @ P[i + 1] @ self.A
            u[i] = K[i] @ x_error

        u_star = u[N - 1]
        return u_star

    def control_loop(self):
        B = self.getB()
        optimal_control_input = self.lqr(B)
        v = optimal_control_input[0]
        w = optimal_control_input[1]

        v = np.clip(v, -self.max_linear_velocity, self.max_linear_velocity)
        w = np.clip(w, -self.max_angular_velocity, self.max_angular_velocity)

        self.send_vel(v, w)

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.left_wheel_pub.publish(Float32(data=self.l_rpm))  # Publish final RPM for left wheel
        self.right_wheel_pub.publish(Float32(data=(self.r_rpm*1.1)))  # Publish final RPM for right wheel

def main(args=None):
    rclpy.init(args=args)
    controller_node = DifferentialDriveController()
    rclpy.spin(controller_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32
# from geometry_msgs.msg import Twist
# import gym
# import numpy as np
# from stable_baselines3 import PPO
# from gym import spaces
# import logging


# class DifferentialDriveEnv(gym.Env):
#     def __init__(self, robot_node):
#         super(DifferentialDriveEnv, self).__init__()

#         self.robot_node = robot_node  # Pass the ROS 2 node for publishing commands
#         self.max_linear_velocity = 1.5  # Maximum linear velocity
#         self.max_angular_velocity = 2.08  # Maximum angular velocity

#         # Define the action space (linear velocity, angular velocity)
#         self.action_space = spaces.Box(
#             low=np.array([-self.max_linear_velocity, -self.max_angular_velocity]),
#             high=np.array([self.max_linear_velocity, self.max_angular_velocity]),
#             dtype=np.float32
#         )

#         # Define the observation space (x, y, theta) - Using simulated state, no odometry data
#         self.observation_space = spaces.Box(
#             low=np.array([-np.inf, -np.inf, -np.pi]),
#             high=np.array([np.inf, np.inf, np.pi]),
#             dtype=np.float32
#         )

#         self.reset()

#     def reset(self):
#         # Initialize state (robot at origin facing along the x-axis)
#         self.state = np.array([0.0, 0.0, 0.0], dtype=np.float32)
#         return self.state

#     def step(self, action):
#         # Unpack action (linear velocity, angular velocity)
#         v, w = action

#         # Ensure velocities are within bounds
#         v = np.clip(v, -self.max_linear_velocity, self.max_linear_velocity)
#         w = np.clip(w, -self.max_angular_velocity, self.max_angular_velocity)

#         # Apply control to the robot using ROS 2
#         self.robot_node.send_vel(v, w)

#         # Simple state update (without actual odometry feedback)
#         # For simplicity, we simulate a simple movement model where the robot moves in a straight line with some angular motion
#         dt = 0.1  # Time step (control loop rate)
#         self.state[0] += v * np.cos(self.state[2]) * dt  # x position
#         self.state[1] += v * np.sin(self.state[2]) * dt  # y position
#         self.state[2] += w * dt  # orientation (theta)

#         # Define a simple reward (negative distance from the goal)
#         goal = np.array([5.0, 5.0])  # Example goal at (5, 5)
#         distance = np.linalg.norm(self.state[:2] - goal)
#         reward = -distance

#         # Done when robot reaches the goal (or some other condition)
#         done = distance < 0.2  # Close enough to goal

#         # Logging the state, reward, and goal distance
#         self.robot_node.get_logger().info(f"State: {self.state}, Reward: {reward}, Distance to goal: {distance}, Done: {done}")

#         return self.state, reward, done, {}

#     def render(self, mode='human'):
#         pass  # Not implementing visualization here


# class DifferentialDriveController(Node):
#     def __init__(self, env=None):
#         super().__init__('diff_drive_rl_controller')

#         # Initialize logging
#         self.get_logger().info("Initializing DifferentialDriveController")

#         # Initialize ROS 2 components and variables
#         self.cpr = 125  # Encoder counts per revolution
#         self.wheel_dia = 0.125  # Wheel diameter in meters
#         self.wheel_separation = 0.43  # Wheel separation in meters
#         self.max_linear_velocity = 1.5
#         self.max_angular_velocity = 2.08

#         # Publishers for control effort (RPM)
#         self.left_wheel_pub = self.create_publisher(Float32, '/left_wheel/control_effort', 10)
#         self.right_wheel_pub = self.create_publisher(Float32, '/right_wheel/control_effort', 10)

#         # Initialize environment and RL model
#         if env is not None:
#             self.env = env
#             self.model = PPO("MlpPolicy", self.env, verbose=1)  # PPO agent
#             self.model.learn(total_timesteps=10000)  # Train the agent for 10k timesteps
#             self.get_logger().info("RL Model trained successfully!")
#         else:
#             self.env = None
#             self.model = None

#         # State and control loop setup
#         self.state = self.env.reset() if self.env is not None else np.zeros(3)
#         self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz control loop

#     def send_vel(self, v, w):
#         msg = Twist()
#         msg.linear.x = v
#         msg.angular.z = w
#         self.left_wheel_pub.publish(Float32(data=v))  # Simplified for testing
#         self.right_wheel_pub.publish(Float32(data=w))  # Simplified for testing

#         # Log the velocity commands
#         self.get_logger().info(f"Sending velocity: Linear velocity = {v}, Angular velocity = {w}")

#     def control_loop(self):
#         if self.env is not None:
#             # Reshape state to match the model's expected input shape (batch of 1)
#             state_batch = np.expand_dims(self.state, axis=0)  # Shape (1, 3)

#             # Log the state before making a prediction
#             self.get_logger().info(f"State before prediction: {self.state}")

#             # Get action from RL agent (using the trained model)
#             action, _states = self.model.predict(state_batch)

#             # Log the action received from the model
#             self.get_logger().info(f"Action predicted: Linear velocity = {action[0]}, Angular velocity = {action[1]}")

#             # Apply the action to the robot
#             v, w = action

#             # Send velocities to robot
#             self.send_vel(v, w)

#             # Get the next state after taking the action
#             self.state, reward, done, _ = self.env.step(action)

#             # Log the reward and whether the episode is done
#             self.get_logger().info(f"Reward: {reward}, Done: {done}")

#             if done:
#                 self.state = self.env.reset()  # Reset if done


# def main(args=None):
#     rclpy.init(args=args)

#     # Create the controller node first (no env yet)
#     controller_node = DifferentialDriveController(env=None)  # Pass None for the env initially

#     # Now create the environment and assign it to the controller
#     controller_node.env = DifferentialDriveEnv(controller_node)  # Initialize environment
#     controller_node.model = PPO("MlpPolicy", controller_node.env, verbose=1)  # Create RL model with environment
#     controller_node.model.learn(total_timesteps=10000)  # Train the RL model for 10k timesteps

#     # Log the start of the main loop
#     controller_node.get_logger().info("RL Model trained successfully! Starting control loop")

#     rclpy.spin(controller_node)
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
