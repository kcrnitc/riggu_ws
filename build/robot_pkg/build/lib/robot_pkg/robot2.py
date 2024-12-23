import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion,Vector3
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
import numpy as np
import math


LIFT_SPEED = 300
MOTOR_SPEED_SCALE_FACTOR = 0.6

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('robot_2')
        self.publisher_ = self.create_publisher(Quaternion,"/motor_vel",1)
        self.publisher_piston = self.create_publisher(Int16,"/piston",1)
        self.publisher_lift = self.create_publisher(Int16,"/lift",1)


        self.subscriber_ = self.create_subscription(Joy,"/joy", self.subscriber_callback,10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer_piston = self.create_timer(timer_period, self.piston_callback)

        self.timer_lift = self.create_timer(timer_period, self.lift_callback)
        self.joy_val = [[0.0,0.0,0.0,0.0],
                        [0.0,0.0],
                        [0.0,0.0],
                        0.0,
                        0.0]
        self.piston_val = []
        self.gripper_state = {"left":0,"right":0}

        self.piston_state = {"lift":0,"throw":0}

        #self.q = LifoQueue()
        self.piston_prev = [0, 0]
        self.gripper_prev = [0, 0]


    def timer_callback(self):
        msg = Quaternion() 
        
        
        vx,vy,omega,m4= self.joy_val[0] 
                           # Controller axes are flipped along the y-axis
        vx=-vx
        print("RAW ", vx, vy, omega)

        print("Transformings")
        transform_matrix = np.array([[-0.33, 0.58, 0.33],
                                     [-0.33, -0.58, 0.33],
                                     [0.67, 0, 0.33]])
        vel_chassis_array = np.array([[vx],
                                      [vy],
                                      [omega]])
        wheel_vel = MOTOR_SPEED_SCALE_FACTOR * np.transpose(np.dot(transform_matrix, vel_chassis_array))
        
        m1 = wheel_vel[0][0] * 1024 
        m3 = wheel_vel[0][1] * 1024
        m2 = wheel_vel[0][2] * 1024
       
        msg.x, msg.y, msg.z, msg.w = m1, m2, m3, m4 *1024
        print(msg.x, msg.y, msg.z, msg.w)
        
        self.publisher_.publish(msg)

    def piston_callback(self):
        msg = Int16()
        piston_state = self.joy_val[3]
        if (piston_state == -1):                #Axis are reversed
            msg.data = 1
            self.publisher_piston.publish(msg)    
        if (piston_state == 1):
            msg.data = 0
            self.publisher_piston.publish(msg)            

    def lift_callback(self):
        msg = Int16()
        msg.data = int(LIFT_SPEED * self.joy_val[4])
        self.publisher_lift.publish(msg)

    def subscriber_callback(self,msg : Joy):
        self.joy_val =  [[msg.axes[0],msg.axes[1],msg.axes[3],msg.axes[4]],
                         [msg.buttons[4],msg.buttons[5]],
                         [msg.buttons[1],msg.buttons[0]], 
                         msg.axes[6], 
                         msg.axes[7]]
        
        # self.q.put(msg)
    
        


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()