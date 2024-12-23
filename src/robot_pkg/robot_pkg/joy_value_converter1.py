import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math
from sensor_msgs.msg import Joy 
speed=0
dirn=0
l1=0
r1=0
class JoyValueConverter(Node):

   def __init__(self):
        super().__init__('joy_value_converter')
        self.subscription = self.create_subscription(Joy, 'joy', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Twist, 'converted_twist', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
   def listener_callback(self, msg):
       global speed, dirn,l1,r1 
       x,y,l,r= msg.axes[0], msg.axes[1], msg.axes[2],msg.axes[5]
       l1=(1-((l+1)/2))*1024
       r1=(1-((r+1)/2))*1024
       speed=(1024*math.sqrt(x**2+y**2))
       if(y > -x and x < y):
          dirn=1
       elif(y < -x and y > x):
          dirn=2
       elif(y < x and y < -x):
          dirn=3
       elif(y < x and y > -x):
          dirn=4        
       elif (y==0 and x==0):
       	  dirn=0

   def timer_callback(self):
      msgs = Twist()
      msgs.linear.x = float(speed)
      msgs.linear.y = float(l1) 
      msgs.linear.z = float(r1) 
      msgs.angular.x = float(dirn)   
      self.publisher_.publish(msgs)
      self.get_logger().info(f'Publishing: linear={msgs.linear.x}, angular={msgs.angular.x}')

def main(args=None):
    rclpy.init(args=args)
    joy_value_converter = JoyValueConverter()
    rclpy.spin(joy_value_converter)
    joy_value_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
