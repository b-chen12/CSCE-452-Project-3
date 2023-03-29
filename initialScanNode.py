# Reed Billedo 228005190; CSCE 452; Project 1
import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from example_interfaces.msg import Int64
from rclpy.node import Node
from turtlesim.msg import Color
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy
import math

# global variables needed across the program
data = []


class CircleAndColorTurtle(Node):

    # initializes the node; publishes messages of type Twist to /turtle1/cmd_vel; subscribes to messages of type Color from /turtle1/color_sensor
    def __init__(self):
        super().__init__('CircleAndColorTurtle')
        
        self.pub = self.create_publisher(PointCloud, '/person_locations', 10)
        # timer = 0.1  # turtle moves every 1 second
        # self.timer = self.create_timer(timer, self.timer_callback)

        self.sub = self.create_subscription(LaserScan, '/scan', self.receive_callback, 10)
        self.sub 

    # creates a message that contains the direction and angle of the turtle to go in a circle
    # def timer_callback(self):
    #     msg = Int64()
        
    #     msg.data = (100)

    #     self.pub.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg)   # can be used to output the dierection / angle of turtle in real time 
    # outputs on the console the message that was received from the topic turtle1/color_sensor
    def parse(self, msg): 
        global data
        if not data:
            data = msg.ranges
        else:
            previousData = data
            data = msg.ranges

            for i in range(len(data)):
                if(previousData[i] == float("inf") and data[i] != float("inf")):
                    self.get_logger().info('Message from /scan: "%s"' % msg.header )
                    curPoint = Point32()
                    
                    if(i <= len(data)/2):
                        x = msg.ranges[i] * math.sin(msg.angle_increment * i) * -1
                        y = msg.ranges[i] * math.cos(msg.angle_increment * i) * -1
                    else:
                        angle = 1.5646604299545288 - msg.angle_increment*i
                        x = msg.ranges[i] * math.sin(angle * i) * -1
                        y = msg.ranges[i] * math.cos(angle * i) * -1
                    z = 0.0
                    curPoint.x = x
                    curPoint.y = y
                    curPoint.z = z
                    temp = PointCloud()
                    temp.header.frame_id = msg.header.frame_id
                    temp.points.append(curPoint)
                    self.pub.publish(temp)



    def receive_callback(self, msg):
        
        

      #  self.get_logger().info('Message from /scan: "%s"' % msg.header )
        self.parse(msg)

       

# the node that allows the turtle to move in a circle and detect color is created; it then is called through spin
def main(args=None):
    rclpy.init(args=args)
    turtle = CircleAndColorTurtle()

    rclpy.spin(turtle)

    # Destroy node
    turtle.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
