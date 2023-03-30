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
    def getX(self, val, i, _size, angleG):
        angle = angleG*i - 1.5646604299545288
        angle2 = angleG*i - 1.5646604299545288
        angle3 = 1.5646604299545288 - angleG*i
        angle4 = 3.14159 - angleG*i
        if(i <= _size/2):
            x = val * math.cos(angle2) 
            y = val * math.sin(angleG * i) 
        else:
            angle = 1.5646604299545288 - angleG*i
            x = val * math.cos(angle4) 
            y = val * math.sin(angle * i) 
        return x
    
    def getY(self, val, i, _size, angleG):
        angle = 1.5646604299545288 - angleG*i
        angle2 = angleG*i - 1.5646604299545288
        angle4 = 3.14159 - angleG*i
        if(i <= _size/2):
            x = val * math.cos(angle2)
            y = val * math.sin(angle2) 
        else:
            
            x = val * math.cos(angle4) 
            y = val * math.sin(angle4) 
        return y
    
    def parse(self, msg): 
        global data
        if not data:
            data = msg.ranges
        else:
            previousData = data
            data = msg.ranges

            

            for i in range(len(data)-5):
                count = 0
                xs = []
                ys = []
                if(previousData[i] == float("inf") and data[i] != float("inf")):
                    count += 1
                    xs.append(self.getX(data[i], i, len(data), msg.angle_increment))
                    ys.append(self.getY(data[i], i, len(data), msg.angle_increment))
                if(previousData[i+1] == float("inf") and data[i+1] != float("inf")):
                    count += 1
                    xs.append(self.getX(data[i+1], i+1, len(data), msg.angle_increment))
                    ys.append(self.getY(data[i+1], i+1, len(data), msg.angle_increment))
                if(previousData[i+2] == float("inf") and data[i+2] != float("inf")):
                    count += 1
                    xs.append(self.getX(data[i+2], i+2, len(data), msg.angle_increment))
                    ys.append(self.getY(data[i+2], i+2, len(data), msg.angle_increment))
                if(previousData[i+3] == float("inf") and data[i+3] != float("inf")):
                    count += 1
                    xs.append(self.getX(data[i+3], i+3, len(data), msg.angle_increment))
                    ys.append(self.getY(data[i+3], i+3, len(data), msg.angle_increment))
                if(previousData[i+4] == float("inf") and data[i+4] != float("inf")):
                    count += 1
                    xs.append(self.getX(data[i+4], i+4, len(data), msg.angle_increment))
                    ys.append(self.getY(data[i+4], i+4, len(data), msg.angle_increment))
                if(count >= 3):
                    self.get_logger().info('Message from /scan: "%s"' % msg.header )
                    totalx = 0
                    totaly = 0
                    for v in xs:
                        totalx += v
                    for v2 in ys:
                        totaly += v2
                    totalx /= count
                    totaly /= count
                    curPoint = Point32()
                    
                    
                    z = 0.0
                    curPoint.x = totalx
                    curPoint.y = totaly
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