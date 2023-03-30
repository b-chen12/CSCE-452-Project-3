import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from example_interfaces.msg import Int64
from rclpy.node import Node
from turtlesim.msg import Color
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy as np
import math
import statistics

# global variables needed across the program
data = []
maxDistances = []
distanceTolerance = 1
iter = 0
class CircleAndColorTurtle(Node):

    # initializes the node; publishes messages of type Twist to /turtle1/cmd_vel; subscribes to messages of type Color from /turtle1/color_sensor
    def __init__(self):
        super().__init__('CircleAndColorTurtle')
        
        self.pub = self.create_publisher(PointCloud, '/person_locations', 10)
        # timer = 0.1  # turtle moves every 1 second
        # self.timer = self.create_timer(timer, self.timer_callback)

        self.sub = self.create_subscription(LaserScan, '/scan', self.receive_callback, 10)
        self.sub 

        self.tol = 0.7
        self.gindex = 0
        self.averages = []
        self.prevAverages = []
        self.currentAmount = 0
        self.prev_scan = None


    def receive_callback(self, msg):
            #self.get_logger().info('Message from /scan: "%s"' % msg.ranges )
            #self.getClusters(msg)
            self.parse(msg)
            #self.test(msg)
    
    def parse(self, msg): 
        global data
        global maxDistances
        global distanceTolerance
        global iter
        distanceTolerance = 10
        personTolerance = 0.1
        
        personPoints = dict(angle = 0, xpoint = 0, ypoint = 0)
        person = []
        totalx = 0
        totaly = 0
        
        if not maxDistances:
            maxDistancesPoints = []
            maxDistances = msg.ranges
            curData = msg.ranges
            for i in range(len(maxDistances)):
                if math.isinf(maxDistances[i]):
                    #self.get_logger().info('CURRENT AMOUNT' )
                    maxDistances[i] = msg.range_max

            
        else:
            previousData = data
            data = msg.ranges
            curData = msg.ranges
            t = 10
            # for(i in range(len(curData)-1)):
            #     if(curData[i] >= curData[i]-self.tol and curData[i] <= curData[i]+self.tol):
                    
            pts = []
            count = 0
            pointcloud_msg = PointCloud()
            pointcloud_msg.header = msg.header
            for i in range(len(msg.ranges)):
                if(msg.ranges[i] > maxDistances[i]  and not math.isinf(msg.ranges[i]) and not math.isnan(msg.ranges[i])):
                    maxDistances[i] = msg.ranges[i]
                angle5 = msg.angle_min + i * msg.angle_increment
                tx = maxDistances[i] * math.cos(angle5)
                ty = maxDistances[i] * math.sin(angle5)
                pointcloud_msg.points.append(Point32(x=tx, y=ty, z=0.0))

            self.pub.publish(pointcloud_msg)
                

        
    def distance_formula(self, x2, y2):
        # Formula for distance between two points: d = sqrt( (x2-x1)^2 + (y2-y2)^2 )
        x = x2[0] - y2[0]
        y = x2[1] - y2[1]
        length = math.sqrt(x**2 + y**2)
        return length

       

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
