# Reed Billedo 228005190; CSCE 452; Project 1
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
                angle5 = msg.angle_min + i * msg.angle_increment
                if(i <= len(msg.ranges)/2):
                    tx = msg.ranges[i] * math.cos(angle5)
                    ty = msg.ranges[i] * math.sin(angle5) 
                else:
                    tx = msg.ranges[i] * math.cos(angle5) 
                    ty = msg.ranges[i] * math.sin(angle5)
                maxDistancesPoints.append([tx,ty])
        else:
            previousData = data
            data = msg.ranges
            curData = msg.ranges
            t = 10
            # for(i in range(len(curData)-1)):
            #     if(curData[i] >= curData[i]-self.tol and curData[i] <= curData[i]+self.tol):
                    
            pts = []
            count = 0
            for i in range(len(msg.ranges)):
                if(curData[i] == float("inf")):
                    msg.ranges[i] = msg.range_max
                    curData[i] = msg.range_max
                if(curData[i] < maxDistances[i] - distanceTolerance and curData[i] != float("inf") and curData[i] != float("nan")):
                    angle2 = msg.angle_increment*i - 1.5646604299545288
                    angle4 = 3.14159 - msg.angle_increment*i
                    angle = msg.angle_min + i * msg.angle_increment
                    if(i <= len(msg.ranges)/2):
                        x = msg.ranges[i] * math.cos(angle)
                        y = msg.ranges[i] * math.sin(angle) 
                        mx = maxDistances[i] *math.cos(angle)
                        my = maxDistances[i] *math.sin(angle)
                    else:
                        x = msg.ranges[i] * math.cos(angle) 
                        y = msg.ranges[i] * math.sin(angle)
                        mx = maxDistances[i] *math.cos(angle)
                        my = maxDistances[i] *math.sin(angle)
                    if not math.isinf(msg.ranges[i]) and not math.isnan(msg.ranges[i]) and self.distance_formula([x,y],[mx,my]) >= t:
                        pts.append([x,y])
            clusters = []
            xtotal = 0
            ytotal = 0
            for point in pts:
                found_cluster = False
                for cluster in clusters:
                    if self.distance_formula(cluster[0], point) < self.tol and point not in maxDistances:
                        cluster.append(point)
                        xtotal += point[0]
                        ytotal += point[1]
                        found_cluster = True
                        break
                if not found_cluster:
                    clusters.append([point])
                    count+=1

            pointcloud_msg = PointCloud()
            pointcloud_msg.header = msg.header
            for cluster in clusters:
                xa = sum([point[0] for point in cluster])/len(cluster)
                ya = sum([point[1] for point in cluster])/len(cluster)
                z=0.0
                
                found = False
                if(self.gindex == 0):
                    self.averages.append([xa, ya])
                else:
                    for p in self.prevAverages:
                        if(self.distance_formula(p, [xa,ya]) <= 0.03):
                            found = True
                    if(found == False):
                        pointcloud_msg.points.append(Point32(x=xa, 
                                                        y=ya, 
                                                        z=0.0))
                    self.averages.append([xa, ya])
                self.prevAverages = self.averages
                
                # pointcloud_msg.points.append(Point32(x=sum([point[0] for point in cluster])/len(cluster), 
                #                                     y=sum([point[1] for point in cluster])/len(cluster), 
                #                                     z=0.0))
            self.gindex += 1
            self.currentAmount = len(pointcloud_msg.points)
            self.pub.publish(pointcloud_msg)
            #self.get_logger().info('CURRENT AMOUNT: "%s"' % self.currentAmount )

            return clusters
        
    def distance_formula(self, x2, y2):
        # Formula for distance between two points: d = sqrt( (x2-x1)^2 + (y2-y2)^2 )
        x = x2[0] - y2[0]
        y = x2[1] - y2[1]
        length = math.sqrt(x**2 + y**2)
        return length

    def getClusters(self, msg):
        pts = []
        count = 0
        for i in range(len(msg.ranges)):
            angle2 = msg.angle_increment*i - 1.5646604299545288
            angle4 = 3.14159 - msg.angle_increment*i
            if not math.isinf(msg.ranges[i]) and not math.isnan(msg.ranges[i]):
                if(i <= len(msg.ranges)/2):
                    x = msg.ranges[i] * math.cos(angle2)
                    y = msg.ranges[i] * math.sin(angle2) 
                else:
                    x = msg.ranges[i] * math.cos(angle4) 
                    y = msg.ranges[i] * math.sin(angle4)
                pts.append([x,y])
        clusters = []
        xtotal = 0
        ytotal = 0
        for point in pts:
            found_cluster = False
            for cluster in clusters:
                if self.distance_formula(cluster[0], point) < self.tol:
                    cluster.append(point)
                    xtotal += point[0]
                    ytotal += point[1]
                    found_cluster = True
                    break
            if not found_cluster:
                clusters.append([point])
                count+=1

        pointcloud_msg = PointCloud()
        pointcloud_msg.header = msg.header
        for cluster in clusters:
            pointcloud_msg.points.append(Point32(x=sum([point[0] for point in cluster])/len(cluster), 
                                                  y=sum([point[1] for point in cluster])/len(cluster), 
                                                  z=0.0))
        self.pub.publish(pointcloud_msg)

        # self.get_logger().info('CLUSTER AMOUNT: "%s"' % count )
        # self.get_logger().info('CLUSTERS: "%s"' % clusters)
        # curPoint = Point32()
        # z = 0.0
        # curPoint.x = totalx/len(person)
        # curPoint.y = totaly/len(person)
        # curPoint.z = z
        # temp = PointCloud()
        # temp.header.frame_id = msg.header.frame_id
        # temp.points.append(curPoint)
        # self.pub.publish(temp)
        return clusters

    def test(self, msg):
        pointcloud_msg = PointCloud()
        pointcloud_msg.header = msg.header
        ps=[]
        if self.prev_scan is not None:
            filtered_ranges = []
            for i, r in enumerate(msg.ranges):
                # Calculate the speed of the object
                if not math.isnan(r):
                    x = r * math.cos(msg.angle_min + i * msg.angle_increment)
                    y = r * math.sin(msg.angle_min + i * msg.angle_increment)
                    x2 = self.prev_scan.ranges[i] * math.cos(msg.angle_min + i * msg.angle_increment)
                    y2 = self.prev_scan.ranges[i] * math.sin(msg.angle_min + i * msg.angle_increment)
                    speed = self.distance_formula([x,y],[x2,y2]) #/ (msg.header.stamp - self.prev_scan.header.stamp).to_sec()
                    # If the object is moving, add it to the filtered ranges
                    if speed > 10000:
                        filtered_ranges.append(r)
                    else:
                        filtered_ranges.append(float('nan'))
            # Publish the filtered ranges as a PointStamped message
            filtered_scan = LaserScan(header=msg.header, angle_min=msg.angle_min, angle_max=msg.angle_max, angle_increment=msg.angle_increment, range_min=msg.range_min, range_max=msg.range_max, ranges=filtered_ranges, intensities=[])
            
            for i, r in enumerate(filtered_ranges):
                if not math.isnan(r) and not math.isinf(r):
                    x = r * math.cos(msg.angle_min + i * msg.angle_increment)
                    y = r * math.sin(msg.angle_min + i * msg.angle_increment)
                    curPoint = Point32()
                    z = 0.0
                    curPoint.x = x
                    curPoint.y = y
                    curPoint.z = z
                    temp = PointCloud()
                    temp.header.frame_id = msg.header.frame_id
                    temp.points.append(curPoint)
                    pointcloud_msg.points.append(curPoint)
                    ps.append([x,y])
            clusters = []
            xtotal = 0
            ytotal = 0
            for point in ps:
                found_cluster = False
                for cluster in clusters:
                    if self.distance_formula(cluster[0], point) < self.tol:
                        cluster.append(point)
                        xtotal += point[0]
                        ytotal += point[1]
                        found_cluster = True
                        break
                if not found_cluster:
                    clusters.append([point])
                   # count+=1

            pointcloud_msg2 = PointCloud()
            pointcloud_msg2.header = msg.header
            for cluster in clusters:
                pointcloud_msg2.points.append(Point32(x=sum([point[0] for point in cluster])/len(cluster), 
                                                    y=sum([point[1] for point in cluster])/len(cluster), 
                                                    z=0.0))
            self.pub.publish(pointcloud_msg2)
        #self.pub.publish(pointcloud_msg)
                    
        self.prev_scan = msg

    def receive_callback(self, msg):
        #self.get_logger().info('Message from /scan: "%s"' % msg.ranges )
        #self.getClusters(msg)
        self.parse(msg)
        #self.test(msg)

       

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
