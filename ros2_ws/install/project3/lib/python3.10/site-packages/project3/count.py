import rclpy
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from example_interfaces.msg import Int64
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32MultiArray

class FilterAndCount(Node):
    # This node publishes onto the three separate topics
    def __init__(self):
        super().__init__('FilterAndCount')
        self.cluster_receiver = self.create_subscription(Float32MultiArray, '/clusters', self.listener_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_info, 10)
        
        self.person_count_curr = self.create_publisher(Int64, '/people_count_current', 10)
        self.person_count_tot = self.create_publisher(Int64, '/people_count_total', 10)
        self.person_location = self.create_publisher(PointCloud, '/person_locations', 10)
        self.laser_msg = None
        self.people = {}
        self.walls = []
        self.id = 0
        self.total = []
        
    
    def listener_callback(self, msg):
        clusters = self.msg_to_clusters(msg.data) # will be passed into wall_filter_total_count

        # If the laser_msg isn't updated then skip this, try again on the next call
        if self.laser_msg != None:
            self.wall_filter_total_count(clusters, self.laser_msg)
    
    def laser_info(self, msg):
        # This sets the laser_msg property to the LaserScan message
        self.laser_msg = msg

    def msg_to_clusters(self, msg):
        clusters = []
        temp_list = []

        # Removing the 100 and sorting coordinates
        for val in msg:
            if val == 100.00:
                if temp_list:
                    clusters.append(temp_list)
                    temp_list = []
            else:
                temp_list.append(val)

        if temp_list:
            clusters.append(temp_list)

        coords = []

        for i in range(len(clusters)):
            coords_row = []
            for j in range(0, len(clusters[i]), 2):
                coords_row.append((clusters[i][j], clusters[i][j + 1]))

            coords.append(coords_row)

        return coords

    # Gets distance between two points
    def euclidean_distance(self, p1, p2):
        
        x1 = p1[0]
        y1 = p1[1]
        x2 = p2[0]
        y2 = p2[1]

        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def wall_filter_total_count(self, clusters, msg):
        
        pointcloud_msg = PointCloud()
        pointcloud_msg.header = msg.header

        # First iteration, get the walls of the map
        if not self.walls:
            # Finding the moving average
            for cluster in clusters:
                xa = sum([point[0] for point in cluster])/len(cluster)
                ya = sum([point[1] for point in cluster])/len(cluster)
 
                self.walls.append([xa,ya])

        # Future iterations, for each cluster check if it is within range of a wall, if it is break; if it is not at it to the Point Cloud
        else:
            for cluster in clusters:
                # Finding the moving average
                xa = sum([point[0] for point in cluster])/len(cluster)
                ya = sum([point[1] for point in cluster])/len(cluster)
                z=0.0
                isWall = False

                for wall in self.walls:
                    # Checking if within range of wall
                    if(self.euclidean_distance((xa,ya), wall) <= 0.8):
                        isWall = True
                        break

                if(isWall == False):
                   # Storing people; go through stored people to see if current person is within range of someone, this would mean they are the same person
                    isNew = True
                    for i in range(len(self.people)):
                        dist =self.euclidean_distance([self.people[i][0], self.people[i][1]],[xa,ya])
                        dist2 =self.euclidean_distance([self.people[i][8], self.people[i][9]],[xa,ya])
                        angle = math.acos((self.people[i][8]-xa)/dist2)*180/math.pi
                        if(self.people[i][10]==0):
                            self.people[i][10]=angle
                        sec = msg.header.stamp.sec
                        nanosec  = msg.header.stamp.nanosec
                        time2 = str(sec) + "." + str(nanosec)
                        currTime = float(time2)

                        if(dist2  <=0.3 and abs(angle-self.people[i][10])<=90): #and self.people[i][7] == 0):
                            isNew = False
                            deltax = self.euclidean_distance([self.people[i][0], self.people[i][1]],[xa,ya])
                            deltay = abs(self.people[i][2] - currTime)
                            velocity = deltax/deltay
                            self.people[i][3] = velocity
                            self.people[i][5]=currTime
                            self.people[i][4]= True
                            self.people[i][8]=xa
                            self.people[i][9]= ya
                            self.people[i][10]=angle
                            break
                        elif(abs(self.people[i][3]*(abs(currTime-self.people[i][5])-dist2))<=0.4):
                            isNew = False

                            deltax = self.euclidean_distance([self.people[i][0], self.people[i][1]],[xa,ya])
                            deltay = abs(self.people[i][2] - currTime)

                            velocity = deltax/deltay
                            self.people[i][3] = velocity
                            self.people[i][5]=currTime
                            self.people[i][4]= True
                            self.people[i][8]=xa
                            self.people[i][9]= ya
                    
                    if(isNew == True):
                        sec = msg.header.stamp.sec
                        nanosec  = msg.header.stamp.nanosec
                        time = str(sec) + "." + str(nanosec)
                        self.people[self.id] = [xa,ya,float(time),0,True,0,0,0,xa,ya,0]
                        self.id+=1
                    
                    pointcloud_msg.points.append(Point32(x=xa,y=ya,z=0.0))

        

        for i in range(len(self.people)):
            self.people[i][4] = False
        # Adds in all of the points to the PointCloud message
        total = Int64()
        total.data = len(self.people)
        self.person_location.publish(pointcloud_msg)
        self.person_count_tot.publish(total)

        self.currentAmount = len(pointcloud_msg.points)
        currAmount_msg = Int64()
        currAmount_msg.data = self.currentAmount
        self.person_count_curr.publish(currAmount_msg)

        self.get_logger().info('TOTAL: "%s"' % total)

      

def main(args=None):
    rclpy.init(args=args)

    topic_pub = FilterAndCount()

    rclpy.spin(topic_pub)

    # Destroy node

    topic_pub.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
