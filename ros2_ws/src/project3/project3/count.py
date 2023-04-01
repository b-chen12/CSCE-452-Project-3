import rclpy
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from example_interfaces.msg import Int64
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32MultiArray
class TopicPublisher(Node):
    # This node publishes onto the three separate topics
    def __init__(self):
        super().__init__('ScanSubscriber')
        self.cluster_receiver = self.create_subscription(Float32MultiArray, '/clusters', self.listener_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_info, 10)
        
        self.person_count_curr = self.create_publisher(Int64, '/person_count_current', 10)
        self.person_count_tot = self.create_publisher(Int64, '/people_count_total', 10)
        self.person_location = self.create_publisher(PointCloud, '/people_locations', 10)
        self.laser_msg = None
        self.prevAverages = []
        self.averages = []
        self.walls = []
        self.total = []
        
    
    def listener_callback(self, msg):
        clusters = self.msg_to_clusters(msg.data) # will be passed into wall_filter

        # Issue is here, msg needs to be from the laser publisher but I'm not sure how to get it
        self.wall_filter(clusters, self.laser_msg)
    
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

    def euclidean_distance(self, p1, p2):
        # Finds the distance between two points
        x1, y1 = p1[0], p1[1]
        x2, y2 = p2[0], p2[1]

        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def wall_filter(self, clusters, msg):
        # For testing rn, to see where the clusters are at, making a PointCloud
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
                        #self.walls.append((xa,ya))
                        # wall[0]=(wall[0]+xa)/2
                        # wall[1]=(wall[1]+ya)/2 
                        break
                if(isWall == False):

                    # Storing people; go through stored people to see if current person is within range of someone, this would mean they are the same person
                    isNew = True
                    for i, center in enumerate(self.total):
                        if(self.euclidean_distance(center,[xa,ya])<=1):
                            isNew = False
                            self.total[i] = [xa,ya]
                            break
                    # New person, add to list
                    if(isNew == True):
                        self.total.append([xa,ya])
                        

                    pointcloud_msg.points.append(Point32(x=xa, 
                                                            y=ya, 
                                                            z=0.0))
        # Adds in all of the points to the PointCloud message
        total = Int64()
        total.data = len(self.total)
        self.person_location.publish(pointcloud_msg)
        self.person_count_tot.publish(total)
        self.get_logger().info('TOTAL: "%s"' % total)

        #self.get_logger().info('Current TOTAL: ' "%s" % len(pointcloud_msg.points))

def main(args=None):
    rclpy.init(args=args)

    topic_pub = TopicPublisher()

    rclpy.spin(topic_pub)

    # Destroy node

    topic_pub.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
