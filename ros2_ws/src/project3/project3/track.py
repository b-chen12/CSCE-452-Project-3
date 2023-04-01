import rclpy
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from example_interfaces.msg import Int64
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32MultiArray

# To install this into ros2 packages, run 'source install/setup.bash'

class ScanSubscriber(Node):
    # This node subcribes to the /scan topic
    def __init__(self):
        super().__init__('ScanSubscriber')
        self.sub = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)

        # Sends the cluster info over to the other
        self.cluster_pub = self.create_publisher(Float32MultiArray, '/clusters', 10)

        self.prevAverages = []
        self.averages = []
        self.walls = []
        self.total = []

    def cluster_to_msg(self, clusters):
        longest_len = 0

        # Encoding portion of the message
        # inf will indicate the end of an array
        for i in range(len(clusters)):
            if len(clusters[i]) >= longest_len:
                longest_len = len(clusters[i])

        for i in range(len(clusters)):
            missing_elements = longest_len - len(clusters[i])

            # List does not have same amount of elements
            if missing_elements > 0:
                placeholder = float(100)
                clusters[i].append(placeholder)

        # Unrolling the array
        flattened = []
        for i in range(len(clusters)):
            for j in range(len(clusters[i])):
                if type(clusters[i][j]) is tuple:
                    flattened.append(clusters[i][j][0])
                    flattened.append(clusters[i][j][1])
                elif clusters[i][j] == 100.00:
                    flattened.append(float(100))

        return flattened

    def listener_callback(self, msg):
        # Currently just prints out the values it heard
        clusters = self.cluster(msg)

        # Creating the message
        msg = Float32MultiArray()
        msg.data = self.cluster_to_msg(clusters)

        self.cluster_pub.publish(msg)

        # self.get_logger().info('TOTAL: "%s"' % str(clusters))

    def polar_to_cartesian(self, msg):
        # Turns the lidar data from polar coordinates to cartesian
        val_array = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        cartesian_array = [] # array contain x and y values

        for i in range(len(val_array)):
            angle = angle_min + (i * angle_increment) # angle in polar coordinates (r, theta)
            x = val_array[i] * math.cos(angle) # x = r * cos(theta)
            y = val_array[i] * math.sin(angle) # y = r * sin(theta)

            cartesian_array.append([x,y]) # array [(x,y), (x,y), .. (x,y)], len = 512

        # Removing inf and nan values from the array
        clean_array = []

        # Go through other array and remove values with inf or nan
        for i in range(len(cartesian_array)):
            if math.isnan(cartesian_array[i][0]) or math.isinf(cartesian_array[i][0]):
                # If x = abs(inf) or nan, don't add it to the new array
                continue
            elif math.isnan(cartesian_array[i][1]) or math.isinf(cartesian_array[i][1]):
                # If y = abs(inf) or nan, don't add it to the new array
                continue
            else:
                # All good, add to array
                clean_array.append(cartesian_array[i])

        return clean_array
    
    def euclidean_distance(self, p1, p2):
        # Finds the distance between two points
        x1, y1 = p1[0], p1[1]
        x2, y2 = p2[0], p2[1]

        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


    def dbscan(self, cartesian_array, epsilon, min_points):
        # This finds the clusters in our data, and is more resilient to noise

        # Parameter Notes:
        # epsilon is the radius parameter
        # min_points is the minimum number of points in a cluster
        
        visited = set()
        noise = set()
        clusters = []

        for point in cartesian_array:
            if tuple(point) in visited:
                continue

            visited.add(tuple(point))

            neighbors = []

            for neighbor in cartesian_array:
                if tuple(neighbor) in visited:
                    continue

                if self.euclidean_distance(np.array(point), np.array(neighbor)) <= epsilon:
                    neighbors.append(neighbor)
                    visited.add(tuple(neighbor))

            if len(neighbors) < min_points:
                noise.add(tuple(point))
            else:
                cluster = set()
                cluster.add(tuple(point))

                for neighbor in neighbors:
                    noise.discard(tuple(neighbor))

                for neighbor in neighbors:
                    if tuple(neighbor) not in visited:
                        visited.add(tuple(neighbor))

                        new_neighbors = []
                        for new_neighbor in cartesian_array:
                            if tuple(new_neighbor) in visited:
                                continue
                            if self.euclidean_distance(np.array(neighbor), np.array(new_neighbor)) <= epsilon:
                                new_neighbors.append(new_neighbor)
                                visited.add(tuple(new_neighbor))

                        if len(new_neighbors) >= min_points:
                            neighbors.extend(new_neighbors)

                    if tuple(neighbor) not in cluster:
                        cluster.add(tuple(neighbor))

                clusters.append(list(cluster))

        return clusters
    
    def find_cluster_center(self, clusters):
        # This function finds the center of the clusters by averaging the points
        cluster_centers = []

        for cluster in clusters:
            x_sum = sum([point[0] for point in cluster])
            y_sum = sum([point[1] for point in cluster])

            x_mean = x_sum / len(cluster)
            y_mean = y_sum / len(cluster)
            
            cluster_centers.append((x_mean, y_mean))
        
        return cluster_centers

    def cluster(self, msg):
        # Finds clusters, and then returns them as a list

        # Sets the LiDAR distance values to array
        cartesian_array = self.polar_to_cartesian(msg)

        # Now we use DBScan in order to find the clusters of points!
        clusters = self.dbscan(cartesian_array, 0.8, 3)

        # Now send the clusters in a topic

        #self.get_logger().info('I heard: "%s"' % str(cluster_centers))
        
        return clusters

class TopicPublisher(Node):
    # This node publishes onto the three separate topics
    def __init__(self):
        super().__init__('ScanSubscriber')
        self.cluster_receiver = self.create_subscription(Float32MultiArray, '/clusters', self.listener_callback, 10)
        self.person_count_curr = self.create_publisher(Int64, '/person_count_current', 10)
        self.person_count_tot = self.create_publisher(Int64, '/person_count_total', 10)
    
    def listener_callback(self, msg):
        self.get_logger().info('Here!')
        self.get_logger().info('I heard: "%s"' % msg.data)

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
    scan_sub = ScanSubscriber()
    topic_pub = TopicPublisher()

    rclpy.spin(scan_sub)
    rclpy.spin(topic_pub)

    # Destroy node
    scan_sub.destroy_node()
    topic_pub.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()