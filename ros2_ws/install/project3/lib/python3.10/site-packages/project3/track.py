import rclpy
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from example_interfaces.msg import Int64
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32MultiArray

# Node organizes the data and breaks it into clusters
class OrganizeAndCluster(Node):
    
    def __init__(self):
        super().__init__('OrganizeAndCluster')
        self.sub = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)

        # Sends cluster infor to other node
        self.cluster_pub = self.create_publisher(Float32MultiArray, '/clusters', 10)

        self.walls = []
        self.total = []

    def cluster_to_msg(self, clusters):
        longest_len = 0

        # encoding portion of the message; inf indicates the end of array
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
        clusters = self.cluster(msg)

        # Create message and publish
        msg_cluster = Float32MultiArray()
        msg_cluster.data = self.cluster_to_msg(clusters)

        self.cluster_pub.publish(msg_cluster)

    # Convert polar to cartesian
    def convert_coordinates(self, msg):
        
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Cartesian points
        new_points = []

        for i in range(len(ranges)):
            # x = r * cos(theta); y = r * sin(theta)
            angle = angle_min + i * angle_increment    
            x = ranges[i] * math.cos(angle) 
            y = ranges[i] * math.sin(angle) 

            new_points.append([x,y]) # array [(x,y), (x,y), .. (x,y)], len = 512

        # Remove garbage values
        final_points = []

        # Remove inf or nan
        for i in range(len(new_points)):
            if not math.isnan(new_points[i][0]) and not math.isinf(new_points[i][0]) and not math.isnan(new_points[i][1]) and not math.isinf(new_points[i][1]):
                final_points.append(new_points[i])

        return final_points
    
    # Distance formula between two points
    def distance_formula(self, p1, p2):
        
        x1 = p1[0]
        y1 = p1[1]
        x2 = p2[0]
        y2 = p2[1]

        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # Finds clusters from cartesian coords; tol is for connecting clusters and size, min_points is the minimum number of points for a cluster
    def getClusters(self, points, tol, min_points):
       
        visited = set()
        noise = set()
        clusters = []

        for point in points:
            #Already seen
            if tuple(point) in visited:
                continue

            visited.add(tuple(point))

            neighbors = []

            for neighbor in points:
                if tuple(neighbor) in visited:
                    continue
                
                # Checking to see if the point is within our tolerance, if it is then it is a neighbor, otherwise leave it
                if self.distance_formula(np.array(point), np.array(neighbor)) <= tol:
                    neighbors.append(neighbor)
                    visited.add(tuple(neighbor))

            # Too small to be a cluster
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
                        for new_neighbor in points:
                            if tuple(new_neighbor) in visited:
                                continue
                            if self.distance_formula(np.array(neighbor), np.array(new_neighbor)) <= tol:
                                new_neighbors.append(new_neighbor)
                                visited.add(tuple(new_neighbor))

                        if len(new_neighbors) >= min_points:
                            neighbors.extend(new_neighbors)
                            
                    if tuple(neighbor) not in cluster:
                        cluster.add(tuple(neighbor))
                clusters.append(list(cluster))

        return clusters
    
    def find_cluster_center(self, clusters):
        # Tfinds the center of the clusters by averaging the points
        cluster_centers = []

        for cluster in clusters:
            x_sum = sum([point[0] for point in cluster])
            y_sum = sum([point[1] for point in cluster])

            x_mean = x_sum / len(cluster)
            y_mean = y_sum / len(cluster)
            
            cluster_centers.append((x_mean, y_mean))
        
        return cluster_centers

    def cluster(self, msg):
        # Convert from polar to cartesian
        cartesian_points = self.convert_coordinates(msg)

        # Get clusters
        clusters = self.getClusters(cartesian_points, 0.8, 3)

        
        return clusters



def main(args=None):
    rclpy.init(args=args)
    scan_sub = OrganizeAndCluster()


    rclpy.spin(scan_sub)


    # Destroy node
    scan_sub.destroy_node()


    rclpy.shutdown()


if __name__ == '__main__':
    main()
