import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from example_interfaces.msg import Int64
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy as np
import math
import statistics
from datetime import timedelta

class ScanSubscriber(Node):
    # This node subcribes to the /scan topic
    def __init__(self):
        super().__init__('ScanSubscriber')
        self.sub = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)

        # Move this to the other node later, here for testing!
        self.person_location = self.create_publisher(PointCloud, '/person_locations', 10)
        self.person_count_tot = self.create_publisher(Int64, '/person_count_total', 10)
        self.prevAverages = []
        self.averages = []
        self.walls = []
        self.total = []
        self.people = {}
        self.id = 0

    def listener_callback(self, msg):
        # Currently just prints out the values it heard
        self.cluster(msg)

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
        # Finds clusters, and then publish the clusters that are found as PointClouds

        # Sets the LiDAR distance values to array
        cartesian_array = self.polar_to_cartesian(msg)

        # Now we use DBScan in order to find the clusters of points!
        clusters = self.dbscan(cartesian_array, 0.8, 5)

        # Finds the center of the clusters
        cluster_centers = self.find_cluster_center(clusters)

        # For testing rn, to see where the clusters are at, making a PointCloud
        pointcloud_msg = PointCloud()
        pointcloud_msg.header = msg.header


        # First iteration, get the walls of the map
        if not self.walls:
            for cluster in clusters:
                xa = sum([point[0] for point in cluster])/len(cluster)
                ya = sum([point[1] for point in cluster])/len(cluster)
 
                self.walls.append([xa,ya])

        # Future iterations, for each cluster check if it is within range of a wall, if it is break; if it is not at it to the Point Cloud
        else:
            for cluster in clusters:
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
                            # deltax = self.euclidean_distance([self.people[i][0], self.people[i][1]],[xa,ya])
                            # deltay = abs(self.people[i][2] - currTime)
                            # velocity = deltax/deltay
                            # self.people[i][3] = velocity
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
                        
                        

                    pointcloud_msg.points.append(Point32(x=xa, 
                                                            y=ya, 
                                                            z=0.0))
        # Adds in all of the points to the PointCloud message
        total = Int64()
        total.data = (self.id)
        self.person_location.publish(pointcloud_msg)
        self.person_count_tot.publish(total)
        self.get_logger().info('TOTAL: "%s"' % total)
        for i in range(len(self.people)):
            #if(self.people[i][4] == False):
                # self.people[i][6] += msg.time_increment
                # self.people[i][7]+=1


            self.people[i][4] = False
        sec = msg.header.stamp.sec
        nanosec  = msg.header.stamp.nanosec
        times = str(sec) + "." + str(nanosec)
       # self.get_logger().info('TOTAL: "%s"' % float(times))
        
       # self.get_logger().info('Current TOTAL: ' "%s" % len(pointcloud_msg.points))

        # After we find the clusters, the goal is to see which clusters don't move so much
        # Once we find those then we can ignore them
        # The guess should get better and better over time!

        #self.get_logger().info('I heard: "%s"' % str(cluster_centers))
        
        return

class TopicPublisher(Node):
    # This node publishes onto the three separate topics
    def __init__(self):
        super().__init__('ScanSubscriber')
        self.person_count_curr = self.create_publisher(Int64, 'person_count_current', 10)
        self.person_count_tot = self.create_publisher(Int64, '/person_count_total', 10)

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