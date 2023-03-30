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

# To install this into ros2 packages, run 'source install/setup.bash'

class ScanSubscriber(Node):
    # This node subcribes to the /scan topic
    def __init__(self):
        super().__init__('ScanSubscriber')
        self.sub = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)

    def listener_callback(self, msg):
        # Currently just prints out the values it heard
        self.get_logger().info('I heard: "%s"' % msg.ranges)
    
    def parse(msg):
        pass


class TopicPublisher(Node):
    # This node publishes onto the three separate topics
    def __init__(self):
        super().__init__('ScanSubscriber')
        self.person_location = self.create_publisher(PointCloud, '/person_locations', 10)
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