from launch import LaunchDescription
from launch_ros.actions import *

def generate_launch():
    # This launches a turtlesim node
    # We'll probably need to change this later to our nodes
    node = Node(package = 'turtlesim',
                executable = 'turtlesim_node')
    
    ld = LaunchDescription([ node ])
    return ld