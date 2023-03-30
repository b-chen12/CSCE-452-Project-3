from launch import LaunchDescription
from launch.actions import *

from launch_ros.actions import *
from launch.substitutions import *

from launch.event_handlers import *
from launch.events import *


# To run this file use ros2 launch project3 launch.py

def generate_launch_description():
    # Argument Object That Takes in Launch Arguments
    bag_in = LaunchConfiguration('bag_in')
    bag_out = LaunchConfiguration('bag_out')

    bag_in_arg = DeclareLaunchArgument(
                            'bag_in',
                            default_value='bags/example1')
    
    bag_out_arg = DeclareLaunchArgument(
                            'bag_out',
                            default_value='bags/out/test')

    # This starts playing the bag on launch
    play_bag = ExecuteProcess(cmd = ['ros2', 'bag', 'play', bag_in])
    
    # For bag recording, my guess is we need to pass in the argument
    # Into the node we create and ros2 bag record bag_out in that node

    # This launches a turtlesim node
    # Should probably be a node dealing with /scan though
    node = Node(package = 'project3',
                executable = 'track')

    # This terminates the program once the bag is done playing
    event_handler = OnProcessExit(target_action = play_bag,
                                  on_exit = [EmitEvent(event = Shutdown())])
    
    terminate_at_end = RegisterEventHandler(event_handler)

    ld = LaunchDescription([ bag_in_arg, bag_out_arg, node, play_bag, terminate_at_end])

    return ld