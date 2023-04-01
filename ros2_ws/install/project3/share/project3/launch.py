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

    # Sets up location of where the bag will be played from
    bag_in_arg = DeclareLaunchArgument(
                            'bag_in',
                            default_value='bags/example1')
    
    # Sets up location of where the bag will be recorded too
    bag_out_arg = DeclareLaunchArgument(
                            'bag_out',
                            default_value='bags/track_result')

    # This starts playing the bag on launch
    play_bag = ExecuteProcess(cmd = ['ros2', 'bag', 'play', bag_in])

    # This records all three topics into a bag
    record_bag = ExecuteProcess(cmd = ['ros2', 'bag', 'record',
                                            '-a', '-o', bag_out])

    # This launches the track node
    node = Node(package = 'project3',
                executable = 'track')
    node2 = Node(package='project3',
                 executable='count')

    # This terminates the program once the bag is done playing
    event_handler = OnProcessExit(target_action = play_bag,
                                  on_exit = [EmitEvent(event = Shutdown())])
    
    terminate_at_end = RegisterEventHandler(event_handler)

    ld = LaunchDescription([ bag_in_arg, bag_out_arg, node, node2, play_bag, terminate_at_end,
                            record_bag ])

    return ld
