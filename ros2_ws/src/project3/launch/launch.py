from launch import LaunchDescription
from launch.actions import *

from launch_ros.actions import *
from launch.substitutions import *

from launch.event_handlers import *
from launch.events import *

def generate_launch_description():
    # Argument Object That Takes in Launch Arguments
    # Default Value can be changed
    # We can also have multiple arguments I believe!
    arg = DeclareLaunchArgument('arg_name',
                                default_value='test')

    # This launches a turtlesim node
    # We'll probably need to change this later to our nodes
    node = Node(package = 'turtlesim',
                executable = 'turtlesim_node')

    # We can use the following in places we're we would normally hard code things,
    # I guess things that rely on arguments:

    # LaunchConfiguration('arg_name')

    # The following can also be used in place of nodes:
    # Will probably come in handy with playing our bags
    ep = ExecuteProcess(cmd = ['ros2', 'bag', 'play', 'bags/example1'])

    # This will help with stopping playback when the original bag file is completed,
    # It also has to be added to our LaunchDescription object

    # event_handler = OnProcessExit(target_action = your_action,
    #                               on_exit = [EmitEvent(event = Shutdown())])
    # terminate_at_end = RegisterEventHandler(event_handler)

    ld = LaunchDescription([ arg, node ])

    return ld