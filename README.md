# CSCE-452-Project-3

WIP: To add the file to your ROS2 package  
First go into ~/ros2_ws/src and run the following to build the package:

```bash
colcon build
```

After that go back to the ~/ros2_ws folder and run the following:

```bash
source install/setup.bash
```

To see if that worked you can run the following command:

```bash
ros2 pkg list
```

If you can find the project3 package in the list then this worked!  
You have to build the package to see any new changes!  

Once that is done you can simply run the following:

```bash
ros2 launch project3 launch.py
```

That will run a bag along with a subscriber to the laser scanner, this should help with using rviz2!