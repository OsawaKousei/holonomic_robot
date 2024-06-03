# requrements
- ROS 2 humble
- gazebo fortless

# usage
## launch
- launch robot in field

`ros2 launch holonomic_sim holonomic_sim.launch.py`

- launch robot in flat ground

`ros2 launch holonomic sim holonomic_sim_flat.launch.py`

## teleop
- user can teleop this holonomic robot using teleop keyboard on GUI
- Or sending ROS2 /cmd_vel topic from rqt
- Ofcourse, user made ros2 node publish /cmd_vel is available