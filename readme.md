# requirements
- ROS 2 humble
- gazebo fortress

# usage
## launch
- launch robot in field

`ros2 launch holonomic_sim holonomic_sim.launch.py`

## teleop
- user can teleop this holonomic robot using teleop keyboard on GUI
- Or sending ROS2 /cmd_vel topic from rqt
- Ofcourse, user made ros2 node publish /cmd_vel is available

# image
## simulation
![alt text](https://github.com/OsawaKousei/holonomic_robot/blob/main/img/sim_img.png)
## visualization using rviz2
![alt text](https://github.com/OsawaKousei/holonomic_robot/blob/main/img/rviz2_img.png)
## watch robot vel using ros2
![alt text](https://github.com/OsawaKousei/holonomic_robot/blob/main/img/total_img.png)
