### Attitude controller

Attitude controller ROS node written as an exercise in writing a node for the existing project, which can be found here: [https://github.com/ntnu-arl/gbplanner_ros](https://github.com/ntnu-arl/gbplanner_ros)

Requirements in addition to the gbplanner ROS stuff: numpy, scipy, and simple_pid (`pip install simple-pid `)

To launch use:
```bash 
roslaunch attitude_control rmf_sim_ttk22.launch world_file:=/path/to/gbplanner2_ws/src/exploration/gbplanner_ros/planner_gazebo_sim/worlds/virginia_mine.world gazebo_gui_en:=true

```
This also includes printing out the received Odometry data and output control signals.
