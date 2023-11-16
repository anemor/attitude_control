### ROS package with an attitude controller node

Requirements in addition to the gbplanner ROS stuff: numpy, scipy, and simple_pid (`pip install simple-pid `)

To launch use:
```bash 
roslaunch attitude_control rmf_sim_ttk22.launch world_file:=/path/to/gbplanner2_ws/src/exploration/gbplanner_ros/planner_gazebo_sim/worlds/virginia_mine.world gazebo_gui_en:=true

```
This also includes printing out the received Odometry data and output control signals.