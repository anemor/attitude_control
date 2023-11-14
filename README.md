### ROS package with an attitude controller node

Requirements in addition to the gbplanner ROS stuff: numpy, scipy, and simple_pid (`pip install simple-pid `)

Add the following lines to `rmf_sim.launch`
``` 
<!-- Launch the nodes needed for TTK22 attitude controller -->
<node pkg="attitude_control" type="attitude_controller_node.py" name="attitude_control" output="screen"/>
<node pkg="rotors_control" type="roll_pitch_yawrate_thrust_controller_node.py" name="rotors_control" output="screen"/>
```

To launch use:
```bash 
roslaunch gbplanner rmf_sim.launch gazebo_gui_en:=true world_file:=/path/to/gbplanner2_ws/src/exploration/gbplanner_ros/planner_gazebo_sim/worlds/virginia_mine.world
```
