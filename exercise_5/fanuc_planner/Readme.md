# Fanuc planner

This package provides a node that performs the a constrained path planning.
The trajectory, that has to be performed, is encoded in "src/fanuc_planner.cpp".
The trajectory is visualized in rviz.
The joints, velocities and acceleration values are visualized in rqt_multiplot.

## How to run the planner

The package contains a launch file in "launch/fanuc_planner.launch" which allows to run the program with a single command.

```bash
roslaunch fanuc_planner fanuc_planner.launch
```

After this command has been executed, load the rqt_multiplot configuration defined in "/conf/ex5_multiplot_conf.xml", then run the plots and follow the instruction displayed on the console.

**Note** <br/>

The program requires the robot descrpition, since the launch file runs the "demo.launch" file, see "exercise_3/fanuc_20ia_moveit_config/launch/demo.launch"