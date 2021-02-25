# Exercise 3

This container contains 3 packages:
 - fanuc_20ia_moveit_config, package created with moveit setup assistant.<br/> Run "demo.launch" file for loading the robot description on the parameter server and visualize it on rviz.
 - fanuc_description, it contains the fanuc 20ia URDF.
 - tf_converter, package that contains the node used for computing the TF trasformation of the end-effector in all the reference frames. <br/>
    ```bash
    roslaunch tf_converter tf_converter.launch
    ```