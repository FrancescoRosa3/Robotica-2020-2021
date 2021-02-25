# Exercise 4

This container contains 3 packages:
- **frw_kinematics**, this packge contains two nodes used form computing the Forward Kinematics by using a Service Comunication Mechanism. The nodes comunicate with the standard **GetPositionFK** serive definition <br/>             
  ```bash
   roslaunch frw_kinematics frw_kinematics.launch
   ```
- **ik_action**, package that contains two nodes used for computing the inverse kinematics bu using an Action Comunication Mechanism. The action is defined in the package **ik_action**. <br/>
   ```bash
   roslaunch inv_kinematics inverse_kinematics.launch
   ```
