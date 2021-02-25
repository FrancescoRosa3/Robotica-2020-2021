#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv){
    
    ros::init(argc, argv, "fanuc_planner");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // create the planning group for the fanuc
    static const std::string PLANNING_GROUP = "fanuc";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    
    // planning scene interface for adding obstacles
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
    std::ostream_iterator<std::string>(std::cout, ", "));

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // define the target pose
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1;
    target_pose1.position.x = 0.4;//0.28;
    target_pose1.position.y = -1;
    target_pose1.position.z = 1;
    move_group.setPoseTarget(target_pose1);

    // call the planner
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // visualize plan
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("flange"), joint_model_group, rvt::LIME_GREEN);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Path Constraints
    // define orientation constraint
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "flange";
    ocm.header.frame_id = "world";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;
    // set the orientation constraint as path constraint
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(test_constraints);

    // set the start pose, this pose must satisfy the constraint
    robot_state::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose start_pose2;
    start_pose2.orientation.w = 1.0;
    start_pose2.position.x = 1.110;
    start_pose2.position.y = 0.0;
    start_pose2.position.z = 1.465;
    start_state.setFromIK(joint_model_group, start_pose2);
    move_group.setStartState(start_state);

    // set the target pose
    move_group.setPoseTarget(target_pose1);

    move_group.setPlanningTime(10.0);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(start_pose2, "start");
    visual_tools.publishAxisLabeled(target_pose1, "goal");
    visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("flange"), joint_model_group, rvt::LIME_GREEN);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    // When done with the path constraint be sure to clear it.
    move_group.clearPathConstraints();


    // Cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose2);

    geometry_msgs::Pose target_pose3 = start_pose2;

    target_pose3.position.z -= 0.2;
    waypoints.push_back(target_pose3);  // down

    target_pose3.position.y -= 0.2;
    waypoints.push_back(target_pose3);  // right

    target_pose3.position.z += 0.2;
    target_pose3.position.y += 0.2;
    target_pose3.position.x -= 0.2;
    waypoints.push_back(target_pose3);  // up and left

    move_group.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


    // Collision object
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    
    collision_object.id = "box1";

    // object shape
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.5;
    primitive.dimensions[2] = 0.4;

    // box pose
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = -0.5;
    box_pose.position.z = 1.0;

    // add collision object into the planning scene
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    // goal behind the obstacle
    move_group.setStartState(*move_group.getCurrentState());
    geometry_msgs::Pose another_pose;
    another_pose.orientation.w = 1.0;
    another_pose.position.x = 0.4;
    another_pose.position.y = -1;
    another_pose.position.z = 1;
    move_group.setPoseTarget(another_pose);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group->getLinkModel("flange"), joint_model_group, rvt::LIME_GREEN);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    // Now, let's remove the collision object from the world.
    ROS_INFO_NAMED("tutorial", "Remove the object from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

    return 0;
}