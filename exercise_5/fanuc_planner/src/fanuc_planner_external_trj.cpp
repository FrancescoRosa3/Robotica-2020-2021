#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit_dp_redundancy_resolution/workspace_trajectory.h>

#include <math.h>

#define PI 3.14

void publish_trajectory_plot(moveit_msgs::RobotTrajectory trajectory, ros::Publisher plot_path_pb){

    // check if a path has been computed
    if(trajectory.joint_trajectory.points.size()>0){
        ROS_INFO("Publishing trajectory points");
        // for each point
        for(int i=0; i<trajectory.joint_trajectory.points.size(); i++){
            trajectory_msgs::JointTrajectoryPoint jtp;
            jtp = trajectory.joint_trajectory.points[i];
            plot_path_pb.publish(jtp);
        }
    }
    ROS_INFO("... Done!");
}


int main(int argc, char** argv){
    
    ros::init(argc, argv, "fanuc_planner");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // define the path publisher
    ros::Publisher plot_path_pb = node_handle.advertise<trajectory_msgs::JointTrajectoryPoint>("plot_planned_path", 10000, true);

    // create the planning group for the fanuc
    static const std::string PLANNING_GROUP = "fanuc";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    
    robot_state::RobotStatePtr initial_robot_state = move_group.getCurrentState();

    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit_msgs::PlannerInterfaceDescription desc;	
    move_group.getInterfaceDescription(desc);
    ROS_INFO_STREAM("Interface Description "<< desc);
    const std::string str =  move_group.getDefaultPlannerId("fanuc");
    ROS_INFO_STREAM("Default ID "<<str);
    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
    std::ostream_iterator<std::string>(std::cout, ", "));

    ROS_INFO("Loading path from file");
    std::string package_path = ros::package::getPath("fanuc_planner");
    std::string trajectory_file_path = package_path + "/trajectory/" + "fanuc_circular.traj";
    moveit_dp_redundancy_resolution::WorkspaceTrajectory ws_trj("fanuc_circular_path", trajectory_file_path);
    std::vector<geometry_msgs::Pose> waypoints = ws_trj.getWaypoints();

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = 0;
    double velocity_scaling_factor = 0.1;
    double acce_scaling_factor = 1;

    ROS_INFO("Velocity Scaling factor: %f", velocity_scaling_factor);
    // set different valocity and acceleration scaling factor;
    move_group.setMaxVelocityScalingFactor(velocity_scaling_factor);
    //move_group.setMaxAccelerationScalingFactor (acce_scaling_factor);
    while(fraction < 1){
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO_STREAM("Fraction "<< fraction);
    }
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    publish_trajectory_plot(trajectory, plot_path_pb);
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    //visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishTrajectoryLine(trajectory, joint_model_group->getLinkModel("flange"), joint_model_group, rvt::LIME_GREEN);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    spinner.stop();
    ros::shutdown();
    exit(0);

}