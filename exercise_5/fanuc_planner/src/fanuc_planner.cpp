#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

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

    // Cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    
    // the center corresponds to the flange origin 
    float y_center = 0;
    float z_center = 1.1;
    float x_center = 1.11;
    float r = 0.2;
    // define the starting point
    geometry_msgs::Pose start_pose;
    start_pose.orientation.x = 0.70;
    start_pose.orientation.z = 0.70;
    start_pose.orientation.w =  0;
    start_pose.position.x = x_center;
    start_pose.position.y = y_center + r;
    start_pose.position.z = z_center;
    waypoints.push_back(start_pose);
    
    // adjacent waypoints differ by 10°
    geometry_msgs::Pose pose;
    float angle_resolution = 10; 
    for (int i = angle_resolution; i<=360; i+=angle_resolution){
        pose.orientation.x = 0.70;
        pose.orientation.z = 0.70;
        pose.orientation.w =  0;
        pose.position.x = x_center;
        pose.position.y = y_center + r*cos((i*2*PI)/360);
        pose.position.z = z_center + r*sin((i*2*PI)/360);
        waypoints.push_back(pose);
        //ROS_INFO_STREAM("Pose "<<pose);
    }



    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    move_group.setMaxVelocityScalingFactor(0.1);
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    //ROS_INFO_STREAM("TRAJCETORY "<< trajectory);

    // visualize path on rviz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    
    // publish path for rqt_multiplot
    publish_trajectory_plot(trajectory, plot_path_pb);
    return 0;
}