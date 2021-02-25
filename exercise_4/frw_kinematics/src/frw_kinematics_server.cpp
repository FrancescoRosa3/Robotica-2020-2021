#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit/robot_state/conversions.h>

#include <eigen_conversions/eigen_msg.h>


// Compute the forward kinematics
bool compute_fk(moveit_msgs::GetPositionFK::Request &request,
                moveit_msgs::GetPositionFK::Response &response
                )
{
    ROS_INFO("Get the model");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    ROS_INFO("Get the kinematic model");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    
    robot_state::RobotState robot_state(kinematic_model);

    if(robotStateMsgToRobotState(request.robot_state, robot_state)){
       ROS_INFO("Robot State Created");
       int fk_link_names_len = request.fk_link_names.size();
       // compute the forward kinematics for each requested link
       for(int i = 0; i<fk_link_names_len; i++){
           ROS_INFO_STREAM("Performing forwark kinematics for "<< request.fk_link_names[i]);
           const Eigen::Isometry3d& target_link_state = robot_state.getGlobalLinkTransform(request.fk_link_names[i]);
           geometry_msgs::PoseStamped ps;
           // convert Isometry3d to Pose
           tf::poseEigenToMsg(target_link_state, ps.pose);
           // fill the response fields
           ps.header.frame_id = "world";
           response.fk_link_names.push_back(request.fk_link_names[i]);
           response.pose_stamped.push_back(ps);
           ROS_INFO_STREAM("Forward kinematics result for: "<< request.fk_link_names[i]<<"\n"<< ps<<"\n");

       }

    }else{
        return false;
    }
     return true;
}


int main(int argc, char**argv){

    ros::init(argc, argv, "frw_kinematics_service_server");
    ros::NodeHandle nodeHandle;
    nodeHandle = ros::NodeHandle();

    ros::ServiceServer service = nodeHandle.advertiseService("compute_forwardKinematics", compute_fk);

    ros::spin();
    return 0;

}