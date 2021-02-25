#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/robot_state/conversions.h>

#include <sensor_msgs/JointState.h>

void print_response(moveit_msgs::GetPositionFK service){

    int fk_link_names_len = service.response.fk_link_names.size();
    for(int i=0; i<fk_link_names_len; i++){
        ROS_INFO_STREAM("Forward Kinematics for "<<service.response.fk_link_names[i]<<"\n"
                        <<service.response.pose_stamped[i])
        ;
    }

}

int main(int argc, char**argv){

    ros::init(argc, argv, "frw_kinematics_service_client");
    ros::NodeHandle nodeHandle;
    nodeHandle = ros::NodeHandle();

    std::string planning_group;
    if(!nodeHandle.getParam("planning_group", planning_group))
    {
        ROS_ERROR("Planning group name not present");
        return 0;
    }

    // Build the robotstate
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotState kinematic_state(kinematic_model);
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(planning_group);
    std::vector<std::string> link_names = joint_model_group->getLinkModelNames();

    boost::shared_ptr<sensor_msgs::JointState const> shared_joint_state;
    // declare request for custom service
    moveit_msgs::GetPositionFK service;

    // service client for "/frw_kinematics_service_server/compute_fk"
    ros::ServiceClient client = nodeHandle.serviceClient<moveit_msgs::GetPositionFK>("/compute_forwardKinematics");
    ros::service::waitForService("/compute_forwardKinematics");
    ROS_INFO("Log to %s", "/compute_forwardKinematics");

    // service client for "/compute_fk" advertised by move_group
    ros::ServiceClient client_move_group = nodeHandle.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
    ros::service::waitForService("/compute_fk");
    ROS_INFO("Log to %s", "/compute_fk");

    ros::Rate looprate(1);

    while(ros::ok()){
        shared_joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nodeHandle);
        // fill the joint_state field 
        if(shared_joint_state != NULL){
            kinematic_state.setVariableValues(*shared_joint_state);
            //kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);

            service.request.fk_link_names = {link_names.back()};
            robotStateToRobotStateMsg(kinematic_state, service.request.robot_state);

            // init request for move_group service
            moveit_msgs::GetPositionFK service_mg;
            service_mg.request.fk_link_names = {link_names.back()};
            robotStateToRobotStateMsg(kinematic_state, service_mg.request.robot_state);

            if(client.call(service)){
                ROS_INFO("Custom Forward Kinematics Service");
                print_response(service);
            }

            if(client_move_group.call(service_mg)){
                ROS_INFO("Move_group Forward Kinematics Service");
                print_response(service_mg);
            }
        }
        ros::spinOnce();
        looprate.sleep();
    }
    
    return 0;

}