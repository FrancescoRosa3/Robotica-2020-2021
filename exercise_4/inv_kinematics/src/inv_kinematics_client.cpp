#include <ros/ros.h>

// action lib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// moveit
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit/robot_state/conversions.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// action
#include <ik_action/IKRequestAction.h>

void print_solution(const moveit_msgs::RobotState &robotstate){

    ROS_INFO_STREAM("Solution found: [ "
                    <<robotstate.joint_state.position[0] << " ,"
                    <<robotstate.joint_state.position[1] << " ,"
                    <<robotstate.joint_state.position[2] << " ,"
                    <<robotstate.joint_state.position[3] << " ,"
                    <<robotstate.joint_state.position[4] << " ,"
                    <<robotstate.joint_state.position[5] << " ]");

}

void doneCb(const actionlib::SimpleClientGoalState & state,
            const ik_action::IKRequestResultConstPtr & result)
{
    ROS_INFO("ActionServer terminated");

    if (state ==  actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
        int num_solutions = result->solutions.size();

        // print all the solutions found
        for(int i=0; i<num_solutions; i++){
            ROS_INFO("Solution number %d ", i);
            print_solution(result->solutions[i]);
        }

        // publish solutions on JointState topic
        ros::NodeHandle nh;
        ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
        sensor_msgs::JointState joint_state_msg;


        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
        robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
        
        std::string planning_group;
        if(!nh.getParam("planning_group", planning_group)){
            ROS_ERROR("Planning group name not defined");
        }

        const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(planning_group);

        joint_state_msg.name = joint_model_group->getVariableNames();
        
        ROS_INFO("Publishing solutions");

        ros::Duration sleep_time(10.0);

        for(int i=0; i < num_solutions; i++)
        {
            sleep_time.sleep();

            joint_state_msg.position = result->solutions[i].joint_state.position;
            joint_state_msg.header.stamp = ros::Time::now();

            joint_state_publisher.publish(joint_state_msg);
        }       

        ROS_INFO("All solutions published");   

    }
    else {
        ROS_INFO_STREAM(" Action Server failed "<<state.getText());
    }
}

void activeCb(){
    ROS_INFO("Goal got active");
}

void feedbackCb(const ik_action::IKRequestFeedbackConstPtr & feedback){
    ROS_INFO("Feedback received");
    print_solution(feedback->current_solution);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "IK_action_client");

    // Create action client
    actionlib::SimpleActionClient<ik_action::IKRequestAction>ac("IK_action_server", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer();
    ROS_INFO("Server connected");

    // load the model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotState kinematic_state(kinematic_model);
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("fanuc");

    // prepare goal;
    ik_action::IKRequestGoal goal;
    /*
    // set random position
    kinematic_state.setToRandomPositions(joint_model_group);
    const Eigen::Affine3d& end_effector_state = kinematic_state.getGlobalLinkTransform("flange");
    // fill the goal field
    tf::poseEigenToMsg(end_effector_state, goal.ik_request);
    // send the goal
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    */
    goal.ik_request.position.x = 1.0;
    goal.ik_request.position.y = 0.0;
    goal.ik_request.position.z = 1.0;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, 0.0);    
    goal.ik_request.orientation = tf2::toMsg(quaternion);
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ac.waitForResult();

    ros::spin();

    return 0;
}