#include <ros/ros.h>

// action lib
#include <actionlib/server/simple_action_server.h>

// moveit
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/robot_state/conversions.h>

#include <eigen_conversions/eigen_msg.h>

#include <ik_action/IKRequestAction.h>

#include <angles/angles.h>
// Class that implements action server for inverse kinematics

class IKAction{

    private:
        ros::NodeHandle nh;
        actionlib::SimpleActionServer<ik_action::IKRequestAction> actionserver;
        std::string action_name;

        std::vector<std::vector<double>> solutions; // solution matrix
        
        std::vector<double> generateSeedState(robot_model::RobotModelPtr &robot_model){
            // generate a new random seed based on joint limits
            std::vector<double> seed_state;
            
            std::vector<std::string> joint_names = robot_model->getVariableNames();

            for(int i=0; i < joint_names.size(); i++)
            {
                double lb = robot_model->getURDF()->getJoint(joint_names[i])->limits->lower;
                double ub = robot_model->getURDF()->getJoint(joint_names[i])->limits->upper;
                double span = ub-lb;
                
                seed_state.push_back((double)std::rand()/RAND_MAX * span + lb);
            }

            return seed_state;


        }

        void normalizeJointPositions(std::vector<double> & solution, const robot_state::JointModelGroup &joint_model_group) const{
            // normalize the angle if the joint type is Revolute
            for(int i=0; i < solution.size(); i++)
            {
                if (joint_model_group.getActiveJointModels()[i]->getType() == robot_model::JointModel::REVOLUTE)
                {
                    solution[i] = angles::normalize_angle(solution[i]);
                }
            }

        }

        bool isSolutionNew(const std::vector<double> &solution, const robot_state::JointModelGroup &joint_model_group) const{

            for(int i=0; i < solutions.size(); i++)
            {
                bool are_solutions_equal = true;

                for(int j=0; j < solutions[i].size() && are_solutions_equal; j++)
                {
                    double diff;

                    if(joint_model_group.getActiveJointModels()[j]->getType() == robot_model::JointModel::REVOLUTE)
                    {
                        diff = angles::shortest_angular_distance(solutions[i][j], solution[j]);
                    }
                    else
                    {
                        diff = solutions[i][j] - solution[j];
                    }

                    if(std::fabs(diff) > 1e-3)
                        are_solutions_equal = false;
                }

                if(are_solutions_equal)
                    return false;
            }

            return true;            
        }


        void executeCB(const ik_action::IKRequestGoalConstPtr &goal){
            // declare action response
            ik_action::IKRequestResult result;

            ROS_INFO("ACTION CALLBACK CALLED");
            ROS_INFO("GET ROBOT MODEL");
            robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
            robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
            ROS_INFO("DEFINE ROBOT STATE");
            robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));

            ROS_INFO("GET Joint Model Group");
            std::string planning_group;
            if(!nh.getParam("planning_group", planning_group)){
                actionserver.setAborted(result, "Could not find any IK solution");
                ROS_ERROR("Planning group name not defined");
            }

            const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(planning_group);

            ROS_INFO("GET SOLVER INSTACE");
            const kinematics::KinematicsBaseConstPtr solver = joint_model_group->getSolverInstance();

            int cnt = 0;



            // call the solver 1000 times
            while(cnt < 1000 && ros::ok()){
                // generate a new seed
                std::vector<double> seed_state = generateSeedState(robot_model);
                
                // call the solver, insert the solution into solution
                std::vector<double> solution;
                moveit_msgs::MoveItErrorCodes error_code;
                solver->getPositionIK(goal->ik_request, seed_state, solution, error_code);

                if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
                    // normalize joint values
                    normalizeJointPositions(solution, *joint_model_group);

                    if(isSolutionNew(solution, *joint_model_group)){
                        // solution is new
                        solutions.push_back(solution);

                        robot_state::RobotState robot_state(robot_model);
                        // set JointState in RobotState to the found values
                        robot_state.setVariablePositions(solution);
                        
                        ik_action::IKRequestFeedback feedback;
                        moveit::core::robotStateToRobotStateMsg(robot_state, feedback.current_solution);
                        
                        // publish the feedback
                        actionserver.publishFeedback(feedback);
                        
                        result.solutions.push_back(feedback.current_solution);

                    }
                }
                
                cnt = cnt + 1;
            }

            if (solutions.size() == 0){
                actionserver.setAborted(result, "Could not find any IK solution");
            }
            else{
                std::ostringstream message;
                message << "Found " << solutions.size() << " solutions";
                
                actionserver.setSucceeded(result, message.str());
            }

        }

          
    public:
        IKAction(std::string name):
            actionserver(nh, name, boost::bind(&IKAction::executeCB, this, _1), false),
            action_name(name)
        {
            // start the action server;
            actionserver.start();
        }
        
};



int main(int argc, char** argv){
    ros::init(argc, argv, "IK_action_server");
    IKAction action("IK_action_server");
    ros::spin();
    return 0;
}