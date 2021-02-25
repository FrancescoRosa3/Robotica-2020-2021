#include "ros/ros.h"
#include "std_msgs/String.h"
#include <controller_pkg/Sensors.h>

class Joint{
    private:
        float joint_ll;
        float joint_up;
        float current_pos;
        float increment;

    public:
        /**
         * Joint initializer
         * joint_ll : joint lower limit
         * joint_up : joint upper limit 
        */
        Joint(float joint_ll, float joint_up){
            this->joint_ll = joint_ll;
            this->joint_up = joint_up;
            this->current_pos = 0;
            this->increment = 0.05;
        }

        /**
         *  Move the joint of a fixed increment
         *  clock_wise: True, move the robot in clock wise (negative) direction, False, anticlockwise
         *  return : the new position
        */
        bool move_joint(bool clock_wise){
            float virtual_position = 0;
            if(clock_wise == true){
                virtual_position = this->current_pos - increment;
            }else{
                virtual_position = this->current_pos + increment;
            }
            
            if(virtual_position >= this->joint_ll && virtual_position <= this->joint_up){
                this->current_pos = virtual_position;
                return true;
            }

            return false;
        }

        float get_current_pos(){
            return this->current_pos;
        }

        void reset_current_pos(){
            this->current_pos = 0;
        }
};

/**
 * Perform a cycle of rotation for a given joint
*/
void perform_rotation(Joint joints[6], int joint_to_rotate, ros::Publisher &sensors_pub){
    controller_pkg::Sensors sensors_value;
    
    ros::Rate rate(5.0);
    while(joints[joint_to_rotate].move_joint(true) == true){
            sensors_value.positions = {joints[0].get_current_pos(),joints[1].get_current_pos(),joints[2].get_current_pos(),
                                        joints[3].get_current_pos(),joints[4].get_current_pos(),joints[5].get_current_pos()};
            sensors_pub.publish(sensors_value);
            ROS_INFO_STREAM( "I published " << sensors_value);
            ros::spinOnce();
            rate.sleep();
    }

    while(joints[joint_to_rotate].move_joint(false) == true){
        sensors_value.positions = {joints[0].get_current_pos(),joints[1].get_current_pos(),joints[2].get_current_pos(),
                                    joints[3].get_current_pos(),joints[4].get_current_pos(),joints[5].get_current_pos()};
        sensors_pub.publish(sensors_value);
        ROS_INFO_STREAM( "I published " << sensors_value);
        ros::spinOnce();
        rate.sleep();
    }

    joints[joint_to_rotate].reset_current_pos();
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "sensors_publisher");

    ros::NodeHandle n;

    ros::Publisher sensors_pub = n.advertise<controller_pkg::Sensors>("sensors", 10);

    ros::Rate loop_rate(1);
    // create the set of joints
    Joint joint1(-2.967, 2.967);
    Joint joint2(-1.4835, 2.7052);
    Joint joint3(-1.570, 1.3962);
    Joint joint4(-3.6651, 3.6651);
    Joint joint5(-2.2689, 2.2689);
    Joint joint6(-47.1238, 47.1238);
    Joint joint_array[6] = {joint1, joint2, joint3, joint4, joint5, joint6};

    
    
    while(ros::ok()){
        perform_rotation(joint_array, 0, sensors_pub);
        perform_rotation(joint_array, 1, sensors_pub);
        perform_rotation(joint_array, 2, sensors_pub);
        perform_rotation(joint_array, 3, sensors_pub);
        perform_rotation(joint_array, 4, sensors_pub);
        perform_rotation(joint_array, 5, sensors_pub);
    }

    return 0;
}