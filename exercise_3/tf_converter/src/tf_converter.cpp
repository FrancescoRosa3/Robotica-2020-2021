#include <iostream>
#include <vector>
#include <string>

#include <ros/console.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * Take in input a TransformStamped and perform the following transformation
 * quaternion -> Rotation Matrix;
 * Rotation Matrix -> Eulero angles (RPY);
 * Quaternion -> Axis-angle
*/
void print_transformation(std::string source_frame, std::string target_frame){
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped ts;  

    tf2::Quaternion quat_tf;
    tf2::Matrix3x3Data rotation_matrix_serialized;

    try{
        // in order to avoid the exception
        ros::Duration(1.0).sleep();
        // get the transformation between source_frame and target_frame
        ts = tfBuffer.lookupTransform(source_frame, target_frame, ros::Time(0));
        // ts = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0));
        
        // convert msg quaternion to tf quaternion
        tf2::fromMsg(ts.transform.rotation , quat_tf);

        // quaternion -> rotation Matrix
        tf2::Matrix3x3 rotation_matrix(quat_tf);

        // rotation Matrix -> RPY
        double roll=0, pitch=0, yaw=0;
        rotation_matrix.getRPY(roll, pitch, yaw);

        // quaternion -> axis/angle
        tf2::Vector3 axis;
        axis = quat_tf.getAxis();
        double angle;
        angle = quat_tf.getAngle();

        ROS_INFO_STREAM("\n**** from " << source_frame << " to " << target_frame << "****"
                        << "\n--- Translation ---\n" << ts.transform.translation
                        << "\n--- Quaternion ---\n" << ts.transform.rotation
                        << "\n--- Rotation Matrix ---\n"
                        << "[ "<< rotation_matrix[0][0] <<" , "<<rotation_matrix[0][1] <<" , "<<rotation_matrix[0][2] << " ]\n"
                        << "[ "<< rotation_matrix[1][0] <<" , "<<rotation_matrix[1][1] <<" , "<<rotation_matrix[1][2] << " ]\n"
                        << "[ "<< rotation_matrix[2][0] <<" , "<<rotation_matrix[2][1] <<" , "<<rotation_matrix[2][2] << " ]\n"
                        << "\n\n--- Euler Angles (RPY) ---\n"
                        << "[ " << roll << " , " << pitch << " , " << yaw << " ]"
                        << "\n\n--- Axis/Angle ---\n"
                        << "Axis = [ " << axis.getX() << " , " << axis.getY() << " , " << axis.getZ() << " ]\n"
                        << "Angle = "<< angle <<"\n"                        
        );

    } catch(tf2::TransformException &exception){
                ROS_WARN("%s", exception.what());
                ros::Duration(1.0).sleep();
    }
    
}


int main(int argc, char**argv){

    ros::init(argc, argv, "tf_converter");
    ros::NodeHandle nodeHandle;

    std::string target_frame = "flange";
    int number_source_frames = 6;
    std::string source_frames[number_source_frames] = { "link1", "link2",  "link3", "link4", "link5", "link6"};

    ros::Rate rate(10.0);
    while(nodeHandle.ok()){
        // for each source frame
        int i = 0;
        for(i=0; i<number_source_frames; i++){
            print_transformation(source_frames[i], target_frame);
        }
        rate.sleep();
    }    
        
    return 0;

}