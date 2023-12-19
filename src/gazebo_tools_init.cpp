#include <ros/ros.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tfMessage.h>
#include <tf/LinearMath/Transform.h>

#include "joint_controller.h"

ros::Publisher UAV_pos_pub;
ros::Publisher arm2_pub;

void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr& msg)
{
    std::string model_name = "gazebo-classic_Crp450";
    size_t model_index = 0;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == model_name) {
            model_index = i;
            break;
        }
    }

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose = msg->pose[model_index];
    UAV_pos_pub.publish(pose);
}


void arm1PoseCallback(const std_msgs::Float32ConstPtr& msg)
{
    std_msgs::Float32 data;
    data.data = msg->data;
    arm2_pub.publish(data);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "transmit_gazebo_pose");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    UAV_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    ros::Subscriber UAV_pos_sub = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback);

    ros::Subscriber arm1_pos_sub = nh.subscribe("/Crp450/Arm1_joint/pose", 10, arm1PoseCallback);

    ros::Publisher arm1_pub = nh.advertise<std_msgs::Float32>("/Crp450/Arm1_joint/pose_cmd", 1);
    arm2_pub = nh.advertise<std_msgs::Float32>("/Crp450/Arm2_joint/pose_cmd", 1);
    std_msgs::Float32 data_temp;
    data_temp.data = 0;
    
    // arm2_pub.publish(data_temp);

    JointController arm1_controller(nh, "Crp450/Arm1_joint");
    JointController arm2_controller(nh, "Crp450/Arm2_joint");

    bool dynamic_param = true;
    nh.getParam("/dynamic_param", dynamic_param);

    while(ros::ok())
    {
        if(dynamic_param)
        {
            arm1_controller.loadParam();
            arm2_controller.loadParam();
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}