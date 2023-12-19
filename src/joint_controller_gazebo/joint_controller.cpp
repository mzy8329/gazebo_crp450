#include "joint_controller.h"

JointController::JointController(ros::NodeHandle nh, std::string joint_name, float loop_rate)
{
    nh_ = nh;
    joint_name_ = joint_name;
    loop_rate_ = loop_rate;
    
    torque_pub_ = nh.advertise<std_msgs::Float32>("/"+joint_name_+"/torque_cmd", 10);

    vel_cmd_sub_ = nh.subscribe<std_msgs::Float32>("/"+joint_name_+"/vel_cmd", 10, &JointController::velCmdCallback, this);
    pose_cmd_sub_ = nh.subscribe<std_msgs::Float32>("/"+joint_name_+"/pose_cmd", 10, &JointController::poseCmdCallback, this);

    vel_sub_ = nh.subscribe<std_msgs::Float32>("/"+joint_name_+"/vel", 10, &JointController::velCallback, this);
    pose_sub_ = nh.subscribe<std_msgs::Float32>("/"+joint_name_+"/pose", 10, &JointController::poseCallback, this);

    loadVelPidParam();
    loadPosePidParam();

}

void JointController::loadParam()
{
    loadVelPidParam();
    loadPosePidParam();
}


void JointController::velCmdCallback(const std_msgs::Float32::ConstPtr &msg)
{
    static std_msgs::Float32 data;
    vel_pid_.ref_ = msg->data;

    vel_pid_.calPIDOutput();
    data.data = vel_pid_.output_;
    torque_pub_.publish(data);
}

void JointController::poseCmdCallback(const std_msgs::Float32::ConstPtr &msg)
{
    static std_msgs::Float32 data;
    pose_pid_.ref_ = msg->data;

    pose_pid_.calPIDOutput();
    vel_pid_.ref_ = pose_pid_.output_;
    vel_pid_.calPIDOutput();
    data.data = vel_pid_.output_;
    torque_pub_.publish(data);
}

void JointController::velCallback(const std_msgs::Float32::ConstPtr &msg)
{ 
    vel_pid_.fdb_ = msg->data;
}

void JointController::poseCallback(const std_msgs::Float32::ConstPtr &msg)
{
    pose_pid_.fdb_ = msg->data;
}



void JointController::loadVelPidParam()
{
    nh_.getParam(std::string("/")+std::string(joint_name_)+std::string("_p_gain_vel"), vel_pid_param_.p);
    nh_.getParam(std::string("/")+std::string(joint_name_)+std::string("_i_gain_vel"), vel_pid_param_.i);
    nh_.getParam(std::string("/")+std::string(joint_name_)+std::string("_d_gain_vel"), vel_pid_param_.d);
    nh_.getParam(std::string("/")+std::string(joint_name_)+std::string("_max_output_vel"), vel_pid_param_.max_output);
    vel_pid_.loadParam(vel_pid_param_.p,
                        vel_pid_param_.i,
                        vel_pid_param_.d,
                        vel_pid_param_.max_output,
                        -vel_pid_param_.max_output);    
}

void JointController::loadPosePidParam()
{
    nh_.getParam(std::string("/")+std::string(joint_name_)+std::string("_p_gain_pose"), pose_pid_param_.p);
    nh_.getParam(std::string("/")+std::string(joint_name_)+std::string("_i_gain_pose"), pose_pid_param_.i);
    nh_.getParam(std::string("/")+std::string(joint_name_)+std::string("_d_gain_pose"), pose_pid_param_.d);
    nh_.getParam(std::string("/")+std::string(joint_name_)+std::string("_max_output_pose"), pose_pid_param_.max_output);
    pose_pid_.loadParam(pose_pid_param_.p,
                        pose_pid_param_.i,
                        pose_pid_param_.d,
                        pose_pid_param_.max_output,
                        -pose_pid_param_.max_output);
}