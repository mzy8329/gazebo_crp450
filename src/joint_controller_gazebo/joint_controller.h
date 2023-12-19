#ifndef _JOINT_CONTROLLER_H_
#define _JOINT_CONTROLLER_H_


#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "PID_controller.h"

typedef struct 
{   
    float p;
    float i;
    float d;
    float max_output;
}PID_Param_s;


class JointController
{
public:
    JointController(ros::NodeHandle nh, std::string joint_name,float loop_rate = 100);
    ~JointController(){;}

    void loadParam();

    PIDController pose_pid_;
    PIDController vel_pid_;

    ros::Subscriber vel_cmd_sub_;
    ros::Subscriber pose_cmd_sub_;

    ros::Subscriber vel_sub_;
    ros::Subscriber pose_sub_;

    ros::Publisher torque_pub_;


private:
    void velCmdCallback(const std_msgs::Float32::ConstPtr &msg);
    void poseCmdCallback(const std_msgs::Float32::ConstPtr &msg);

    void velCallback(const std_msgs::Float32::ConstPtr &msg);
    void poseCallback(const std_msgs::Float32::ConstPtr &msg);

    void loadVelPidParam();
    void loadPosePidParam();
    

    std::string joint_name_;

    ros::NodeHandle nh_;
    float loop_rate_;

    PID_Param_s pose_pid_param_;
    PID_Param_s vel_pid_param_;
    
};


#endif // _JOINT_CONTROLLER_H_