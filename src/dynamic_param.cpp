#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
//TutorialsConfig.h为 #3.2 编译过程中自动生成头文件
#include <gazebo_crp450/arm_pidConfig.h>



void callback(dynamic_arm_pid::arm_pidConfig &config, uint32_t level, ros::NodeHandle &nh) 
{  
    nh.setParam("/Crp450/Arm1_joint_p_gain_pose", config.arm1_p_gain_pose);
    nh.setParam("/Crp450/Arm1_joint_i_gain_pose", config.arm1_i_gain_pose);
    nh.setParam("/Crp450/Arm1_joint_d_gain_pose", config.arm1_d_gain_pose);
    nh.setParam("/Crp450/Arm1_joint_max_output_pose", config.arm1_max_output_pose);
    nh.setParam("/Crp450/Arm1_joint_p_gain_vel", config.arm1_p_gain_vel);
    nh.setParam("/Crp450/Arm1_joint_i_gain_vel", config.arm1_i_gain_vel);
    nh.setParam("/Crp450/Arm1_joint_d_gain_vel", config.arm1_d_gain_vel);
    nh.setParam("/Crp450/Arm1_joint_max_output_vel", config.arm1_max_output_vel);

    nh.setParam("/Crp450/Arm2_joint_p_gain_pose", config.arm2_p_gain_pose);
    nh.setParam("/Crp450/Arm2_joint_i_gain_pose", config.arm2_i_gain_pose);
    nh.setParam("/Crp450/Arm2_joint_d_gain_pose", config.arm2_d_gain_pose);
    nh.setParam("/Crp450/Arm2_joint_max_output_pose", config.arm2_max_output_pose);
    nh.setParam("/Crp450/Arm2_joint_p_gain_vel", config.arm2_p_gain_vel);
    nh.setParam("/Crp450/Arm2_joint_i_gain_vel", config.arm2_i_gain_vel);
    nh.setParam("/Crp450/Arm2_joint_d_gain_vel", config.arm2_d_gain_vel);
    nh.setParam("/Crp450/Arm2_joint_max_output_vel", config.arm2_max_output_vel);
}

int main(int argc, char **argv)
{    
	ros::init(argc, argv, "dynamic_param");
    ros::NodeHandle nh("~/dynamic_param");

	dynamic_reconfigure::Server<dynamic_arm_pid::arm_pidConfig> server;    
	dynamic_reconfigure::Server<dynamic_arm_pid::arm_pidConfig>::CallbackType f;    

	f = boost::bind(&callback, _1, _2, nh);    
	server.setCallback(f);
    
	ros::spin();    
	return 0;
}
