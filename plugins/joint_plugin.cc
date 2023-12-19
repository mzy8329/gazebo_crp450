#ifndef _JOINT_PLUGIN_HH_
#define _JOINT_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include <iostream>
#include <string>
#include <boost/algorithm/string/replace.hpp>
#include <dynamic_reconfigure/server.h>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"


namespace gazebo
{
	class JointPlugin : public ModelPlugin
	{
		public: JointPlugin() {}
		public: virtual void Load(physics::ModelPtr _mode, sdf::ElementPtr _sdf)
		{
			if (_mode->GetJointCount() == 0)
			{
				std::cerr << "Invalid joint count, position plugin not loaded\n";
				return;
			}
			
			if (_sdf->HasElement("jointname"))
				joint_name_ori = _sdf->Get<std::string>("jointname");
				joint_name_gz = _mode->GetScopedName() + "::" + joint_name_ori;
				joint_name = _mode->GetScopedName() + "/"  + joint_name_ori; 
				boost::replace_all(joint_name, "::", "/");
				boost::replace_all(joint_name, "-", "_");

				std::cerr << "We find the joint [" <<
				joint_name << "]\n";

			if (!ros::isInitialized())
			{
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, joint_name + "_" + "node",
					ros::init_options::NoSigintHandler);
			}
			nh = ros::NodeHandle(joint_name + "_" + "Handle");
			this->rosNode.reset(&nh);


			use_param = false;
			if(_sdf->HasElement("use_param") && _sdf->Get<double>("use_param"))
			{
				use_param = true;
			}
			this->model = _mode;

			std::cerr << "\n The model's name is [" <<_mode->GetScopedName() << "]\n";
			this->joint = _mode->GetJoint(joint_name_gz);

			this->model->GetJointController()->SetPositionTarget(joint_name_gz, position);

			// ros::SubscribeOptions pos_so = ros::SubscribeOptions::create<std_msgs::Float32>(
			// 		"/" + joint_name + "/pos_cmd",
			// 		1,
			// 		boost::bind(&JointPlugin::OnPosMsg, this, _1),
			// 		ros::VoidPtr(), &this->rosQueue);
			
			// ros::SubscribeOptions vel_so = ros::SubscribeOptions::create<std_msgs::Float32>(
			// 		"/" + joint_name + "/vel_cmd",
			// 		1,
			// 		boost::bind(&JointPlugin::OnVelMsg, this, _1),
			// 		ros::VoidPtr(), &this->rosQueue);
			ros::SubscribeOptions torque_so = ros::SubscribeOptions::create<std_msgs::Float32>(
					"/" + joint_name + "/torque_cmd",
					1,
					boost::bind(&JointPlugin::OnTorqueMsg, this, _1),
					ros::VoidPtr(), &this->rosQueue);

			// this->pos_rosSub = this->rosNode->subscribe(pos_so);
			// this->vel_rosSub = this->rosNode->subscribe(vel_so);
			this->torque_rosSub = this->rosNode->subscribe(torque_so);


			ros::AdvertiseOptions pos_ao = ros::AdvertiseOptions::create<std_msgs::Float32>(
					"/" + joint_name + "/pose",
					1,
					boost::bind(&JointPlugin::ConnectCb, this),
					boost::bind(&JointPlugin::disConnectCb, this),
					ros::VoidPtr(), &this->rosQueue);
			this->pos_rosPub = this->rosNode->advertise(pos_ao);

			ros::AdvertiseOptions vel_ao = ros::AdvertiseOptions::create<std_msgs::Float32>(
					"/" + joint_name + "/vel",
					1,
					boost::bind(&JointPlugin::ConnectCb, this),
					boost::bind(&JointPlugin::disConnectCb, this),
					ros::VoidPtr(), &this->rosQueue);
			this->vel_rosPub = this->rosNode->advertise(vel_ao);


			this->rosQueueThread = std::thread(std::bind(&JointPlugin::QueueThread, this));
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&JointPlugin::OnUpdate, this));
		}

		public: void OnUpdate()
		{
			pose.data = this->joint->Position();
			pos_rosPub.publish(pose);

			velocity.data = this->joint->GetVelocity(0);
			vel_rosPub.publish(velocity);
		}


		// public: void OnPosMsg(const std_msgs::Float32ConstPtr &_msg)
		// {
		// 	this->model->GetJointController()->SetPositionTarget(joint_name_gz, _msg->data);
		// }
		// public: void OnVelMsg(const std_msgs::Float32ConstPtr &_msg)
		// {
		// 	this->model->GetJointController()->SetVelocityTarget(joint_name_gz, _msg->data);
		// }
		public: void OnTorqueMsg(const std_msgs::Float32ConstPtr &_msg)
		{
			this->model->GetJointController()->SetForce(joint_name_gz, _msg->data);
		}


		public: void ConnectCb(){}
		public: void disConnectCb(){}

		private: void QueueThread()
		{
			static const double timeout = 0.01;
			while (this->rosNode->ok())
			{
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
			}
		}
	

		private: event::ConnectionPtr updateConnection;
		private: physics::ModelPtr model;
		private: physics::JointPtr joint;
		private: common::PID pid_pos, pid_vel;

		private: std::unique_ptr<ros::NodeHandle> rosNode;
		private: ros::Subscriber pos_rosSub;
		private: ros::Subscriber vel_rosSub;
		private: ros::Subscriber torque_rosSub;
		private: ros::Publisher pos_rosPub;
		private: ros::Publisher vel_rosPub;

		private: std_msgs::Float32 pose;
		private: std_msgs::Float32 velocity;
		private: bool use_param;
		private: ros::NodeHandle nh;

		/// \brief A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;
		/// \brief A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;
		private: double position = 0.0, p_gain_pos, i_gain_pos, d_gain_pos, p_gain_vel, i_gain_vel, d_gain_vel;

		public: std::string joint_name, joint_name_ori, joint_name_gz;

	};

	GZ_REGISTER_MODEL_PLUGIN(JointPlugin)
}
#endif