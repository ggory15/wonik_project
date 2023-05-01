#ifndef __wonik_drive__
#define __wonik_drive__

#include <sstream>
#include <iostream>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/BatteryState.h>
#include <trajectory_msgs/JointTrajectory.h>

// ROS service includes
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

// Wonik Drive for Fastech
#include "wonik_drive/drive.hpp"
#include "wonik_drive/StrUtil.h"
#include "wonik_drive/DriveParam.h"

// Fastech Motor
#include "fastech_ros_sdk/fastech/fastech_step.hpp"
#include "fastech_ros_sdk/ServoOn.h"

class WonikDriveNode
{
public:
	// create a handle for this node, initialize node
	ros::NodeHandle n;

	// Drives:
	ros::Publisher topicPub_drives;
	ros::Subscriber topicSub_drives;

	// Constructor
	WonikDriveNode();

	// Destructor
	~WonikDriveNode();

	// ---- Comm Handler ----
	int init();
	int HandleCommunication();

	// ---- Topic Callbacks ----
	void getNewVelocitiesFromTopic(const trajectory_msgs::JointTrajectory jt);

	void setJointVelocity(trajectory_msgs::JointTrajectory jt, int i);

	// ---- Pubisher functions ----
	// Motors
	void PublishJointStates();
	void getJointState(int i);

	// --- Timer Callbacks ---
	void print_info(const ros::TimerEvent&);

	// --- Motor Servo on with thread ---
	void servo_on(int  motor_index);


private:
	// ---- Motors ----
	DriveParam m_Drives[4];
	std::shared_ptr<FastechStepWrapper> m_ctrl[4];

	// ---- Msg Handling ------------
	ros::Time m_tCurrentTimeStamp;
	ros::Time m_last_trajectory_time;
	
	double m_tMotorDelay = 0;
	double m_trajectory_timeout = 1.0;
	bool is_trajectory_timeout = false;

	ros::Timer m_info_timer;
	trajectory_msgs::JointTrajectoryPoint prev_jt_;
	sensor_msgs::JointState state;

};
#endif