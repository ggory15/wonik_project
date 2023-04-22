#include "wonik_kinematics_mecanum/MecanumKinematics.h"

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>


class PlatformCtrlNode 
{
public:
	virtual ~PlatformCtrlNode();

	ros::NodeHandle nh;
	ros::Publisher topicPub_Odometry;	
	ros::Subscriber topicSub_DriveState;
	ros::Publisher topicPub_DriveCommands;	
	ros::Subscriber topicSub_ComVel;
	tf::TransformBroadcaster odom_broadcaster;

	int init();
	void receiveCmd(const geometry_msgs::Twist& twist);
	void sendOdom(const sensor_msgs::JointState& js);

private:
	boost::mutex mutex;
	OdomPose pose;
	Mecanum4WKinematics* kin = 0;
	bool sendTransform = false;
	std::string odom_tf_name = "/odom";

};

PlatformCtrlNode::~PlatformCtrlNode()
{
	delete kin;
}

int PlatformCtrlNode::init()
{
	kin = new Mecanum4WKinematics();

	pose.xAbs = 0;
	pose.yAbs = 0;
	pose.phiAbs = 0;

	double wheelDiameter, axisWidth, axisLength;
	double devX, devY, devZ, devRoll, devPitch, devYaw, factor;

	nh.param("wheelDiameter", wheelDiameter, 0.3);
	nh.param("robotWidth", axisWidth, 0.5);
	nh.param("robotLength", axisLength, 0.5);
	nh.param("devX", devX, 0.1);
	nh.param("devY", devY, 0.1);
	nh.param("devZ", devZ, 0.1);
	nh.param("devRoll", devRoll, 0.1);
	nh.param("devPitch", devPitch, 0.1);
	nh.param("devYaw", devYaw, 0.1);
	nh.param("pacticalFactor", factor, 1.0);

	axisWidth = axisWidth / factor;
	axisLength = axisLength /factor;

	nh.param<bool>("sendTransform", sendTransform, false);
	nh.param<std::string>("odom", odom_tf_name, "/odom");

	kin->setWheelDiameter(wheelDiameter);
	kin->setAxis1Length(axisWidth);
	kin->setAxis2Length(axisLength);

	if(sendTransform) {
        ROS_INFO("wonik_kinematics_mecanum_node: sending transformation");
	} else {
        ROS_INFO("wonik_kinematics_mecanum_node: sending no transformation");
	}

	kin->setStdDev(devX, devY, devZ, devRoll, devPitch, devYaw);
	
	topicPub_Odometry = nh.advertise<nav_msgs::Odometry>(odom_tf_name.c_str(), 1);
    topicSub_DriveState = nh.subscribe("/joint_states", 1, &PlatformCtrlNode::sendOdom, this);
    topicPub_DriveCommands = nh.advertise<trajectory_msgs::JointTrajectory>("/drives/joint_trajectory", 1);
	topicSub_ComVel = nh.subscribe("/cmd_vel", 1, &PlatformCtrlNode::receiveCmd, this);
	return 0;
}

void PlatformCtrlNode::receiveCmd(const geometry_msgs::Twist& twist)
{
	boost::lock_guard<boost::mutex> lock(mutex);

	trajectory_msgs::JointTrajectory traj;
	kin->execInvKin(twist, traj);
	topicPub_DriveCommands.publish(traj);
}

void PlatformCtrlNode::sendOdom(const sensor_msgs::JointState& js)
{
	boost::lock_guard<boost::mutex> lock(mutex);

    //check if js has data from 4 motors
	if(js.velocity.size() < 4) {
		return;
	}

	//odometry msg
	nav_msgs::Odometry odom;
	odom.header.stamp = js.header.stamp;
	odom.header.frame_id = odom_tf_name;
	odom.child_frame_id = "base_link";
	kin->execForwKin(js, odom, pose);

	// ROS_WARN_STREAM(odom);
	topicPub_Odometry.publish(odom);

	//odometry transform:
	if(sendTransform)
	{
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = odom.header.stamp;
		odom_trans.header.frame_id = odom.header.frame_id;
		odom_trans.child_frame_id = odom.child_frame_id;
		odom_trans.transform.translation.x = odom.pose.pose.position.x;
		odom_trans.transform.translation.y = odom.pose.pose.position.y;
		odom_trans.transform.translation.z = odom.pose.pose.position.z;
		odom_trans.transform.rotation = odom.pose.pose.orientation;
		odom_broadcaster.sendTransform(odom_trans);
	}
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "wonik_kinematics_mecanum_node");

	PlatformCtrlNode node;
    if(node.init() != 0) {
    	ROS_ERROR("wonik_kinematics_mecanum_node: init failed!");
    }

	ros::spin();

	return 0;
}

