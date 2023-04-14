#include "wonik_kinematics_mecanum/Kinematics.h"
#include "wonik_kinematics_mecanum/MecanumKinematics.h"

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>
#include<std_msgs/Float64.h>


class WonikMecKinSimNode
{
public:
	WonikMecKinSimNode();
	virtual ~WonikMecKinSimNode();

	// ROS-Node handle 
	ros::NodeHandle nh;

	// Subscribers 
	ros::Subscriber topicSub_GazeboLinkState;
	ros::Subscriber topicSub_ComVel;
	ros::Subscriber topicsub_Joint_States;

	// Publishers
	ros::Publisher wheel0;
	ros::Publisher wheel1;
	ros::Publisher wheel2;
	ros::Publisher wheel3;
	ros::Publisher topicPub_Odometry;
	ros::Publisher topicPub_DriveCommands;

	// Message declarations
    geometry_msgs::Quaternion odom_quat;
    geometry_msgs::Twist twist;
	ros::Time joint_state_odom_stamp_;
	tf::TransformBroadcaster odom_broadcaster;

	// int init();
	void receiveCmd(const geometry_msgs::Twist& msg);
	void HeaderStampCB(const sensor_msgs::JointState::ConstPtr& msg);

private:
	boost::mutex mutex;
	Mecanum4WKinematics* kin = 0;
	OdomPose pose;
	bool sendTransform = false;
};



WonikMecKinSimNode::WonikMecKinSimNode()
{
	kin = new Mecanum4WKinematics();

	pose.xAbs = 0;
	pose.yAbs = 0;
	pose.phiAbs = 0;

	double wheelDiameter, axisWidth, axisLength;
	double devX, devY, devZ, devRoll, devPitch, devYaw;

	nh.param("wheelDiameter", wheelDiameter, 0.3);
	nh.param("robotWidth", axisWidth, 0.5);
	nh.param("robotLength", axisLength, 0.5);
	nh.param("devX", devX, 0.1);
	nh.param("devY", devY, 0.1);
	nh.param("devZ", devZ, 0.1);
	nh.param("devRoll", devRoll, 0.1);
	nh.param("devPitch", devPitch, 0.1);
	nh.param("devYaw", devYaw, 0.1);
	nh.param<bool>("sendTransform", sendTransform, false);
	kin->setWheelDiameter(wheelDiameter);
	kin->setAxis1Length(axisWidth);
	kin->setAxis2Length(axisLength);

	if(sendTransform) {
        ROS_INFO("wonik_kinematics_mecanum_sim_node: sending transformation");
	} else {
        ROS_INFO("wonik_kinematics_mecanum_sim_node: sending no transformation");
	}

	kin->setStdDev(devX, devY, devZ, devRoll, devPitch, devYaw);
	
	// Subscribe to joint_states 
	topicsub_Joint_States = nh.subscribe("/joint_states", 1, &WonikMecKinSimNode::HeaderStampCB, this); 

	// Subscribe to cmd_vel for the converstion to joint velocity
	topicSub_ComVel = nh.subscribe("/cmd_vel", 1, &WonikMecKinSimNode::receiveCmd, this);

	// Virtual controllers created for simulating the wheels  
	wheel0 = nh.advertise<std_msgs::Float64>("/wonik_robot_omni_wheel_back_left_controller/command", 1);
	wheel1 = nh.advertise<std_msgs::Float64>("/wonik_robot_omni_wheel_back_right_controller/command", 1);
	wheel2 = nh.advertise<std_msgs::Float64>("/wonik_robot_omni_wheel_front_left_controller/command", 1);
	wheel3 = nh.advertise<std_msgs::Float64>("/wonik_robot_omni_wheel_front_right_controller/command", 1);
}

WonikMecKinSimNode::~WonikMecKinSimNode()
{}


void WonikMecKinSimNode::HeaderStampCB(const sensor_msgs::JointState::ConstPtr& msg)
{
	joint_state_odom_stamp_ = msg->header.stamp;
}

void WonikMecKinSimNode::receiveCmd(const geometry_msgs::Twist& msg)
{
	twist = msg;
	trajectory_msgs::JointTrajectory traj;
	kin->execInvKin(twist, traj);
	std_msgs::Float64 f1,f2,f3,f4;
	f1.data = traj.points[0].velocities[0];
	f2.data = -traj.points[0].velocities[1];
	f3.data = traj.points[0].velocities[2];
	f4.data = -traj.points[0].velocities[3];

	wheel0.publish(f1);
	wheel1.publish(f2);
	wheel2.publish(f3);
	wheel3.publish(f4);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "wonik_kinematics_mecanum_sim_node");

	WonikMecKinSimNode node;

	ros::Rate loop_rate(100); 
	while (ros::ok())
   	{
      loop_rate.sleep();   
      ros::spinOnce();   		
   	}  
	return 0;
}