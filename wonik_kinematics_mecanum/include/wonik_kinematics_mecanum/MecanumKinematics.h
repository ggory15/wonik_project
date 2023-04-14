
#ifndef wonik_diffdrivekinematics_h_
#define wonik_diffdrivekinematics_h_

#include "Kinematics.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>

class Mecanum4WKinematics : public Kinematics {
public:
	Mecanum4WKinematics();
	void execForwKin(const sensor_msgs::JointState& js, nav_msgs::Odometry& odom, OdomPose& cpose);
	void execInvKin(const geometry_msgs::Twist& twist, trajectory_msgs::JointTrajectory& traj);
	/*
	 robot:	l1: m_dAxis1Length
		l2: m_dAxis2Length

	      --|##2##|        |##4##|
	      ^   ##################
	  l1  ¦   ##################             ^ y
	      ¦   ##################             ¦
	      v   ##################       x     ¦
	      --|##1##|        |##3##|     <-----¦-
		   |       l2     |
		   |<------------>|
	*/
	void setAxis1Length(double dLength);
	void setAxis2Length(double dLength);
	void setWheelDiameter(double dDiam);
	void setStdDev(double dStdDevX, double dStdDevY, double dStdDevZ, double dStdDevRoll, double dStdDevPitch, double dStdDevYaw);

private:
	double m_dAxis1Length;
	double m_dAxis2Length;
	double m_dDiam;
	double m_dStdDevX;
	double m_dStdDevY;
	double m_dStdDevZ;
	double m_dStdDevRoll;
	double m_dStdDevPitch;
	double m_dStdDevYaw;
};


#endif //wonik_diffdrivekinematics_h_
