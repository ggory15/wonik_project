#include "wonik_drive/drive.hpp"

int main(int argc, char **argv)
{
	// initialize ROS
	ros::init(argc, argv, "wonik_drive_node");

	// keep a node handle outside the loop to prevent auto-shutdown
	ros::NodeHandle nh;

	while (ros::ok())
	{
		WonikDriveNode node;

		// initialize node
		if (node.init() != 0)
			return 1;

		// get parameters
		double request_rate;   // [1/s]
		node.n.param("request_rate", request_rate, 25.0); // Hz

		// frequency of publishing states (cycle time)
		ros::Rate rate(request_rate);

		while (nh.ok())
		{
			const ros::Time cycleStartTime = ros::Time::now();

			// Communication
			const int comState = node.HandleCommunication();

			// Motors
			node.PublishJointStates();

			ros::spinOnce();

			const ros::Duration cycleTime = ros::Time::now() - cycleStartTime;

			rate.sleep();
		}
	}

	return 0;
}
