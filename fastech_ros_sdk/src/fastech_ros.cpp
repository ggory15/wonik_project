#include "ros/ros.h"

#include "fastech_ros_sdk/fastech/fastech_step.hpp"
#include "fastech_ros_sdk/ServoOn.h"
#include "sensor_msgs/JointState.h"


std::shared_ptr<FastechStepWrapper> ctrl_;
ros::Publisher joint_state_pub_;

std::string name_;
bool servo_on_ = false;

sensor_msgs::JointState joint_state_;

bool servoOnCallback(fastech_ros_sdk::ServoOn::Request & req, fastech_ros_sdk::ServoOn::Response & res){
	int iret = 0;
    servo_on_ = req.state;
    ros::Duration duration(0.1);
	if (req.state){
		iret = ctrl_->Reset();
		iret = ctrl_->ServoEnable(req.state);
        duration.sleep();   
		iret = ctrl_->SetVelocity(500000, true); // default velocity
		iret = ctrl_->SetVelocityOveride(0);
	}
	else{
        iret = ctrl_->MotorStop();
        duration.sleep();   
		// iret = ctrl_->ServoEnable(req.state);
	}

	if (iret == FMM_OK)
		res.result = true;
	else{
		res.result = false;
		ROS_ERROR_STREAM("Failed to serve on/off");
	}
	return res.result;
}

void setVelocityCallback(const sensor_msgs::JointState::ConstPtr & msg){
    if (!servo_on_){
        ROS_WARN_STREAM("Motor should be servo on state.");
    }
    else{
        int iret = 0;
        if (fabs(joint_state_.velocity[0]) >= 0.1)
            iret = ctrl_->SetVelocityOveride((int) ( RADSEC2PPS(msg->velocity[0] )) );
        else{
            iret = ctrl_->SetVelocity(500000, true); // default velocity
		    iret = ctrl_->SetVelocityOveride((int) ( RADSEC2PPS(msg->velocity[0] )) );
        }
    }

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"fastech_ros_sdk");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    std::string ip_address, name;
    int port;

    nh.getParam("fastech_ros_sdk/motor_ip", ip_address);
    nh.getParam("fastech_ros_sdk/port", port);
    nh.getParam("fastech_ros_sdk/motor_name", name);

    ctrl_ = std::make_shared<FastechStepWrapper>(ip_address, port);

    ctrl_->Connect();        
    // ctrl_->SetParameter(44, 0);
    
    joint_state_.name.resize(1);
    joint_state_.velocity.resize(1);
    joint_state_.name[0] = ip_address;
    joint_state_.velocity[0] = 0.0;

    ros::ServiceServer Servo_srv = nh.advertiseService("fastech_ros_sdk/" + name + "/servo_on", servoOnCallback);
    ros::Subscriber Set_Velocity_sub = nh.subscribe("fastech_ros_sdk/" + name + "/set_velocity", 10, setVelocityCallback);
    joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/fastech_ros_sdk/" + name + "/joint_states", 100);

    while(ros::ok())
    {
        ctrl_->GetActualVel();
        ctrl_->Recieve();

        joint_state_.velocity[0] = PPS2RADSEC(ctrl_->get_velocity());
        if (servo_on_)
            joint_state_pub_.publish(joint_state_);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ctrl_->Disconnect();


    return 0;
}