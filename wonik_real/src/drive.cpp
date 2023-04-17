#include <math.h>
#include <wonik_drive/drive.hpp>
#include <thread>
#include <mutex>

WonikDriveNode::WonikDriveNode(){
    m_Drives[0].sName = "FR";
    m_Drives[1].sName = "FL";
    m_Drives[2].sName = "RR";
    m_Drives[3].sName = "RL";  
}

WonikDriveNode::~WonikDriveNode(){
    for (int i=0; i<4; i++){
        m_ctrl[i]->ServoEnable(false);
        m_ctrl[i]->Disconnect();
    }
}

void WonikDriveNode::servo_on(int i){

    std::string msg = "Motor " + std::to_string(i) + " servo on";
    ROS_WARN_STREAM(msg);

    static std::mutex m;
    std::lock_guard<std::mutex> gaurd(m);
    m_ctrl[i] = std::make_shared<FastechStepWrapper>(m_Drives[i].ip_address,  m_Drives[i].port);
    int iret = m_ctrl[i]->Connect();

    if (iret != FMM_OK){
        ROS_ERROR("INIT_CONFIG_MOTOR %d", i);
        for (int j=0; j<i; j++){
            iret = m_ctrl[i]->MotorStop();
        }
    }

    ros::Duration duration(0.5);       
    
    m_ctrl[i]->Reset();
    m_ctrl[i]->ServoEnable(true);
    m_Drives[i].servo_on = true;

    duration.sleep();  
    iret = m_ctrl[i]->SetVelocity(500000, true); 
    iret = m_ctrl[i]->SetVelocityOveride(30000);

    m_Drives[i].servo_on = true;
}

int WonikDriveNode::init()
{

    for (int i=0; i<4; i++){
        const std::string parent = "drive" + std::to_string(i + 1) + "/";
        n.param(parent + "ip_address", m_Drives[i].ip_address, std::string("192.168.0.3"));
        n.param(parent + "port", m_Drives[i].port, 3002);
        n.param(parent + "gear_ratio", m_Drives[i].gear_ratio, 8);
        n.getParam(parent + "joint_name", m_Drives[i].sName);
        
        m_Drives[i].servo_on = false;
    }

    std::vector<std::thread> threads;

    threads.emplace_back(&WonikDriveNode::servo_on, this, 1); // motor 1 servo on
    threads.emplace_back(&WonikDriveNode::servo_on, this, 2); // motor 2 servo on
    threads.emplace_back(&WonikDriveNode::servo_on, this, 3); // motor 3 servo on
    threads.emplace_back(&WonikDriveNode::servo_on, this, 4); // motor 4 servo on
    
    for (auto &thread : threads)
        thread.join();
    
    topicPub_drives = n.advertise<sensor_msgs::JointState>("/drives/joint_states", 1);
    topicSub_drives = n.subscribe("/drives/joint_trajectory", 1, &WonikDriveNode::getNewVelocitiesFromTopic, this);


    m_info_timer = n.createTimer(ros::Duration(1), &WonikDriveNode::print_info, this);
	return 0;
}

int WonikDriveNode::HandleCommunication()
{
    const ros::Time now = ros::Time::now();

	// check for input timeout
	if ((now - m_last_trajectory_time).toSec() > m_trajectory_timeout)
	{
		if (!is_trajectory_timeout && !m_last_trajectory_time.isZero()) {
			ROS_WARN_STREAM("joint_trajectory input timeout! Stopping now.");
		}
		is_trajectory_timeout = true;
	}
	else {
		is_trajectory_timeout = false;
	}

	if (is_trajectory_timeout) {
		for (int i = 0; i < 4; i++) {
            m_ctrl[i]->SetVelocityOveride(0.0); // if there is somewhat wrong from nav2 pkg, i will set the motor speed as zero.
		}
	}

    return 0;
}


void WonikDriveNode::PublishJointStates()
{
	long lEnc[8] = {0, 0, 0, 0};

	sensor_msgs::JointState state;
	state.header.stamp = m_tCurrentTimeStamp - ros::Duration(m_tMotorDelay);

	// Publish Data for all possible Motors
	state.name.resize(4);
	state.position.resize(4);
	state.velocity.resize(4);

	for (int i = 0; i < 4; i++)
	{
        m_ctrl[i]->GetActualVel();
        m_ctrl[i]->Recieve();
        state.name[i] = m_Drives[i].sName.c_str();
        state.velocity[i] = PPS2RADSEC(m_ctrl[i]->get_velocity()) / (double)m_Drives->gear_ratio;	
	}

	topicPub_drives.publish(state);
}

void WonikDriveNode::getNewVelocitiesFromTopic(const trajectory_msgs::JointTrajectory jt)
{   
    for (int i = 0; i<4; i++){
	    if (!m_Drives[i].servo_on)
		    return;
    }

	trajectory_msgs::JointTrajectoryPoint d_point = jt.points[0];

    int iret;
    
	for (int i = 0; i < 4; i++)
	{
        double vel = RADSEC2PPS(d_point.velocities[i] * (double)m_Drives[i].gear_ratio); //rad/sec * gear_ratio = pps

        ROS_INFO_STREAM(vel);
    
		if (fabs(vel >= 0.1)){
            iret = m_ctrl[i]->SetVelocityOveride((int)vel );
        }
        else{
            iret = m_ctrl[i]->SetVelocity(500000, true); // default velocity
		    iret = m_ctrl[i]->SetVelocityOveride((int)vel );
        }
	}

	m_last_trajectory_time = ros::Time::now();
}

void WonikDriveNode::print_info(const ros::TimerEvent&)
{
	// if(m_enable_charging) {
	// 	int iChargingState = 0;
	// 	m_SerRelayBoard->getChargingState(&iChargingState);
	// 	ROS_INFO_STREAM("Charging: state = " << iChargingState
	// 			<< ", current = " << last_bstate_msg.current
	// 			<< " A, voltage = " << last_bstate_msg.voltage
	// 			<< " V, percentage = " << 100 * last_bstate_msg.percentage << " %");
	// }
}
