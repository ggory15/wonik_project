#include <math.h>
#include <wonik_drive/drive.hpp>
#include <thread>
#include <mutex>

WonikDriveNode::WonikDriveNode(){
    m_Drives[0].sName = "FR";
    m_Drives[1].sName = "FR";
    m_Drives[2].sName = "FL";
    m_Drives[3].sName = "RR";
}

WonikDriveNode::~WonikDriveNode(){
    for (int i=0; i<4; i++){
        m_ctrl[i]->ServoEnable(false);
        m_ctrl[i]->Disconnect();
    }
}

void WonikDriveNode::servo_on(int i){

    std::string msg = "Motor " + std::to_string(i+1) + " servo on";
    
    static std::mutex m;
    std::lock_guard<std::mutex> gaurd(m);
    m_ctrl[i] = std::make_shared<FastechStepWrapper>(m_Drives[i].ip_address,  m_Drives[i].port);
    
    int iret = m_ctrl[i]->Connect();
 
    if (iret != FMM_OK){
        ROS_ERROR("INIT_CONFIG_MOTOR %d", i);
    }


    m_ctrl[i]->Reset();
    m_ctrl[i]->SetParameter(PARAM_POS_TRACKING_LIMIT, 10000000);
    m_ctrl[i]->SetParameter(PARAM_RUN_CURRENT, 15);
    m_ctrl[i]->SetParameter(PARAM_STOP_CURRENT, 8);
    m_ctrl[i]->SetParameter(PARAM_AXIS_ACC_TIME, 3000);
    m_ctrl[i]->SetParameter(PARAM_AXIS_DEC_TIME, 3000);
    m_ctrl[i]->SetParameter(PARAM_POS_GAIN, 0);
    m_ctrl[i]->SetParameter(PARAM_POS_OVERFLOW_LIMIT, 100000);
    
    m_ctrl[i]->ServoEnable(true);
    m_Drives[i].servo_on = true;

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    iret = m_ctrl[i]->SetVelocity(100000, true); 
    iret = m_ctrl[i]->SetVelocityOveride(0);
    ROS_WARN_STREAM(msg);

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
    threads.emplace_back(&WonikDriveNode::servo_on, this, 0); // motor 1 servo on
    threads.emplace_back(&WonikDriveNode::servo_on, this, 1);
    threads.emplace_back(&WonikDriveNode::servo_on, this, 2);
    threads.emplace_back(&WonikDriveNode::servo_on, this, 3);

    for (auto &thread : threads)
        thread.join();
    
    // m_ctrl[0]->enable_debug_ = true;

    topicPub_drives = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    topicSub_drives = n.subscribe("/drives/joint_trajectory", 1, &WonikDriveNode::getNewVelocitiesFromTopic, this);

    state.name.resize(4);
	state.position.resize(4);
	state.velocity.resize(4);


    m_info_timer = n.createTimer(ros::Duration(1), &WonikDriveNode::print_info, this);

	return 0;
}

int WonikDriveNode::HandleCommunication()
{
    const ros::Time now = ros::Time::now();

    m_tCurrentTimeStamp = ros::Time::now();
    prev_jt_.velocities.resize(4);

    for (int i=0; i<4; i++)
        prev_jt_.velocities[i] = 0.0;

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

void WonikDriveNode::getJointState(int i){
    static std::mutex m;
    std::lock_guard<std::mutex> gaurd(m);
}

void WonikDriveNode::PublishJointStates()
{
	long lEnc[4] = {0, 0, 0, 0};

	
	state.header.stamp = m_tCurrentTimeStamp - ros::Duration(m_tMotorDelay);

	// Publish Data for all possible Motors
	
    int iret, iret2;
	for (int i = 0; i < 4; i++)
	{   
        // m_ctrl[i]->EmptyRead();
        iret2 = m_ctrl[i]->GetActualVel();
        // iret2 = m_ctrl[i]->Recieve();
        
        state.name[i] = m_Drives[i].sName.c_str();
        double vel = m_ctrl[i]->get_velocity();
        state.velocity[i] = PPS2RADSEC(vel) / (double)m_Drives->gear_ratio;	
        m_Drives[i].current_vel = vel;
	}

    state.velocity[0] *= -1.;
    state.velocity[2] *= -1.;

    if (iret2 == FMM_OK)
	    topicPub_drives.publish(state);
}

void WonikDriveNode::setJointVelocity(trajectory_msgs::JointTrajectory jt, const int i){

    static std::mutex m;
    std::lock_guard<std::mutex> gaurd(m);

    trajectory_msgs::JointTrajectoryPoint d_point = jt.points[0];
    d_point.velocities[0] *= -1.;
    d_point.velocities[2] *= -1.;
    

    if (d_point.velocities[i] > 8.0)
        d_point.velocities[i] = 8.0;
    if (d_point.velocities[i] < -8.0)
        d_point.velocities[i] = -8.0;

    double vel = RADSEC2PPS(d_point.velocities[i] * (double)m_Drives[i].gear_ratio); //rad/sec * gear_ratio = pps
    double deadzone = 5000.0;
    double acc_limit = 5001.0;

    m_ctrl[i]->ClearPosition();
    
    if(vel >= 0.) {          
        // if (m_ctrl[i]->get_velocity() <= 0)
        {
            m_ctrl[i]->SetVelocity(65536, true); // default velocity
        }
        if (fabs(vel) < deadzone){
            vel = 0.;
        }
        m_ctrl[i]->SetVelocityOveride((int)vel);
    }
    else{
        // if (m_ctrl[i]->get_velocity() >= 0)
        {
            m_ctrl[i]->SetVelocity(65536, false); // default velocity
        }
        if (fabs(vel) < deadzone){
            vel = 0.;                    
        }
         m_ctrl[i]->SetVelocityOveride((int)vel);

    }
    
}
void WonikDriveNode::getNewVelocitiesFromTopic(const trajectory_msgs::JointTrajectory jt)
{   
    for (int i = 0; i<4; i++){
	    if (!m_Drives[i].servo_on)
		    return;
    }

	

    int iret;
    
    std::vector<std::thread> threads_2;
    threads_2.emplace_back(&WonikDriveNode::setJointVelocity, this, jt, 0); // motor 1 servo on
    threads_2.emplace_back(&WonikDriveNode::setJointVelocity, this, jt, 1);
    threads_2.emplace_back(&WonikDriveNode::setJointVelocity, this, jt, 2);
    threads_2.emplace_back(&WonikDriveNode::setJointVelocity, this, jt, 3);

    for (auto &thread : threads_2)
        thread.join();

// 	for (int i = 0; i < 4; i++)
// 	{   
//         d_point.velocities[0] *= -1.;
//         d_point.velocities[2] *= -1.;
        

//         if (d_point.velocities[i] > 8.0)
//             d_point.velocities[i] = 8.0;
//         if (d_point.velocities[i] < -8.0)
//             d_point.velocities[i] = -8.0;

//         double vel = RADSEC2PPS(d_point.velocities[i] * (double)m_Drives[i].gear_ratio); //rad/sec * gear_ratio = pps
//         double deadzone = 20000.0;
//         double acc_limit = 5001.0;

//         // if (vel > m_Drives[i].current_vel + acc_limit)
//         //     vel = m_Drives[i].current_vel + acc_limit;
//         // else if (vel <m_Drives[i].current_vel - acc_limit)
//         //     vel = m_Drives[i].current_vel - acc_limit;

//         m_ctrl[i]->ClearPosition();
//         // ROS_INFO_STREAM(m_Drives[i].current_vel );
//         // ROS_INFO_STREAM(vel);
// /*
// 		if (fabs(m_Drives[i].current_vel) >= 100){
//             iret = m_ctrl[i]->SetVelocityOveride((int)vel );
//         }
//         else{*/
//             // if(i==0 || i == 2){
//             if(vel >= 0.) {          
//                 if (m_ctrl[i]->get_velocity() <= 0)
//                 {
//                     // iret = m_ctrl[i]->SetVelocityOveride(0);
//                     // iret = m_ctrl[i]->SetVelocityOveride(0);
//                     // iret = m_ctrl[i]->SetVelocityOveride(0);
//                     // iret = m_ctrl[i]->SetVelocityOveride(0);
//                     iret = m_ctrl[i]->SetVelocity(65536, true); // default velocity
//                     // iret = m_ctrl[i]->SetVelocity(500000, true); // default velocity
//                 }
//                 if (fabs(vel) < deadzone){
//                     vel = 0.;
//                     // iret = m_ctrl[i]->MotorStop();
//                     // ROS_WARN_STREAM("deadzone");
//                 }
// 		        iret = m_ctrl[i]->SetVelocityOveride((int)vel);

                

//                 // if (prev_jt_.velocities[i] >= 0.0)
//                 //     iret = m_ctrl[i]->SetVelocityOveride((int)vel );
//                 // else    
//                 //     iret = m_ctrl[i]->SetVelocityOveride(0);
//             }
//             else{
//                 if (m_ctrl[i]->get_velocity() >= 0)
//                 {
//                     // iret = m_ctrl[i]->SetVelocityOveride(0);
//                     // iret = m_ctrl[i]->SetVelocityOveride(0);
//                     // iret = m_ctrl[i]->SetVelocityOveride(0);
//                     // iret = m_ctrl[i]->SetVelocityOveride(0);
//                     iret = m_ctrl[i]->SetVelocity(65536, false); // default velocity
//                 }
//                 // iret = m_ctrl[i]->SetVelocity(-500000, false); // default velocity
//                 if (fabs(vel) < deadzone){
//                     vel = 0.;
//                     // iret = m_ctrl[i]->MotorStop();
//                     //  ROS_WARN_STREAM("deadzone");
//                 }
//                 // iret = m_ctrl[i]->SetVelocityOveride(0);
//                 // iret = m_ctrl[i]->SetVelocityOveride(0);
//                 // iret = m_ctrl[i]->SetVelocityOveride(0);
//                 // iret = m_ctrl[i]->SetVelocityOveride(0);
//                 // iret = m_ctrl[i]->SetVelocity(65536, false); // default velocity
//                 iret = m_ctrl[i]->SetVelocityOveride((int)vel);
//                 // if (prev_jt_.velocities[i] <= 0.0)
//                 //     iret = m_ctrl[i]->SetVelocityOveride((int)vel );
//                 // else    
//                 //     iret = m_ctrl[i]->SetVelocityOveride(0);
//             }
//         }
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
