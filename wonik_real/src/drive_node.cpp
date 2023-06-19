#include "md_motor_ros_sdk/com.hpp"

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include <thread>
#include <mutex>

ROBOT_PARAMETER_t robotParamData;

SETTINNG_PARAM_STEP_t byCntInitStep;
volatile uint16_t appTick;
volatile uint16_t req_pid_pnd_main_data_count;
uint16_t byCntComStep[2];
uint32_t velCmdUpdateCount;
uint32_t velCmdRcvCount;

static uint8_t byCntCmdVel;
static uint8_t fgSendCmdVel;

static uint8_t byCntCase[10];
static uint8_t byFglIO;
INIT_SETTING_STATE_t fgInitsetting;

double goal_cmd_speed;             // m/sec
double goal_cmd_ang_speed;         // radian/sec
bool reset_odom_flag;
bool reset_alarm_flag;

ros::Publisher topicPub_drives;

sensor_msgs::JointState state;

std::shared_ptr<MDWrapper> m_ctrl[2];

void getNewVelocitiesFromTopic(const trajectory_msgs::JointTrajectory jt)
{   
    trajectory_msgs::JointTrajectoryPoint d_point = jt.points[0];
    d_point.velocities[0] *= -1.;
    d_point.velocities[2] *= -1.;


    PID_PNT_VEL_CMD_t pid_pnt_vel_cmd, *p;

    p = &pid_pnt_vel_cmd;
    p->enable_id1 = 1;
    p->rpm_id1 = 0; // rpm
    p->enable_id2 = 1;
    p->rpm_id2 = 0; // rpm
    p->req_monitor_id = REQUEST_PNT_MAIN_DATA;

    m_ctrl[0]->PutMdData(PID_PNT_VEL_CMD, robotParamData.nRMID, (const uint8_t *)&pid_pnt_vel_cmd, sizeof(pid_pnt_vel_cmd));
    m_ctrl[1]->PutMdData(PID_PNT_VEL_CMD, robotParamData.nRMID, (const uint8_t *)&pid_pnt_vel_cmd, sizeof(pid_pnt_vel_cmd));


} // velocity command


void servo_on(int i){
    static std::mutex m;
    std::lock_guard<std::mutex> gaurd(m);

    m_ctrl[i]->InitSerialComm();     //communication initialization in com.cpp

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void AppTickTimerCallback(const ros:: TimerEvent&)
{
    appTick++;
}

void InitMotorParameter(int drive_num)
{
    switch(byCntInitStep)
    {
        case SETTING_PARAM_STEP_PID_PNT_VEL_CMD:
        {
            PID_PNT_VEL_CMD_t cmd_data, *p;

#if 0
            ROS_INFO("SET PID_PNT_VEL_CMD(%d)", PID_PNT_VEL_CMD);
            ROS_INFO("size of PID_PNT_VEL_CMD_t: %ld", sizeof(PID_PNT_VEL_CMD_t));
#endif

            p = &cmd_data;
            p->enable_id1 = 1;
            p->rpm_id1 = 0;
            p->enable_id2 = 1;
            p->rpm_id2 = 0;
            p->req_monitor_id = REQUEST_PNT_MAIN_DATA;
            m_ctrl[drive_num]->PutMdData(PID_PNT_VEL_CMD, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data)); // 207

            byCntInitStep = SETTING_PARAM_STEP_PID_ROBOT_PARAM;
            break;
        }

        case SETTING_PARAM_STEP_PID_ROBOT_PARAM:
        {
            if(robotParamData.use_MDUI == 1) {  // If using MDUI
                if(robotParamData.nRMID == robotParamData.nIDMDUI) 
                {
                    PID_ROBOT_PARAM_t cmd_data, *p;

    #if 0
                    ROS_INFO("SETTING_PARAM_STEP_PID_ROBOT_PARAMv(%d)", SETTING_PARAM_STEP_PID_ROBOT_PARAM);
                    ROS_INFO("size of PID_ROBOT_PARAM_t: %ld", sizeof(c));
    #endif
                    p = &cmd_data;
                    p->nDiameter = (uint16_t)robotParamData.nDiameter;
                    p->nWheelLength = (uint16_t)robotParamData.nWheelLength * 1000;                 // m unit --> mm unit
                    p->nGearRatio = (uint16_t)robotParamData.nGearRatio;
                    
                    m_ctrl[drive_num]->PutMdData(PID_ROBOT_PARAM, MID_MDUI, (const uint8_t *)p, sizeof(cmd_data));     // 247
                    

                    byCntInitStep = SETTING_PARAM_STEP_PID_POSI_RESET;
                }
                else {
                    ROS_INFO("err.mismatch ID(RMID(%d), MDUI(%d))", robotParamData.nRMID, robotParamData.nIDMDUI);
                    fgInitsetting = INIT_SETTING_STATE_ERROR;
                    m_ctrl[drive_num]->setInitSetting(fgInitsetting);
                }
            }
            else {
                byCntInitStep = SETTING_PARAM_STEP_PID_POSI_RESET;
            }
            break;
        }

        case SETTING_PARAM_STEP_PID_POSI_RESET:
        {
#if 0
            ROS_INFO("PID_POSI_RESET(%d)", PID_POSI_RESET);
#endif

            m_ctrl[drive_num]->PutMdData(PID_POSI_RESET, robotParamData.nRMID, NULL, 0);

            byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_START;
            break;
        }

        case SETTING_PARAM_STEP_PID_SLOW_START:
        {
            PID_SLOW_START_t cmd_data, *p;

#if 0
            ROS_INFO("PID_SLOW_START(%d)", PID_SLOW_START);
            ROS_INFO("size of PID_SLOW_START_t: %ld", sizeof(PID_SLOW_START_t));
#endif

            p = &cmd_data;
            p->value = robotParamData.nSlowstart;

            m_ctrl[drive_num]->PutMdData(PID_SLOW_START, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data));

            byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_DOWN;
            break;
        }

        case SETTING_PARAM_STEP_PID_SLOW_DOWN:
        {
            PID_SLOW_DOWN_t cmd_data, *p;

#if 0
            ROS_INFO("PID_SLOW_DOWN(%d)", PID_SLOW_DOWN);
            ROS_INFO("size of PID_SLOW_DOWN_t: %ld", sizeof(PID_SLOW_DOWN_t));
#endif

            p = &cmd_data;
            p->value = robotParamData.nSlowdown;

            m_ctrl[drive_num]->PutMdData(PID_SLOW_DOWN, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data));

            byCntInitStep = SETTING_PARAM_STEP_PID_GAIN;
            break;
        }

        case SETTING_PARAM_STEP_PID_GAIN:
        {
            PID_GAIN_t cmd_data, *p;

#if 0
            ROS_INFO("PID_GAIN(%d)", PID_GAIN);
            ROS_INFO("size of PID_GAIN: %ld", sizeof(PID_GAIN_t));
#endif

            p = &cmd_data;
            p->position_proportion_gain = robotParamData.position_proportion_gain;
            p->speed_proportion_gain = robotParamData.speed_proportion_gain;
            p->integral_gain = robotParamData.integral_gain;

            m_ctrl[drive_num]->PutMdData(PID_GAIN, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data));

            byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD;
            break;
        }

        case SETTING_PARAM_STEP_PID_INV_SIGH_CMD:       // Left motor
        {
            uint8_t cmd_data;

#if 1
            ROS_INFO("PID_INV_SIGN_CMD(%d)", PID_INV_SIGN_CMD);
#endif

            if(robotParamData.reverse_direction == 0) {
                cmd_data = 1;
            }
            else {
                cmd_data = 0;
            }

            m_ctrl[drive_num]->PutMdData(PID_INV_SIGN_CMD, robotParamData.nRMID, (const uint8_t *)&cmd_data, 1);

            byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD2;
            break;
        }

        case SETTING_PARAM_STEP_PID_INV_SIGH_CMD2:      // Right motor
        {
            uint8_t cmd_data;

#if 1
            ROS_INFO("PID_INV_SIGN_CMD2(%d)", PID_INV_SIGN_CMD2);
#endif

            if(robotParamData.reverse_direction == 0) {
                cmd_data = 0;
            }
            else {
                cmd_data = 1;
            }

            m_ctrl[drive_num]->PutMdData(PID_INV_SIGN_CMD2, robotParamData.nRMID, (const uint8_t *)&cmd_data, 1);

            byCntInitStep = SETTING_PARAM_STEP_PID_USE_EPOSI;
            break;
        }

        case SETTING_PARAM_STEP_PID_USE_EPOSI:
        {
            uint8_t cmd_data;

#if 1
            ROS_INFO("PID_USE_POSI(%d)", PID_USE_POSI);
#endif

            if(robotParamData.motor_position_type == 0) {
                cmd_data = 0;               // hall sensor
            }
            else {
                cmd_data = 1;               // encoder
            }

            m_ctrl[drive_num]->PutMdData(PID_USE_POSI, robotParamData.nRMID, (const uint8_t *)&cmd_data, 1);

            byCntInitStep = SETTING_PARAM_STEP_PID_PPR;
            break;
        }

        case SETTING_PARAM_STEP_PID_PPR:
        {
            PID_PPR_t cmd_data, *p;

#if 1
            ROS_INFO("PID_PPR(%d)", PID_PPR);
#endif
            p = &cmd_data;

            p->PPR = robotParamData.encoder_PPR;

            m_ctrl[drive_num]->PutMdData(PID_PPR, robotParamData.nRMID, (const uint8_t *)&cmd_data, sizeof(PID_PPR_t));

            byCntInitStep = SETTING_PARAM_STEP_DONE;

            fgInitsetting = INIT_SETTING_STATE_OK;
            m_ctrl[drive_num]->setInitSetting(fgInitsetting);
            break;
        }

        default:
            break;
    }
}

void RequestRobotStatusTask(int drive_num)
{
    int nArray[5];
    uint8_t req_pid;

    switch(byCntComStep[drive_num])
    {
        case 0:
        {
            req_pid = PID_PNT_MAIN_DATA;            //PID 210
            m_ctrl[drive_num]->PutMdData(PID_REQ_PID_DATA, robotParamData.nRMID, (const uint8_t *)&req_pid, 1);

            if(robotParamData.use_MDUI == 1) {  // If using MDUI
                byCntComStep[drive_num] = 1;
            }
            else {
                byCntComStep[drive_num] = 3;
            }
            break;
        }

        case 1:
        {
            if(robotParamData.use_MDUI == 1) {  // If using MDUI
                req_pid = PID_ROBOT_MONITOR2;            // PID 224, Only MDUI
                m_ctrl[drive_num]->PutMdData(PID_REQ_PID_DATA, MID_MDUI, (const uint8_t *)&req_pid, 1);
            }

            byCntComStep[drive_num] = 3;
            break;
        }

#if 0
        case 2:
        {
            if(robotParamData.use_MDUI == 1) {  // If using MDUI
                req_pid = PID_PNT_IO_MONITOR;               // PID 241, Only MDUI, but not using
                PutMdData(PID_REQ_PID_DATA, MID_MDUI, (const uint8_t *)&req_pid, 1);
            }

            byCntComStep[drive_num] = 3;
            break;
        }
#endif

        case 3:
        {
            if(m_ctrl[drive_num]->curr_pid_robot_monitor2.byPlatStatus.bits.bEmerSW == 1)
            {
                PID_PNT_TQ_OFF_t pid_pnt_tq_off, *p;

                fgSendCmdVel = 0;
                pid_pnt_tq_off.enable_id1 = 1;
                pid_pnt_tq_off.enable_id2 = 1;
                pid_pnt_tq_off.req_monitor_id = REQUEST_PNT_MAIN_DATA;
                m_ctrl[drive_num]->PutMdData(PID_PNT_TQ_OFF, robotParamData.nRMID, (const uint8_t *)&pid_pnt_tq_off, sizeof(pid_pnt_tq_off));
            }
            else if(reset_odom_flag == true) {
                reset_odom_flag = false;

                m_ctrl[drive_num]->PutMdData(PID_POSI_RESET, robotParamData.nRMID, NULL, 0);
            }
            else if(reset_alarm_flag == true) {
                uint8_t cmd_pid;

                reset_alarm_flag = false;

                cmd_pid = CMD_ALARM_RESET;
                m_ctrl[drive_num]->PutMdData(PID_COMMAND, robotParamData.nRMID, (const uint8_t *)&cmd_pid, 1);
            }

            byCntComStep[drive_num] = 0;
            break;
        }

        case 4:
            req_pid = PID_GAIN;            // PID: 203
            m_ctrl[drive_num]->PutMdData(PID_REQ_PID_DATA, robotParamData.nIDMDT, (const uint8_t *)&req_pid, 1);

            byCntComStep[drive_num] = 0;
            break;

        default:
            byCntComStep[drive_num] = 0;
            break;
    }
}


int main(int argc, char **argv)
{
	// initialize ROS
	ros::init(argc, argv, "wonik_drive_node");

	// keep a node handle outside the loop to prevent auto-shutdown
	ros::NodeHandle nh;

    ros::Subscriber topicSub_drives = nh.subscribe("/drives/joint_trajectory", 1, getNewVelocitiesFromTopic);
    topicPub_drives = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    state.name.resize(4);
	state.position.resize(4);
	state.velocity.resize(4);

    reset_odom_flag = false;
    reset_alarm_flag = false;

    fgSendCmdVel = 1;
    fgInitsetting = INIT_SETTING_STATE_NONE;

    nh.getParam("use_MDUI", robotParamData.use_MDUI);
    nh.getParam("wheel_radius", robotParamData.wheel_radius);               // m unit
    nh.getParam("wheel_length", robotParamData.nWheelLength);                // m unit
    nh.getParam("reduction", robotParamData.nGearRatio);
    nh.getParam("motor_pole", robotParamData.motor_pole);
    nh.getParam("reverse_direction", robotParamData.reverse_direction);
    nh.getParam("maxrpm", robotParamData.nMaxRPM);
    nh.getParam("motor_posi", robotParamData.motor_position_type);
    nh.getParam("encoder_PPR", robotParamData.encoder_PPR);
    nh.getParam("position_proportion_gain", robotParamData.position_proportion_gain);
    nh.getParam("speed_proportion_gain", robotParamData.speed_proportion_gain);
    nh.getParam("integral_gain", robotParamData.integral_gain);
    nh.getParam("slow_start", robotParamData.nSlowstart);
    nh.getParam("slow_down", robotParamData.nSlowdown);

    robotParamData.nIDPC = MID_PC;         // Platform mini-PC ID
    robotParamData.nIDMDUI = MID_MDUI;       // MDUI ID
    robotParamData.nIDMDT = MID_MDT;        // MD750T, MD400T, MD200T ID
    if(robotParamData.use_MDUI == 1) {  // If using MDUI
        robotParamData.nRMID = robotParamData.nIDMDUI;
        ROS_INFO("Using MDUI");
    }
    else {
        robotParamData.nRMID = robotParamData.nIDMDT;
        ROS_INFO("Not using MDUI");
    }

    robotParamData.nBaudrate = 57600;               // fixed

    ROS_INFO("PC ID          : %d", robotParamData.nIDPC);
    ROS_INFO("MDUI ID        : %d", robotParamData.nIDMDUI);
    ROS_INFO("MDT ID         : %d", robotParamData.nIDMDT);
    ROS_INFO("Receving ID    : %d", robotParamData.nRMID);
    ROS_INFO("baudrate       : %d", robotParamData.nBaudrate);
    ROS_INFO("Wheel Radius(m): %f", robotParamData.wheel_radius);
    ROS_INFO("WheelLength(m) : %f", robotParamData.nWheelLength);
    ROS_INFO("Reduction rate : %d", robotParamData.nGearRatio);
    ROS_INFO("Motor pole     : %d", robotParamData.motor_pole);
    if(robotParamData.motor_position_type == 0) {
        ROS_INFO("motor position detection: hall sensor");
    }
    else {
        ROS_INFO("motor position detection: encoder");
    }

    // If use hall sensor: 3 x pole no x reduction rate
    // If use encoder: 4 x encder x reduction rate
    ROS_INFO("PPR: %d", robotParamData.encoder_PPR);

    if(robotParamData.motor_position_type == 0) {
        ROS_INFO("Robot direction: Forward");
    }
    else {
        ROS_INFO("Robot direction: Reverse");
    }
    ROS_INFO("Max RPM        : %d",             robotParamData.nMaxRPM);
    ROS_INFO("Position proportion gain: %d",    robotParamData.position_proportion_gain);
    ROS_INFO("Speed proportion gain   : %d",    robotParamData.speed_proportion_gain);
    ROS_INFO("Integral gain  : %d",             robotParamData.integral_gain);
    ROS_INFO("Slow start     : %d",             robotParamData.nSlowstart);
    ROS_INFO("Slow down      : %d\r\n",         robotParamData.nSlowdown);

    if(robotParamData.motor_position_type == 0) {
        robotParamData.motor_count = robotParamData.motor_pole * 3 * robotParamData.nGearRatio;
    }
    else {
        robotParamData.motor_count = robotParamData.encoder_PPR * 4 * robotParamData.nGearRatio;
    }

    robotParamData.motor_count_per_degree = (double)(360.0 / (double)robotParamData.motor_count);
    ROS_INFO("count per degree: %f", robotParamData.motor_count_per_degree);

    robotParamData.nDiameter = (int)(robotParamData.wheel_radius * 2.0 * 1000.0);              // nDiameter is (mm) unit

    std::string port1, port2;
    nh.getParam("port1", port1);
    nh.getParam("port2", port2);
    m_ctrl[0] = std::make_shared<MDWrapper>(port1, robotParamData, fgInitsetting);
    m_ctrl[1] = std::make_shared<MDWrapper>(port2, robotParamData, fgInitsetting);

    // Initialize
    std::vector<std::thread> threads;
    threads.emplace_back(&servo_on, 0); // motor 1 servo on
    threads.emplace_back(&servo_on, 1);

    for (auto &thread : threads)
        thread.join();

    ros::Rate r(1000);       //Set the loop period -> 1ms.
    ros::Time start_time = ros::Time::now();
    ros::Duration start_delay(1.5);
    double start_delay_sec = ros::Time::now().toSec();

    start_delay_sec += start_delay.toSec();

    //---------------------------------------------------------------------------------------------------------
    // Start delay: 1.5sec
    //---------------------------------------------------------------------------------------------------------
    while(ros::ok())
    {
        if(ros::Time::now().toSec() >= start_delay_sec) {
            break;
        }

        m_ctrl[0]->ReceiveDataFromController();
        m_ctrl[1]->ReceiveDataFromController();

        ros::spinOnce();
        r.sleep();
    }

    appTick = 0;
    byCntInitStep = SETTING_PARAM_STEP_PID_PNT_VEL_CMD;
    ros::Timer app_tick_timer = nh.createTimer(ros::Duration(0.500), AppTickTimerCallback);
    app_tick_timer.start();

    while(ros::ok())
    {
        m_ctrl[0]->ReceiveDataFromController();
        m_ctrl[1]->ReceiveDataFromController();

        if(appTick > 0) {
            appTick = 0;

            InitMotorParameter(0);
            byCntInitStep = SETTING_PARAM_STEP_PID_PNT_VEL_CMD;
            InitMotorParameter(1);
            
            if(fgInitsetting != INIT_SETTING_STATE_NONE) {
                break;
            }
        }

        ros::spinOnce();
        r.sleep();
    }

    app_tick_timer.stop();

    if(fgInitsetting != INIT_SETTING_STATE_OK) {
        ROS_INFO("error.init ROBOT");
    }
    else {
        ROS_INFO("Init done");
    }

    byCntComStep[0]= 0;
    byCntComStep[1]= 0;
    
    appTick = 0;
    app_tick_timer.setPeriod(ros::Duration(0.02));
    app_tick_timer.start();

    while(ros::ok())
    {
        m_ctrl[0]->ReceiveDataFromController();
        m_ctrl[1]->ReceiveDataFromController();

        if(appTick > 0) {
            appTick = 0;

            RequestRobotStatusTask(0);
            RequestRobotStatusTask(1);
        }

        long lEnc[4] = {0, 0, 0, 0};
        state.header.stamp = ros::Time::now();

        state.velocity[0] = m_ctrl[0]->v_id1_;
        state.velocity[1] = m_ctrl[0]->v_id2_;
        state.velocity[2] = m_ctrl[1]->v_id1_;
        state.velocity[3] = m_ctrl[1]->v_id2_;
        

        // // Publish Data for all possible Motors
        // int iret, iret2;
        // for (int i = 0; i < 2; i++)
        // {   
        //     iret2 = m_ctrl[i]->GetActualVel();
        //     // iret2 = m_ctrl[i]->Recieve();
            
        //     state.name[i] = m_Drives[i].sName.c_str();
        //     double vel = m_ctrl[i]->get_velocity();
        //     state.velocity[i] = PPS2RADSEC(vel) / (double)m_Drives->gear_ratio * 5.0;	
        //     m_Drives[i].current_vel = vel;
        // }
        
        // // ROS_WARN_STREAM(m_ctrl[3]->get_velocity());
        // state.velocity[0] *= -1.;
        // state.velocity[2] *= -1.;

        // if (iret2 == FMM_OK)
        topicPub_drives.publish(state);

        ros::spinOnce();
        r.sleep();
    }


	return 0;
}
