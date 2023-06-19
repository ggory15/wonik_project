#include "md_motor_ros_sdk/com.hpp"

using namespace std;

MDWrapper::MDWrapper(const std::string & port, ROBOT_PARAMETER_t robotParamData,  INIT_SETTING_STATE_t fgInitsetting){
    port_ = port;
    robotParamData_ = robotParamData;
    fgInitsetting_ = fgInitsetting;
}

int MDWrapper::InitSerialComm()
{
    try
    {
        ser.setPort(port_);
        ser.setBaudrate(robotParamData_.nBaudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1667); //1667 when baud is 57600, 0.6ms
        ser.setTimeout(to);                                        //2857 when baud is 115200, 0.35ms
        ser.open();
    }

    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
        return 1;
    }
    else {
        return -1;
    }
} // Initialize

uint8_t MDWrapper::CalCheckSum(uint8_t *pData, uint16_t length)
{
    uint8_t sum;

    sum = 0;
    for(int i = 0; i < length; i++) {
        sum += *pData++;
    }

    sum = ~sum + 1; //check sum

    return sum;
}

int MDWrapper::PutMdData(PID_CMD_t pid, uint16_t rmid, const uint8_t *pData, uint16_t length)
{
    uint8_t *p;
    uint16_t len;

    len = 0;
    serial_comm_snd_buff[len++] = rmid;
    serial_comm_snd_buff[len++] = robotParamData_.nIDPC;
    serial_comm_snd_buff[len++] = 1;
    serial_comm_snd_buff[len++] = (uint8_t)pid;
    serial_comm_snd_buff[len++] = length;

    p = (uint8_t *)&serial_comm_snd_buff[len];
    memcpy((char *)p, (char *)pData, length);
    len += length;

    serial_comm_snd_buff[len++] = this->CalCheckSum(serial_comm_snd_buff, len);

#if 0
    {
        if(fgInitsetting == INIT_SETTING_STATE_NONE) {
            for(int i = 0; i < len; i++) {
                ROS_INFO("%2d: 0x%02x, %3d", i, serial_comm_snd_buff[i], serial_comm_snd_buff[i]);
            }
        }
    }
#endif

    if(ser.isOpen() == true) {
        ser.write(serial_comm_snd_buff, len);
    }

    return 1;
}

int MDWrapper::MdReceiveProc() //save the identified serial data to defined variable according to PID NUMBER data
{
    uint8_t *pRcvBuf;
    uint8_t *pRcvData;
    uint8_t byRcvPID;
    uint8_t byRcvDataSize;

    pRcvBuf = serial_comm_rcv_buff;

    byRcvPID      = pRcvBuf[MD_PROTOCOL_POS_PID];
    byRcvDataSize = pRcvBuf[MD_PROTOCOL_POS_DATA_LEN];
    pRcvData      = &pRcvBuf[MD_PROTOCOL_POS_DATA_START];

    switch(byRcvPID)
    {
        case PID_GAIN:
        {
            if(byRcvDataSize == sizeof(PID_GAIN_t)) {
                memcpy((char *)&curr_pid_gain, (char *)pRcvData, sizeof(PID_GAIN_t));

#if 0
                ROS_INFO("gain 1:%d", curr_pid_gain.position_proportion_gain);
                ROS_INFO("gain 2:%d", curr_pid_gain.speed_proportion_gain);
                ROS_INFO("gain 3:%d", curr_pid_gain.integral_gain);
#endif
            }
            break;
        }

        case PID_PNT_MAIN_DATA: // 210
        {
            if(fgInitsetting_ != INIT_SETTING_STATE_OK) {
                break;
            }

            if(byRcvDataSize == sizeof(PID_PNT_MAIN_DATA_t)) {
                memcpy((char *)&curr_pid_pnt_main_data, (char *)pRcvData, sizeof(PID_PNT_MAIN_DATA_t));

                //get encoder information
                v_id1_ = curr_pid_pnt_main_data.rpm_id1;
                if (v_id1_ > 0){
                    v_id1_ = v_id1_ + (robotParamData_.nGearRatio / 2);
                }
                else
                    v_id1_ = v_id1_ - (robotParamData_.nGearRatio / 2);

                v_id1_ /= robotParamData_.nGearRatio;

                v_id2_ = curr_pid_pnt_main_data.rpm_id2;
                if (v_id2_ > 0){
                    v_id2_ = v_id2_ + (robotParamData_.nGearRatio / 2);
                }
                else
                    v_id2_ = v_id2_ - (robotParamData_.nGearRatio / 2);

                v_id2_ /= robotParamData_.nGearRatio;

                v_id1_ *= 2.0 * M_PI / 60.;
                v_id2_ *= 2.0 * M_PI / 60.;
            }
            break;
        }

        case PID_ROBOT_PARAM:  // 247
        {
            if(byRcvDataSize == sizeof(PID_ROBOT_PARAM_t)) {
                PID_ROBOT_PARAM_t *p;

                p = (PID_ROBOT_PARAM_t *)pRcvData;
                robotParamData_.sSetDia      = p->nDiameter;             // mm unit
                robotParamData_.sSetWheelLen = p->nWheelLength;          // mm unit
                robotParamData_.sSetGear     = p->nGearRatio;
            }
            break;
        }

        case PID_ROBOT_MONITOR2:        // 224
        {
            if(byRcvDataSize == sizeof(PID_ROBOT_MONITOR2_t)) {
                memcpy((char *)&curr_pid_robot_monitor2, (char *)pRcvData, sizeof(PID_ROBOT_MONITOR2_t));
#if 0
                ROS_INFO("0x%x", curr_pid_robot_monitor2.sVoltIn);
                ROS_INFO("0x%x", curr_pid_robot_monitor2.byUS1);
                ROS_INFO("0x%x", curr_pid_robot_monitor2.byUS2);
                ROS_INFO("0x%x", curr_pid_robot_monitor2.byUS3);
                ROS_INFO("0x%x", curr_pid_robot_monitor2.byUS4);
                ROS_INFO("0x%x", curr_pid_robot_monitor2.byPlatStatus);
                ROS_INFO("0x%x", curr_pid_robot_monitor2.byDockingStatus);
                ROS_INFO("0x%x", curr_pid_robot_monitor2.head_motor_speed_ctrl_signal);
                ROS_INFO("");
#endif                
            }
            break;
        }

        case PID_PNT_IO_MONITOR:        // 241
        {
            if(byRcvDataSize == sizeof(PID_PNT_IO_MONITOR_t)) {
                memcpy((char *)&curr_pid_pnt_io_monitor, (char *)pRcvData, sizeof(PID_PNT_IO_MONITOR_t));
            }
            break;
        }
    }
    return 1;
}

int MDWrapper::AnalyzeReceivedData(uint8_t byArray[], uint8_t byBufNum) //Analyze the communication data
{
    ros::NodeHandle n;
    ros::Time stamp;
    uint8_t i, j;
    uint8_t data;
    static uint8_t byChkSec;
    static long lExStampSec, lExStampNsec;
    static uint32_t byPacketNum;
    static uint32_t rcv_step;
    static uint8_t byChkSum;
    static uint16_t byMaxDataNum;
    static uint16_t byDataNum;

    if(byPacketNum >= MAX_PACKET_SIZE)
    {
        rcv_step = 0;
        byPacketNum = 0;

        return 0;
    }

    for(j = 0; j < byBufNum; j++)
    {
        data = byArray[j];
#if 0
        ROS_INFO("%2d: %02x, %3d", j, data, data);
#endif
        switch(rcv_step) {
            case 0:    //Put the reading machin id after checking the data
                if(data == robotParamData_.nIDPC)
                {
                    byPacketNum = 0;
                    byChkSum = data;
                    serial_comm_rcv_buff[byPacketNum++] = data;

                    rcv_step++;
                }
                else
                {
                    byPacketNum = 0;
                }
                break;
            case 1:    //Put the transmitting machin id after checking the data
                if((data == robotParamData_.nIDMDUI) || (data == robotParamData_.nIDMDT))
                {
                    byChkSum += data;
                    serial_comm_rcv_buff[byPacketNum++] = data;

                    rcv_step++;
                }
                else
                {
                    rcv_step = 0;
                    byPacketNum = 0;

                }
                break;

            case 2:    //Check ID
                if(data == 1 || data == ID_ALL)
                {
                    byChkSum += data;
                    serial_comm_rcv_buff[byPacketNum++] = data;

                    rcv_step++;
                }
                else
                {
                    rcv_step = 0;
                    byPacketNum = 0;
                }
                break;
             case 3:    //Put the PID number into the array
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                rcv_step++;
                break;

             case 4:    //Put the DATANUM into the array
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                byMaxDataNum = data;
                byDataNum = 0;

                rcv_step++;
                break;

             case 5:    //Put the DATA into the array
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                if(++byDataNum >= MAX_DATA_SIZE)
                {
                    rcv_step = 0;
                    break;
                }

                if(byDataNum >= byMaxDataNum) {
                    rcv_step++;
                }
                break;

             case 6:    //Put the check sum after Checking checksum
                byChkSum += data;
                serial_comm_rcv_buff[byPacketNum++] = data;

                if(byChkSum == 0)
                {
                    MdReceiveProc();                                 //save the identified serial data to defined variable
                }

                rcv_step = 0;
                break;

            default:
                rcv_step = 0;
                break;
        }
    }
    return 1;
}

int MDWrapper::ReceiveDataFromController() //Analyze the communication data
{
    uint8_t byRcvBuf[250];
    uint8_t byBufNumber;

    byBufNumber = ser.available();
    if(byBufNumber != 0)
    {
        if(byBufNumber > sizeof(byRcvBuf)) {
            byBufNumber = sizeof(byRcvBuf);
        }

        ser.read(byRcvBuf, byBufNumber);
        this->AnalyzeReceivedData(byRcvBuf, byBufNumber);
    }

    return 1;
}