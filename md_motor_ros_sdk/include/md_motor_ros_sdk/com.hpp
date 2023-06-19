#ifndef __md_wrapper__
#define __md_wrapper__

#include "md_motor_ros_sdk/global.hpp"

class MDWrapper{
    public:
        MDWrapper(const std::string & port,  ROBOT_PARAMETER_t robotParamData, INIT_SETTING_STATE_t fgInitsetting);
        ~MDWrapper(){};

        int InitSerialComm();
        uint8_t CalCheckSum(uint8_t *pData, uint16_t length);
        int PutMdData(PID_CMD_t pid, uint16_t rmid, const uint8_t *pData, uint16_t length);
        int ReceiveDataFromController();
        int MdReceiveProc();
        int AnalyzeReceivedData(uint8_t byArray[], uint8_t byBufNum);
        void setInitSetting(INIT_SETTING_STATE_t fgInitsetting){
            fgInitsetting_ = fgInitsetting;
        }


        serial::Serial ser;

        PID_PNT_MAIN_DATA_t curr_pid_pnt_main_data;
        PID_ROBOT_MONITOR2_t curr_pid_robot_monitor2;
        PID_PNT_IO_MONITOR_t curr_pid_pnt_io_monitor;
        PID_ROBOT_MONITOR_t curr_pid_robot_monitor;
        PID_GAIN_t curr_pid_gain;

        uint8_t serial_comm_rcv_buff[MAX_PACKET_SIZE];
        uint8_t serial_comm_snd_buff[MAX_PACKET_SIZE];

        string port_;
        ROBOT_PARAMETER_t robotParamData_;
        INIT_SETTING_STATE_t fgInitsetting_;

        double v_id1_;
        double v_id2_;

};

#endif