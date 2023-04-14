#ifndef __fastech_step_wrapper__
#define __fastech_step_wrapper__

#include "ros/ros.h"

#include "fastech_ros_sdk/fastech/util/Definition.h"

#include "fastech_ros_sdk/ServoOn.h"
#include "sensor_msgs/JointState.h"

class FastechStepWrapper{
    public:
        FastechStepWrapper(const std::string & motor_ip, const int & port);
        ~FastechStepWrapper(){};

        // motor control;
        int Connect();
        int Send();
        int Unpack(unsigned char* recv_data,unsigned char& comm_status, unsigned char* unpacked_data);
        int Recieve();
        void Disconnect();

        int Reset();
        int SetParameter(unsigned int paramno ,unsigned int paramval);
        int GetParameter(unsigned int paramno, unsigned int paramval );
        int SaveAllParam();
        int GetActualVel();
        int GetMotionStatus();
        int GetAlarmLogs();
        int ResetAlarmLogs();
        int ServoEnable(bool enable);
        int SetVelocity(unsigned int speed, bool direction);
        int MotorStop();    
        int EmergencyStop();
        int SetVelocityOveride(unsigned int speed );
        // int SetVelocityEx(unsigned int speed, bool direction, bool flagoption, unsigned short accdectime);
      
        // helper function;
        void UInt2UBytes(unsigned int num, unsigned char* bytes);
        unsigned int UBytes2UInt(unsigned char* bytes);
        void UShort2UBytes(unsigned short num, unsigned char* bytes);
        unsigned short UBytes2Ushort(unsigned char* bytes);
        void print_hex(const unsigned char* buf,int size);

        int get_velocity(){
            return motor_vel_;
        }
        

    private:
        std::string motor_ip_;
        unsigned int syncNo_ = 1;       

        // UDP sock defined variables
        int sock_, port_;
        unsigned char send_msg_[BUF_SIZE];
        unsigned char recieve_msg_[BUF_SIZE+5];

        int send_msg_size_, recieve_msg_size_;
        int str_len_;
        socklen_t adr_sz_;
        struct sockaddr_in serv_adr_, from_adr_;

        // Recieve Data
        int motor_vel_;

};



#endif