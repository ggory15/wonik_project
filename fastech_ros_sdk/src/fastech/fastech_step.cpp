#include "fastech_ros_sdk/fastech/fastech_step.hpp"

using namespace std;
FastechStepWrapper::FastechStepWrapper(const std::string & motor_ip, const int & port){
    motor_ip_ = motor_ip;
    port_ = port;
}

int FastechStepWrapper::Connect(){
    sock_ = socket(PF_INET, SOCK_DGRAM, 0);
   if(sock_==-1){
     return FMM_CONNECT_ERROR;
   }
   memset(&serv_adr_, 0, sizeof(serv_adr_));
   serv_adr_.sin_family=AF_INET;
   serv_adr_.sin_addr.s_addr=inet_addr(motor_ip_.data());
   serv_adr_.sin_port=htons(port_);
   return FMM_OK;
}
int FastechStepWrapper::Send(){
    try{
        sendto(sock_, send_msg_, send_msg_size_, 0, (struct sockaddr*)&serv_adr_,sizeof(serv_adr_));
        syncNo_++;
        return FMM_OK;
    }
    catch(std::string err)
    {
        std::cout << err << std::endl;
        return FMM_UDP_SEND_ERROR;
    }
}
int FastechStepWrapper::Unpack(unsigned char* recv_data,unsigned char& comm_status, unsigned char* unpacked_data){
    try{
       int size = sizeof(recv_data);
       if(sizeof(recv_data) > 5)
       {
            comm_status = recv_data[5];
            printf("comm status : 0x%02x ",comm_status);
            printf("recv data : ");
            for(size_t i=6;i<sizeof(recv_data);i++)
            {
                unpacked_data[i] = recv_data[i];
                printf("0x%02x ",unpacked_data[i]);
            }
            printf("\n");
       }
       
       return FMM_OK;
   }
   catch(std::string err)
   {
        std::cout << err << std::endl;
        return FMM_PACKET_UNPACK_ERROR;
   }
}
void FastechStepWrapper::EmptyRead(){

	try{
		do{
			adr_sz_ = sizeof(from_adr_);
			str_len_= recvfrom(sock_, recieve_msg_, sizeof(recieve_msg_), MSG_DONTWAIT, (struct sockaddr*)&from_adr_, &adr_sz_);
			// std::cout <<"str_len_ : " << str_len_ << std::endl;
		} while(str_len_ > 0);
	}
	catch(std::string err)
	{
		std::cout << err << std::endl;
	}
}
int FastechStepWrapper::Recieve(){
    try{
		adr_sz_ = sizeof(from_adr_);
        str_len_ = recvfrom(sock_, recieve_msg_, sizeof(recieve_msg_), 0, (struct sockaddr*)&from_adr_, &adr_sz_);
        if(str_len_!=-1){
			unsigned char* res = reinterpret_cast<unsigned char*>(recieve_msg_);
			if (send_msg_[7] == 0xFF && send_msg_[8] == 0xFF && recieve_msg_[4] == 0x55 ){
				unsigned char tmp_buf[4] = {res[sizeof(res)-2], res[sizeof(res)-1], 0xFF, 0xFF};
				motor_vel_ = (int)UBytes2UInt(tmp_buf);
			}
			else if (recieve_msg_[4]=0x55){
				unsigned char tmp_buf[4] = {res[sizeof(res)-2], res[sizeof(res)-1], 0x00, 0x00};
				motor_vel_ = (unsigned int)UBytes2UInt(tmp_buf);
			}
		}
        return FMM_OK;
   }
   catch(std::string err)
   {
        std::cout << err << std::endl;
        return FMM_UDP_RECV_ERROR;
   }
}
void FastechStepWrapper::Disconnect(){
    close(sock_);
}

int FastechStepWrapper::Reset(){
    try{
		unsigned char AlarmReset[] = { 0xAA, 0x03, 0x00, 0x00, 0x2B };
		AlarmReset[2] = (unsigned char)syncNo_;
		for (size_t idx = 0; idx < sizeof(AlarmReset); idx++)
			send_msg_[idx] = AlarmReset[idx];
			send_msg_size_ = sizeof(AlarmReset);

		this->Send();
		return FMM_OK;
	}
	catch (std::string err)
	{
		std::cout << err << std::endl;
		return FMM_PACKET_BUILD_ERROR;
	}
}
int FastechStepWrapper::SetParameter(unsigned int paramno ,unsigned int paramval){
    try{
		unsigned char SetParameter[] = { 0xAA, 0x08, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, };
		SetParameter[2] = (unsigned char)syncNo_;
		
		if (paramno > 50) return FMM_PACKET_BUILD_ERROR;
		SetParameter[5] = (unsigned char)paramno;

		unsigned char pv[4];
		UInt2UBytes(paramval, pv);
		for (size_t idx = 0; idx < sizeof(pv); idx++)
			SetParameter[idx + 6] = pv[idx];

		for (size_t idx = 0; idx < sizeof(SetParameter); idx++)
			send_msg_[idx] = SetParameter[idx];

		send_msg_size_ = sizeof(SetParameter);
		this->Send();
		return FMM_OK;
	}
	catch (std::string err)
	{
		std::cout << err << std::endl;
		return FMM_PACKET_BUILD_ERROR;
	}
}
int FastechStepWrapper::GetParameter(unsigned int paramno, unsigned int paramval){
    try{
		unsigned char GetParameter[] = { 0xAA, 0x04, 0x00, 0x00, 0x13, 0x00 };
		GetParameter[2] = (unsigned char)syncNo_;

		if (paramno > 50) return FMM_PACKET_BUILD_ERROR;
		GetParameter[5] = (unsigned char)paramno;

		for (size_t idx = 0; idx < sizeof(GetParameter); idx++)
			send_msg_[idx] = GetParameter[idx];
		send_msg_size_ = sizeof(GetParameter);
		this->Send();
		return FMM_OK;
	}
	catch (std::string err)
	{
		std::cout << err << std::endl;
		return FMM_PACKET_BUILD_ERROR;
	}
}
int FastechStepWrapper::SaveAllParam(){
    try{
		unsigned char SaveAllParam[] = { 0xAA, 0x03, 0x00, 0x00, 0x10 };
		SaveAllParam[2] = (unsigned char)syncNo_;

		for (size_t idx = 0; idx < sizeof(SaveAllParam); idx++)
			send_msg_[idx] = SaveAllParam[idx];
		send_msg_size_ = sizeof(SaveAllParam);

		this->Send();
		return FMM_OK;
	}
	catch (std::string err)
	{
		std::cout << err << std::endl;
		return FMM_PACKET_BUILD_ERROR;
	}
}

int FastechStepWrapper::GetActualVel(){
    try{
		unsigned char GetActualVel[] = { 0xAA, 0x03, 0x00, 0x00, 0x55 };
		GetActualVel[2] = (unsigned char)syncNo_;
		for (size_t idx = 0; idx < sizeof(GetActualVel); idx++)
			send_msg_[idx] = GetActualVel[idx];
		send_msg_size_ = sizeof(GetActualVel);
		this->Send();
		return FMM_OK;
	}
	catch (std::string err)
	{
		std::cout << err << std::endl;
		return FMM_PACKET_BUILD_ERROR;
	}
}
int FastechStepWrapper::GetMotionStatus(){
    try{
		unsigned char GetMotionStatus[] = { 0xAA, 0x03, 0x00, 0x00, 0x42 };
		GetMotionStatus[2] = (unsigned char)syncNo_;
		for (size_t idx = 0; idx < sizeof(GetMotionStatus); idx++)
			send_msg_[idx] = GetMotionStatus[idx];
		send_msg_size_ = sizeof(GetMotionStatus);
		this->Send();
		return FMM_OK;
	}
	catch (std::string err)
	{
		std::cout << err << std::endl;
		return FMM_PACKET_BUILD_ERROR;
	}
}
int FastechStepWrapper::GetAlarmLogs(){
    try{
		unsigned char GetAlarmLogs[] = { 0xAA, 0x03, 0x00, 0x00, 0x9C };
		GetAlarmLogs[2] = (unsigned char)syncNo_;
		for (size_t idx = 0; idx < sizeof(GetAlarmLogs); idx++)
			send_msg_[idx] = GetAlarmLogs[idx];
		send_msg_size_ = sizeof(GetAlarmLogs);	
		this->Send();

		return FMM_OK;
	}
	catch (std::string err)
	{
		std::cout << err << std::endl;
		return FMM_PACKET_BUILD_ERROR;
	}
}
int FastechStepWrapper::ResetAlarmLogs(){
    try{
		unsigned char ResetAlarmLogs[] = { 0xAA, 0x03, 0x00, 0x00, 0x9D };
		ResetAlarmLogs[2] = (unsigned char)syncNo_;
		for (size_t idx = 0; idx < sizeof(ResetAlarmLogs); idx++)
			send_msg_[idx] = ResetAlarmLogs[idx];
		send_msg_size_ = sizeof(ResetAlarmLogs);
		this->Send();

		return FMM_OK;
	}
	catch (std::string err)
	{
		std::cout << err << std::endl;
		return FMM_PACKET_BUILD_ERROR;
	}
}
int FastechStepWrapper::ServoEnable(bool enable){
    try{
		unsigned char ServoEnable[] = { 0xAA, 0x04, 0x00, 0x00, 0x2A, 0x00 };
		ServoEnable[2] = (unsigned char)syncNo_;
		if (enable) ServoEnable[5] = 0x01;
		else ServoEnable[5] = 0x00;
		for (size_t idx = 0; idx < sizeof(ServoEnable); idx++)
			send_msg_[idx] = ServoEnable[idx];
		send_msg_size_ = sizeof(ServoEnable);
		this->Send();

		return FMM_OK;
	}
	catch (std::string err)
	{
		std::cout << err << std::endl;
		return FMM_PACKET_BUILD_ERROR;
	}
}
int FastechStepWrapper::SetVelocity(unsigned int speed, bool direction){
    try{
		unsigned char MoveVelocity[] = { 0xAA, 0x08, 0x00, 0x00, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00 };
		MoveVelocity[2] = (unsigned char)syncNo_;
		unsigned char spd[4];
		UInt2UBytes(speed, spd);
		for (size_t idx = 0; idx < sizeof(spd); idx++)
			MoveVelocity[idx + 5] = spd[idx];
		if (direction) MoveVelocity[9] = 0x01;
		else MoveVelocity[9] = 0x00;
		for (size_t idx = 0; idx < sizeof(MoveVelocity); idx++)
			send_msg_[idx] = MoveVelocity[idx];
		send_msg_size_ = sizeof(MoveVelocity);
		this->Send();

		return FMM_OK;
	}
	catch (std::string err)
	{
		std::cout << err << std::endl;
		return FMM_PACKET_BUILD_ERROR;
	}
}
int FastechStepWrapper::MotorStop(){
    try{
		unsigned char MoveStop[] = { 0xAA, 0x03, 0x00, 0x00, 0x31 };
		MoveStop[2] = (unsigned char)syncNo_;
		for (size_t idx = 0; idx < sizeof(MoveStop); idx++)
			send_msg_[idx] = MoveStop[idx];
		send_msg_size_ = sizeof(MoveStop);

		this->Send();
		return FMM_OK;
	}
	catch (std::string err)
	{
		std::cout << err << std::endl;
		return FMM_PACKET_BUILD_ERROR;
	}
}
int FastechStepWrapper::EmergencyStop(){
    try{
		unsigned char EmergencyStop[] = { 0xAA, 0x03, 0x00, 0x00, 0x32 };
		EmergencyStop[2] = (unsigned char)syncNo_;
		for (size_t idx = 0; idx < sizeof(EmergencyStop); idx++)
			send_msg_[idx] = EmergencyStop[idx];
		send_msg_size_ = sizeof(EmergencyStop);
		this->Send();

		return FMM_OK;
	}
	catch (std::string err)
	{
		std::cout << err << std::endl;
		return FMM_PACKET_BUILD_ERROR;
	}
}
int FastechStepWrapper::SetVelocityOveride(unsigned int speed){
    try{
		unsigned char VelocityOverride[] = { 0xAA, 0x07, 0x00, 0x00, 0x3A, 0x00, 0x00, 0x00, 0x00 };
		VelocityOverride[2] = (unsigned char)syncNo_;
		unsigned char spd[4];
		UInt2UBytes(speed, spd);		
		for (size_t idx = 0; idx < sizeof(spd); idx++)
			VelocityOverride[idx + 5] = spd[idx];
		for (size_t idx = 0; idx < sizeof(VelocityOverride); idx++)
			send_msg_[idx] = VelocityOverride[idx];
		send_msg_size_ = sizeof(VelocityOverride);
		this->Send();

		return FMM_OK;
	}
	catch (std::string err)
	{
		std::cout << err << std::endl;
		return FMM_PACKET_BUILD_ERROR;
	}
}


// helper functions
void FastechStepWrapper::UInt2UBytes(unsigned int num, unsigned char* bytes)
{
	bytes[0] = num & 0xFF;
	bytes[1] = (num >> 8) & 0xFF;
	bytes[2] = (num >> 16) & 0xFF;
	bytes[3] = (num >> 24) & 0xFF;
}

unsigned int FastechStepWrapper::UBytes2UInt(unsigned char* bytes)
{
	return (unsigned int)bytes[0] | (unsigned int)bytes[1] << 8 | (unsigned int)bytes[2] << 16 | (unsigned int)bytes[3] << 24;
}

void FastechStepWrapper::UShort2UBytes(unsigned short num, unsigned char* bytes)
{
	bytes[0] = num & 0xFF;
	bytes[1] = (num >> 8) & 0xFF;
}

unsigned short FastechStepWrapper::UBytes2Ushort(unsigned char* bytes)
{
	return (unsigned short)bytes[0] | (unsigned short)bytes[1] << 8;
}

void FastechStepWrapper::print_hex(const unsigned char* buf,int size)
{
    for(size_t idx=0; idx<size;idx++)
        printf("0x%02X ",buf[idx]);
    printf("\n");


	// // char tmp_char[4];
	// printf("%ld\n", strtol(strcat( &buf[size-1], &buf[size-2]), 0, 16));
	
}



