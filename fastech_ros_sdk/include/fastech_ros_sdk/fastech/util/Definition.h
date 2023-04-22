#ifndef __fastech_defition__
#define __fastech_defition__

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define IPADDRESS "192.168.0.2"
#define UDP_PORT 3002
#define BUF_SIZE 1024*4
#define FMM_OK					0
#define FMM_NOT_OPEN			1
#define FMM_INVALID_PORT_NUM	2
#define FMM_INVALID_SLAVE_NUM	3
#define FMM_POSTABLE_ERROR		9
#define FMC_DISCONNECTED		5
#define FMC_TIMEOUT_ERROR		6
#define FMC_RECVPACKET_ERROR	8

#define FMP_FRAMETYPEERROR		128
#define FMP_DATAERROR		    129
#define FMP_RUNFAIL	            133
#define FMP_RESETFAIL		    134
#define FMP_SERVOONFAIL1		135
#define FMP_SERVOONFAIL2    	136
#define FMP_SERVOONFAIL3		137
#define FMP_PACKETERROR     	130

#define FMM_CONNECT_ERROR 	300
#define FMM_PACKET_BUILD_ERROR  301
#define FMM_UDP_SEND_ERROR      302
#define FMM_UDP_RECV_ERROR      303
#define FMM_PACKET_UNPACK_ERROR 304

#define RPM2RADSEC(x) x*0.10471975511965977
#define RADSEC2RPM(x) x*9.54929658551

#define STEPANGLE 0.036 //deg -> 8 : 10,000 pulse per rev
#define PPS2RPM(x) x*STEPANGLE*60.0/360.0
#define RPM2PPS(x) x*360.0/(60.0*STEPANGLE)

#define PPS2RADSEC(x) x*STEPANGLE*60.0/360.0*0.10471975511965977
#define RADSEC2PPS(x) x*360.0/(60.0*STEPANGLE)*9.54929658551

#endif