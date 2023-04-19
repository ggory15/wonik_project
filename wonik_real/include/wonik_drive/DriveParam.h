#ifndef DRIVEPARAM_INCLUDEDEF_H
#define DRIVEPARAM_INCLUDEDEF_H

#include "StrUtil.h"

const double PI = 3.14159265358979323846;

/**
 * Parameters and conversion functionality of a motor drive.
 */
class DriveParam
{
public:
	DriveParam()
	{
		ip_address = "192.168.0.2";
		port = 3006;
		gear_ratio = 8;
		servo_on = false;
		current_vel = 0.0;
	}

	std::string ip_address;
	int port;
	int gear_ratio;
    std::string sName;
	bool servo_on;
	double current_vel;



};

#endif
