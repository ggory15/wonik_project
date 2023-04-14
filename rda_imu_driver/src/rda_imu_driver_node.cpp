#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/fcntl.h>

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#define SERIAL_PORT		"/dev/imu"
#define SERIAL_SPEED		B115200
#define COMM_RECV_TIMEOUT	30	

#define TRACE	printf

std::string port;
int32_t baud;
int serial_fd = -1;

int serial_open ()
{
	TRACE ("Try to open serial: %s\n", SERIAL_PORT); 

	serial_fd = open(SERIAL_PORT, O_RDWR|O_NOCTTY); // |O_NDELAY);
	if (serial_fd < 0) {
		printf ("Error unable to open %s\n", SERIAL_PORT);
		return -1;
	}
  	TRACE ("%s open success\n", SERIAL_PORT);

  	struct termios tio;
  	tcgetattr(serial_fd, &tio);
  	cfmakeraw(&tio);
	tio.c_cflag = CS8|CLOCAL|CREAD;
  	tio.c_iflag &= ~(IXON | IXOFF);
  	cfsetspeed(&tio, SERIAL_SPEED);
  	tio.c_cc[VTIME] = 0;
  	tio.c_cc[VMIN] = 0;

  	int err = tcsetattr(serial_fd, TCSAFLUSH, &tio);
  	if (err != 0) {
    	TRACE ("Error tcsetattr() function return error\n");
    	close(serial_fd);
		serial_fd = -1;
    	return -1;
  	}
	return 0;
}

static unsigned long GetTickCount() 
{
    struct timespec ts;
   
    clock_gettime (CLOCK_MONOTONIC, &ts);

    return ts.tv_sec*1000 + ts.tv_nsec/1000000;
}

int SendRecv(const char* command, double* returned_data, int data_length)
{
	char temp_buff[256];
	read (serial_fd, temp_buff, 256);

	int command_len = strlen(command);
	int n = write(serial_fd, command, command_len);
	if (n < 0) return -1;

	const int buff_size = 1024;
	int  recv_len = 0;
	char recv_buff[buff_size + 1];	

	unsigned long time_start = GetTickCount();

	while (recv_len < buff_size) {
		int n = read (serial_fd, recv_buff + recv_len, buff_size - recv_len);
		if (n < 0) {
			return -1;
		}
		else if (n == 0) {
			usleep(1000);
		}
		else if (n > 0) {
			recv_len += n;

			if (recv_buff[recv_len - 1] == '\r' || recv_buff[recv_len - 1] == '\n') {
				break;
			}
		}

		unsigned long time_current = GetTickCount();
		unsigned long time_delta = time_current - time_start;

		if (time_delta >= COMM_RECV_TIMEOUT) break;
	}
	recv_buff[recv_len] = '\0';

	if (recv_len > 0) {
		if (recv_buff[0] == '!') {
			return -1;
		}
	}

	
	if (strncmp(command, recv_buff, command_len - 1) == 0) {
		if (recv_buff[command_len - 1] == '=') {
			int data_count = 0;

			char* p = &recv_buff[command_len];
			char* pp = NULL;

			for (int i = 0; i < data_length; i++) {
				if (p[0] == '0' && p[1] == 'x') {	
					returned_data[i] = strtol(p+2, &pp, 16);
					data_count++;
				}
				else {
					returned_data[i] = strtod(p, &pp);
					data_count++;
				}

				if (*pp == ',') {
					p = pp + 1;
				}
				else {
					break;
				}
			}
			return data_count;
		}
	}
	return 0;
}

void publishImuMessage(ros::Publisher& imu_pub, double roll,double pitch, double yaw, double x_acc, double y_acc, double z_acc)
{
  double linear_acceleration_stddev = 0;
  double angular_velocity_stddev = 0;
  double orientation_stddev = 0;
 
  // calculate measurement time
  ros::Time measurement_time = ros::Time::now(); // + ros::Duration(time_offset_in_seconds);

  sensor_msgs::Imu imu;
  imu.header.stamp = measurement_time;
  imu.header.frame_id = "base_link";

  /* covariance matrices */
  imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

  imu.angular_velocity_covariance[0] = angular_velocity_stddev;
  imu.angular_velocity_covariance[4] = angular_velocity_stddev;
  imu.angular_velocity_covariance[8] = angular_velocity_stddev;

  imu.orientation_covariance[0] = orientation_stddev;
  imu.orientation_covariance[4] = orientation_stddev;
  imu.orientation_covariance[8] = orientation_stddev;
  /* acceleration */
  imu.linear_acceleration.x = x_acc;
  imu.linear_acceleration.y = y_acc;
  imu.linear_acceleration.z = z_acc;

  /* Orientation */
  tf::Quaternion orientation;
  orientation.setRPY(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180);

  // Convert Quaternion to Quaternion msg
  tf::quaternionTFToMsg(orientation, imu.orientation);

  /* angular velocity */
  // gyro velocity is set to 0. This can be calculated by comparing the diff in rotation over time. 
  int16_t gx = 0;
  int16_t gy = 0;
  int16_t gz = 0;

  // convert to rad/sec (like acceleration data)
  double gxf = gx * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
  double gyf = gy * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
  double gzf = gz * (4000.0/65536.0) * (M_PI/180.0) * 25.0;

  imu.angular_velocity.x = gxf;
  imu.angular_velocity.y = gyf;
  imu.angular_velocity.z = gzf;

  imu_pub.publish(imu);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rda_imu_driver");

  //ros::NodeHandle node;
  ros::Time::init();

  /* Node */
  ros::NodeHandle node("imu");

  /* publisher */
  ros::Publisher imu_pub = node.advertise<sensor_msgs::Imu>("data", 1);

  port = "/dev/imu";
  baud = 115200;

  const int max_data = 3;
  double euler_data[max_data];
  double accel_data[max_data];

  ros::NodeHandle nh("~");
  nh.param<std::string>("port", port, port);
  
  serial_open();

  if (serial_fd >= 0) {
	  ROS_INFO("Connection Succesful");
  }
  else{
	  ROS_INFO("Problem connecting to serial device (number of attempts is 100)");
          return 1;
  }
  
  ros::Rate r(100); // 500 hz
  while (ros::ok())
  {
    
     int euler_no_data = SendRecv("e\n", euler_data, max_data);  // Read Euler angle
     int accel_no_data = SendRecv("a\n", accel_data, max_data);              // Read Accel

     if (euler_no_data >= 3 && accel_no_data >=3){
	//printf("euler = %f, %f, %f\n",euler_data[0], euler_data[1], euler_data[2]);
        //printf("Acc = %f, %f, %f\n",  accel_data[0], accel_data[1], accel_data[2]);
         publishImuMessage(imu_pub, euler_data[0],euler_data[1],euler_data[2],accel_data[0],accel_data[1],accel_data[2]);
    }
    ros::spinOnce();
    r.sleep();
  }
}

