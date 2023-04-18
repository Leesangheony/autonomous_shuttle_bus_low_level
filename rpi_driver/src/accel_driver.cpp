#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
#include <vector>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "Adafruit_MCP4725.h"
#include <wiringSerial.h>

// check limit
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define BAUD 115200

using namespace std;

// accel command
// accelCmd[0] -> Pi Enable Command (0 or nonzero value)
// accelCmd[1] -> Desired SIG1 Voltage 
// accelCmd[2] -> Desired SIG2 Voltage 
float accelCmd[3] = {0, 0, 0};
float pre_accelCmd[3] = {0, 0, 0};
// file descriptor
int fd;
	
void accelCmdCallback(const std_msgs::Float32MultiArray::ConstPtr& accelCmd);
void cmd2ascii(float sig1, float sig2);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "accel_driver_node");
	ros::NodeHandle nh;
	ros::Subscriber accelSub = nh.subscribe("cmd/accel", 100, accelCmdCallback);
	
	ros::Rate loop_rate(100);
	
	
	if((fd = serialOpen("/dev/ttyUSB0", BAUD)) < 0)
	{
		ROS_INFO("Port Open Failed");
		return 1;
	}
	
	// DAC setVolate function input value range is 0~4095(12bit)
	float dac_input[2] = {0, 0};
	
	while(ros::ok())
	{
		ros::spinOnce();
		
		// Exception Handling
		dac_input[0] = constrain(accelCmd[1], 0.54199, 3.06152); 	// SIG1
		dac_input[1] = constrain(accelCmd[2], 0.26855, 1.4209);		// SIG2	
		
		// Case1 : Pi Enabled -> Use RPi's accel Command
		if (!accelCmd[0]){
			// ROS_INFO("Pi Enabled!\n");
			
			// Execute command when command is changed.
			if (pre_accelCmd[1] != accelCmd[1] || pre_accelCmd[2] != accelCmd[2]){
				ROS_INFO("dac_input[0] = %1.5f", dac_input[0]);
				ROS_INFO("dac_input[1] = %1.5f\n", dac_input[1]);
				cmd2ascii(dac_input[0], dac_input[1]);
			}
				/*	
				if(serialDataAvail(fd))
				{
					unsigned char rec = serialGetchar(fd);
					cout << rec << endl;
				}*/
		}
		// Case2 : Pi Disabled -> Use Accel pedal Command
		else{
			ROS_INFO("Pi Disabled!\n");
		}	
			
		// save previous command
		pre_accelCmd[0] = accelCmd[0];
		pre_accelCmd[1] = accelCmd[1];
		pre_accelCmd[2] = accelCmd[2];
		
		loop_rate.sleep();	
	}
	
    serialClose(fd);
	return 0;
}


// store subscribed array to accelCmd array
void accelCmdCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	int i = 0;
	for(vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		accelCmd[i] = *it;
		i++;
	}
}

void cmd2ascii(float sig1, float sig2)
{
	// SIG 1
	// #01C0+0 is fixed
	
	serialPutchar(fd, 35); // #
	serialPutchar(fd, 48); // 0
	serialPutchar(fd, 49); // 1
	serialPutchar(fd, 67); // C
	serialPutchar(fd, 48); // 0
	serialPutchar(fd, 43); // +
	serialPutchar(fd, 48); // 0
	
	// e.g. cmd = 1.234
	int num1 = sig1 * 1000;							// 1234
	serialPutchar(fd, (num1 / 1000 + 48));			// 1
	serialPutchar(fd, 46);							// .
	serialPutchar(fd, ((num1 % 1000) / 100 + 48));	// 2
	serialPutchar(fd, ((num1 % 100) / 10 + 48));	// 3
	serialPutchar(fd, ((num1 % 10) + 48));			// 4
	serialPutchar(fd, 13);	// carriage return
	
	// sleep for next command
	usleep(5000);

	// SIG 2
	// #01C1+0 is fixed
	serialPutchar(fd, 35); // #
	serialPutchar(fd, 48); // 0
	serialPutchar(fd, 49); // 1
	serialPutchar(fd, 67); // C
	serialPutchar(fd, 49); // 1
	serialPutchar(fd, 43); // +
	serialPutchar(fd, 48); // 0
	
	int num2 = sig2 * 1000;
	serialPutchar(fd, (num2 / 1000 + 48));			
	serialPutchar(fd, 46);							
	serialPutchar(fd, ((num2 % 1000) / 100 + 48));	
	serialPutchar(fd, ((num2 % 100) / 10 + 48));		
	serialPutchar(fd, ((num2 % 10) + 48));			
	serialPutchar(fd, 13);	// carriage return
}

