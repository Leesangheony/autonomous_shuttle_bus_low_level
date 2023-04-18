/*
 ============================================================================
 Name        : ads1115_example.c
 Author      : Giovanni Bauermeister
 Description : Read analog values from potentiometer using ADS1115 and prints to terminal
 ============================================================================
 */

#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "ads1115_rpi.h"

using namespace std;

int main(int argc, char **argv) {

    // Initialize
    ros::init(argc,argv, "brake_enc_node");
    ros::NodeHandle nh;

    // Pub
	ros::Publisher encPub = nh.advertise<std_msgs::Int32>("brake_enc", 10);
    
    // Rate
    ros::Rate loop_rate(20);

    std_msgs::Int32 brake_enc;

    float encVal;
    int encVal_int;
    float encVal_H = 1.4;
    float encVal_L = 0.3;

	if(openI2CBus("/dev/i2c-0") == -1)
	{
		return EXIT_FAILURE;
	}
	setI2CSlave(0x48);
	while(ros::ok())
	{
        encVal =  readVoltage(0);
        encVal_int = 100/(encVal_H-encVal_L)*(encVal-encVal_L);
        brake_enc.data = encVal_int;

        // publish
        encPub.publish(brake_enc);


		printf("CH_0 = %.2f V | \n", encVal);

        ros::spinOnce();
        loop_rate.sleep();
	}

	return EXIT_SUCCESS;
}