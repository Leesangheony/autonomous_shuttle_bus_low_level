#include "ros/ros.h"
#include <can_msgs/Frame.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_bridge/topic_to_socketcan.h>
#include <std_msgs/Int32.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

char d2h_mg(int a, int b);

std_msgs::Int32 steer_cmd;
void steer_cb(const std_msgs::Int32::ConstPtr& msg){
    steer_cmd = *msg;
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "steering_driver_node");
	ros::NodeHandle nh;
	
    ros::Subscriber steer_sub = nh.subscribe<std_msgs::Int32>
            ("cmd/steer", 10, steer_cb);
	ros::Publisher publisher_ = nh.advertise<can_msgs::Frame>("sent_messages", 10);
	
	ros::Rate loop_rate(10);

    bool initial = true;
    int prev_data = 0;
    
    steer_cmd.data = 0;
	
	int count = 0;
    int ticc = 0;
	while(ros::ok())
	{
        ros::spinOnce();
        can_msgs::Frame msg;
        if (initial) {
            msg.is_extended = true;
            msg.is_rtr = false;
            msg.is_error = false;
            msg.id = 0x06000002|CAN_EFF_FLAG;
            msg.dlc = 8;
            msg.data[0] = 0x23; //35
            msg.data[1] = 0x0D; //13
            msg.data[2] = 0x20; //32
            msg.data[3] = 0x01; //1
            msg.data[4] = 0x00; //0
            msg.data[5] = 0x00; //0
            msg.data[6] = 0x00; //0
            msg.data[7] = 0x00; //0

            msg.header.frame_id = "0";  // "0" for no frame.
            msg.header.stamp = ros::Time::now();
            if(ticc>11){
                initial = false;
            }
            
            printf("initialized\n");
            publisher_.publish(msg);
            
        } else if ((prev_data!=steer_cmd.data)||(ticc==12)) {
            int inputcmd = (int)(steer_cmd.data*10000/360);
            msg.is_extended = true;
            msg.is_rtr = false;
            msg.is_error = false;
            msg.id = 0x06000002|CAN_EFF_FLAG;
            msg.dlc = 8;
            msg.data[0] = 0x23; //35
            msg.data[1] = 0x02; //2
            msg.data[2] = 0x20; //32
            msg.data[3] = 0x01; //1
            msg.data[4] = d2h_mg(inputcmd,1); //4
            msg.data[5] = d2h_mg(inputcmd,0);; //8
            msg.data[6] = d2h_mg(inputcmd,3);; //0
            msg.data[7] = d2h_mg(inputcmd,2);; //0
            printf("input: %d | output: %X, %X, %X, %X\n",steer_cmd.data, msg.data[4], msg.data[5], msg.data[6], msg.data[7]);

            msg.header.frame_id = "0";  // "0" for no frame.
            msg.header.stamp = ros::Time::now();
            prev_data = steer_cmd.data;
            publisher_.publish(msg);
        } 
        ticc++;
        //printf("ticc : %d\n", ticc);
        
        

        // send the can_frame::Frame message to the sent_messages topic.
        //publisher_.publish(msg);
        
        
        loop_rate.sleep();
        count++;
 

    }
    return 0;
}

char d2h_mg(int a, int b) {
    int mask = 0b11111111;
    unsigned char result;
    
    short a_tmp = a>>(8*b);
    result = a_tmp&mask;
    
    return result;
}