#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <phidget21.h>
#include <std_msgs/String.h>
#include <argyris/servo_reference.h>
#include <argyris/servo_params.h>

CPhidgetAdvancedServoHandle servo;






int main(int argc, char* argv[])
{
	ros::init(argc, argv, "phservo");
	ros::NodeHandle n;
	ros:: NodeHandle nh("~");
	int serial_number = -1;
	nh.getParam("serial", serial_number);
	std::string name = "servos";


}
