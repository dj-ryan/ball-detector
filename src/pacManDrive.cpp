#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "balboa_core/balboaLL.h"
#include "ball_detector/ballLocation.h"


#include <sstream>
#include <cmath>

balboa_core::balboaLL robot;
ball_detector::ballLocation ball;


void callbackIR(const balboa_core::balboaLL &data)
{
	robot = data;
	if(robot.rangeSensor > 100){	//if wall is within buffer distance -- range value ^ when object is closer
		pubMotor.publish(/*turn right 115 degrees*/);
	}
}

void callbackBall(const ball_detector::ballLocation &data)
{
	ball = data;
}

int main(int argc, char ** argv)
{
	ROS_INFO("STARTEDs");
	ros::init(argc, argv, "pacManDrive");
	ros::NodeHandle node;

	ros::Subscriber subIR = node.subscribe("balboaLL", 1000, callbackIR);
	ros::Subscriber subBall = node.subscribe("ballLocation", 1000, callbackBall);
	ros::Publisher pubMotor = node.advertise<std_msgs::Int32>("motorSpeeds", 1000);

	ros:: Rate loop_rate(10);

	while(ros::ok())
	{
//		pubMotor.publish(5);
		ROS_INFO("...");
		ROS_INFO("%d",robot.distanceRight);
		ROS_INFO("%f", ball.x);
		ros::spinOnce();
		loop_rate.sleep();
	}
return 0;
}