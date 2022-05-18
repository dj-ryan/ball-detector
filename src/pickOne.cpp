#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "balboa_core/balboaLL.h"
#include "ball_detector/ballLocation.h"
#include "balboa_core/balboaMotorSpeeds.h"
#include "landmark_self_sim/landmarkLocation.h"


#include <sstream>
#include <cmath>


int32_t targetLandmark = 1; // this is the landmark that we want to go to

// declare global vars
double angp = 0.05;
int dashDelay = 3;
ros::Time lastLandmarkSeen;

landmark_self_sim::landmarkLocation landmark;
balboa_core::balboaMotorSpeeds motor;

// landmark location callback 
void callback(const landmark_self_sim::landmarkLocation &data) {
	landmark = data;	
}

// main loop
int main(int argc, char ** argv)
{
	// startup
	ROS_INFO("STARTED");
	ros::init(argc, argv, "pickOne");
	ros::NodeHandle node;

	// Publishers and Subscribers
	ros::Subscriber sub = node.subscribe("landmarkLocation", 1000, callback);
	ros::Publisher pubMotor = node.advertise<balboa_core::balboaMotorSpeeds>("motorSpeeds", 1000);
	
	ros::Rate loop_rate(10);

	// while loop
	while(ros::ok())
	{	
		//ROS_INFO("looped");
	
		ros::spinOnce();
	
		// if landmark message is the desired landmark		
		if(landmark.code == targetLandmark) {
			double error = landmark.xbottom - 300; // center error at middle of robot FOV (600 pixels)
			
			double pidValue = error * angp; // PID - P calculation
			
			// set motor speeds based on PID 
			motor.left = 60 + (int)pidValue; 
			motor.right = 60 - (int)pidValue;
			
			ROS_INFO("error: %f - left: %d | right: %d",pidValue,motor.left, motor.right);
			
			// publish motor speeds
			pubMotor.publish(motor);
			
			// update time stamp
			lastLandmarkSeen = ros::Time::now();
			
		}	
		
		// determine time since last landmark seen
		ros::Time check = ros::Time::now();
		ros::Duration timeDelta = check - lastLandmarkSeen;
		
		// if no landmark seen, stop robot motion
		if(timeDelta.toSec() > dashDelay) {
			ROS_INFO("cant find landmark");
			motor.left = 0;
			motor.right = 0;
			pubMotor.publish(motor); // publish motor speeds
		}
			
		ros::spinOnce(); 
		loop_rate.sleep();

	}
return 0;
}
