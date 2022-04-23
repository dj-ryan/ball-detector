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


double angp = 0.05;
int dashDelay = 3;
ros::Time lastLandmarkSeen;


landmark_self_sim::landmarkLocation landmark;
balboa_core::balboaMotorSpeeds motor;

void callback(const landmark_self_sim::landmarkLocation &data) {
	landmark = data;	
}

int main(int argc, char ** argv)
{
	ROS_INFO("STARTED");
	ros::init(argc, argv, "pickOne");
	ros::NodeHandle node;

	ros::Subscriber sub = node.subscribe("landmarkLocation", 1000, callback);
	ros::Publisher pubMotor = node.advertise<balboa_core::balboaMotorSpeeds>("motorSpeeds", 1000);
	

	ros::Rate loop_rate(10);

	while(ros::ok())
	{	
		//ROS_INFO("looped");
	
		ros::spinOnce();
	
		// set motors to turn right
		//motor.left = 50;
		//motor.right = -50;
		//pubMotor.publish(motor);
		
		//while(landmark.code != targetLandmark){
			//// delay until found target
			//ros::spinOnce();
			//loop_rate.sleep();
			//pubMotor.publish(motor);
			//ROS_INFO("looking for target... | %d | %d",motor.left, motor.right);
		//}	
		//ROS_INFO("FOUND TARFGET!!!");
		
		//// drive straight
		//motor.left = 80;
		//motor.right = 80;
		//pubMotor.publish(motor);
		
	
		//while(true){ } //infinte loop
		

		
		if(landmark.code == targetLandmark) {
			double error = landmark.xbottom - 300;
			
			double pidValue = error * angp;
			
			motor.left = 60 + (int)pidValue;
			motor.right = 60 - (int)pidValue;
			
			ROS_INFO("error: %f - left: %d | right: %d",pidValue,motor.left, motor.right);
						
			pubMotor.publish(motor);
			
			lastLandmarkSeen = ros::Time::now();
			
			
		}	
		
		ros::Time check = ros::Time::now();
		ros::Duration timeDelta = check - lastLandmarkSeen;
		
		if(timeDelta.toSec() > dashDelay) {
			ROS_INFO("cant find landmark");
			motor.left = 0;
			motor.right = 0;
			pubMotor.publish(motor);
		}
			
		ros::spinOnce(); 
		loop_rate.sleep();

	}
return 0;
}
