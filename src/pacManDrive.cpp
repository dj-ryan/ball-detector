#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

#include "balboa_core/balboaLL.h"
#include "ball_detector/ballLocation.h"
#include "balboa_core/balboaMotorSpeeds.h"

#include <sstream>
#include <cmath>


// Robot states
#define emptyState 0 	// default
#define trackState 1 	// drive towards ball that has been found
#define lungeState 2 	// drive foward when close to ball
#define searchState 3 	// 360 degree scan to find balls
#define turnState 4 	// avoid wall by turnning if ir sensor trips
#define roamState 5 	// robot moves randomly
#define pauseState 6 	// set motors to zero


// Global data vars
balboa_core::balboaLL robot;
ball_detector::ballLocation ball;
balboa_core::balboaMotorSpeeds motor;

int robotState = searchState; 	// starting state 
ros::Time lastBallSeen; 		// last time the ball was seen
int32_t forgetBallDelay = 3; 	// time to forget a ball was identified (sec)
int32_t lungeDelay = 2; 		// time lunge lasts (sec)
int32_t lungeRadiusThresh = 180;// max radius before activate lunge mode
int32_t avgRadiusThresh = 50;	// average radius before activite track mode 
int32_t radiusAverage;			// ball radius average 
int32_t oldRadiusAve;			// old ball radius average


// Global PID vars
double oldAngleError = 0;
double oldDistanceError = 0;
double angleRunningAvg = 0;
double distanceRunningAvg = 0;


// PID values
double pAngle = 0.09;		// 0.09
double dAngle = 0.07;
double pDistance = 1;		// 0.6
double dDistance = 0.05;


// Target distance when in track state
double targetDistance = (double)lungeRadiusThresh; 

double alpha = 0.1; // length of running average (1/alpha) i.e. 1/0.1 = 10


// Update robot variables
void callbackLL(const balboa_core::balboaLL &data)
{
	// TODO: implement turn state update
	robot = data;
	
	if(robot.rangeSensor > 100){	//if wall is within buffer distance -- range value ^ when object is closer
	//	pubMotor.publish(/*turn right 115 degrees*/);
	//robotState = turnState;
	}
	
}


// Update ball detection variables
void callbackBall(const ball_detector::ballLocation &data)
{
	// Running average to filter out bad ball radius values / fake balls for past 100 balls
	radiusAverage = 0.99*(oldRadiusAve) + 0.01*(data.radius);
	
	// If ball is not close
	if(radiusAverage < lungeRadiusThresh) {
		
		// If real ball, enter track state
		if(radiusAverage > avgRadiusThresh){
			
			robotState = trackState;
			radiusAverage = 0;
		}		
	} 
	
	// If ball is really close, lunge for ball
	else {
		robotState = lungeState;
	}
	
	// Update variables 
	radiusAverage = oldRadiusAve;
	lastBallSeen = ros::Time::now();
}


// Roam State
void roamAround() {
	
		// TODO: implement roam funciton
	
	}


// Track State
void trackBallPID(){
	
	//Calculate error
	double angleError = ball.x;
    double distanceError = ball.radius - targetDistance;
    //ROS_INFO("angleError: %f", angleError);
    
    //PID EQUATIONS
    double anglePIDValue = (pAngle * angleError) + (dAngle * (angleError - oldAngleError));
    double distancePIDValue = (pDistance * distanceError) + (dDistance * (distanceError - oldDistanceError));

	//ROS_INFO("-------------------------");

	//ROS_INFO("anglePIDValue: %f", anglePIDValue);

	// running average calculations to filter out random balls
    angleRunningAvg = (angleRunningAvg * (1.0 - alpha)) + (anglePIDValue * alpha);
    distanceRunningAvg = (distanceRunningAvg * (1.0 - alpha)) + (distancePIDValue * alpha);

	//ROS_INFO("angleRunningAvg: %f", angleRunningAvg);


    //Save old error values for derivative pid equation
    oldAngleError = angleError;
    oldDistanceError = distanceError;

    //Assign motor speeds
    //motor.left = distancePIDValue + anglePIDValue
    //motor.right = distancePIDValue - anglePIDValue

    motor.left = distanceRunningAvg - angleRunningAvg;
    motor.right = distanceRunningAvg + angleRunningAvg;
		
	//ROS_INFO("-------------------------");
    //ROS_INFO("Radius: %d", ball.radius);
    //ROS_INFO("x: %d", ball.x);
    //ROS_INFO("left motor: %f | right motor: %f", motor.left, motor.right);
}


// Lunge State
void lungeFowardToBall(){
		ROS_INFO("lunged foward");
		// TODO: pub motors staright
		//pub motor straight
		motor.left = 100;
		motor.right = 100;
		robotState = searchState;
	}


// Turn State
void turnAwayFromWall(){
		// TODO: implement turn function
	
		// turn motors
		
		robotState = searchState;
	
	}


// Search State
void searchForBall(){
		// TODO: implement search function
		
		motor.right = -150;
		motor.left = 150;
		
		ROS_INFO("searching for ball");
		
		// possibly don't need to do any logic here
		//if(ball -> track){
		//	robotState = trackState;
		//}
	}
	
	
// Ball Timer 
void ballTimer(){
			
		ros::Time begin = ros::Time::now();
		ros::Duration ballDelta = begin - lastBallSeen;
		if(ballDelta.toSec() > forgetBallDelay){
			ROS_INFO("ball seen a while ago...");
			robotState = searchState; // this should eventually be roam state
		}
		
	}


// Main node loop
int main(int argc, char ** argv)
{
	// Node details
	ROS_INFO("STARTED");
	ros::init(argc, argv, "pacManDrive");
	ros::NodeHandle node;

	// Subscriptions
	ros::Subscriber subLL = node.subscribe("balboaLL", 1000, callbackLL);
	ros::Subscriber subBall = node.subscribe("ballLocation", 1000, callbackBall);
	
	// Publishers
	ros::Publisher pubMotor = node.advertise<balboa_core::balboaMotorSpeeds>("motorSpeeds", 1000);

	// Loop rate
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
      // pubMotor.publish(5);
	  // ROS_INFO("...");
	  // ROS_INFO("%d",robot.distanceRight);
	  // ROS_INFO("%f", ball.x);

		//ballTimer();
	
		// Robot State Machine	
		switch(robotState) {
			case searchState: // Search for balls
				ROS_INFO("=> search state");
				searchForBall();
				break;
				
			case trackState: // Track balls once seen 
				ROS_INFO("=> track state");
				trackBallPID();
				break;
				
			case lungeState: // Hit ball once in range
				ROS_INFO("=> lunge state");
				lungeFowardToBall();
				pubMotor.publish(motor);
				ros::Duration(lungeDelay).sleep(); // should not update msgs
				break;
				
			case turnState: // Turn to avoid wall
				ROS_INFO("=> turn state");
				turnAwayFromWall();
				break;
				
			case pauseState: // Turn off robot motors
				ROS_INFO("=> pause state");
				motor.right = 0;
				motor.left = 0;
				
			default:
				ROS_INFO("=> no state");
				// go to search state?
				break;
		}
		
		// Publish motors & print values 
		ROS_INFO("Publish: %d, %d",motor.left, motor.right);
		pubMotor.publish(motor);
	
		// Spin loop and sleep
		ros::spinOnce();
		loop_rate.sleep();
	}
	
return 0;
}
