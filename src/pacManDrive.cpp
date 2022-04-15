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
#define searchState 0 // same as roam state
#define trackState 1 // drive towards ball that has been found
#define lungeState 2 // drive foward when close to ball
#define scanState 3 // 360 degree scan to find balls
#define turnState 4 // avoid wall by turnning if ir sensor trips
#define roamState 5 //robot moves randomly


// global data vars
balboa_core::balboaLL robot;
ball_detector::ballLocation ball;
balboa_core::balboaMotorSpeeds motor;

int robotState = searchState; // starting state 
ros::Time lastBallSeen; // last time the ball was seen
int32_t forgetBallDelay = 3; // time to forget a ball was identified (sec)
int32_t lungeDelay = 2; // time lunge lasts (sec)
int32_t lungRadiusThresh = 200; // max radius before activate lunge mode


// global pid vars
double oldAngleError = 0;
double oldDistanceError = 0;
double angleRunningAvg = 0;
double distanceRunningAvg = 0;


// pid values
double pAngle = 0.015;
double dAngle = 0.05;
double pDistance = 0.07;
double dDistance = 0.01;

// target distance when in track state
double targetDistance = (double)lungRadiusThresh; 

double alpha = 0.1; // length of running average (1/alpha) i.e. 1/0.1 = 10

void callbackLL(const balboa_core::balboaLL &data)
{
	// TODO: implement turn state update
	robot = data;
	
	if(robot.rangeSensor > 100){	//if wall is within buffer distance -- range value ^ when object is closer
	//	pubMotor.publish(/*turn right 115 degrees*/);
	robotState = turnState;
	}
	
}

void callbackBall(const ball_detector::ballLocation &data)
{
	ball = data;
	
	//ROS_INFO("====> ball data");
	
	// may need to implement some sort of filter here 
	// to filter out false balls

	if(ball.radius < lungRadiusThresh) {
		
		robotState = trackState;
	} else {
		robotState = lungeState;
	}
	lastBallSeen = ros::Time::now();
}

void roamAround() {
	
		// TODO: implement roam funciton
	
	}

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

    motor.left = distanceRunningAvg + angleRunningAvg;
    motor.right = distanceRunningAvg - angleRunningAvg;
		
	//ROS_INFO("-------------------------");
    //ROS_INFO("Radius: %d", ball.radius);
    //ROS_INFO("x: %d", ball.x);
    ROS_INFO("left motor: %f | right motor: %f", motor.left, motor.right);
}

void lungeFowardToBall(){
		ROS_INFO("lunged foward");
		// TODO: pub motors staright
		//pub motor straight
		ros::Duration(lungeDelay).sleep(); // should not update msgs
		robotState = searchState;
	
	
	}


void turnAwayFromWall(){
		// TODO: implement turn function
	
		// turn motors
		
		robotState = searchState;
	
	}

void searchForBall(){
		// TODO: implement search function
	}


int main(int argc, char ** argv)
{
	ROS_INFO("STARTED");
	ros::init(argc, argv, "pacManDrive");
	ros::NodeHandle node;

	ros::Subscriber subLL = node.subscribe("balboaLL", 1000, callbackLL);
	ros::Subscriber subBall = node.subscribe("ballLocation", 1000, callbackBall);
	ros::Publisher pubMotor = node.advertise<std_msgs::Int32>("motorSpeeds", 1000);

	ros:: Rate loop_rate(10);

	while(ros::ok())
	{
//		pubMotor.publish(5);
//		ROS_INFO("...");
//		ROS_INFO("%d",robot.distanceRight);
//		ROS_INFO("%f", ball.x);
		
		
		//pubMotor.publish(motor)
		
		
		

		ros::Time begin = ros::Time::now();
		ros::Duration ballDelta = begin - lastBallSeen;
		if(ballDelta.toSec() > forgetBallDelay){
			ROS_INFO("ball seen a while ago...");
			robotState = searchState;
		}
		
		
		
		
		switch(robotState) {
			case searchState:
				ROS_INFO("=> seach state");
				searchForBall();
				break;
			case trackState:
				ROS_INFO("=> track state");
				trackBallPID();
				break;
			case lungeState:
				ROS_INFO("=> lunge state");
				lungeFowardToBall();
				break;
			case turnState:
				ROS_INFO("=> turn state");
				turnAwayFromWall();
				break;
			default:
				ROS_INFO("=> no state");
				// go to search state?
				break;
		}
		
		
		
		ros::spinOnce();
		loop_rate.sleep();
	}
return 0;
}
