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
#define searchState 1 	// scan to find balls
#define roamState 2 	// robot moves randomly
#define trackState 3 	// drive towards ball that has been found
#define lungeState 4 	// drive foward when close to ball
#define backupState 5	// robot moves backwards
#define turnState 6 	// avoid wall by turnning if IR sensor trips
#define pauseState 7 	// set motors to zero


// Global data vars
balboa_core::balboaLL robot;
ball_detector::ballLocation ball;
balboa_core::balboaMotorSpeeds motor;
ros::Time lastBallSeen; 		// last time the ball was seen

int robotState = searchState; 	// starting state
int searchCount = 0; 			// counting variable for switching to roam state
int roamCount = 0;				// counting variable for avoiding obstacles in roam state

double forgetBallDelay = 2; 	// time to forget a ball was identified (sec)
double lungeDelay = 1.5; 		// time lunge lasts (sec)
double searchDelay = 0;  		// time search turn lasts (sec), reasigned
double turnDelay = 1.2;	 		// time robot turns away from wall/obstacle (sec)
double backDelay = 0.5;			// time robot backs up (sec)
double roamDelay = 1;			// time robot roams looking for balls (sec)
double radiusAverage;			// ball radius average
double oldRadiusAve;			// old ball radius average
double avgRadiusThresh = 4;		// average radius before activate track mode
double lungeRadiusThresh = 140;	// max radius before activate lunge mode
double alpha = 0.1; 			// length of running average (1/alpha) i.e. 1/0.1 = 10
double targetDistance = 180;    // Target distance when in track state

bool searchFlag = false;		// tflipflop var for search
bool directionFlag = false;		// tflipflop direction var for search
//bool roamFlag = false;


// Global PID vars
double oldAngleError = 0;
double oldDistanceError = 0;
double sumAngleError = 0;
double sumDistanceError = 0;
double angleRunningAvg = 0;
double distanceRunningAvg = 0;


// PID values
double pAngle = 0.1;		// 0.09
double dAngle = 0.2;		// 0.07
double iAngle = 0.0;		// 0.0
double pDistance = 0.6;		// 0.6
double dDistance = 0.5;		// 0.05
double iDistance = 0.01;	// 0.01


// Update ball detection variables
void callbackBall(const ball_detector::ballLocation &data)
{
    ball = data;
    
    // Running average to filter out bad ball radius values / fake balls for past 100 balls
    radiusAverage = (1-alpha)*(oldRadiusAve) + alpha*(data.radius);

    // If ball is not close
    if(data.radius < lungeRadiusThresh) {

        // If real ball, enter track state
        if(radiusAverage > avgRadiusThresh) {
			
		    ROS_INFO("Average: %f", radiusAverage);
            robotState = trackState;
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


// Update robot variables
void callbackLL(const balboa_core::balboaLL &data)
{
    robot = data;

    // if wall is within buffer distance/range value, backup robot
    if(robot.rangeSensor > 400) {
        robotState = backupState;
    }

}


// Update track PID variables 
void resetIntegralVars() {
    sumAngleError = 0;
    sumDistanceError = 0;
}


// Search State
void searchForBall() {
	
    if((searchFlag == 0) && (directionFlag == 0)) { // Spin to search CW

        motor.right = -125;
        motor.left = 125;
        searchDelay = 0.2;

    } else if((searchFlag == 0) && (directionFlag == 1)) { // Spin to search CCW 
		motor.right = 125;
        motor.left = -125;
        searchDelay = 0.2;
        
	} else { // Pause to look for balls
        motor.right = 0;
        motor.left = 0;
        searchDelay = 0.6;
    }
    
    // Wait for ball detection
    searchFlag = !searchFlag; // invert flag
    resetIntegralVars();
    ROS_INFO("searching for ball... %d", searchCount);

    // Exit condition if no balls
    if (searchCount > 8) {
        ROS_INFO("enterring roamState");
        directionFlag = !directionFlag; // invert flag
        
        robotState = roamState;
    }
  
	roamCount = 0; // reset roam variable  
    searchCount++; // index counting variable
}


// Roam State
void roamAround() {
    ROS_INFO("going to balls...");

	motor.right = 100;
	motor.left = 110;

    searchCount = 0; // reset search variable
    roamCount++; 	 // increment roam count variable
    
    if (roamCount > 2) {
		robotState = searchState;
	}
}


// Track State
void trackBallPID() {

    searchCount = 0; // reset search variable
    radiusAverage = 0; // reset average variable

    // Calculate error
    double angleError = ball.x;
    double distanceError = targetDistance - ball.radius;

    // add up errors for i term
    sumAngleError +=  angleError;
    sumDistanceError += distanceError;

    // PID EQUATIONS
    double anglePIDValue = (pAngle * angleError) +
                           (dAngle * (angleError - oldAngleError)) +
                           (iAngle * sumAngleError);

    double distancePIDValue = (pDistance * distanceError) +
                              (dDistance * (distanceError - oldDistanceError)) +
                              (iDistance * sumDistanceError);

    // running average calculations to filter out random balls
    angleRunningAvg = (angleRunningAvg * (1.0 - alpha)) + (anglePIDValue * alpha);
    distanceRunningAvg = (distanceRunningAvg * (1.0 - alpha)) + (distancePIDValue * alpha);

    // Save old error values for derivative pid equation
    oldAngleError = angleError;
    oldDistanceError = distanceError;


    // Cast double math as an int
    int motorLeft = (int)(distanceRunningAvg + angleRunningAvg);
    int motorRight = (int)(distanceRunningAvg - angleRunningAvg);

    // speed limit for the motors
    if(motorLeft > 125) {
        motorLeft = 125;
    }
    if(motorLeft < -125) {
        motorLeft = -125;
    }
    if(motorRight > 125) {
        motorRight = 125;
    }
    if(motorRight < -125) {
        motorRight = -125;
    }

    // cast to 8 bit
    motor.left = (int8_t)motorLeft;
    motor.right = (int8_t)motorRight;

    ROS_INFO("A-ravg: %f | D-ravg: %f | Lm: %d, Rm: %d", angleRunningAvg, distanceRunningAvg, motor.left, motor.right);
}


// Lunge State
void lungeFowardToBall() {
    ROS_INFO("lunged foward");
    // pub motor straight
    motor.left = 110; // delta for weak wheel (4/28) - updated by rachael (4/30)
    motor.right = 100; 

    resetIntegralVars();

    robotState = searchState;
}


// Backup State
void backup() {
	motor.left = -60;
	motor.right = -60;
	
	robotState = turnState;	
	
	}	
	
	
// Turn State
void turnAwayFromWall() {

	motor.right = -100;
    motor.left = 100;

	robotState = roamState;
}
	

// Ball Timer
void ballTimer() {
    ros::Time begin = ros::Time::now(); // get current time
    ros::Duration ballDelta = begin - lastBallSeen; // determine time since last ball seen
    if(ballDelta.toSec() > forgetBallDelay) { // if ball seen a while ago
        //ROS_INFO("ball seen a while ago...");
        
        if(robotState == trackState) { // exit trackState and return to searchState
            robotState = searchState;
            ROS_INFO("ball seen a while ago...");
        }
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
		
        // Robot State Machine
        switch(robotState) {
        case searchState: // Search for balls
            ROS_INFO("=> search state");
            ros::Duration(searchDelay).sleep(); // should not update msgs
            searchForBall(); 
            break;

        case roamState: // Look for balls out of sight
            ROS_INFO("=> roam state");
            roamAround();
            pubMotor.publish(motor); // publish motor speeds
            ros::Duration(roamDelay).sleep(); // delay while driving
            break;

        case trackState: // Track balls once seen
            ROS_INFO("=> track state");
            trackBallPID();
            break;

        case lungeState: // Hit ball once in range
            ROS_INFO("=> lunge state");
            lungeFowardToBall();
            pubMotor.publish(motor); // publish motor speeds
            ros::Duration(lungeDelay).sleep(); // delay while driving
            break;
            
        case backupState: // Back up to avoid ghosts or walls
			ROS_INFO("=> backup state");
			backup();
			pubMotor.publish(motor); // publish motor speeds
			ros::Duration(backDelay).sleep(); // delay while driving
			break;

        case turnState: // Turn to avoid wall
            ROS_INFO("=> turn state");
            turnAwayFromWall();
            pubMotor.publish(motor); // publish motor speeds
            ros::Duration(turnDelay).sleep(); // delay while driving
            break;

        case pauseState: // Turn off robot motors
            ROS_INFO("=> pause state");
            motor.right = 0;
            motor.left = 0;
            break;

        default:
            ROS_INFO("=> no state");
            break;
        }

		// Re-enter search state if ball subscription has not been updated/no balls seen
        ballTimer();

        // Publish motors & print values
        ROS_INFO("Publish: %d, %d",motor.left, motor.right);
        pubMotor.publish(motor);

        // Spin and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
