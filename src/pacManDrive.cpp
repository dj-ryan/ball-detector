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
#define backupState 6	// robot moves backwards
#define pauseState 7 	// set motors to zero


// Global data vars
balboa_core::balboaLL robot;
ball_detector::ballLocation ball;
balboa_core::balboaMotorSpeeds motor;

int robotState = searchState; 	// starting state

ros::Time lastBallSeen; 		// last time the ball was seen

double forgetBallDelay = 2; 	// time to forget a ball was identified (sec)
double lungeDelay = 2; 			// time lunge lasts (sec)
double searchDelay = 0;  		// time search turn lasts (sec), reasigned
double turnDelay = 1;	 		// time robot turns away from wall/obstacle (sec)
double backDelay = 0.5;			// time robot backs up (sec)
double roamDelay = 3;			// time robot roams looking for balls (sec)
double radiusAverage;			// ball radius average
double oldRadiusAve;			// old ball radius average
double avgRadiusThresh = 50;	// average radius before activite track mode
double lungeRadiusThresh = 170;	// max radius before activate lunge mode

int searchCount = 0; 			// counting variable for switching to roam state

bool searchFlag = false;		// tflipflop var for search
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


// Target distance when in track state
double targetDistance = lungeRadiusThresh;

double alpha = 0.1; // length of running average (1/alpha) i.e. 1/0.1 = 10


// Update robot variables
void callbackLL(const balboa_core::balboaLL &data)
{
    // TODO: implement turn state update
    robot = data;

    //if wall is within buffer distance -- range value ^ when object is closer
    if(robot.rangeSensor > 400) {
        //	pubMotor.publish(/*turn right 115 degrees*/);

        robotState = backupState;
    }

}


// Update ball detection variables
void callbackBall(const ball_detector::ballLocation &data)
{
    ball = data;
    // Running average to filter out bad ball radius values / fake balls for past 100 balls
    radiusAverage = 0.9*(oldRadiusAve) + 0.1*(data.radius);
    ROS_INFO("Average: %f", radiusAverage);

    // If ball is not close
    if(radiusAverage < lungeRadiusThresh) {

        // If real ball, enter track state
        if(radiusAverage > avgRadiusThresh) {

            robotState = trackState;
            radiusAverage = 0;
        }
    }



    // If ball is really close, lunge for ball
    else {
        robotState = lungeState;
    }

    if(data.radius < lungeRadiusThresh) {
        robotState = trackState;
    } else {
        robotState = lungeState;
    }

    // Update variables

    radiusAverage = oldRadiusAve;
    lastBallSeen = ros::Time::now();
}


// Roam State
void roamAround() {
    ROS_INFO("going to balls...");

	motor.right = 100;
	motor.left = 100;

    searchCount = 0; // reset search variable
    robotState = searchState;
}

void resetIntegralVars() {
    sumAngleError = 0;
    sumDistanceError = 0;
}



// Track State
void trackBallPID() {

    searchCount = 0; // reset search variable

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
    motor.left = 100; // delta for weak wheel (4/28) removed by rw (4/28)
    motor.right = 100;

    resetIntegralVars();

    robotState = searchState;
}


// Backup State
void backup() {
	motor.left = -80;
	motor.right = -80;
	
	robotState = turnState;	
	
	}

// Turn State
void turnAwayFromWall() {

	motor.right = -100;
    motor.left = 100;

	robotState = searchState;
}


// Search State
void searchForBall() {
    // TODO: implement search function

    if(searchFlag) {

        motor.right = -125;
        motor.left = 125;
        searchDelay = 0.2;

    } else {
        motor.right = 0;
        motor.left = 0;
        searchDelay = 0.5;
    }
    // Wait for ball detection
    searchFlag = !searchFlag; // invert flag

    resetIntegralVars();
    ROS_INFO("searching for ball... %d", searchCount);

    // exit condition
    if (searchCount > 18) {
        ROS_INFO("searchCount > 50");
        robotState = roamState;
    }
    searchCount++; // index counting variable
}


// Ball Timer
void ballTimer() {
    ros::Time begin = ros::Time::now();
    ros::Duration ballDelta = begin - lastBallSeen;
    if(ballDelta.toSec() > forgetBallDelay) {
        ROS_INFO("ball seen a while ago...");
        if(robotState != roamState) {
            robotState = searchState;
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
        // pubMotor.publish(5);
        // ROS_INFO("...");
        // ROS_INFO("%d",robot.distanceRight);
        // ROS_INFO("%f", ball.x);

        //robotState = trackState;

        ballTimer();

        // Robot State Machine
        switch(robotState) {
        case searchState: // Search for balls
            ROS_INFO("=> search state");
            ros::Duration(searchDelay).sleep();
            searchForBall();
            break;

        case roamState: // Look for balls out of sight
            ROS_INFO("=> roam state");
            roamAround();
            pubMotor.publish(motor);
            ros::Duration(roamDelay).sleep();
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
            pubMotor.publish(motor);
            ros::Duration(turnDelay).sleep();
            break;
            
        case backupState: // Back up to avoid ghosts or walls
			ROS_INFO("=> backup state");
			backup();
			pubMotor.publish(motor);
			ros::Duration(backDelay).sleep(); 
			break;

        case pauseState: // Turn off robot motors
            ROS_INFO("=> pause state");
            motor.right = 0;
            motor.left = 0;
            break;

        default:
            ROS_INFO("=> no state");
            // go to search state?
            break;
        }

        // Publish motors & print values
        ROS_INFO("Publish: %d, %d",motor.left, motor.right);
        pubMotor.publish(motor);

        // Spin and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
