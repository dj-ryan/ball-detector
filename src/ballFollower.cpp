#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "ballDetector/ballLocation.h" 

#include <sstream>

// Initialize variables 
std_msgs::Int32 targetAngle;
std_msgs::Int32 targetDistance;

// 
void ballLocationCallback(const ballDetector::ballLocation ballLocation)
{
    ROS_INFO("Image Width: %d | Image Height: %d | X Pos: %f | Y Pos: %f | Radius: %f", ballLocation.imageWidth, ballLocation.imageHeight, ballLocation.x, ballLocation.y, ballLocation.radius);

    if (ballLocation.x > 0) {

        targetAngle = -ballLocation.x; 
    }
    
    else if (ballLocation.x < 0) {

        targetAngle = abs(ballLocation.x);
    }

    else if (ballLocation.radius > 500) {

        targetDistance = -100;
    }
    
    else if (ballLocation.radius < 500 {

        targetDistance = 100;
    }

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ballFollower");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("ballLocation", 1000, ballLocationCallback);
    
    ros::Publisher pubRot = n.advertise<std_msgs::Int32>("rotation", 1000);
    ros::Publisher pubDist = n.advertise<std_msgs::Int32>("distance", 1000);
    
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        pubRot.publish(targetAngle);
        pubDist.publish(targetDistance);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}