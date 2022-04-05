#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

void ballLocationCallback(const std_msgs::Int64 data)
{
    // do some math
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ballFollower");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("ballLocation", 1000, ballLocationCallback);
    
    ros::Publisher chatter_pub = n.advertise<std_msgs::Int64>("BalboaLL", 1000);

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        chatter_pub.publish(/*something*/);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}