#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

// Drives the robot in a square
int main(int argc, char **argv)
{
    double iteration_rate = 10;
    ros::init(argc, argv, "HuskyForward");
    ros::NodeHandle n;
    ros::Publisher huskycmd_pub = n.advertise<geometry_msgs::Twist>("husky/cmd_vel", 1000);
    ros::Rate loop_rate(iteration_rate);
    int count;
    for (int i = 0; i < 4; i++)
    {
        // linear movement
        double linear_time=3;
        double linear_rate=.5;
        count = 0;
        while (ros::ok() && count < linear_time*iteration_rate)
        {
            geometry_msgs::Twist msg;

            msg.linear.x = linear_rate;

            huskycmd_pub.publish(msg);

            ros::spinOnce();

            loop_rate.sleep();
            ++count;
        }
        // rotation
        count = 0;
        double turn_time=3;
        double turn_rate=3.1415/turn_time;
        while (ros::ok() && count < turn_time*iteration_rate)
        {
            geometry_msgs::Twist msg;

            msg.angular.z = turn_rate;

            huskycmd_pub.publish(msg);

            ros::spinOnce();

            loop_rate.sleep();
            ++count;
        }
    }

    return 0;
}
