#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "clearpath_base/Encoders.h"

#include <sstream>

double trvl; //Travel Distance

//function to parse the data from the encoders
void encoderCallback(const clearpath_base::Encoders::ConstPtr &msg)
{
    trvl = msg->encoders[0].travel; //update total travel distance
}

int main(int argc, char **argv)
{
    double iteration_rate = 10; //iteration rate in Hz

    ros::init(argc, argv, "HuskySquare"); //initialize the node running this code
    ros::NodeHandle n; //call this node n
    ros::Publisher huskycmd_pub = n.advertise<geometry_msgs::Twist>("husky/cmd_vel", 1000); //allow publishing of Twist messages to cmd_vel with buffer of 1000
    ros::Subscriber sub = n.subscribe("husky/data/encoders", 1000, encoderCallback); //subscribe to encoder data and call function when recieved
    ros::Rate loop_rate(iteration_rate); //Set the minimum time per loop iteration
    int count;

    double linear_dist = 3; //set distance to travel (m)
    double linear_rate = .5; //set speed at which to move (m/s)
    count = 0;
    while (ros::ok() && trvl < linear_dist) //run code if master is running and while robot has distance to go
    {
        geometry_msgs::Twist msg; //initialize message
        msg.linear.x = linear_rate; //set speed
        huskycmd_pub.publish(msg); //send command
        ros::spinOnce(); //spin to keep from shutting down
        loop_rate.sleep(); //wait until the timestep is reached
        ++count;
    }


return 0;
}
