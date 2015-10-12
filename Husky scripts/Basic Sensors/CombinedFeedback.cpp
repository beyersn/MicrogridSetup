#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "clearpath_base/Encoders.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"


double imuL; //Travel Distance Left side
double imuR; //Travel Distance Right side
double lidarDist; //Average distance to obstacle

//function to parse the data from the encoders
void encoderCallback(const clearpath_base::Encoders::ConstPtr &msg)
{
    imuL = msg->encoders[0].travel; //update travel distances
    imuR = msg->encoders[1].travel; 
}

//function to parse the data from the lidar
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    half_range_width = 30; //double this to get number of datapoints to use
    int lower_range = sizeof(msg->ranges) / 2 - half_range_width; //determine lower bound
    int upper_range = sizeof(msg->ranges) / 2 + half_range_width; //determine upper bound
    double sum = 0;
    for(int i=lower_range; i <= upper_range; i++){
        sum += msg->ranges[i]; //add all distances in this range
    }
    lidarDist = sum / (upper_range-lower_range); //take the average distance
}

//function to parse the data from the imu
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //Put data to use here
}

int main(int argc, char **argv)
{
    double iteration_rate = 10; //iteration rate in Hz

    ros::init(argc, argv, "HuskySquare"); //initialize the node running this code
    ros::NodeHandle n; //call this node n
    ros::Publisher huskycmd_pub = n.advertise<geometry_msgs::Twist>("husky/cmd_vel", 1000); //allow publishing of Twist messages to cmd_vel with buffer of 1000
    // Setup all subscribers
    ros::Subscriber subEnc = n.subscribe("husky/data/encoders", 1000, encoderCallback);
    ros::Subscriber subLid = n.subscribe("scan", 1000, lidarCallback);
    ros::Subscriber subImu = n.subscribe("imu/data", 1000, imuCallback);

    ros::Rate loop_rate(iteration_rate); //Set the minimum time per loop iteration
    int count;

    double linear_dist = 3; //set distance to travel (m)
    double linear_rate = .5; //set speed at which to move (m/s)
    double stopThresh = .2; //proximity where robot will stop (m)
    
    count = 0;
    while (ros::ok() && trvl < imuL && lidarDist > stopThresh) //run code if master is running and while robot has distance to go
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
