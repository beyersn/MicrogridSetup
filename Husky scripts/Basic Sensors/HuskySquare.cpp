#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "clearpath_base/Encoders.h"

#include <sstream>

double trvl; //Travel Distance
int count = 0;
//function to parse the data from the encoders
void encoderCallback(const clearpath_base::Encoders::ConstPtr &msg)
{
    trvl = msg->encoders[1].travel; //update total travel distance
    count ++;
    ROS_INFO("Updating info");
}

int main(int argc, char **argv)
{
    double iteration_rate = 10; //iteration rate in Hz

    ros::init(argc, argv, "HuskySquare"); //initialize the node running this code
    ros::NodeHandle n; //call this node n
    ros::Publisher huskycmd_pub = n.advertise<geometry_msgs::Twist>("husky/cmd_vel", 1000); //allow publishing of Twist messages to cmd_vel with buffer of 1000
    ros::Subscriber sub = n.subscribe("husky/data/encoders", 1000, encoderCallback); //subscribe to encoder data and call function when recieved
    ros::Rate loop_rate(iteration_rate); //Set the minimum time per loop iteration

    double linear_dist = 1.0; //set distance to travel (m)
    double turn_dist = 0.864; // Roughly the distance the wheel travels to turn 90 degrees.
    double linear_rate = .5; //set speed at which to move (m/s)
    double turn_rate = .5; //speed at which to turn (rad/s)
    while(count==0){
    	ros::spinOnce();
    }
    double current;
	for(int i=0; i<4; i++){ //loop 4 times to complete one square
		current = trvl + linear_dist;
		while(ros::ok() && trvl < current){
	        geometry_msgs::Twist msg; //initialize message
	        msg.linear.x = linear_rate; //set speed
	        huskycmd_pub.publish(msg); //send command
	        ros::spinOnce(); //spin to keep from shutting down
	        loop_rate.sleep(); //wait until the timestep is reached
		}
		current = trvl + turn_dist;
		while(ros::ok() && trvl < current){
	        geometry_msgs::Twist msg; //initialize message
	        msg.angular.z = turn_rate; //set speed
	        huskycmd_pub.publish(msg); //send command
	        ros::spinOnce(); //spin to keep from shutting down
	        loop_rate.sleep(); //wait until the timestep is reached
		}
    }


return 0;
}
