#!/usr/bin/env python

import rospy # Needed for ROS command w/ python
from geometry_msgs.msg import Twist # Used for cmd_vel messages sent to robot
from sensor_msgs.msg import LaserScan # Messages from Lidar System


def readDistance(data): # Takes Lidar data as the input and interpret it
    print "test 2" # Does the code make it to this point
    lower_range = int(len(data.ranges) / 2 - half_range_width) # midpoint plus how many terms (30 here)
    upper_range = int(len(data.ranges) / 2 + half_range_width)
    twist_msg = Twist() # initialize command
    message = "@%s: Lower index is %d, Upper index is: %d" % (
        rospy.get_time(), lower_range, upper_range) # show the time and range of the data
    rospy.loginfo(message) # print message information
    if rospy.is_shutdown(): # test if rospy is working
        print "is down"
    else:
        print "is not down"

    print "test3" # does the code make it to this point
    distance_array = data.ranges[lower_range:upper_range] # take data from upper to lower range
    distance = float(sum(distance_array)) / len(distance_array) # take average distance in this range

    velocity = keepDistance(distance) # use function to find appropriate velocity

    twist_msg.linear.x = velocity # set velocity in command
    print distance,velocity,twist_msg # display what the command will be
    pub.publish(twist_msg) # send the command



def keepDistance(distance): # simple proportional controller for holding distance
    angular_coef = 1 # for now just a coefficient for the controller
    limit = 2 # How far away should we stay? (2 meters)
    velocity = (distance - limit) * angular_coef # Actual Math for the control
    return velocity


if __name__ == "__main__":
    machine = 'husky' 
    try:
        rospy.init_node('simple_feedback') # configure as a node
        message = "@%s: Node is being initiated" % (rospy.get_time()) #show that the node is set up

        message = "@%s: Running readDistance Function" % (rospy.get_time())
        rospy.loginfo(message) 

        pub = rospy.Publisher('/husky/cmd_vel', Twist, queue_size=1) # allow sending of velocity commands

        message = "@%s: Running publisher" % (rospy.get_time()) 
        rospy.loginfo(message)
        rate = rospy.Rate(10) # Set rate to send messages

        half_range_width = 30 # How many datapoints to take the average from
        rospy.Subscriber('/scan', LaserScan, readDistance) # path, datatype, function to call
        print "test1"
        rospy.spin() # Tells the program to keep doing this

    except rospy.ROSInterruptException, e:
        print e
        message = "@%s: Feedback navigation terminated." % (rospy.get_time())
        rospy.loginfo(message)

    finally:
        message = "@%s: Node is shutting down." % (rospy.get_time())
        rospy.loginfo(message)

        pub.publish(Twist())
