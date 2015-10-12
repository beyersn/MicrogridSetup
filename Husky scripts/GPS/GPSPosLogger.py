#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix
from time import time, sleep
import logging
import csv



latitude = 0.0
longitude = 0.0
altitude = 0.0

def updatePosition(data):
	global latitude
	latitude = data.latitude
	global longitude
	longitude = data.longitude
	global altitude
	altitude = data.altitude
	print "position coordinates recieved. Logging data..."

logging.basicConfig(filename='control.log', format=50 * '=' +
                    '\n%(asctime)s %(message)s', level=logging.DEBUG) #set up debug logging


class posLogger(object):

    def __init__(self):
        rospy.init_node('posLogger') #start the control node

        rospy.Subscriber('/navsat/fix', NavSatFix, updatePosition) #start listening for GPS data
        self.rate = rospy.Rate(10) #set rate to 10 Hz

        print 'Starting To Recieve Pose Data.'
        try:
            self.__run__() #start motion
        except Exception, e:
            print e
            print "Error during runtime. Exiting."
            logging.exception(e)
        finally:
            print "Run complete."


    def __run__(self):
        sleep(5) #delay before startup
        while not rospy.is_shutdown():
            # get position
            self.lat = latitude
            self.long = longitude
            self.alt = altitude


            self.rate.sleep()


if __name__ == "__main__":
    try:
        gpsLog = posLogger()
    except rospy.ROSInterruptException, e:
        print e
