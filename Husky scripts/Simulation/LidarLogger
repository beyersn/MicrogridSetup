#!/usr/bin/env python

import rospy
import numpy as np
from time import time, sleep
from datetime import datetime
import logging
from sensor_msgs.msg import LaserScan # Messages from Lidar System
import sys



logging.basicConfig(filename='control.log', format=50 * '=' +
                    '\n%(asctime)s %(message)s', level=logging.DEBUG) #set up debug logging


class LidarLogger(object):

    def __init__(self):
        rospy.init_node('lidarLogger') #start the control node
        self.logger = []
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.readDistance)
        self.rate = rospy.Rate(10) #set rate to 10 Hz
        # read = 0
        while not rospy.is_shutdown() and len(self.logger)<1:
            self.rate.sleep()
            #rospy.spin()

        try:
            pass
            # self.__run__() #start motion
        except Exception, e:
            print e
            print "Error during runtime. Exiting."
            logging.exception(e)
        finally:
            print "Run complete."
            print self.logger # REMOVE LATER
            np.save('scan'+ str(datetime.now()),self.logger)


    # def __run__(self):
    #     sleep(5) #delay before startup
    #     while not rospy.is_shutdown():
    #         self.rate.sleep()


    def readDistance(self,data):
        scan = data.ranges
        print scan
        self.logger = scan
        print "Lidar Scan Recieved. Logging data..."
        self.subscriber.unregister()
        

if __name__ == "__main__":
    try:
        lidarLog = LidarLogger()
    except rospy.ROSInterruptException, e:
        print e