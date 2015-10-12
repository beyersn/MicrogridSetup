#!/usr/bin/env python

import rospy
import numpy as np
import pathGenerator as pthgen
from geometry_msgs.msg import Pose2D, Twist
import socket
import sys
import struct
from math import pi, sin, cos
from time import time, sleep
import PID
import logging
from datetime import datetime

logging.basicConfig(filename='control.log', format=50 * '=' +
                    '\n%(asctime)s %(message)s', level=logging.DEBUG) #set up debug logging


class navigation_control(object):

    def __init__(self):
        self.logger = [[0, 0, 0, 0, 0, 0, 0, 0]]
        # kp_longitudinal = float(raw_input('Set longitudinal KP:'))
        self.longitudinal_pid = PID.PID(4, 0, 0)  # 1.6, 1.70484816196, 1.00106666667
        # kp_lateral = float(raw_input('Set lateral KP:'))
        self.lateral_pid = PID.PID(5, 0, 0)  # 2.0, 2.87999906616, 0.925926226157
        # kp_angle = float(raw_input('Set lateral KP:'))
        self.angle_pid = PID.PID(4, 0, 0)  # 2.8, 4.26087605796, 1.22666479747

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #configure network connection to camera system
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(1)
        ip_address = '192.168.0.25' #address for the qualisys camera system
        server_address = (ip_address, 1895) #combine with port number
        print >>sys.stderr, 'Connecting To %s Port %s' % server_address #display the connection info
        self.sock.connect(server_address) #connect to the camera system

        rospy.init_node('navigation_control') #start the control node
        self.pub_pose = rospy.Publisher('qualisys_pose', Pose2D, queue_size=1) #allow publishing of the camera position
        self.pub_cmd = rospy.Publisher('/husky/cmd_vel', Twist, queue_size=1) #allow publishing to motor controller
        self.rate = rospy.Rate(40) #set rate to 40 Hz

        self.pose_msg = Pose2D() #create a 2D pose message and initialize values
        self.pose_msg.x = float('nan')
        self.pose_msg.y = float('nan')
        self.pose_msg.theta = float('nan')
        self.twist_msg = Twist() #create the message that the motor commands will be sent to

        print 'Connection Established.'
        print 'Starting To Recieve And Publish Pose Data.'
        try:
            self.__run__() #start motion
        except Exception, e:
            np.save('long ' + str(self.longitudinal_pid) + ' lat ' +
                    str(self.lateral_pid) + str(datetime.now()), self.logger)
            logging.exception(e)
        finally:
            self.sock.close()
            np.save('long ' + str(self.longitudinal_pid) + ' lat ' +
                    str(self.lateral_pid) + ' theta ' + str(self.angle_pid) + ' ' + str(datetime.now()), self.logger)

    def __run__(self):
        pth = pthgen.PathGenerator(path_type='circle', speed=.3) #create the path to be used
        sleep(5) #delay before startup
        degree_to_rad = pi / 180 #useful conversion
        reference_x, reference_y = pth.getPosition() #get next position target
        while not rospy.is_shutdown():
            # get position
            check = struct.unpack('<B', self.sock.recv(1))[0] #recieve packet from camera
            if check is 2: #check if recieved
                self.pub_pose.publish(self.pose_msg)
                self.rate.sleep()
            else:
                print 'Warning: Bad Qualisys Packet'
                continue
            recieved_data = self.sock.recv(4096)
            if len(recieved_data) < 12: #check for proper message length
                print 'bad 2'
                self.pub_cmd.publish(Twist()) #send empty message (stop command)
                continue
            self.pose_msg.x = struct.unpack('<f', recieved_data[:4])[0]
            self.pose_msg.y = struct.unpack('<f', recieved_data[4:8])[0]
            self.pose_msg.theta = struct.unpack('<f', recieved_data[8:12])[0] * degree_to_rad
            if self.pose_msg.x == float('nan'):
                print 'Recieved "NaN" For Pose'
                self.pub_cmd.publish(Twist())
                continue

            # calculate error
            reference_x_temp, reference_y_temp = pth.getPosition()

            reference_angle = np.arctan2(reference_y_temp - reference_y, reference_x_temp - reference_x) #determine angle of travel from previous point
            angle_error = reference_angle - self.pose_msg.theta #determine difference between current angle and target
            angle_error_wrapped = (angle_error + np.pi) % (2 * np.pi) - np.pi #find least angle?
            reference_x = reference_x_temp #update previous points with current ones
            reference_y = reference_y_temp
            diff_x = reference_x - self.pose_msg.x / 1000 #pos message must be in (mm)?
            diff_y = reference_y - self.pose_msg.y / 1000
            longitudinal_error = cos(self.pose_msg.theta) * diff_x + sin(self.pose_msg.theta) * diff_y
            lateral_error = cos(self.pose_msg.theta) * diff_y - sin(self.pose_msg.theta) * diff_x
            # lateral_error_linearized = np.arcsin(lateral_error / np.hypot(diff_x, diff_y))
            feedback_linear = self.longitudinal_pid.calculate(longitudinal_error)
            feedback_angular = self.lateral_pid.calculate(lateral_error) + self.angle_pid.calculate(angle_error_wrapped)
            # print feedback_linear, feedback_angular

            # Calculate actuator command
            limit = 1
            feedback_linear = min(limit, max(feedback_linear, -limit))
            limit = 2
            feedback_angular = min(limit, max(feedback_angular, -limit))
            self.twist_msg.linear.x = feedback_linear
            self.twist_msg.angular.z = feedback_angular
            self.pub_cmd.publish(self.twist_msg)
            self.logger = np.append(self.logger, [[longitudinal_error, lateral_error,
                                                   reference_x, reference_y,
                                                   self.pose_msg.x, self.pose_msg.y,
                                                   self.pose_msg.theta, time()]], axis=0)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        nav = navigation_control()
    except rospy.ROSInterruptException, e:
        np.save('long ' + str(nav.longitudinal_pid.kp) + 'lat ' +
                str(nav.lateral_pid.kp) + str(datetime.now()), nav.logger)
        logging.exception(e)
        print e
