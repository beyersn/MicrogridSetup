#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D, Twist
import socket
import sys
from select import select
import struct
from math import pi, sin, cos


class navigation_control(object):

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(1)

        timeout = 1
        ip_address = '192.168.0.25'
        print "[Optional] Enter IP (Defualt=" + ip_address + "):"
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            ip_address = sys.stdin.readline()
        else:
            ip_address = '192.168.0.25'

        print ip_address, 'Has Been Selected.'
        server_address = (ip_address, 1895)
        print >>sys.stderr, 'Connecting To %s Port %s' % server_address

        self.sock.connect(server_address)

        rospy.init_node('navigation_control')
        self.pub_pose = rospy.Publisher('qualisys_pose', Pose2D, queue_size=1)
        self.pub_cmd = rospy.Publisher('/husky/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(40)

        self.pose_msg = Pose2D()
        self.pose_msg.x = float('nan')
        self.pose_msg.y = float('nan')
        self.pose_msg.theta = float('nan')
        self.twist_msg = Twist()
        self.reference_x = float(raw_input('Set X(meters):')) #prompt for destination coordinates
        self.reference_y = float(raw_input('Set Y(meters):'))
        print 'Connection Established.'
        print 'Starting To Recieve And Publish Pose Data.'
        self.__run__()

    def __run__(self):
        while not rospy.is_shutdown():
            # get position
            check = struct.unpack('<B', self.sock.recv(1))[0]
            if check is 2:
                self.pub_pose.publish(self.pose_msg)
                self.rate.sleep()
            else:
                print 'BAD'
                continue
            recieved_data = self.sock.recv(4096)
            if len(recieved_data) < 12:
                print 'bad 2'
                self.pub_cmd.publish(Twist())
                continue
            self.pose_msg.x = struct.unpack('<f', recieved_data[:4])[0]
            self.pose_msg.y = struct.unpack('<f', recieved_data[4:8])[0]
            self.pose_msg.theta = struct.unpack('<f', recieved_data[8:12])[0]
            if self.pose_msg.x == float('nan'):
                print 'Recieved "NaN" For Pose'
                self.pub_cmd.publish(Twist())
                continue

            # calculate error
            diff_x = self.reference_x - self.pose_msg.x/1000
            diff_y = self.reference_y - self.pose_msg.y/1000
            feedback_longitudinal_coef = .1
            feedback_lateral_coef = 2
            # feedback_angular_coef = .1
            longitudinal_error = cos(self.pose_msg.theta * pi / 180)\
                * diff_x + sin(self.pose_msg.theta * pi / 180) * diff_y
            lateral_error = cos(self.pose_msg.theta * pi / 180) * \
                diff_y - sin(self.pose_msg.theta * pi / 180) * diff_x
            feedback_linear = feedback_longitudinal_coef * longitudinal_error
            feedback_angular = feedback_lateral_coef * lateral_error
            print '\r','linear',longitudinal_error,'lateral',lateral_error,

            # Calculate actoator command
            limit = 1
            feedback_linear = min(limit, max(feedback_linear, -limit))
            limit = .2
            feedback_angular = min(limit, max(feedback_angular, -limit))
            self.twist_msg.linear.x = feedback_linear
            self.twist_msg.angular.z = feedback_angular
            self.pub_cmd.publish(self.twist_msg)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        navigation_control()
    except rospy.ROSInterruptException, e:
        print e
        pass
