#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
import socket
import sys
from select import select
import struct


def talker():
    # setup a socket with additional options
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.settimeout(1)

    timeout = 1
    ip_address = '192.168.0.25' #is this the address of the camera system?
    print "[Optional] Enter IP (Defualt=" + ip_address + "):" # prompt for ip input
    rlist, _, _ = select([sys.stdin], [], [], timeout) # read from stdin, wait for timeout for input
    if rlist:
        ip_address = sys.stdin.readline()
    else:
        ip_address = '192.168.0.25'

    print ip_address, 'Has Been Selected.'
    server_address = (ip_address, 1895) # touple: ip and port number
    print >>sys.stderr, 'Connecting To %s Port %s' % server_address
    sock.connect(server_address) #connect to the server on the camera system

    pub = rospy.Publisher('qualisys_pose', Pose2D, queue_size=1) #publish to, message type, buffer size
    rospy.init_node('get') #setup a node called get
    rate = rospy.Rate(50) #send messages at 50hz

    pose_msg = Pose2D() #expresses position on a plane
    pose_msg.x = float('nan')
    pose_msg.y = float('nan')
    pose_msg.theta = float('nan')
    print 'Connection Established.'
    print 'Starting To Recieve And Publish Pose Data.'
    while not rospy.is_shutdown():
        check = struct.unpack('<B', sock.recv(1))[0]
        if check is 2:
            pub.publish(pose_msg)
            rate.sleep()
        else:
            print 'BAD'
            continue
        recieved_data = sock.recv(12)
        if len(recieved_data) is not 12:
            continue
        pose_msg.x = struct.unpack('<f', recieved_data[:4])[0]
        pose_msg.y = struct.unpack('<f', recieved_data[4:8])[0]
        pose_msg.theta = struct.unpack('<f', recieved_data[8:12])[0]


if __name__ == "__main__": #run talker unless there is an error
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
