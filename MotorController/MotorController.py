#!/usr/bin/env python

import rospy
import threading
from std_msgs.msg import Float32MultiArray
from MainController import *
from myserial import *

# ROS names
nodeName = 'PathController'
subName = "/matlab_velocity"

# COM
com = MotorControler('COM1')

def callback(commandMessage):
    controller = MainController(commandMessage, com) 
	rospy.loginfo('New path received')
    while status:
        output = controller.mainProcess()
        if not output['status']: break
        rospy.loginfo('Vel_R = '+ str( round( output['VR'], 2 )) 
                      + ' Vel_L = ' + str( round( output['VL'], 2 )) )


def listener():
    rospy.init_node(nodeName, anonymous = True)
    rospy.Subscriber(subName, Float32MultiArray , callback)
    rospy.spin()

if __name__ == '__main__':
    listener()