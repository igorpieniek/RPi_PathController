#!/usr/bin/env python

import rospy
import threading
from geometry_msgs.msg import Vector3Stamped
from MainController import *
from myserial import MotorControler

# ROS names
nodeName = 'PathController'
subName = "/matlab_velocity"


# COM
for i in range(4):
        try:
                com = MotorControler('/dev/ttyUSB'+ str(i))
        except:
                print('problem z ustawieniem portu nr'+ str(i))
        else: break

controller = MainController(com) 

def callback(msg):
    rospy.loginfo('New msg received: '+ msg.header.frame_id)
    status  = controller.addPathPoint(msg.vector, msg.header.frame_id)	
    while status:
        output = controller.mainProcess()
        if not output['status']: break
        rospy.loginfo('Vel_R = '+ str( round( output['VR'], 2 )) 
                        + ' Vel_L = ' + str( round( output['VL'], 2 )) )


def listener():
    rospy.init_node(nodeName, anonymous = True)
    rospy.Subscriber(subName, Vector3Stamped , callback)
    rospy.spin()

if __name__ == '__main__':
    listener()