#!/usr/bin/env python

import rospy
import threading
from geometry_msgs.msg import Vector3Stamped
from MainController import *
from myserial import MotorControler
import time
# ROS names
nodeName = 'PathController'
subName = "/matlab_velocity"


# COM
for i in range(4):
        try:
                com = MotorControler('/dev/ttyACM'+ str(i))
        except:
                print('problem z ustawieniem portu nr'+ str(i))
        else: break

controller = MainController(com)
status = False

def callback(msg):
    global status
    rospy.loginfo('New msg received: '+ msg.header.frame_id)
    status  = controller.addPathPoint(msg.vector, msg.header.frame_id)	


def listener():
    global rate
    rospy.init_node(nodeName, anonymous = True)
    rospy.Subscriber(subName, Vector3Stamped , callback)
    rate = rospy.Rate(50)


    #rospy.spin()

if __name__ == '__main__':
    listener()
    print('PATH CONTROLLER INIT DONE!')
    while not rospy.is_shutdown():
       # print("in the while loop")
        global status, rate
        if status:
            output = controller.mainProcess()
           # if not output['status']: rospy.loginfo('Vel_R = '+ str( round( output['VR'], 2 ))+ ' Vel_L = ' + str( round( output['VL'], 2 )) )
        rate.sleep()

