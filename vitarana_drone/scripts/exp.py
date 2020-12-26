#!/usr/bin/env python
import rospy
import sys
import time
from vitarana_drone.msg import *
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest
from std_msgs.msg import String


def callback_check(msg):
        print "Hello"


r = rospy.Rate(16)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
while not rospy.is_shutdown():      # run pid() until rospy is shutdown
    rospy.wait_for_service('/edrone/activate_gripper')
    hold = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
    rospy.Subscriber('/edrone/gripper_check' , String , callback_check)

    if bool(hold):
        print("picked up successfully")
    else:
        print("pick up faied")
           
    r.sleep()

    
