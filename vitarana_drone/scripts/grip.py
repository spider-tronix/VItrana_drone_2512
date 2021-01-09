#!/usr/bin/env python
import rospy
import sys
from vitarana_drone.msg import *
from std_msgs.msg import String
from vitarana_drone.srv import Gripper,GripperResponse,GripperRequest



rospy.init_node('grip')

def grip(status):
    rospy.wait_for_service('/edrone/activate_gripper')
    hold = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
    out = hold(status)
    return out.result

def gripper_check_callback(msg):
    response_pub = rospy.Publisher('/gripper_response',String,queue_size =1)
    global deactivate
    if msg.data=="True" and deactivate=="False":
        grip(True)
        if grip(True):
            print("picked up")
            response_pub.publish("true")
            
        else:
            print("not picked up") 
            response_pub.publish("false")                 
    else:
        print("not yet reached parcle")
        response_pub.publish("false")


def deactivate_callback(msg):
    global deactivate
    deactivate = msg.data
    if(msg.data=="True"):
        grip(False)
        

if __name__ == '__main__':
    
    
    r = rospy.Rate(10)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():      # run pid() until rospy is shutdown
        try:
         global deactivate
         rospy.Subscriber('/deactivate_gripper',String, deactivate_callback)
         rospy.Subscriber('/edrone/gripper_check', String, gripper_check_callback)            # execute pid equation
         r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException: pass  #return error is anything goees wwrong
                    