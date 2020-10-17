#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


PI = 3.15
Radius = 1
Speed = 1




def pose_callback(msg):
	global reached , theta , theta_moved , prevTheta
	#defining twist varible
	vel_msg = Twist()

	#absolute value of current angle
	theta = abs(msg.theta)

	#angle moved till now
	theta_moved = theta_moved + abs(theta - prevTheta)

	prevTheta = theta
	
	
	
	#if angular distance is less than 2*PI
	if(theta_moved<2*PI):
		rospy.loginfo("Moving in circle:" + "\n" + str(theta_moved))

		#angilar_velocity * radius = linear_velocity
		#for Radius  = 1
		#angular velocity = linear velocity
		
		vel_msg.linear.x = 1
		vel_msg.angular.z = 1
		pub.publish(vel_msg)
	
	#if angular diatance more than 2*Pi and not reached to goal yet.
	if(theta_moved>2*PI and reached ==0):
		
		vel_msg.linear.x = 0
		vel_msg.angular.z =0
		pub.publish(vel_msg)
		rospy.loginfo("Reached To Goal")
		reached=1



if __name__ == '__main__':
	#varible checks if reached to goal or not
	#varible prevTheta stores previous value of theta
	global reached , prevTheta
	try:
		prevTheta = 0
		theta_moved =0
		#initialization of node
		rospy.init_node('node_turtle_revolve',anonymous=True)

		#Publishing message to our publisher (/cmd_vel)
		pub = rospy.Publisher('turtle1/cmd_vel', Twist , queue_size = 10)

		#Subscribe messages from turtls1/pose
		sub = rospy.Subscriber('turtle1/pose',Pose , pose_callback)

		reached =0 
		rospy.spin()
		
	except rospy.ROSInterruptException:
		pass