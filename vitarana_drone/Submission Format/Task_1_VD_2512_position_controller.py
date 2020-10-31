#!/usr/bin/env python

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf

class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')
        
        self.gps_value = [19.0, 72.0, 0.31]
        self.setpoint_gps_value = [19.0, 72.0, 0.31]


        self.checkpoint_1 =0
        self.checkpoint_2 =0


        self.command_value = edrone_cmd()
        self.command_value.rcRoll = 0.0
        self.command_value.rcPitch = 0.0
        self.command_value.rcYaw = 0.0
        self.command_value.rcThrottle = 0.0

    
        self.error = [0,0,0]
        self.errorI = [0,0,0]
        self.errorD = [0,0,0]

        self.prev_error = [0,0,0] 

        self.Kp = [6000000, 6000000, 15]
        self.Ki = [0, 0, 0]
        self.Kd = [300000000, 300000000, 600]

       

        self.sample_time = 0.060  # in seconds

        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=16)
        self.lat_error_pub = rospy.Publisher('/lat_error',Float32,queue_size = 1)

        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.latitude_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.longitude_set_pid)
         

    def gps_callback(self, msg):
        self.gps_value[0] = msg.latitude
        self.gps_value[1] = msg.longitude
        self.gps_value[2] = msg.altitude

    def latitude_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 60000
        self.Ki[0] = roll.Ki * 0.008
        self.Kd[0] = roll.Kd * 3000000

    def longitude_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp 
        self.Ki[1] = pitch.Ki
        self.Kd[1] = pitch.Kd
    
    def pid(self):

        if(self.setpoint_gps_value[0]==19 and self.setpoint_gps_value[1]==72 and self.setpoint_gps_value[2]==0.31):
            rospy.sleep(1)
            self.setpoint_gps_value[2] = 3.0
        
           
        if(self.checkpoint_1==0 and self.gps_value[2]>2.99):
            rospy.sleep(2)
            self.setpoint_gps_value = [19.0000451704,72.0,3.0]   
            self.checkpoint_1 =1

        if(self.checkpoint_2==0 and self.gps_value[0]>19.00004517035):
            rospy.sleep(2)
            self.setpoint_gps_value = [19.0000451704,72.0,0.35]
            self.checkpoint_2=1
        
        
        self.error[0] = (self.setpoint_gps_value[0] - self.gps_value[0])
        self.error[1] = (self.setpoint_gps_value[1] - self.gps_value[1])
        self.error[2] = (self.setpoint_gps_value[2] - self.gps_value[2])

       
        self.errorI[0] += self.error[0]
        self.errorI[1] += self.error[1]
        self.errorI[2] += self.error[2]

        self.errorD[0] = self.error[0] - self.prev_error[0]
        self.errorD[1] = self.error[1] - self.prev_error[1]
        self.errorD[2] = self.error[2] - self.prev_error[2]
        
        self.prev_error[0]= self.error[0]
        self.prev_error[1]= self.error[1]
        self.prev_error[2]= self.error[2]

        self.output_latitude = (self.error[0]*self.Kp[0])+(self.errorI[0]*self.Ki[0])+(self.errorD[0]*self.Kd[0])
        self.output_longitude = (self.error[1]*self.Kp[1])+(self.errorI[1]*self.Ki[1])+(self.errorD[1]*self.Kd[1])
        self.output_altitude = (self.error[2]*self.Kp[2])+(self.errorI[2]*self.Ki[2])+(self.errorD[2]*self.Kd[2])

        self.command_value.rcThrottle = self.output_altitude

        self.command_value.rcRoll = 1500 + self.output_latitude  

        self.command_value.rcPitch = 1500 + self.output_longitude
     

        self.cmd_pub.publish(self.command_value)             
        
if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(16)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        try:
         e_drone.pid()
         r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException: pass
