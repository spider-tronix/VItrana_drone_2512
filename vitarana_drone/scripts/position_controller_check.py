#!/usr/bin/env python

# Import the required libraries

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
        rospy.init_node('position_controller')  # initializing ros node with name drone_control
        
        # initialize drone's latitude, longitude and altitude (start with origin values)
         # latitude, longitude and altitude values at origin
        self.current_position = [19.0000271037, 71.9999999998, 0.31]

        # initialize setpoint(destination) values of drone (latitude, longitude, altitude) as origin
         # latitude, longitude and altitude values at origin
        self.setpoint_position = [19.0000271037, 71.9999999998, 0.31]
        
        # variables those get updated to 1 when drones reaches desired position
           # checkpoint_1 updates when drone reaches almost 3m altitude
           # checkpoint_2 updates when dronee reaches almost 19.0000451704 latitude
        self.checkpoint_1 =0
        self.checkpoint_2 =0
        self.checkpoint_3=0

        # Declaring command_value of message type edrone_cmd and initializing values
        self.command_value = edrone_cmd()
        self.command_value.rcRoll = 0.0
        self.command_value.rcPitch = 0.0
        self.command_value.rcYaw = 0.0
        self.command_value.rcThrottle = 0.0

         # ---------------------------adding other variables for pid here -------------------------------------------------

        self.error = [0,0,0]       # initialize current errors in position(gps values) w.r.t setpoints
        self.errorI = [0,0,0]      # initialize sum of errors (for integral) for latitude, longitude and altitude
        self.errorD = [0,0,0]      # initialize change in error (for derivative) for latitude, longitude and altitude
        self.prev_error = [0,0,0]  # initialize errors in position w.r.t setpoints in previous cycle of pid

         # initial setting of Kp, Kd and ki for [latitude, longitude, altitude]
        # after tuning and computing corresponding PID parameters
        self.Kp = [6000000, 6000000,15.2]
        self.Ki = [0, 0, 0]
        self.Kd = [300000000, 300000000, 1000]

       
        # # This is the sample time in which you need to run pid
        self.sample_time = 0.060  # in seconds

        # publish /drone_command and /lat_error
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=16)
        self.lat_error_pub = rospy.Publisher('/lat_error',Float32,queue_size = 1)

        # Subscribe to /edrone/gps
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        
         
       
       # -----------------------------------------------call back function -------------------------------------------------------
    
    
    # gps call back function
    # this function gets executed each time when gps publishes /edrone/gps
    def gps_callback(self, msg):
        self.current_position[0] = msg.latitude
        self.current_position[1] = msg.longitude
        self.current_position[2] = msg.altitude
               
       # --------------------------------------------------------------------------------------------------------------------------

    
    def pid(self):
        # writing PID algorithm for latitude, longitude and altitude
       
    ##### -------------------------------- logic for problem statement -----------------------------------------------------------#####
        '''
        checkpoints are basically that checks whether it reaches to that setpoint(1 or 2) or not 
        '''

        # set setpoint of (altitude as 3m)
        if(self.setpoint_position[0]==19.0000271037 and self.setpoint_position[1]==71.9999999998 and self.setpoint_position[2]==0.31):
            rospy.sleep(1)
            self.setpoint_position[2] = 3.0
        
        
        # set setpoint of latitude to 19.0000451704       
        if(self.checkpoint_1==0 and self.current_position[2]>2.99):
            rospy.sleep(1)
            self.setpoint_position = [19.0,71.9999999998,3.0]   
            self.checkpoint_1 =1

        
        # set setpoint of altitude to 0.31    
        if(self.checkpoint_2==0 and self.current_position[0]<19.00000004):
            rospy.sleep(1)
            self.setpoint_position = [19.0,71.9999999998,0.31]
            self.checkpoint_2=1

        if(self.checkpoint_2==1 and self.checkpoint_3==0 and self.current_position[2]<0.315):
            rospy.sleep(1)
            self.setpoint_position = [19.0,71.9999999998,25]
            self.checkpoint_3=1

        if(self.current_position[2]>24):
            self.setpoint_position[2] = 8.44

        

       
           


        

    ##### -----------------------------------------------------------------------------------------------------------------------#####   
        
        # Computing errors(for proportional) for latitide, longitude and altitude
        self.error[0] = (self.setpoint_position[0] - self.current_position[0])
        self.error[1] = (self.setpoint_position[1] - self.current_position[1])
        self.error[2] = (self.setpoint_position[2] - self.current_position[2])

        # Compute sum of errors (for integral) for latitide, longitude and altitude
        self.errorI[0] += self.error[0]
        self.errorI[1] += self.error[1]
        self.errorI[2] += self.error[2]
        
        # Compute change in errors (for derivative) for latitude, longitude and altitude
        self.errorD[0] = self.error[0] - self.prev_error[0]
        self.errorD[1] = self.error[1] - self.prev_error[1]
        self.errorD[2] = self.error[2] - self.prev_error[2]
        
        # Calculate the pid output required for latitude, longitude and altitude
        self.output_latitude = (self.error[0]*self.Kp[0])+(self.errorI[0]*self.Ki[0])+(self.errorD[0]*self.Kd[0])
        self.output_longitude = (self.error[1]*self.Kp[1])+(self.errorI[1]*self.Ki[1])+(self.errorD[1]*self.Kd[1])
        self.output_altitude = (self.error[2]*self.Kp[2])+(self.errorI[2]*self.Ki[2])+(self.errorD[2]*self.Kd[2])

        # Use this computed output value in the equations to compute the roll, pitch and throttle forr drone
        self.command_value.rcThrottle = self.output_altitude
        self.command_value.rcRoll = 1500 + self.output_latitude  
        self.command_value.rcPitch = 1500 + self.output_longitude
        
        #updating previous error of latitude, longitude and altitude
        self.prev_error[0]= self.error[0]
        self.prev_error[1]= self.error[1]
        self.prev_error[2]= self.error[2]
        
        # publishing the roll, pitch and throttle values ( which will be subscribed by attitude controller )
        self.cmd_pub.publish(self.command_value)             



   



   
if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(16)  # specify rate in Hz based upon your desired PID sampling time
    while not rospy.is_shutdown():      # run pid() until rospy is shutdown
        try:
         e_drone.pid()                 # execute pid equation
         r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException: pass    #return error is anything goees wwrong
