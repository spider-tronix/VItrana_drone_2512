#!/usr/bin/env python

# Import the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int8
import rospy
import time
import tf
import math

class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node with name drone_control
        
        # initialize drone's latitude, longitude and altitude (start with origin values)
         # latitude, longitude and altitude values at origin
        self.current_position = [0,0,0]

        # initialize setpoint(destination) values of drone (latitude, longitude, altitude) as origin
         # latitude, longitude and altitude values at origin
        self.setpoint_position = [0,0,0]
        self.starting_position = [0,0,0]
         
        # variables those get updated to 1 when drones reaches desired position
           # checkpoint_1 updates when drone reaches almost 3m altitude
           # checkpoint_2 updates when dronee reaches almost 19.0000451704 latitude
        self.checkpoint_1 =0
        self.checkpoint_2 =0

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
        self.Kp = [33,33,60]
        self.Ki = [0, 0, 0]
        self.Kd = [2500,2500, 1300]

        self.obstacle_right = float('inf')
        self.obstacle_left = float('inf')
        self.obstacle_right = float('inf')
        self.obstacle_front = float('inf')
        
        # # This is the sample time in which you need to run pid
        self.sample_time = 0.060  # in seconds

        # publish /drone_command and /lat_error
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=16)
        self.lat_error_pub = rospy.Publisher('/lat_error',Float32,queue_size = 1)
        self.build_num_pub = rospy.Publisher('/build_num',Int8,queue_size=1)
        self.Z_m_pub = rospy.Publisher('/z_m',Float64,queue_size=10)

        self.check_pub = rospy.Publisher('check_4', Int8 , queue_size=1)
        self.error_x_pub = rospy.Publisher('err_x' , Float64 , queue_size=10)
        self.error_y_pub = rospy.Publisher('err_y' , Float64 , queue_size=10)



        self.following_wall =0
        self.moving_forward =0
        self.moving_left = 0  
        self.moving_right =0
        self.intact_to_wall = 0
        self.picked =0
        self.moving_vertically =0
        self.cruising =0
        self.count = 1
        self.check=0
        self.new_initial_setpoint = [0,0,0]
        self.multiplier = 6
        self.final_setpoint_position = [0,0,0]
        self.prev_lower_setpoint = 25
        self.checkpoint_1 =0
        self.checkpoint_2 =0
        self.checkpoint_3 =0
        self.checkpoint_4 =0
        self.X_marker=0
        self.Y_marker=0
        self.build_num=2
        self.is_reached =0


        # Subscribe to /edrone/gps
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/edrone/range_finder_top',LaserScan,self.range_finder_top_callback)
        rospy.Subscriber('/edrone/range_finder_bottom',LaserScan,self.range_finder_bottom_callback)
        rospy.Subscriber('/gripper_response',String , self.gripper_callback)
        rospy.Subscriber('/final_coordinates_x',Float32,self.x_callback)
        rospy.Subscriber('/final_coordinates_y',Float32,self.y_callback)
        rospy.Subscriber('/final_coordinates_z',Float32,self.z_callback)
        rospy.Subscriber('/x_marker',Float64 , self.x_marker_callback)
        rospy.Subscriber('/y_marker',Float64 , self.y_marker_callback)

        #rospy.Subscriber('/pid_tuning_roll',PidTune , self.set_roll_pid)


    #callbacks for getting location through QR code
    def x_callback(self,msg):
        self.final_setpoint_position[0]=self.lat_to_x(msg.data)

    def y_callback(self,msg):
        self.final_setpoint_position[1]=self.long_to_y(msg.data)
    
    def z_callback(self,msg):
        self.final_setpoint_position[2]=msg.data

    def x_marker_callback(self,msg):
        self.X_marker = msg.data

    def y_marker_callback(self,msg):
        self.Y_marker = msg.data
    
    def gripper_callback(self,msg):
        if(msg.data=='true'):
            self.picked=1
        else:
            self.picked=0



    def range_finder_top_callback(self,msg):
        self.obstacle_front = msg.ranges[0]
        self.obstacle_right = msg.ranges[1]
        self.obstacle_back = msg.ranges[2]
        self.obstacle_left = msg.ranges[3]

    def range_finder_bottom_callback(self,msg):
        self.obstacle_bottom = msg.ranges[0]


    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19)

    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)   
       # -----------------------------------------------call back function -------------------------------------------------------
    
    
    def set_roll_pid(self,roll):
        self.Kp[1] = roll.Kp*0.06
        self.Ki[1] = roll.Ki*0.00008
        self.Kd[1] = roll.Kd



    # gps call back function
    # this function gets executed each time when gps publishes /edrone/gps
    def gps_callback(self, msg):
        self.current_position[0] = self.lat_to_x(msg.latitude)
        self.current_position[1] = self.long_to_y(msg.longitude)
        self.current_position[2] = msg.altitude
               
       # --------------------------------------------------------------------------------------------------------------------------

    
    def pid(self):
       
        if(self.build_num==2):
            self.build_num_pub.publish(2)
            self.Z_m_pub.publish(10)
            self.z_m=10
            
        if(self.build_num==1):
            self.build_num_pub.publish(1)
            self.Z_m_pub.publish(9.25)
            self.z_m=9.25

        if(self.build_num==3):
            self.build_num_pub.publish(3)
            self.Z_m_pub.publish(9.3)
            self.z_m=9.3


        # Algorithm for navigation towards the buildings 
        # It reaches to a height of 25 meters then navigates to each building so o need to obstacle avoidance
        # After detection it continiously move to dercease that error
        # It is based on checkpoint if it reaches to a point algo updates to what to do next

        if(self.build_num==2):
            if(self.checkpoint_1==0):
                self.setpoint_position = [-84,19,25]
            
            if(self.current_position[2]>24.90 and self.checkpoint_1==0):
                self.setpoint_position=[-100,10,25]
                self.checkpoint_1=1

            if(self.current_position[0]<-99.94 and self.current_position[1]<10.03 and self.checkpoint_2==0 and self.checkpoint_1==1):
                self.Kp[2] =30
                self.setpoint_position = [-100,10,23.2]
                self.checkpoint_2=1

            if(self.current_position[2]<23.23 and self.checkpoint_3==0 and self.checkpoint_2==1):
                self.Kp[2] =60
                self.setpoint_position = [-96,16,32.2]
                self.checkpoint_3=1
                

            if(self.current_position[0]>-96.02 and self.checkpoint_3==1 and self.checkpoint_4==0): 
                self.focal_length = 238.350719
                print self.X_marker
                print self.Y_marker

                print 10*self.X_marker/self.focal_length
                print 10*self.Y_marker/self.focal_length

                self.setpoint_position[0] = 10*self.X_marker/self.focal_length + self.current_position[0]
                self.setpoint_position[1] = -(10*self.Y_marker/self.focal_length) + self.current_position[1]
                self.checkpoint_4=1

            if(self.checkpoint_4==1 and abs(self.setpoint_position[0]-self.current_position[0])<0.05 and abs(self.setpoint_position[1]-self.current_position[1])<0.05):
                self.build_num=1
                self.checkpoint_1=0
                self.checkpoint_2=0
                self.checkpoint_3=0
                self.checkpoint_4=0


        if(self.build_num==1):
            if(self.checkpoint_1==0):
                self.setpoint_position = [-100,-7,26]
                self.checkpoint_1 =1 

            if(self.current_position[1]<-6.99 and self.current_position[0]<-99.97 and self.checkpoint_2==0 and self.checkpoint_1==1):
                self.Kp[2]=30
                self.setpoint_position = [-100,-7,11.75]
                self.checkpoint_2=1


            if(self.current_position[2]<11.80 and self.checkpoint_3==0 and self.checkpoint_2==1):
                self.Kp[2] =60
                self.setpoint_position = [-103,-10,20]
                self.checkpoint_3=1

            if(self.current_position[0]<-102.97 and self.checkpoint_3==1 and self.checkpoint_4==0):
                self.focal_length = 238.350719
                print self.X_marker
                print self.Y_marker

                print 9.25*self.X_marker/self.focal_length
                print 9.25*self.Y_marker/self.focal_length

                self.setpoint_position[0] = 9.25*self.X_marker/self.focal_length + self.current_position[0]
                self.setpoint_position[1] = -(9.25*self.Y_marker/self.focal_length) + self.current_position[1]
                self.checkpoint_4=1

            if(self.checkpoint_4==1 and abs(self.setpoint_position[0]-self.current_position[0])<0.05 and abs(self.setpoint_position[1]-self.current_position[1])<0.05):
                self.build_num=3
                self.checkpoint_1=0
                self.checkpoint_2=0
                self.checkpoint_3=0
                self.checkpoint_4=0

        if(self.build_num==3):
            if(self.checkpoint_1==0):
                self.Kp[0] = 25
                self.setpoint_position = [-70,-6,20]
                self.checkpoint_1 =1 

            if(self.current_position[0]>-70.05 and self.checkpoint_2==0 and self.checkpoint_1==1):
                self.Kp[0] = 33
                self.Kp[2] = 30  
                self.setpoint_position = [-70,-6,11.70]
                self.checkpoint_2=1


            if(self.current_position[2]<11.75 and self.checkpoint_3==0 and self.checkpoint_2==1):
                self.Kp[2] =60
                self.setpoint_position = [-70,-2,20]
                self.checkpoint_3=1

            if(self.current_position[1]>-2.05 and self.checkpoint_3==1 and self.checkpoint_4==0):
                self.focal_length = 238.350719
                print self.X_marker
                print self.Y_marker

                print 9.3*self.X_marker/self.focal_length
                print 9.3*self.Y_marker/self.focal_length

                self.setpoint_position[0] = 9.3*self.X_marker/self.focal_length + self.current_position[0]
                self.setpoint_position[1] = -(9.3*self.Y_marker/self.focal_length) + self.current_position[1]
                self.checkpoint_4=1

            if(self.checkpoint_4==1 and abs(self.setpoint_position[0]-self.current_position[0])<0.05 and abs(self.setpoint_position[1]-self.current_position[1])<0.05):
                self.build_num=3
                self.is_reached=1
                
                
       

        if(self.is_reached==1):
            self.setpoint_position[2]=11.1
        '''
        if(self.checkpoint_4==1):
            self.focal_length = 238.350719
            print self.X_marker
            print self.Y_marker

            print 10*self.X_marker/self.focal_length
            print 10*self.Y_marker/self.focal_length

            self.setpoint_position[0] = self.z_m*self.X_marker/self.focal_length + self.current_position[0]
            self.setpoint_position[1] = -(self.z_m*self.Y_marker/self.focal_length) + self.current_position[1]

        '''


        self.check_pub.publish(self.checkpoint_4)
        print self.setpoint_position
        print self.current_position
        self.lat_error_pub.publish(self.error[0])

        
       
        self.error[0] = (self.setpoint_position[0] - self.current_position[0])
        self.error[1] = (self.setpoint_position[1] - self.current_position[1])
        self.error[2] = (self.setpoint_position[2] - self.current_position[2])

        self.error_x_pub.publish(self.error[0])
        self.error_y_pub.publish(self.error[1])

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
        self.command_value.rcPitch = 1500 - self.output_longitude
        
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
