#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /roll_error             /pid_tuning_altitude
        /pitch_error            /pid_tuning_pitch
        /yaw_error              /pid_tuning_roll
        /edrone/pwm             /edrone/imu/data
                                /edrone/drone_command

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf

PI = 3.1415926535897

class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('attitude_controller')  # initializing ros node with name drone_control
        
        
        # This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        self.drone_orientation_euler_deg = [0.0,0.0,0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [1500.0, 1500.0, 1500.0]
        self.setpoint_throttle =0

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]


        #self.position_controller_active checks whether position controller is active or not : if not then gives 0 as pwm values
        self.position_controller_active =0 

        # Declaring pwm_cmd of message type prop_speed and initializing values
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
        # after tuning and computing corresponding PID parameters
        self.Kp = [0.3, 0.3, 46.0]
        self.Ki = [0, 0, 0]
        self.Kd = [7.5, 7.5, 420]
        
        # -----------------------Add other required variables for pid here ----------------------------------------------
        
        self.prev_errors = [0,0,0]                  #error from IMU data w.r.t setpoints in previous cycle of pid
        self.max_values = [1024,1024,1024,1024]     #maximum permisiable value for PWM output
        self.min_values = [0,0,0,0]                 #minimum permisiable value for PWM output
        self.errorI = [0,0,0]                       #error from integration branch
        self.errorD = [0,0,0]                       #error from derivative branch
        self.error=[0,0,0]                          #present error of IMU data w.r.t setpoints

        

        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [roll, pitch, yaw]
        #        Add variables for limiting the values like self.max_values = [1024, 1024, 1024, 1024] corresponding to [prop1, prop2, prop3, prop4]
        #                                                   self.min_values = [0, 0, 0, 0] corresponding to [prop1, prop2, prop3, prop4]
        #
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds

        

        # Publishing /edrone/pwm
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        
        # -----------------------------------------------------------------------------------------------------------

        
        # Subscribing to /drone_command
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        # Subscribing to /imu/data
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        
        




        # ------------------------------------------------------------------------------------------------------------

    # Imu callback function
    # The function gets executed each time when imu publishes /edrone/imu/data

    # Note: The imu publishes various kind of data viz angular velocity, linear acceleration, magnetometer reading (if present),
    # but here we are interested in the orientation which can be calculated by a complex algorithm called filtering which is not in the scope of this task,
    # so for your ease, we have the orientation published directly BUT in quaternion format and not in euler angles.
    # We need to convert the quaternion format to euler angles format to understand the orienataion of the edrone in an easy manner.
    # Hint: To know the message structure of sensor_msgs/Imu, execute the following command in the terminal
    # rosmsg show sensor_msg/Imu

    def imu_callback(self, msg):
       
        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w
        

        # -------------------------------------------------------------------------------------------------------------

    

    # Callbackfunction for /drone_command
    # This function gets executed each time when /drone_command is published i.e setpoints for roll,piych,yaw and throttle
        
    def drone_command_callback(self, msg):
        self.position_controller_active = 1
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.setpoint_throttle  = msg.rcThrottle    


    # ----------------------------------------------------------------------------------------------------------------------
    
    def pid(self):
        
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

        # Steps:
        #   1. Convert the quaternion format of orientation to euler angles
        #   2. Convert the setpoin that is in the range of 1000 to 2000 into angles with the limit from -10 degree to 10 degree in euler angles
        #   3. Compute error in each axis. eg: error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0], where error[0] corresponds to error in roll...
        #   4. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
        #   5. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
        #   6. Use this computed output value in the equations to compute the pwm for each propeller. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
        #   7. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
        #   8. Limit the output value and the final command value between the maximum(0) and minimum(1024)range before publishing. For eg : if self.pwm_cmd.prop1 > self.max_values[1]:
        #                                                                                                                                      self.pwm_cmd.prop1 = self.max_values[1]
        #   8. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
        #   9. Add error_sum to use for integral component

        
        
         # fix yaw at 1500 ( 0 degree) since we wont be working with it
        

        # Convert the quaternion format of orientation to euler angles
        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll, pitch and yaw axes
        self.setpoint_cmd[2] = 1500
        self.setpoint_euler[0] = self.setpoint_cmd[0] * 0.02 - 30
        self.setpoint_euler[1] = self.setpoint_cmd[1] * 0.02 - 30
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30

        # converting the range of 1000 to 2000 to 0 to 1024 for throttle here itslef
        #converting throttle value
        
        
        # converting euler angles(interms of pi) to degrees
        self.drone_orientation_euler_deg[1] = ( self.drone_orientation_euler[0]/PI )*180
        self.drone_orientation_euler_deg[0] = ( self.drone_orientation_euler[1]/PI )*180
        self.drone_orientation_euler_deg[2] = ( self.drone_orientation_euler[2]/PI )*180
        

        # Computing error(for proportional) in each axis
        self.error[0] = self.setpoint_euler[0] - self.drone_orientation_euler_deg[0]
        self.error[1] = self.setpoint_euler[1] - self.drone_orientation_euler_deg[1]
        self.error[2] = self.setpoint_euler[2] - self.drone_orientation_euler_deg[2]

        if(self.error[0] > 0.7):
            self.error[0] = 0.7

        if(self.error[1] > 0.7):
            self.error[1] = 0.7

        if(self.error[2] > 0.7):
            self.error[2] = 0.7


        if(self.error[0] < -0.7):
            self.error[0] = -0.7

        if(self.error[1] < -0.7):
            self.error[1] = -0.7

        if(self.error[2] < -0.7):
            self.error[2] = -0.7

        



        
        # Compute change in error (for derivative) in each axis
        self.errorD[0] = self.error[0] - self.prev_errors[0]
        self.errorD[1] = self.error[1] - self.prev_errors[1]
        self.errorD[2] = self.error[2] - self.prev_errors[2]
        
        
        # Compute sum of errors (for integral) in each axis
        self.errorI[0] = self.errorI[0] + self.error[0]
        self.errorI[1] = self.errorI[1] + self.error[1]
        self.errorI[2] = self.errorI[2] + self.error[2]
        
        
        # Calculate the pid output required for each axis and throttle
        self.out_roll = self.Kp[0]*self.error[0] + self.Kd[0]*self.errorD[0] + self.Ki[0]*self.errorI[0]
        self.out_pitch = self.Kp[1]*self.error[1] + self.Kd[1]*self.errorD[1] + self.Ki[1]*self.errorI[1]
        self.out_yaw = self.Kp[2]*self.error[2] + self.Kd[2]*self.errorD[2] + self.Ki[2]*self.errorI[2]

        # throttle setpointt is the pid output of altitude, so it becomes direct output with second pid
        self.out_throttle = self.setpoint_throttle
        

        # Use this computed output value in the equations to compute the pwm for each propeller
        # drones thrust will be equal to gravitational force at 508.9 PWM input for propellors (FOUND WITH TRAIL AND ERROR)
        if(self.position_controller_active==1):
            self.pwm_cmd.prop1 = 508.9 + self.out_throttle - self.out_roll + self.out_pitch - self.out_yaw
            self.pwm_cmd.prop2 = 508.9 + self.out_throttle - self.out_roll - self.out_pitch + self.out_yaw
            self.pwm_cmd.prop3 = 508.9 + self.out_throttle + self.out_roll - self.out_pitch - self.out_yaw
            self.pwm_cmd.prop4 = 508.9 + self.out_throttle + self.out_roll + self.out_pitch + self.out_yaw

        # Limit the output value and the final command value between the maximum(0) and minimum(1024)range before publishing
        if self.pwm_cmd.prop1 > self.max_values[0]:
            self.pwm_cmd.prop1 = self.max_values[0]

        if self.pwm_cmd.prop2 > self.max_values[1]:
            self.pwm_cmd.prop2 = self.max_values[1]

        if self.pwm_cmd.prop3 > self.max_values[2]:
            self.pwm_cmd.prop3 = self.max_values[2]

        if self.pwm_cmd.prop4 > self.max_values[3]:
            self.pwm_cmd.prop4 = self.max_values[3]
        


        #updating previous error of roll, pitch, yaw
        self.prev_errors[0]=self.error[0]
        self.prev_errors[1]=self.error[1]
        self.prev_errors[2]=self.error[2]

        
        # publish the PWM values for four propellors
        self.pwm_pub.publish(self.pwm_cmd)


if __name__ == '__main__':
    
    e_drone = Edrone() 
    
    r = rospy.Rate(16)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():      # run pid() until rospy is shutdown
        try:
         e_drone.pid()      # execute pid equation
         r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException: pass  #return error is anything goees wwrong
        
        
    