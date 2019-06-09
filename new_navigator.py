#!/usr/bin/env python

#import statements:
import rospy
import math
import sys
import time

from mavros_msgs.msg import OpticalFlowRad #import optical flow message structure
from mavros_msgs.msg import State  #import state message structure
from sensor_msgs.msg import Range  #import range message structure
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose #import position message structures
from geometry_msgs.msg import TwistStamped #used to set velocity messages
from mavros_msgs.srv import *   #import for arm and flight mode setting
from tf.transformations import euler_from_quaternion

#global altitude
#global x
#global y
#global z

#altitude = 0.0
#x = 0.0
#y = 0.0

class velControl:
    def __init__(self, attPub):  #attPub = attitude publisher
        self._attPub = attPub
        self._setVelMsg = TwistStamped()
        self._targetVelX = 0
        self._targetVelY = 0
        self._targetVelZ = 0

    
    def setVel(self, coordinates):
        self._targetVelX = float(coordinates[0])
        self._targetVelY = float(coordinates[1])
        self._targetVelZ = float(coordinates[2])
        #rospy.logwarn("Target velocity is \nx: {} \ny: {} \nz: {}".format(self._targetVelX,self._targetVelY, self._targetVelZ))


    def publishTargetPose(self, stateManagerInstance):
        self._setVelMsg.header.stamp = rospy.Time.now()    #construct message to publish with time, loop count and id
        self._setVelMsg.header.seq = stateManagerInstance.getLoopCount()
        self._setVelMsg.header.frame_id = 'fcu'

        self._setVelMsg.twist.linear.x = self._targetVelX
        self._setVelMsg.twist.linear.y = self._targetVelY
        self._setVelMsg.twist.linear.z = self._targetVelZ
        
        self._attPub.publish(self._setVelMsg) 
        
        
        
        
        
class stateManager: #class for monitoring and changing state of the controller
    def __init__(self, rate):
        self._rate = rate
        self._loopCount = 0
        self._isConnected = 0
        self._isArmed = 0
        self._mode = None
    
    def incrementLoop(self):
        self._loopCount = self._loopCount + 1

    def getLoopCount(self):
        return self._loopCount

    def stateUpdate(self, msg):
        self._isConnected = msg.connected
        self._isArmed = msg.armed
        self._mode = msg.mode
        rospy.logwarn("Connected is {}, armed is {}, mode is {} ".format(self._isConnected, self._isArmed, self._mode)) #some status info

    def armRequest(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            modeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode) #get mode service and set to offboard control
            modeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("Service mode set faild with exception: %s"%e)
    
    def offboardRequest(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool) #get arm command service and arm
            arm(True)
        except rospy.ServiceException as e:   #except if failed
            print("Service arm failed with exception :%s"%e)


    def waitForPilotConnection(self):   #wait for connection to flight controller
        rospy.logwarn("Waiting for pilot connection")
        while not rospy.is_shutdown():  #while not shutting down
            if self._isConnected:   #if state isConnected is true
                rospy.logwarn("Pilot is connected")
                return True
            self._rate.sleep
        rospy.logwarn("ROS shutdown")
        return False


# PID controller Class used for setting DOF velocities 
class PID:
    def __init__(self, weights):
        self._weightKp = float(weights[0])
        self._weightKi = float(weights[1])
        self._weightKd = float(weights[2])

        self._error = 0
        self._integral = 0
        self._derivative = 0
        self._output = 0
        self._prevError = 0
        self._dt = 0

        self._iteration = 1

    def PIDcontroller(self, setPoint, processVariable):

        if self._iteration == 1:
            self._time = rospy.Time.now()
            self._iteration = self._iteration + 1
        else:
            self._dt = rospy.Time.now() - self._time
            self._dt = self._dt.to_nsec()*10**(-6)
            self._time = rospy.Time.now()
            self._error = float(setPoint) - float(processVariable)
            self._derivative = (self._error - self._prevError)/self._dt
            self._integral = (self._prevError + 0.5*self._derivative*self._dt)*self._dt
            self._output = self._weightKp*self._error + self._weightKi*self._integral + self._weightKd*self._derivative
            self._prevError = self._error
        return self._output


def distanceCheck(msg):
    global altitude    #import global range
    altitude = msg.range #set range = recieved range

    altitude = altitude*math.cos(pitch)

def PosCheck(msg):
    global x
    global y
    global z
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    global pitch

    #get pitch angle:
    quaternion = (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w)

    euler = euler_from_quaternion(quaternion)
    pitch = euler[1]

def callback(msg):
    global xflow
    global yflow
    global zflow
    xflow = msg.integrated_x
    yflow = msg.integrated_y
    zflow = msg.distance
    #rospy.loginfo("\n Position x: {} \n Optical Flow y: {} \n Optical Flow z: {} \n ---".format(x, y, z))

def main():
    rospy.init_node('navigator')   # make ros node
    


    rate = rospy.Rate(20) # rate will update publisher at 20hz, higher than the 2hz minimum before tieouts occur
    stateManagerInstance = stateManager(rate) #create new statemanager

    #Subscriptions
    rospy.Subscriber("/mavros/state", State, stateManagerInstance.stateUpdate)  #get autopilot state including arm state, connection status and mode
    rospy.Subscriber("/mavros/distance_sensor/hrlv_ez4_pub", Range, distanceCheck)  #get current distance from ground 
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, PosCheck)
    rospy.Subscriber("/mavros/px4flow/raw/optical_flow_rad", OpticalFlowRad, callback)  #subscribe to position messages
    
    global altitude #import global range variable
    global x # from local_position
    global y
    global z

    global xflow
    global yflow
    global zflow
    global travel
    global height

    global pitch

    global offboard_status

    x = 0.0
    y = 0.0
    z = 0.0
    altitude = 0.0
    xflow = 0.0
    yflow = 0.0
    zflow = 0.0
    
    offboard_status = False


    #Publishers
    velPub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=2) ###Change to atti


    controller = velControl(velPub) #create new controller class and pass in publisher and state manager
    stateManagerInstance.waitForPilotConnection()   #wait for connection to flight controller
   
    # PID controller for each degree of freedom
    # Kp, Ki, Kd
    xpositionControl = PID([0.1, 0.012,0.005])
    ypositionControl = PID([0.1, 0.047, 0.0038])    
    zpositionControl = PID([0.1, 0.0804, 0.00199])
    #yawControl = PID([1, 0, 0])

    
    # Navigation part of the code
    # Initialising logic statements to make sure drone is in correct trajectory stage 
    (J,K,L,M,N,Trigger) = (True,True,True,True,True,False)
    T = 1
  
    # Initialising variables for time measurement 
    (duration_1, duration_2) = (0,0)
    (t_initial_1, t_now_1, t_initial_2, t_now_2) = (0,0,0,0)

    # Target X-position
    Target = 6

    
    while not rospy.is_shutdown():

##########  DO NOT MODIFY ABOVE THIS LINE   ####################

#TAKEOFF
#------------------------------------------------------------------------------------------------------------------------------------
 
        if altitude < 1.5 and N == True:  
           xVelocity = xpositionControl.PIDcontroller(0, x)
           yVelocity = ypositionControl.PIDcontroller(0, y)
           zVelocity = zpositionControl.PIDcontroller(1.5, altitude)
           if zVelocity > 0.2:
	      zVelocity = 0.2

           Vcom = [0, 0, zVelocity]
           

#STABILISE FOR >1.5s
#-------------------------------------------------------------------------------------------------------------------------------------
	
	elif altitude > 1.5 and M == True:
             
           # This is used to calculate time hovering above take-off point to stabilise
           if T == 1:
                t_initial_1 = rospy.Time.now()
                T = T+1
           else:
                t_now_1 = rospy.Time.now()
                duration_1 = (t_now_1 - t_initial_1)
                duration_1 = duration_1.to_sec()

           rospy.loginfo("\n Duration_1 in seconds : {} ".format(duration_1))
           zVelocity = zpositionControl.PIDcontroller(1.5, altitude)
           xVelocity = xpositionControl.PIDcontroller(0, x)
           yVelocity = ypositionControl.PIDcontroller(0, y)
	   Vcom = [xVelocity,yVelocity,zVelocity]

           # Makes sure drone moves to next stage after a certain set duration (5 sec)
           if duration_1 > 5:
              N = False
              M = False 

	     			
#MOVING FORWARDS
#-------------------------------------------------------------------------------------------------------------------------------------     

	elif duration_1 > 5 and x < Target and J == True: 

	     
	     zVelocity = zpositionControl.PIDcontroller(1.5, altitude)
             # Setting speed limits on z-velocity
	     if zVelocity > 0.5:
		zVelocity = 0.5
	     elif zVelocity < -0.5:
		zVelocity = -0.5
	        
	     xVelocity = xpositionControl.PIDcontroller(Target, x)
             # Setting speed above ramp at lower velocity for stability
	     if 1 < x < 4.5 and xVelocity > 0.4:
	        xVelocity = 0.4
             # Setting speed limits on x-velocity
	     elif xVelocity > 0.5:
	        xVelocity = 0.5  

             yVelocity = ypositionControl.PIDcontroller(0, y)

             Vcom = [xVelocity,yVelocity,zVelocity]
	
	


# Stopping to Land
#-------------------------------------------------------------------------------------------------------------------------------------- 
           
	elif (x > (Target) and K == True) or Trigger == True:
             
            # Makes sure drone doesn't go back to previous stage once it has passed target
             J = False
	     Trigger = True

            # This is used to calculate time hovering above landing point to stabilise
             if T == 2:
                  t_initial_2 = rospy.Time.now()
                  T = T+1
             else:
                  t_now_2 = rospy.Time.now()
                  duration_2 = (t_now_2 - t_initial_2)
                  duration_2 = duration_2.to_sec()

             
             zVelocity = zpositionControl.PIDcontroller(1.5, altitude)
             xVelocity = xpositionControl.PIDcontroller(Target, x)
             # Setting speed limits on x-velocity
             if xVelocity > 0.5:
	        xVelocity = 0.5
             yVelocity = ypositionControl.PIDcontroller(0, y)
  
             Vcom = [xVelocity,yVelocity,zVelocity]

             # Makes sure drone goes to next stage after certain time has passed
             if duration_2 > 3:
                K = False
		Trigger = False
# Landing 
#--------------------------------------------------------------------------------------------------------------------------------------------
        elif Trigger == False:
	
             zVelocity = zpositionControl.PIDcontroller(0, altitude)
             xVelocity = xpositionControl.PIDcontroller(Target, x)
             yVelocity = ypositionControl.PIDcontroller(0, y)
  
             Vcom = [xVelocity, 0, -0.5]

             # Making sure drones lands at lower speed
             if altitude < 0.5:
		Vcom = [xVelocity, 0, -0.2]
	     print('landing on')
		
#===================================
				

##### DO NOT MODIFY BELOW THIS LINE ################

        controller.setVel(Vcom)
        controller.publishTargetPose(stateManagerInstance)
        stateManagerInstance.incrementLoop()
        rate.sleep()    #sleep at the set rate
        if ((stateManagerInstance.getLoopCount() > 100) & (offboard_status 
== False)):   #need to send some position data before we can switch to offboard mode otherwise offboard is rejected
            stateManagerInstance.offboardRequest()  #request control from external computer
            stateManagerInstance.armRequest()   #arming must take place after offboard is requested
            offboard_status = True
    rospy.spin()    #keeps python from exiting until this node is stopped
    


if __name__ == '__main__':
    main()



