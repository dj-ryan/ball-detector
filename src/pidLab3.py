#!/usr/bin/env python
from __future__ import print_function
from ball_detector.msg import ballLocation
from balboa_core.msg import balboaMotorSpeeds
from balboa_core.msg import balboaLL
import rospy

# Must start camera & detect ball to run PID 

# Declare variables 
pubMotor = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size = 10)
motor = balboaMotorSpeeds()

targetAngle = 0
targetDistance = 0

# PID constants
pValAng = 0.0001
dValAng = 0.00000001
pValDis = 0.015
dValDis = 0.0001

# We are using hard-coded constants, not from a launch file
# pValAng = rospy.get_param('/pidLab1/pValAng')
# dValAng = rospy.get_param('/pidLab1/dValAng')
# pValDis = rospy.get_param('/pidLab1/pValDis')
# dValDis = rospy.get_param('/pidLab1/dValDis')

# initFlag = 0
initIMU = balboaLL()

# PID function based on robot data (/balboaLL)
def callbackIMU(data):
    global initFlag
    global targetAngle
    global targetDistance
    global oldAngError
    global oldDisError
    
    # We do not take our initial first position to feed as the target, we start directly from the camera info
    # if(initFlag == 0):
    #     initIMU = data
    #     initFlag = 1
    #     targetAngle = initIMU.angleX
    #     targetDistance = initIMU.distanceRight

    # PID Error calculations for both angle and distance
    angError = (targetAngle - data.angleX)
    disError = (targetDistance - data.distanceRight)

    # PID value calculations for both angle and distance
    angPidVal = (pValAng * angError) + (dValAng * (angError - oldAngError)) 
    disPidVal = (pValDis * disError) + (dValDis * (disError - oldDisError))
    
    # Updated commanded motors values based on angle and distance PID values 
    motor.right = angPidVal + disPidVal
    motor.left = -angPidVal + disPidVal

    # Set derivative values 
    oldAngError = angError
    oldDisError = disError

# Ball follower function    
def callbackTarget(ballInfo):

    rospy.loginfo("Radius: %d", ballInfo.radius)

    #     if (ballLocation.x > 0) {

    #     targetAngle = -ballLocation.x; 
    # }
    
    # else if (ballLocation.x < 0) {

    #     targetAngle = abs(ballLocation.x);
    # }

    # else if (ballLocation.radius > 500) {

    #     targetDistance = -100;
    # }
    
    # else if (ballLocation.radius < 500 {

    #     targetDistance = 100;
    # }

def main(): 
    rospy.init_node('pidLab3')
    rospy.loginfo("STARTED")
    rospy.Subscriber('/ballLocation',ballLocation,callbackTarget)
    rate = rospy.Rate(10)   

    # Continously publish motor speeds based on PID
    while not rospy.is_shutdown():
        pubMotor.publish(motor)
        rate.sleep()    



