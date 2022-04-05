#!/usr/bin/env python
from __future__ import print_function
from geometry_msgs.msg import Twist
from balboa_core.msg import balboaMotorSpeeds
from balboa_core.msg import balboaLL
import rospy

pubMotor = rospy.Publisher('/motorSpeeds', balboaMotorSpeeds, queue_size = 10)
motor = balboaMotorSpeeds()

targetAngle = 0
targetDistance = 0

pValAng = 0.0001
dValAng = 0.00000001
pValDis = 0.015
dValDis = 0.0001

# pValAng = rospy.get_param('/pidLab1/pValAng')
# dValAng = rospy.get_param('/pidLab1/dValAng')
# pValDis = rospy.get_param('/pidLab1/pValDis')
# dValDis = rospy.get_param('/pidLab1/dValDis')


initFlag = 0
initIMU = balboaLL()



def callbackIMU(data):
    global initFlag
    global targetAngle
    global targetDistance
    global oldAngError
    global oldDisError
    
    # if(initFlag == 0):
    #     initIMU = data
    #     initFlag = 1
    #     targetAngle = initIMU.angleX
    #     targetDistance = initIMU.distanceRight

    angError = (targetAngle - data.angleX)
    disError = (targetDistance - data.distanceRight)

    angPidVal = (pValAng * angError) + (dValAng * (angError - oldAngError)) 
    disPidVal = (pValDis * disError) + (dValDis * (disError - oldDisError))
    
    motor.right = angPidVal + disPidVal
    motor.left = -angPidVal + disPidVal

    oldAngError = angError
    oldDisError = disError
def 



