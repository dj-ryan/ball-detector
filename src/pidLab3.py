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
#ball = ballLocation()

oldAngleError = 0
oldDistanceError = 0
angleRunningAvg = 0
distanceRunningAvg = 0

#initFlag = 0

# Ball follower function    
def callbackTarget(ball):
    global oldAngleError
    global oldDistanceError
    global angleRunningAvg
    global distanceRunningAvg

#    global initFlag

    # Start with zero motor speed initially
 #   if(initFlag == 0):
 #       initIMU = ball
 #       initFlag = 1
 #       targetAngle = 0
 #       targetDistance = 140

    # PID VALUES
    pAngle = 0.015
    dAngle = 0.05
    pDistance = 0.07
    dDistance = 0.01

    # target distance
    targetDistance = 130
    
    # Calculate error
    angleError = ball.x
    distanceError = ball.radius - targetDistance
    
    # PID EQUATIONS
    anglePIDValue = (pAngle * angleError) + (dAngle * (angleError - oldAngleError)) 
    distancePIDValue = (pDistance * distanceError) + (dDistance * (distanceError - oldDistanceError))

    angleRunningAvg = (angleRunningAvg * 0.9) + (anglePIDValue * 0.1)
    distanceRunningAvg = (distanceRunningAvg * 0.9) + (distancePIDValue * 0.1)


    # Save old error values for derivative pid equation
    oldAngleError = angleError
    oldDistanceError = distanceError

    # Assign motor speeds
    #motor.left = distancePIDValue + anglePIDValue
    #motor.right = distancePIDValue - anglePIDValue

    motor.left = distanceRunningAvg + angleRunningAvg
    motor.right = distanceRunningAvg - angleRunningAvg

    # debug output
    rospy.loginfo("-------------------------")
    rospy.loginfo("Radius: %d", ball.radius)
    rospy.loginfo("x: %d", ball.x)
    rospy.loginfo("left motor: %f", motor.left)
    rospy.loginfo("right motor: %f", motor.right)

def mainFunction(): 
    rospy.init_node('pidLab3')
    rospy.loginfo("STARTED")
    rospy.Subscriber('/ballLocation',ballLocation,callbackTarget)
    rate = rospy.Rate(10)   

    # Continously publish motor speeds based on PID
    while not rospy.is_shutdown():
        pubMotor.publish(motor)
        motor.left = 0
        motor.right = 0
        rate.sleep()    

if __name__ == '__main__':
    try:
        mainFunction()
    except rospy.ROSInterruptException:
        pass

