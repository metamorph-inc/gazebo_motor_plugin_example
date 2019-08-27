#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: simple_state_machine.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/05/2017
# Edit Date: 10/05/2017
#
# Description:
# Simple state machine controlling the position of an robot arm
#
# Attribution:
# Code examples from
# Programming Robots with ROS by Morgan Quigley, Brian Gerkey, and William D. Smart
# (O'Reilly), Chapter 13. Copyright 2015 Morgan Quigley, Brian Gerkey, and William D. Smart,
# 978-1-4493-2389-9.
'''

import rospy
from smach import State, StateMachine

from std_msgs.msg import Float32
import time, math

# Global objects - they're not pretty...
g_pub = None  # ros publisher
g_rate = None  # ros rate
g_prev_position = None
g_prev_target = None

# Helper function to calculate shortest distance/path along circumference of circle
def calculate_target_joint_position(prev_position, prev_target, target_position):
    targetYRad = None
    prevOrientationYRad = prev_position
    prevTargetYRad = prev_target
    orientationYRad = target_position

    if (prevTargetYRad >= 0.0 and prevTargetYRad <= math.pi):  # less than 180 degrees counterclockwise from origin - more accurate
        if (orientationYRad < 0.0):
            if ((math.pi - abs(orientationYRad)) < prevOrientationYRad):
                targetYRad = 2*math.pi + orientationYRad  # counterclockwise rotation
            else:
                targetYRad = orientationYRad  # clockwise rotation
        else:  # (orientationYRad >= 0.0)
            targetYRad = orientationYRad  # counter/counterclockwise rotation

    elif (prevTargetYRad >= -math.pi and prevTargetYRad < 0.0):  # less then 180 degrees clockwise from origin - more accurate
        if (orientationYRad > 0.0):
            if ((math.pi - orientationYRad) < abs(prevOrientationYRad)):
                targetYRad = -2*math.pi + orientationYRad  # clockwise rotation
            else:
                targetYRad = orientationYRad  # counterclockwise rotation
        else:  # (orientationYRad <= 0.0)
            targetYRad = orientationYRad  # counter/counterclockwise rotation

    else:  # more than 180 degrees clockwise/counterclockwise from origin
        if (prevOrientationYRad >= 0.0):
            if (orientationYRad < 0.0):
                if ((math.pi - abs(orientationYRad)) < prevOrientationYRad):
                    targetYRad = prevTargetYRad - prevOrientationYRad + 2*math.pi + orientationYRad  # counterclockwise rotation
                else:
                    targetYRad = prevTargetYRad - prevOrientationYRad + orientationYRad  # clockwise rotation
            else:  # (orientationYRad >= 0.0)
                targetYRad = prevTargetYRad - prevOrientationYRad + orientationYRad  # counter/counterclockwise rotation
        else:  # (prevOrientationYRad < 0.0)
            if (orientationYRad > 0.0):
                if ((math.pi - orientationYRad) < abs(prevOrientationYRad)):
                    targetYRad = prevTargetYRad - prevOrientationYRad - 2*math.pi + orientationYRad  # clockwise rotation
                else:
                    targetYRad = prevTargetYRad - prevOrientationYRad + orientationYRad  # counterclockwise rotation
            else:  # (orientationYRad <= 0.0)
                targetYRad = prevTargetYRad - prevOrientationYRad + orientationYRad  # counter/counterclockwise rotation
    return targetYRad

class MoveArm(State):
    def __init__(self, targetPosition, moveTime):
        State.__init__(self, outcomes=['success'])
        self.targetPosition = targetPosition
        self.moveTime = moveTime

    def execute(self, userdata):
        print("Moving to " + str(self.targetPosition))

        # Calculate target joint position
        global g_prev_position, g_prev_target
        targetRotation = calculate_target_joint_position(g_prev_position, g_prev_target, self.targetPosition)
        g_prev_position = self.targetPosition
        g_prev_target = targetRotation

        curTime = time.time()
        endTime = curTime + self.moveTime
        while curTime < endTime:
            g_pub.publish(targetRotation)
            g_rate.sleep()
            curTime = time.time()

        print("Move command published for " + str(self.moveTime))
        return 'success'

if __name__ == '__main__':
    # ROS stuff
    rospy.init_node('simple_state_machine')
    g_pub = rospy.Publisher('/boilerplate_motor_plugin_test/pos_cmd', Float32, queue_size=1)
    g_rate = rospy.Rate(100)  # 100hz

    # smach stuff
    clockwise = StateMachine(outcomes=['success'])
    with clockwise:
        StateMachine.add('3', MoveArm(0.0, 1.0), transitions={'success':'4'})
        StateMachine.add('4', MoveArm(-0.3926991, 1.0), transitions={'success':'5'})
        StateMachine.add('5', MoveArm(-1.178097, 1.0), transitions={'success':'6'})
        StateMachine.add('6', MoveArm(-1.5708, 1.0), transitions={'success':'7'})
        StateMachine.add('7', MoveArm(-1.9634954, 1.0), transitions={'success':'8'})
        StateMachine.add('8', MoveArm(-2.7488936, 1.0), transitions={'success':'9'})
        StateMachine.add('9', MoveArm(3.14159, 1.0), transitions={'success':'10'})
        StateMachine.add('10', MoveArm(2.7488936, 1.0), transitions={'success':'11'})
        StateMachine.add('11', MoveArm(1.9634954, 1.0), transitions={'success':'12'})
        StateMachine.add('12', MoveArm(1.5708, 1.0), transitions={'success':'1'})
        StateMachine.add('1', MoveArm(1.178097, 1.0), transitions={'success':'2'})
        StateMachine.add('2', MoveArm(0.3926991, 1.0), transitions={'success':'3f'})
        StateMachine.add('3f', MoveArm(0.0, 1.0), transitions={'success':'success'})

    counterclockwise = StateMachine(outcomes=['success'])
    with counterclockwise:
        StateMachine.add('2', MoveArm(0.3926991, 1.0), transitions={'success':'1'})
        StateMachine.add('1', MoveArm(1.178097, 1.0), transitions={'success':'12'})
        StateMachine.add('12', MoveArm(1.5708, 1.0), transitions={'success':'11'})
        StateMachine.add('11', MoveArm(1.9634954, 1.0), transitions={'success':'10'})
        StateMachine.add('10', MoveArm(2.7488936, 1.0), transitions={'success':'9'})
        StateMachine.add('9', MoveArm(3.14159, 1.0), transitions={'success':'8'})
        StateMachine.add('8', MoveArm(-2.7488936, 1.0), transitions={'success':'7'})
        StateMachine.add('7', MoveArm(-1.9634954, 1.0), transitions={'success':'6'})
        StateMachine.add('6', MoveArm(-1.5708, 1.0), transitions={'success':'5'})
        StateMachine.add('5', MoveArm(-1.178097, 1.0), transitions={'success':'4'})
        StateMachine.add('4', MoveArm(-0.3926991, 1.0), transitions={'success':'3'})
        StateMachine.add('3', MoveArm(0.0, 1.0), transitions={'success':'success'})

    compasspoints = StateMachine(outcomes=['success'])
    with compasspoints:
        StateMachine.add('NORTH', MoveArm(1.5708, 3.0), transitions={'success':'SOUTHWEST'})
        StateMachine.add('SOUTHWEST', MoveArm(-2.35619, 3.0), transitions={'success':'EAST'})
        StateMachine.add('EAST', MoveArm(0.0, 3.0), transitions={'success':'success'})

    coolpattern = StateMachine(outcomes=['success'])
    with coolpattern:
        g_prev_position = 0.0  # initial position [radians]
        g_prev_target = 0.0  # initial target [radians]
        StateMachine.add('CLOCKWISE', clockwise, transitions={'success':'COUNTERCLOCKWISE1'})
        StateMachine.add('COUNTERCLOCKWISE1', counterclockwise, transitions={'success':'COUNTERCLOCKWISE2'})
        StateMachine.add('COUNTERCLOCKWISE2', counterclockwise, transitions={'success':'COMPASSPOINTS'})
        StateMachine.add('COMPASSPOINTS', compasspoints, transitions={'success':'success'})

    # Give Gazebo some time to start up
    user_input = raw_input("Please press the 'Return/Enter' key to start executing 'coolpattern'\n")

    print("Input received. Executing 'coolpattern'...")
    coolpattern.execute()
    rospy.spin()
