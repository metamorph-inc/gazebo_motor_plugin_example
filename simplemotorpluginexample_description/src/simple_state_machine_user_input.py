#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: simple_state_machine_user_input.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/05/2017
# Edit Date: 10/05/2017
#
# Description:
# Simple state machine controlling the position of an robot arm using
# user keyboard input to transition between states.
#
# Attribution:
# Code examples from
# Programming Robots with ROS by Morgan Quigley, Brian Gerkey, and William D. Smart
# (O'Reilly), Chapter 13. Copyright 2015 Morgan Quigley, Brian Gerkey, and William D. Smart,
# 978-1-4493-2389-9.
'''

import rospy
from smach import State, StateMachine
import smach_ros

from std_msgs.msg import Float32, String
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
    def __init__(self, targetPosition, moveTime, subTopic, msgType):
        State.__init__(self, outcomes=['forward', 'reverse', 'exit'])
        self.targetPosition = targetPosition
        self.moveTime = moveTime
        self.subTopic = subTopic
        self.msgType = msgType
        self.keyMapping = { 's': 'hold', 'a': 'reverse', 'd': 'forward', 'x': 'exit'}
        self.nextState = 'hold'  # hold, forward, reverse

    def execute(self, userdata):
        self.sub = rospy.Subscriber(self.subTopic, self.msgType, self.callback)
        print("Moving to " + str(self.targetPosition))

        # Calculate target joint position
        global g_prev_position, g_prev_target
        targetRotation = calculate_target_joint_position(g_prev_position, g_prev_target, self.targetPosition)
        g_prev_position = self.targetPosition
        g_prev_target = targetRotation

        while True:
            time.sleep(self.moveTime)
            if (self.nextState == 'hold'):
                self.nextState = 'hold'
                g_pub.publish(targetRotation)
                g_rate.sleep()
            else:  # self.nextState == 'reverse', 'forward', or 'exit'
                self.sub.unregister()  # unregister from topic when not active (overhead of creating Subscriber every time node executes vs duplicate callback functions)
                returnState = self.nextState
                self.nextState = 'hold'  # reset this state in case we return to it later
                return returnState

    def callback(self, msg):
        if len(msg.data) == 0 or not msg.data[0] in self.keyMapping:
            return  # unknown key
        print("callback: setting nextState")
        self.nextState = self.keyMapping[msg.data[0]]

if __name__ == '__main__':
    # ROS stuff
    rospy.init_node('simple_state_machine')

    # Publisher
    g_pub = rospy.Publisher('/boilerplate_motor_plugin_test/pos_cmd', Float32, queue_size=1)
    g_rate = rospy.Rate(100)  # 100hz

    # smach stuff
    turn = StateMachine(outcomes=['success'])
    with turn:
        g_prev_position = 0.0  # initial position [radians]
        g_prev_target = 0.0  # initial target [radians]
        StateMachine.add('3',  MoveArm(0.0       , 0.1, 'keys', String), transitions={'forward':'4',  'reverse':'2',  'exit':'success'})
        StateMachine.add('4',  MoveArm(-0.3926991, 0.1, 'keys', String), transitions={'forward':'5',  'reverse':'3',  'exit':'success'})
        StateMachine.add('5',  MoveArm(-1.178097 , 0.1, 'keys', String), transitions={'forward':'6',  'reverse':'4',  'exit':'success'})
        StateMachine.add('6',  MoveArm(-1.5708   , 0.1, 'keys', String), transitions={'forward':'7',  'reverse':'5',  'exit':'success'})
        StateMachine.add('7',  MoveArm(-1.9634954, 0.1, 'keys', String), transitions={'forward':'8',  'reverse':'6',  'exit':'success'})
        StateMachine.add('8',  MoveArm(-2.7488936, 0.1, 'keys', String), transitions={'forward':'9',  'reverse':'7',  'exit':'success'})
        StateMachine.add('9',  MoveArm(3.14159   , 0.1, 'keys', String), transitions={'forward':'10', 'reverse':'8',  'exit':'success'})
        StateMachine.add('10', MoveArm(2.7488936 , 0.1, 'keys', String), transitions={'forward':'11', 'reverse':'9',  'exit':'success'})
        StateMachine.add('11', MoveArm(1.9634954 , 0.1, 'keys', String), transitions={'forward':'12', 'reverse':'10', 'exit':'success'})
        StateMachine.add('12', MoveArm(1.5708    , 0.1, 'keys', String), transitions={'forward':'1',  'reverse':'11', 'exit':'success'})
        StateMachine.add('1',  MoveArm(1.178097  , 0.1, 'keys', String), transitions={'forward':'2',  'reverse':'12', 'exit':'success'})
        StateMachine.add('2',  MoveArm(0.3926991 , 0.1, 'keys', String), transitions={'forward':'3',  'reverse':'1',  'exit':'success'})

        ## if we wanted to be cute about this, we could...
        #  for i in range(0, 12):
        #      StateMachine.add(str((i+3)%12) ...

    # Create and start the introspection server - for visualization / debugging
    sis = smach_ros.IntrospectionServer('simple_state_machine_user_input', turn, '/SM_ROOT')
    sis.start()

    # Give Gazebo some time to start up
    user_input = raw_input("Please press the 'Return/Enter' key to start executing 'turn'\n")

    # Execute the state machine
    print("Input received. Executing 'turn'...")
    turn.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
