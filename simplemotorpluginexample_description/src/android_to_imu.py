#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: android_to_imu.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/03/2017
# Edit Date: 10/03/2017
#
# Description:
# Connects to port on localhost and receives Android sensor data.
# Source: https://www.dropbox.com/s/zgxrrje17hhn8vj/androidSensor123.py
'''

import rospy
from std_msgs.msg import Float32
import socket, traceback, math
#import tf_conversions  # Really should switch to tf2, but I haven't figured out how to use the new library yet...

# global variables
host = ""  # or "localhost"
port = 8000

if __name__ == '__main__':

    rospy.init_node('android_to_imu')
    pos_cmd_pub = rospy.Publisher('/boilerplate_motor_plugin_test/pos_cmd', Float32, queue_size=1)  # TODO: change to class? https://answers.ros.org/question/62327/how-to-create-a-combining-subscriber-and-publisher-node-in-python/?answer=62424#post-id-62424
    rate = rospy.Rate(100)  # 100 Hz

    firstMessage = True
    prevOrientationYRad = None
    targetYRad = None
    prevTargetYRad = None

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((host, port))
    print("Successful socket binding to port " + str(port))
    while not rospy.is_shutdown():
        try:
            message, address = s.recvfrom(8192)
            messageString = message.decode("utf-8")

            orientationY = float(messageString.split(",")[-2])

            # Convert from degrees to radians
            orientationYRad = orientationY * math.pi / 180.0

            if firstMessage:
                prevOrientationYRad = orientationYRad
                targetYRad = orientationYRad
                prevTargetYRad = orientationYRad
                firstMessage = False
            else:
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

                print("orientationY = " + str(orientationY) + " [degrees] | targetY = " + str(targetYRad * 180 / math.pi) + "[degrees]")
                prevOrientationYRad = orientationYRad
                prevTargetYRad = targetYRad

            pos_cmd_pub.publish(targetYRad)

            ## I've had enough of this quaternion nonsense
            # # Extract the x, y, and z components of the unit quaternion
            # rotationVector = messageString.split(",")[-3:]
            # rotationVector = [float(i) for i in rotationVector]
            #
            # # Calculate the w component of the unit quaternion
            # w = math.sqrt(abs(1.0 - rotationVector[0]**2 + rotationVector[1]**2 + rotationVector[2]**2))  # abs() prevents domain-errors caused by rounding
            # rotationVector.insert(0, w)
            #
            # # Convert from quaternion to euler
            # quaternion = (
            #     rotationVector[0],
            #     rotationVector[1],
            #     rotationVector[2],
            #     rotationVector[3])
            #
            # euler = tf_conversions.transformations.euler_from_quaternion(quaternion, 'szxy')  # [roll, pitch, yaw]
            # yaw = euler[1]  # measure phone yaw (in Android coordinate system) [radians]
            # print("roll: " + str(euler[0]) + "| pitch: " + str(euler[1]) + "| yaw: " + str(euler[2]))

        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            traceback.print_exc()

        rate.sleep()
