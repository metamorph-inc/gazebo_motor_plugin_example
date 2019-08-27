#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: imu_to_pos.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 10/03/2017
# Edit Date: 10/03/2017
#
# Description:
# Subscribes to imu data published on the /gyro topic and publishes
# roll in radians to the /gazebo_client/pos_cmd topic
'''

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import tf_conversions  # Really should switch to tf2, but I haven't figured out how to use the new library yet...

# global variable
g_pos_cmd_pub = None

# callback function is called every time a new message is published onto the /gyro topic
def callback(data):
    # Extract /gyro topic orientation data (published in quaternion form)
    quaternion = (
        data.orientation.w,
        data.orientation.x,
        data.orientation.y,
        data.orientation.z)

    # Convert quaternion to [roll, pitch, yaw]
    euler = tf_conversions.transformations.euler_from_quaternion(quaternion, 'sxyz')
    roll = euler[0]  # measured roll (radians)
    g_pos_cmd_pub.publish(roll)

def listener():
    rospy.init_node('imu_to_pos')
    rospy.Subscriber('gyro', Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    g_pos_cmd_pub = rospy.Publisher('/boilerplate_motor_plugin_test/pos_cmd', Float32, queue_size=1)  # TODO: change to class? https://answers.ros.org/question/62327/how-to-create-a-combining-subscriber-and-publisher-node-in-python/?answer=62424#post-id-62424
    listener()
