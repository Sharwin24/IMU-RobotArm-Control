#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The IMUController is a ROS node that subscribes to the IMU data and computes the angles for
each link of the robot. It then publishes the angles to the robot controller via rosserial.
"""

import rospy
from robotic_arm_control.msg import ArduinoCommand, Vectornav

def elbowImuCallback(data):
	"""Handles the input elbow IMU data and computes the angle for link 2.
		Each link should have a "zero" angle which is calibrated on initialization or during
   	the first moments of operation. The angles should be relative to the "zero" angle,
    and then will be published/dispatched to the robot controller (Arduino) via rosserial.

	Args:
			data (Vectornav): The IMU data
	"""
	pass


def shoulderImuCallback(data):
	"""Handles the input shoulder IMU data and computes the angle for link 1.
		Each link should have a "zero" angle which is calibrated on initialization or during
   	the first moments of operation. The angles should be relative to the "zero" angle,
    and then will be published/dispatched to the robot controller (Arduino) via rosserial.

	Args:
			data (Vectornav): The IMU data
	"""
	pass

def imuListener():
	rospy.init_node('imuListener', anonymous=True)
	rospy.Subscriber("elbow_imu", Vectornav, elbowImuCallback)
	rospy.Subscriber("shoulder_imu", Vectornav, shoulderImuCallback)
	rospy.spin()

if __name__ == '__main__':
	imuListener()