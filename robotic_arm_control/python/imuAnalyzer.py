#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The IMUController is a ROS node that subscribes to the IMU data and computes the angles for
each link of the robot. It then publishes the angles to the robot controller via rosserial.
"""

import rospy
from robotic_arm_control.msg import ArduinoCommand, Vectornav
import numpy as np

# The length of the calibration period, which will assign the "zero" values [ms]
calibration_length = 5000 # [ms]

# The zero angles for each link, which are calibrated on initialization or during the first moments of operation
link1ZeroAngle = 0
link2ZeroAngle = 0

# Lists of the target angles for each link, the latest angle is the last element in the list
link1TargetAngles = []
link2TargetAngles = []

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
	rospy.Subscriber("elbow_imu", Vectornav, elbowImuCallback)
	rospy.Subscriber("shoulder_imu", Vectornav, shoulderImuCallback)

def imuPublisher():
	rospy.Publisher('arduino_command', ArduinoCommand, queue_size=10)

def main():
	rospy.init_node('imuAnalyzer', anonymous=True)
	imuListener()
	imuPublisher()
	rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass