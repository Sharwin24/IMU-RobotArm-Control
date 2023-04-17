#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The IMUController is a ROS node that subscribes to the IMU data and computes the angles for
each link of the robot. It then publishes the angles to the robot controller via rosserial.
"""

import rospy
from robotic_arm_control.msg import ArduinoCommand, Vectornav
import numpy as np
from typing import Tuple

# The length of the calibration period, which will assign the "zero" values [ms]
calibration_length = 5000 # [ms]

# The zero angles for each link, which are calibrated on initialization or during the first moments of operation
zeroAngles = {1: 0, 2: 0, 3: 0} # [degrees]

# Lists of the target angles for each link, the latest angle is the last element in the list
targetAngles = {1: [], 2: [], 3: []} # [degrees]

class IMUPubSub:
	def __init__(self):
		rospy.init_node('imuAnalyzer', anonymous=True)
		self.elbowIMUSub = rospy.Subscriber("elbow_imu", Vectornav, self.elbowImuCallback)
		self.shoulderIMUSub = rospy.Subscriber("shoulder_imu", Vectornav, self.shoulderImuCallback)
		self.arduinoCmdPub = rospy.Publisher('arduino_command', ArduinoCommand, queue_size=5)
		
	def elbowImuCallback(self, data):
		"""Handles the input elbow IMU data and computes the angle for link 2.
			Each link should have a "zero" angle which is calibrated on initialization or during
			the first moments of operation. The angles should be relative to the "zero" angle,
			and then will be published/dispatched to the robot controller (Arduino) via rosserial.

		Args:
				data (Vectornav): The IMU data
		"""
	pass

	def shoulderImuCallback(self, data):
		"""Handles the input shoulder IMU data and computes the angle for link 1.
			Each link should have a "zero" angle which is calibrated on initialization or during
			the first moments of operation. The angles should be relative to the "zero" angle,
			and then will be published/dispatched to the robot controller (Arduino) via rosserial.

		Args:
				data (Vectornav): The IMU data
		"""
	pass

	def publishToArduino(self, linkAngles: Tuple[float]):
		"""Given the angles for each link as a tuple, this function publishes the angles to the
			robot controller (Arduino) via rosserial to the topic "arduino_command" using the
			ArduinoCommand message type via the arduinoCmdPub publisher.

		Args:
				linkAngles (Tuple[float]): A tuple of the angles for each link [link1, link2, link3] in degrees
		"""
		# Validate input Tuple, must be length 3 and contain floats
		if len(linkAngles) != 3:
			raise ValueError("linkAngles must be a tuple of length 3")
		elif not isinstance(linkAngles[0], float) or not isinstance(linkAngles[1], float) or not isinstance(linkAngles[2], float):
			raise ValueError("linkAngles must be a tuple of floats")
		arduinoCmd = ArduinoCommand()
		arduinoCmd.link1Angle = linkAngles[0]
		arduinoCmd.link2Angle = linkAngles[1]
		arduinoCmd.link3Angle = linkAngles[2]
		self.arduinoCmdPub.publish(arduinoCmd)

def main():
	"""The main function of the IMUAnalyzer node"""
	IMUAnalyzer = IMUPubSub()
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print("ROS Interrupt Exception, exiting...")