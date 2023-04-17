#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The IMUController is a ROS node that subscribes to the IMU data and computes the angles for
each link of the robot. It then publishes the angles to the robot controller via rosserial.
"""

import rospy
from robotic_arm_control.msg import Vectornav
from std_msgs.msg import String
import numpy as np
import sys

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
		self.arduinoCmdPub = rospy.Publisher('arduino_command', String, queue_size=5)
		
	def elbowImuCallback(self, data):
		"""Handles the input elbow IMU data and computes the angle for link 2.
			Each link should have a "zero" angle which is calibrated on initialization or during
			the first moments of operation. The angles should be relative to the "zero" angle,
			and then will be published/dispatched to the robot controller (Arduino) via rosserial.

		Args:
				data (Vectornav): The IMU data
		"""
		rospy.logdebug("Elbow IMU data received")
		self.publishToArduino("0,0,0")

	def shoulderImuCallback(self, data):
		"""Handles the input shoulder IMU data and computes the angle for link 1.
			Each link should have a "zero" angle which is calibrated on initialization or during
			the first moments of operation. The angles should be relative to the "zero" angle,
			and then will be published/dispatched to the robot controller (Arduino) via rosserial.

		Args:
				data (Vectornav): The IMU data
		"""
		rospy.logdebug("Shoulder IMU data received")
		self.publishToArduino("1,1,1")

	def publishToArduino(self, linkAngles: String):
		"""Given the angles for each link as a tuple, this function publishes the angles to the
			robot controller (Arduino) via rosserial to the topic "arduino_command" using the
			ArduinoCommand message type via the arduinoCmdPub publisher.

		Args:
				linkAngles (String): A string of the angles for each link in degrees with the format
					"link1Angle,link2Angle,link3Angle"
		"""
		anglesList = linkAngles.split(",")
		arduinoCmd = String()
		arduinoCmd.data = f"{anglesList[0]},{anglesList[1]},{anglesList[2]}"
		rospy.logdebug(f"Publishing to Arduino: {arduinoCmd.data}")
		self.arduinoCmdPub.publish(arduinoCmd)

def main():
	"""The main function of the IMUAnalyzer node"""
	IMUAnalyzer = IMUPubSub()
	rospy.spin()

if __name__ == '__main__':
	# args = rospy.myargv(argv=sys.argv)
	# arduinoPort = args[1]
	try:
		main()
	except rospy.ROSInterruptException:
		print("ROS Interrupt Exception, exiting...")