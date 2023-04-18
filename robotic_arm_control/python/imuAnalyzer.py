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
from RobotArm import RobotArm
from typing import List


class IMUBuffer:
	"""Class that holds a Buffer for the IMU Data. The IMU Data is held in this buffer which only contains
		 up to a certain number of data points. The buffer is updated externally but the length of the buffer
	   is maintained internally.
    
		 The data for this buffer is a list of floats representing IMU Angles in degrees.
    
	Attributes:	
			size (int): The size of the buffer, which is the maximum number of data points
			buffer (List[float]): The buffer of IMU data
	"""
	def __init__(self, size: int = 50):
		self.size = size
		self.buffer = []
  
	def update(self, data: float):
		"""Updates the buffer with a new data point.
		"""
		self.buffer.append(data)
		if len(self.buffer) > self.size:
			self.buffer.pop(0)

	def getBuffer(self) -> List[float]:
		"""Returns the buffer.
		"""
		return self.buffer

	def getAverage(self) -> float:
		return np.mean(self.buffer)

	def getLatest(self) -> float:
		return self.buffer[-1]


class IMUPubSub:
	def __init__(self, arduinoPort, arduinoBaud):
		self.elbowIMUSub = rospy.Subscriber("/elbow_imu", Vectornav, self.elbowImuCallback)
		self.shoulderIMUSub = rospy.Subscriber("/shoulder_imu", Vectornav, self.shoulderImuCallback)
		self.arduinoCmdPub = rospy.Publisher('/arduino_command', String, queue_size=10)
		rospy.loginfo("IMU PubSub initialized")
		self.robotarm = RobotArm(arduinoPort, arduinoBaud)
  	# The zero angles for each link, which are calibrated on initialization or during the first moments of operation
		self.zeroAngles = {1: 0, 2: 0, 3: 0}  # [degrees]
		# Buffers for each link's target angle so we can store a list of angles
		self.link1TargetAngleBuffer = IMUBuffer()
		self.link2TargetAngleBuffer = IMUBuffer()
		self.link3TargetAngleBuffer = IMUBuffer()
  
	def calibrateZeroAngles(self, calibration_length: float = 2):
		"""Calibrates the zero angles for each link by averaging the angles over the given period of time.

		Args:
				calibration_length (float, optional): The length of time to calibrate the zero angles [sec], defaults to 2
  	"""
		# Starts a timer to keep track of the calibration period and initializes the lists of angles
		# The IMU data is automatically updated in the callback functions to the IMUBuffers for each link
		start_time = rospy.get_time()
		while rospy.get_time() - start_time <= calibration_length:
			pass
		# Once the calibration period is over, we average the angles in the buffers to get the zero angles
		self.zeroAngles[1] = self.link1TargetAngleBuffer.getAverage()
		self.zeroAngles[2] = self.link2TargetAngleBuffer.getAverage()
		self.zeroAngles[3] = self.link3TargetAngleBuffer.getAverage()

	
		
	def elbowImuCallback(self, data):
		"""Handles the input elbow IMU data and computes the angle for link 2.
			Each link should have a "zero" angle which is calibrated on initialization or during
			the first moments of operation. The angles should be relative to the "zero" angle,
			and then will be published/dispatched to the robot controller (Arduino) via rosserial.

		Args:
				data (Vectornav): The IMU data
		"""
		# self.robotarm.forwardKinematics(45,30,15)
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


if __name__ == '__main__':
	# args = rospy.myargv(argv=sys.argv)
	# arduinoPort = args[1]
	try:
		rospy.init_node('imuAnalyzer', anonymous=True)
		IMUAnalyzer = IMUPubSub(arduinoPort="/dev/ttyACM0", arduinoBaud=9600)
		rospy.spin()
	except rospy.ROSInterruptException:
		print("ROS Interrupt Exception, exiting...")