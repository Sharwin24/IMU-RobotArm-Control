#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The IMUAnalyzer is a ROS node that subscribes to the IMU data and computes the angles for
each link of the robot. It then publishes the angles to the robot controller (Arduino) via serial.
"""

import rospy
from robotic_arm_control.msg import Vectornav
from std_msgs.msg import String
import numpy as np
from RobotArm import RobotArm
from typing import List
import math


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
		return self.buffer

	def getAverage(self) -> float:
		try:
			return np.mean(self.buffer)
		except:
			return 0

	def getLatest(self) -> float:
		try:
			return self.buffer[-1]
		except IndexError:
			return 0
	
	def getOldest(self) -> float:
		try:
			return self.buffer[0]
		except IndexError:
			return 0


class IMUPubSub:
	def __init__(self, arduinoPort, arduinoBaud):
		self.elbowIMUSub = rospy.Subscriber("/elbow_imu", Vectornav, self.elbowImuCallback)
		self.shoulderIMUSub = rospy.Subscriber("/shoulder_imu", Vectornav, self.shoulderImuCallback)
		self.arduinoCmdPub = rospy.Publisher('/arduino_command', String, queue_size=10)
		rospy.loginfo("IMU PubSub initialized")
		self.robotarm = RobotArm(arduinoPort, arduinoBaud)
  	# The zero angles for each link, which are calibrated on initialization or during the first moments of operation
		self.zeroAngles: dict[int, float] = {1: 0, 2: 0, 3: 0}  # [degrees]
		# Buffers for each link's target angle and velocity
		self.linkAngleBuffers: dict[int, IMUBuffer] = {1: IMUBuffer(), 2: IMUBuffer(), 3: IMUBuffer()}
		self.linkVelocityBuffers: dict[int, IMUBuffer] = {1: IMUBuffer(10), 2: IMUBuffer(10), 3: IMUBuffer(10)}
		self.runningRobotControl = False
		self.useVariableSpeed = False

	def calibrateZeroAngles(self, calibration_length: float = 2):
		"""Calibrates the zero angles for each link by averaging the angles over the given period of time.
		Args:
			calibration_length (float, optional): The length of time to calibrate the zero angles [sec], defaults to 2
		"""
		# Starts a timer to keep track of the calibration period and initializes the lists of angles
		# The IMU data is automatically updated in the callback functions to the IMUBuffers for each link
		rospy.loginfo("Calibrating zero angles...")
		start_time = rospy.get_time()
		while rospy.get_time() - start_time < calibration_length:
			# While we are calibrating, yield to other tasks
			rospy.sleep(0.01)  # [sec]
		# Once the calibration period is over, we average the angles in the buffers to get the zero angles
		for linkNumber in self.linkAngleBuffers:
			self.zeroAngles[linkNumber] = self.linkAngleBuffers[linkNumber].getAverage()
		rospy.loginfo("Calibration complete -> Zero angles: {}".format(self.zeroAngles))

	def quat2eul(self, orientation) -> float:
		"""Converts orientation object from a given quaternion measured in radians to a z-axis angle in degrees.
		"""
		w = orientation.w
		x = orientation.x
		y = orientation.y
		z = orientation.z
  
		t0 = 2.0 * (w * z + x * y)
		t1 = 1.0 - 2.0 * (y * y + z * z)
		yaw_z = (180.0/math.pi) * math.atan2(t0, t1)
		return float(yaw_z)

	def elbowImuCallback(self, data):
		"""Handles the input elbow IMU data and computes the angle for link 2.
			Each link should have a "zero" angle which is calibrated on initialization or during
			the first moments of operation. The angles should be relative to the "zero" angle.

			This function should only update the IMUBuffers for link 2.
			The elbow IMU rotates on the Z axis.

		Args:
				data (Vectornav): The IMU data
		"""

		relAngle = float(self.quat2eul(data.imu.orientation) - self.zeroAngles[2])
		self.linkAngleBuffers[2].update(relAngle)

		# assume all link starting velocities are 0
		self.linkVelocityBuffers[2].update(data.imu.angular_velocity.z)
		
	def shoulderImuCallback(self, data):
		"""Handles the input shoulder IMU data and computes the angle for link 1.
			Each link should have a "zero" angle which is calibrated on initialization or during
			the first moments of operation. The angles should be relative to the "zero" angle.

			This function should only update the IMUBuffers for link 1.
			The shoulder IMU rotates on the Z axis.

		Args:
				data (Vectornav): The IMU data
		"""
		relAngle = float(self.quat2eul(data.imu.orientation) - self.zeroAngles[1])
		self.linkAngleBuffers[1].update(relAngle)

		# assume all link starting velocities are 0
		self.linkVelocityBuffers[1].update(data.imu.angular_velocity.z)
  
	def validateTargetAngle(self, linkNumber: int, angle: float) -> bool:
		"""Validates the given target angle for the given link number

		Args:
				linkNumber (int): the link number
				angle (float): the target angle [degrees]
		"""
		# Switch on the link number
		match linkNumber: 
			case 1:
				# Link 1 should be between -90 and 90 degrees
				return -90 <= angle <= 90
			case 2:
				# Link 2 should be between -90 and 90 degrees
				return -90 <= angle <= 90
			case 3:
				return True
			case _:
				# If the link number is not 1, 2, or 3, then it is invalid
				return False

	def shouldContinueControl(self) -> bool:
		"""Determines if the control loop should continue running. This is used to stop the control loop.

		Returns:
				bool: True if the control loop should continue running, False otherwise
		"""

  	# TODO: Implement this function as we find out what aspects we run into that require stopping the control loop
		return True 

	def runRobotArmControl(self):
		"""Runs the Robot Arm Control loop. This is expected to run after calibration of the zero angles.
			The Robot Arm Control loop obtains the latest target angles from the IMU buffers and then 
			sends the commands to the robot controller (Arduino) via serial.
   
			This control loop should also validate the values as much as possible before sending them to the Arduino.
			The Arduino also runs validation which will reject commands when:
			 - The target angle is close to the current angle
			 - The robot arm is currently moving
		"""
		self.runRobotArmControl = True
		while (self.runRobotArmControl):
    	# Get the latest target angles from the IMU buffers
			link1TargetAngle = self.linkAngleBuffers[1].getLatest() - self.zeroAngles[1]
			link2TargetAngle = self.linkAngleBuffers[2].getLatest() - self.zeroAngles[2]
			link3TargetAngle = self.linkAngleBuffers[3].getLatest() - self.zeroAngles[3]
			# Get the most recent angular velocity from the IMU buffers
			link1Velocity = self.linkVelocityBuffers[1].getAverage()
			link2Velocity = self.linkVelocityBuffers[2].getAverage()
			link3Velocity = self.linkVelocityBuffers[3].getAverage()
			# Validate the target angles
			link1Validation = self.validateTargetAngle(1, link1TargetAngle)
			link2Validation = self.validateTargetAngle(2, link2TargetAngle)
			link3Validation = self.validateTargetAngle(3, link3TargetAngle)
			if link1Validation and link2Validation and link3Validation:
				# Send the target angles to the Arduino if they are valid
				if self.useVariableSpeed:
					self.robotarm.setSpeed(1, link1Velocity)
					self.robotarm.setSpeed(2, link2Velocity)
					self.robotarm.setSpeed(3, link3Velocity)
				self.robotarm.forwardKinematics(link1TargetAngle, link2TargetAngle, link3TargetAngle)
			# Yield to other tasks for a short (but not too short) period of time and determine if control should continue
			rospy.sleep(0.1)  # [sec]
			self.runRobotArmControl = self.shouldContinueControl()

if __name__ == '__main__':
	try:
		rospy.init_node('imuAnalyzer', anonymous=True)
		IMUAnalyzer = IMUPubSub(arduinoPort="/dev/ttyACM0", arduinoBaud=9600)
		IMUAnalyzer.calibrateZeroAngles()
		IMUAnalyzer.runRobotArmControl()
		rospy.spin()
	except rospy.ROSInterruptException:
		print("ROS Interrupt Exception, exiting...")
