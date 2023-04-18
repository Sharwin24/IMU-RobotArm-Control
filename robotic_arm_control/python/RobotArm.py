#usr/bin/env python3
from Arduino import Arduino

class RobotArm(Arduino):
	def __init__(self, port: str, baud=9600):
		super().__init__(port, baud)
  
	def read_handler(self, command, value):
		if command == "STATE":
			with self.stateLock:
				self.state = value
    
	def forwardKinematics(self, link1Angle: float, link2Angle: float, link3Angle: float):
		"""Sends the forward kinematics command to the Arduino

		Args:
				link1Angle (float): Angle of the first link in [degrees]
				link2Angle (float): Angle of the second link in [degrees]
				link3Angle (float): Angle of the third link in [degrees]
    """
		self.sendCommand("fk", [link1Angle, link2Angle, link3Angle])
	
	def setSpeed(self, linkNumber: int, speed: float):
		"""Sets the speed of the specified link to the given speed. If the given speed is zero, ignore the command

		Args:
				linkNumber (int): link number
				speed (float): Speed of the link in RPM
		"""
		if speed != 0:
			self.sendCommand("speed", [linkNumber, speed])