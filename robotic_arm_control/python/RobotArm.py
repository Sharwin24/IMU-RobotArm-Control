#usr/bin/env python3
from Arduino import Arduino

class RobotArm(Arduino):
	def __init__(self, port: str, baud=9600):
		super().__init__(port, baud)
  
	def read_handler(self, command, value):
		if command == "STATE":
			with self.stateLock:
				self.state = value
    
	def forwardKinematics(self, link1Angle, link2Angle, link3Angle):
		self.sendCommand("fk", [link1Angle, link2Angle, link3Angle])