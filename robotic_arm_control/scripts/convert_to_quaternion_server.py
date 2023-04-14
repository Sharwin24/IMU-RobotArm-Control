#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from navigation.srv import convert_to_quaternion, convert_to_quaternionResponse
import numpy as np

def euler2Quaternion(roll, pitch, yaw):
	"""Given yaw, pitch, roll in radians, returns the corresponding quaternion as a tuple (qx, qy, qz, qw)

	Args:
			yaw (float): The yaw angle (Z-Axis) [rad]
			pitch (float): The pitch angle (Y-Axis) [rad]
			roll (float): The roll angle (X-Axis) [rad]

	Returns:
			Tuple[float,float,float,float]: The quaternion as a tuple (qx, qy, qz, qw)
	"""
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	return (qx, qy, qz, qw)

def handle_convert_to_quaternion(req):
  # print("[Euler XYZ (%f,%f,%f) -> Quaternion XYZW (%.3f,%.2f,%.3f,%.3f)]"%(req.eulerX, req.eulerY, req.eulerZ, *euler2Quaternion(req.eulerX, req.eulerY, req.eulerZ)))
  (qx, qy, qz, qw) = euler2Quaternion(req.eulerX, req.eulerY, req.eulerZ)
  return convert_to_quaternionResponse(quaternionX=qx, quaternionY=qy, quaternionZ=qz, quaternionW=qw)

def convert_to_quaternion_server():
	rospy.init_node('convert_to_quaternion_server')
	s = rospy.Service('convert_to_quaternion', convert_to_quaternion, handle_convert_to_quaternion)
	# print("Ready to convert euler angles to quaternion.")
	rospy.spin()

if __name__ == "__main__":
	convert_to_quaternion_server()