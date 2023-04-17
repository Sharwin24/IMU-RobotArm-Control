#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import sys
from robotic_arm_control.msg import Vectornav
from robotic_arm_control.srv import convert_to_quaternion
import numpy as np

# Debug Flag will print intermediate debugging statements
DEBUG = False

# $VNYMR,
# -165.964, # Yaw
# -037.285, # Pitch
# +001.249, # Roll
# +00.2880, # Mag X
# +00.0749, # Mag Y
# +00.7428, # Mag Z
# -05.966, # Accel X
# -00.169, # Accel Y
# -07.846, # Accel Z
# -00.000235, # Gyro X
# +00.000263, # Gyro Y
# -00.000486*63 # Gyro Z

def convert_to_quaternion_client(roll, pitch, yaw):
  rospy.wait_for_service('convert_to_quaternion')
  if DEBUG:
    print("convert_to_quaternion service found")
  try:
    euler2Quaternion = rospy.ServiceProxy('convert_to_quaternion', convert_to_quaternion)
    response = euler2Quaternion(roll, pitch, yaw)
    return (response.quaternionX, response.quaternionY, response.quaternionZ, response.quaternionW)
  except rospy.ServiceException as e:
    print("Service call failed: %s" % e)


def configure_vectornav(serialPort, frequency=40):
    """Configure the Vectornav IMU to output data at the given frequency [Hz].
    This can be done by writing a string to the register of the VectorNav IMU.

    Args:
        serialPort (serial.Serial): A serial.Serial object to write to
        frequency (int): the frequency to output data [Hz], default is 40Hz
    """
    write_string = "$VNWRG,07," + str(frequency) + "*XX\r"
    serialPort.write(bytes(write_string, 'utf-8'))

def serial_port_setup(default_port='/dev/pts/5', default_baud=115200, default_sampling_rate=1.0):
    """Setup serial port with defaults

    Returns:
        serial.Serial: A serial.Serial object
    """
    SERIAL_PORT = rospy.get_param('~port', default_port)
    BAUD_RATE = rospy.get_param('~baud', default_baud)
    SAMPLING_RATE = rospy.get_param('~sampling_rate', default_sampling_rate)
    print(f'Starting Elbow IMU Driver: \nSerial Port: {SERIAL_PORT}\nBaud Rate: {BAUD_RATE}\nSampling Rate: {SAMPLING_RATE}')
    port = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SAMPLING_RATE / 2.0)
    configure_vectornav(port)
    return port


def convert_to_float(value):
    """Converts a string from the message to a float"""
    sign = value[0]
    contains_checksum = value.__contains__('*')
    if (contains_checksum):
        value = value[1:value.find('*')]
    if (sign == '-'):
        return -1 * float(value[1:])
    elif (sign == '+'):
        return float(value[1:])
    else:
        try:
            return float(value)
        except:
            return 0

def parse(line: str, header ="VNYMR") -> Vectornav:
    """Parses a line from the serial port into a GNSS message

    Args:
        line (str): A line from the serial port

    Return
        Vectornav
    """
    if line is None or line == "":
        return None
    decoded_line = line.decode('utf-8').strip()
    dataList = decoded_line.split(',')
    if DEBUG:
        print("dataList -> ", dataList)
    if len(dataList) == 0:
        print("msg is empty")
        return None
    # Parse data and copy to local variables
    log_header = dataList[0]
    if not log_header.__contains__(header):
        if DEBUG:
            print("Log header is not" + header)
            return ""
    # Read in data
    if len(dataList) == 1 and dataList[0] == "":
        if DEBUG:
            print("No data in message")
        return None
    try: 
        yaw = convert_to_float(dataList[1])
        pitch = convert_to_float(dataList[2])
        roll = convert_to_float(dataList[3])
        magX = convert_to_float(dataList[4]) / 10e5 # Convert to Teslas from Gauss
        magY = convert_to_float(dataList[5]) / 10e5 # Convert to Teslas from Gauss
        magZ = convert_to_float(dataList[6]) / 10e5 # Convert to Teslas from Gauss
        accelX = convert_to_float(dataList[7])
        accelY = convert_to_float(dataList[8])
        accelZ = convert_to_float(dataList[9])
        gyroX = convert_to_float(dataList[10])
        gyroY = convert_to_float(dataList[11])
        gyroZ = convert_to_float(dataList[12])
    except:
        if DEBUG:
            print("Error parsing data, skipping this data point")
        return None
    # Process data
    # (qx, qy, qz, qw) = euler2Quaternion(roll, pitch, yaw)
    if DEBUG:
        print("Using service to convert to quaternion")
    # Convert RPY to radians
    roll = np.deg2rad(roll)
    pitch = np.deg2rad(pitch)
    yaw = np.deg2rad(yaw)
    (qx, qy, qz, qw) = convert_to_quaternion_client(roll, pitch, yaw)
    if DEBUG:
        print("qx, qy, qz, qw -> ", qx, qy, qz, qw)
    # Create IMU message and populate fields
    now = rospy.get_rostime()
    msg = Vectornav()
    msg.Vectornav.frame_id = "imu1_frame"
    msg.Vectornav.stamp.secs = now.secs
    msg.Vectornav.stamp.nsecs = now.nsecs
    msg.imu.header.frame_id = "imu1_frame"
    msg.imu.header.stamp.secs = now.secs
    msg.imu.header.stamp.nsecs = now.nsecs
    msg.imu.orientation.x = qx
    msg.imu.orientation.y = qy
    msg.imu.orientation.z = qz
    msg.imu.orientation.w = qw
    # msg.imu_orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
    msg.imu.angular_velocity.x = gyroX
    msg.imu.angular_velocity.y = gyroY
    msg.imu.angular_velocity.z = gyroZ
    # msg.imu_angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
    msg.imu.linear_acceleration.x = accelX
    msg.imu.linear_acceleration.y = accelY
    msg.imu.linear_acceleration.z = accelZ
    # msg.imu_linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
    msg.mag_field.header.frame_id = "imu1_frame"
    msg.mag_field.header.stamp.secs = now.secs
    msg.mag_field.header.stamp.nsecs = now.nsecs
    msg.mag_field.magnetic_field.x = magX
    msg.mag_field.magnetic_field.y = magY
    msg.mag_field.magnetic_field.z = magZ
    # msg.mag_field.magnetic_field_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
    return msg


def driver_publish(serialPort: serial.Serial, driverName: str, topicName: str, msgType):
    """Publishes data from serial port to a ROS topic

    Args:
        serialPort (serial.Serial): A serial.Serial object
        driverName (str): Name of the ROS node.
        topicName (str): Name of the ROS topic to publish to.
        msgType (msg): ROS message type.
    """
    ros_publisher = rospy.Publisher(topicName, msgType, queue_size=10)
    rospy.init_node(driverName, anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        line = serialPort.readline()
        if DEBUG:
            print("Serial.readLine() -> ", line)
        msg = parse(line)
        if msg is None or msg == "":
            rospy.logwarn('Unable to parse message')
        else:
            rospy.loginfo(msg)
            ros_publisher.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    serialPort = serial_port_setup(args[1])
    topic = "elbow_imu"
    try:
        driver_publish(serialPort, driverName='imu_driver', topicName=topic, msgType=Vectornav)
    except rospy.ROSInterruptException:
        serialPort.close()
        print("ROS Interrupted, closing serial port")
