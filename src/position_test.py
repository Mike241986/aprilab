#!/usr/bin/env python3
""" Reads NMEA0183 messages from a serial port and publishes them to ROS
"""
import io
import serial
import numpy as np

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32

""" serial_update_rate = 15.0  # Hz
baudrate = 38400  # 4800 for sonar, 38400 for GPS
port = '/dev/ttyACM0'  # /dev/ttyUSB0 for sonar, /dev/ttyUSB1 for GPS 
ser = serial.Serial(port=port, baudrate=baudrate, timeout=1/serial_update_rate)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))"""

class NMEAReader():
	def __init__(self) -> None:
		rospy.init_node('nmea_reader')
		self.ll_pub = rospy.Publisher('nmea_lla', PointStamped, queue_size=100)
		self.depth_pub = rospy.Publisher('nmea_depth', Float32, queue_size=100)
		rospy.loginfo("NMEA Reader Initialized")

		self.debug = False
		self.ll = np.array([0, 0])
		self.depth_m = 0

	"""def read_nmea0183(self, event):
		Reads NMEA0183 messages from a serial port
			Refer to pynmea2 documentation for more info on sentences (types of messages)
			or print the repr(msg) to see the raw messages. 
		
		parse_warn = False
		try:
			line = sio.readline()
			msg = pynmea2.parse(line)

			if self.debug:
				print("Type: ", type(msg))
				print(repr(msg))

			if type(msg) == pynmea2.types.talker.MTW:
				self.water_temp_C = msg.temperature  # degrees C
				# print(type(msg.temperature)) 
			elif type(msg) == pynmea2.types.talker.GGA or type(msg) == pynmea2.types.talker.GLL:
				if msg.lat != '':
					# Convert from degrees and decimal minutes to decimal degrees
					latitude = float(msg.lat[:2]) + float(msg.lat[2:]) / 60
					longitude = float(msg.lon[:3]) + float(msg.lon[3:]) / 60
					
					# Check direction and apply signs
					if msg.lat_dir == 'S':
						latitude = -latitude
					
					if msg.lon_dir == 'W':
						longitude = -longitude
					
					self.ll = np.array([latitude, longitude])
					utc_time = msg.timestamp
				else:
					rospy.logwarn("Empty lat")
			elif type(msg) == pynmea2.types.talker.HDG:
				heading = msg.heading
			elif type(msg) == pynmea2.types.talker.DBT:
				self.depth_m = msg.depth_meters
		except serial.SerialException as e:
			rospy.logerr('Device error: {}'.format(e))
		except pynmea2.ParseError as e:
			rospy.logwarn('Parse error: {}'.format(e)) if parse_warn else None
	  """
	def pub_nmea(self, event):
		""" Publishes nmea data to ROS """
		point_stmp_msg = PointStamped()
		point_stmp_msg.header.stamp = rospy.Time.now()
		point_stmp_msg.point.x = 29.99
		point_stmp_msg.point.y = -82.98
		point_stmp_msg.point.z = 0
		self.ll_pub.publish(point_stmp_msg)
		#self.depth_pub.publish(Float32(self.depth_m))


def main():
	reader = NMEAReader()
	#rospy.Timer(rospy.Duration(1.0 / serial_update_rate), reader.read_nmea0183)
	rospy.Timer(rospy.Duration(1.0 / 1.0), reader.pub_nmea)

	rospy.spin()

if __name__ == '__main__':
	main()	