#!/usr/bin/env python

import rosbag
import rospy
from sensor_msgs.msg import Image

def cbImageRecord(data):
	global i
	# Camera will nominally operate at 15 hz, so i%val = image rate in rosbag.
	val = 7.5
	if i%val == 0:
		bag.write('/camera/image_raw', data)
	i = i + 1

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('/camera/image_raw', Image, cbImageRecord)
	rospy.spin()

if __name__ == '__main__':
	global i
	i = 0
	bag = rosbag.Bag('rest.bag', mode='w', compression='bz2')
	listener()
	while not rospy.is_shutdown():
		pass
	bag.close()
