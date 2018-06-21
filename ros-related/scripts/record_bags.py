#!/usr/bin/env python

import rosbag
import rospy
from sensor_msgs.msg import Image

def cbImageRecord(data):
	global i
	# Camera will nominally operate at 15 hz, so i%val = image rate in rosbag.
	val = 1
	if i%val == 0:
		if i < 61:
			bag.write('/camera/image_mono', data)
		else:
			print 'Done'
	i = i + 1

def listener():
	rospy.init_node('imageRecordListener', anonymous=True)
	rospy.Subscriber('/camera/image_mono', Image, cbImageRecord)
	rospy.spin()

if __name__ == '__main__':
	global i
	i = 0
	bag = rosbag.Bag('testc.bag', mode='w', compression='lz4')
	listener()
	while not rospy.is_shutdown():
		pass
	bag.close()
