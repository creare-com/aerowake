#!/usr/bin/env python

import rosbag
import rospy

def cbImageRecord(data):
	global i
	if i%10 == 0:
		bag.write('/camera/image_raw/compressed', data)
		print '2'
	i = i + 1

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('/turtle1/cmd_vel', Twist, callback)
	print '1'
	rospy.spin()

if __name__ == '__main__':
	global i
	i = 0
	print '0'
	bag = rosbag.Bag('rest.bag', 'w')
	listener()
	while not rospy.is_shutdown():
		pass
	print '-1'
	bag.close()
	print '-2'