#! /usr/bin/env python
import rospy
import os
import rosbag
from sensor_msgs.msg import Image
 

bag = None
cnt = 100
def kinect_image_cb(msg):
	global bag, image_sub, cnt
   	bag.write('/camera/rgb/image_color', msg)
	cnt -= 1
	if cnt == 0:
		image_sub.unregister()
		print ("Finished")
		rospy.signal_shutdown("waahhhh")



if __name__ == "__main__":
	rospy.init_node("bagger")
	bag = rosbag.Bag('Image_and_vid.bag', 'w')
	image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, kinect_image_cb)

	period = rospy.Duration(0.25)
	while not rospy.is_shutdown():
		period.sleep()
		continue

	bag.close()
