#! /usr/bin/env python
import rospy
import os
import rosbag
from sensor_msgs.msg import Image

bagvid = None 
def kinect_image_cb(msg):
        global bagvid
        bagvid.write('/camera/rgb/image_color', msg)

def kinect_caller(vid_num,spinner):
	global bagvid
	bagvid = rosbag.Bag('Image_and_vid'+ str(vid_num) +'.bag', 'w')
	image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, kinect_image_cb)

        #period = rospy.Duration(0.25)
        while not rospy.is_shutdown() and spinner == True:
		#period.sleep()
		continue 
	
	print spinner
	image_sub.unregister()
       	bagvid.close()
