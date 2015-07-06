#! /usr/bin/env python
import rospy
import os
import rosbag
from sensor_msgs.msg import Image

bagvid = None
spinner = True

def stop_vid_record():
	global spinner
	spinner = False

def kinect_image_cb(msg):
        global bagvid
        bagvid.write('/camera/rgb/image_color', msg)

def kinect_caller(vid_num):
	global bagvid, spinner
	bagvid = rosbag.Bag('Image_and_vid'+ str(vid_num) +'.bag', 'w')
	image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, kinect_image_cb)

        period = rospy.Duration(0.25)
        while not rospy.is_shutdown() and spinner == True:
		rospy.sleep(period)
		continue 
	
	print spinner

	# Reset the spinner for the next trial/grasp
	spinner = True
	image_sub.unregister()
       	bagvid.close()
