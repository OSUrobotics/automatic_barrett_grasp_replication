#! /usr/bin/env python
import rospy
import os
import time
from threading import Thread, Lock
import rosbag
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2


bagvid = None
bagimg = None
bagpcl = None
spinner = True
grasp_trial = None
grasp_num = None
grasp_test_bool = True 
vid_start_lock = None 

bag_file_location = "/home/roboticslab/AlphaTrialVideos/"

def change_trial(trial):
	global grasp_trial
	grasp_trial = trial

def kinect_pcl_cb(msg):
	global bagpcl
	bagpcl.write('camera/depth_registered/points', msg)
 
def point_cloud_record(grasp_num, pcl_num):
	global bagpcl, grasp_trial, bag_file_location
        bagpcl = rosbag.Bag(bag_file_location + 'Grasp_' + str(grasp_num) +'_trial_'+ str(grasp_trial) + str(pcl_num) +'.bag', 'w')
        pcl_sub = rospy.Subscriber("camera/depth_registered/points", PointCloud2, kinect_pcl_cb)
	
	myint = 3
	while myint != 0:
		myint -=1

        pcl_sub.unregister()
        bagpcl.close()

def kinect_image(msg):
        global bagimg
        bagimg.write('/camera/rgb/image_color', msg)

def Image_record(grasp_num,imag_num):
	global bagImg, bag_file_location
	bagImg = rosbag.Bag(bag_file_location + 'Grasp_' + str(grasp_num) +'_trial_'+ str(grasp_trial) + str(imag_num) +'.bag', 'w')
	image = rospy.Subscriber("/camera/rgb/image_color", Image, kinect_image)
	
	myint = 30 
	while myint != 0:
		myint -= 1

	image.unregister()
	bagImg.close()

def init_vid_thread():
	global vid_start_lock
	vid_start_lock = Lock()
	vid_start_lock.acquire()

	t1 = Thread(target = kinect_caller)
	t1.start()

	return t1

def stop_vid_record():
	global spinner, vid_start_lock
	spinner = False
	vid_start_lock.acquire()

def start_vid_record(grasp_num_l, grasp_trial_l):
	global spinner, vid_start_lock, grasp_num, grasp_trial
	spinner = True
	grasp_num = grasp_num_l
	grasp_trial = grasp_trial_l

	# Start the recording
	vid_start_lock.release()

def kinect_image_cb(msg):
        global bagvid
	try:
        	bagvid.write('/camera/rgb/image_color', msg)
	except:
		rospy.logerr("Trouble writing to bag file. Image stamp: " + str(msg.header.stamp))

def kinect_caller():
	global bagvid, spinner, grasp_trial, grasp_test_bool, grasp_num, vid_start_lock, bag_file_location
	
	while True:
		# Pause until the main thread has loaded the video paramters
		vid_start_lock.acquire()
		if rospy.is_shutdown():
			return

		bag_name = bag_file_location + 'Grasp_' + str(grasp_num) +'_trial_'+ str(grasp_trial) + '_vid' +'.bag'
		rospy.loginfo("Recording to bag: " + bag_name)
		bagvid = rosbag.Bag(bag_name, 'w')
		image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, kinect_image_cb, queue_size=1)

       		period = rospy.Duration(0.25)
        	while not rospy.is_shutdown() and spinner == True:
			rospy.sleep(period)
			continue 
	
	
		# Reset the spinner for the next trial/grasp
		vid_start_lock.release()
		image_sub.unregister()
		del image_sub
       		bagvid.close()
		print "Kinect Img bag close " + bag_name + ", time: ", time.time()
		time.sleep(1)
	
