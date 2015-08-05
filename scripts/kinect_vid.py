#! /usr/bin/env python
import rospy
import os
import time
from threading import Thread, Lock
import rosbag
from  cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2


bagvid = None
name = None
bagpcl = None
cnt = 3
pcl_var = False
spinner = True
grasp_trial = None
grasp_num = None
grasp_test_bool = True 
vid_start_lock = None 

bag_file_location = "/home/roboticslab/AlphaTrialVideos/"


def kinect_pcl_cb(msg):
	global bagpcl, cnt, pcl_var
	if pcl_var == True:
		bagpcl.write('camera/depth_registered/points', msg) #writes the pcl into the bagfile
		cnt -= 1
	pcl_var = False
 
def point_cloud_record(pcl_num):
	global bagpcl, grasp_trial, bag_file_location, cnt, pcl_var
	#creates a new file folder if one already exist
	if not os.path.exists(bag_file_location +'Grasp' + str(grasp_num)):
		os.makedirs(bag_file_location +'Grasp' + str(grasp_num))
	
	bagname = bag_file_location +'Grasp' + str(grasp_num) +'/'+ 'Grasp_' + str(grasp_num) +'_trial_'+ str(grasp_trial) + str(pcl_num) +'.bag'
        bagpcl = rosbag.Bag(bagname, 'w')
        pcl_sub = rospy.Subscriber("camera/depth_registered/points", PointCloud2, kinect_pcl_cb, queue_size=1) #subscribes to the kinect 
	
	while cnt > 0:
		pcl_var = True
		continue
	
	cnt = 3
	print "wacky done"
	pcl_sub.unregister()
	del pcl_sub 
        bagpcl.close()

class image_converter:

	global img_num, bag_file_location, grasp_num, grasp_trial
	
	def __init__(self, img_num):
		self.looper = True
		self.img_num = img_num
		self.img_pub = rospy.Publisher("/camera/rgb/image_color", Image)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.callback, queue_size=1)
		self.name = None
		while self.looper == True:
			continue

	def callback(self, data):
		#makes an image object that is converited from Image data to cv2 data
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print e
		#names the image and saves it 
		self.name = bag_file_location +'Grasp' + str(grasp_num) +'/'+ 'Grasp_' + str(grasp_num) +'_trial_'+ str(grasp_trial) + str(self.img_num) +'.png'
		cv2.imwrite(self.name, cv_image) 
		
		self.image_sub.unregister()
		self.looper = False

	def img_name(self):
		return self.name
	
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
        	bagvid.write('/camera/rgb/image_color/compressed', msg)
	except:
		rospy.logerr("Trouble writing to bag file. Image stamp: " + str(msg.header.stamp))

def kinect_caller():
	global bagvid, spinner, grasp_trial, grasp_test_bool, grasp_num, vid_start_lock, bag_file_location
	
	while True:
		# Pause until the main thread has loaded the video paramters
		vid_start_lock.acquire()
		if rospy.is_shutdown():
			return
		
		if not os.path.exists(bag_file_location +'Grasp' + str(grasp_num)):
			os.makedirs(bag_file_location +'Grasp' + str(grasp_num))

		bag_name = bag_file_location + 'Grasp' + str(grasp_num) +'/' + 'Grasp_' + str(grasp_num) +'_trial_'+ str(grasp_trial) + '_vid' +'.bag'	
		bagvid = rosbag.Bag(bag_name, 'w')
		image_sub = rospy.Subscriber("/camera/rgb/image_color/compressed", CompressedImage, kinect_image_cb, queue_size=1)

       		period = rospy.Duration(0.25)
        	while not rospy.is_shutdown() and spinner == True:
			rospy.sleep(period)
			continue 
	
	
		# Reset the spinner for the next trial/grasp
		vid_start_lock.release()
		image_sub.unregister()
		del image_sub
       		bagvid.close()
		#print "Kinect Img bag close " + bag_name + ", time: ", time.time()
		
	
