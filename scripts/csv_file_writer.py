#! /usr/bin/env python
import rospy
import rosbag
import csv

file_location = "/home/roboticslab/AlphaGraspData/"
grasp = None

def open_csv_writer(graspl):
	global grasp
	grasp = graspl
	with open(file_location + "GraspInfo_Grasp" + str(grasp) + ".csv", "wb") as cf:
		f = csv.writer(cf)
       		f.writerow(["Grasp_Trial", "Success","Image_1","Reset_okay","Image_2","Image_3","Image_4", "Video"])
	

def csv_appender( trial, img1, img2, img3, img4, vid):
	global grasp
	with open( file_location + "GraspInfo_Grasp" + str(grasp) + ".csv", "ab") as cf:
		f = csv.writer(cf)
		f.writerow([trial, "", img1,"", img2, img3, img4, vid])





