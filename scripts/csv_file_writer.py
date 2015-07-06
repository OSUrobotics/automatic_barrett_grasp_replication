import rospy
import rosbag
import csv
 
def csv_writer():
        with open("GraspInfo.csv", "wb") as cf:
		f = csv.writer(cf)
       		f.writerow([ "Grasp Trial", "Success","Image 1","Reset okay","Image 2","Image 3","Image 4", "Video"])
	

def csv_appender(grasp, img1, img2, img3, img4, vid):
	with open("GraspInfo.csv", "ab") as cf:
		f = csv.writer(cf)
		f.write(grasp, "", img1,"", img2, img3, img4, vid)

