import rospy
import rosbag
import csv
 
graspNumb = None
def csv_writer(grasp):
	global graspNumb
	graspNumb = grasp
        with open("GraspInfo_Grasp" + str(graspNumb) + ".csv", "wb") as cf:
		f = csv.writer(cf)
       		f.writerow(["Grasp Trial", "Success","Image 1","Reset okay","Image 2","Image 3","Image 4", "Video"])
	

def csv_appender(grasp, img1, img2, img3, img4, vid):
	global graspNumb
	with open("GraspInfo_Grasp" + str(graspNumb) + ".csv", "ab") as cf:
		f = csv.writer(cf)
		f.writerow(grasp, "", img1,"", img2, img3, img4, vid)

