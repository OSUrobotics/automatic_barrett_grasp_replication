#! /usr/bin/env python
import rospy
import rosbag
import csv
import xlsxwriter

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

def xls_open_n_write(grasp, dic):
	#make the excel sheet and get the set up ready for information to come. 
	workbook = xlsxwriter.Workbook(file_location + "GraspInfo_Grasp" + str(grasp) + ".xlsx")
	worksheet = workbook.add_worksheet()
	worksheet.set_column('A:A', 10)
	worksheet.set_column('B:B', 10)
	worksheet.set_column('C:C', 10)
	worksheet.write('A1', "Grasp_Trials")
	worksheet.write('B1', "Success")
	worksheet.write('C1', "Reset Okay")
	worksheet.write('D1', "Image 0")
	worksheet.write('E1', "Image 1")
	worksheet.write('F1', "Image 2")
 	worksheet.write('G1', "Image 3")
	worksheet.set_column('D:D', 30)
	worksheet.set_column('E:E', 30)
	worksheet.set_column('F:F', 30)
	worksheet.set_column('G:G', 30)
	
	row = 2
	count = 0 
	while count < len(dic):
		worksheet.write('A'+str(row), "grasp " + str(count))
		print dic
		worksheet.set_row(row-1,115)
		worksheet.insert_image('D' + str(row), str(dic[str(count)][0]), {'x_scale': 0.3, 'y_scale': 0.3})
		worksheet.insert_image('E' + str(row), str(dic[str(count)][1]), {'x_scale': 0.3, 'y_scale': 0.3})
		worksheet.insert_image('F' + str(row), str(dic[str(count)][2]), {'x_scale': 0.3, 'y_scale': 0.3})
		worksheet.insert_image('G' + str(row), str(dic[str(count)][3]), {'x_scale': 0.3, 'y_scale': 0.3}) 
		row += 1
		count += 1

	workbook.close()




