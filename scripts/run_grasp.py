#! /usr/bin/env python
import rospkg
import rosbag
import os
from sensor_msgs.msg import JointState
from csv_file_writer import *
from kinect_vid import *
from Raspberry_pi_com import *
from joint_operations import *
import math, struct
import time
from numpy import *
from osu_ros_adept.srv import *
from hand_control import send_hand_position
from sensor_msgs.msg import Image


#Global variable to keep track of trials, and other thing related
grasp_trial_num = 0
grasp_trial_bool = True
grasp_num_dic = {} #dic to map the type of grasp and number of trials for that grasp
grasp_trial_dic = {} #dic to make trials and data
 


def parse_grasp_file(filename):
    rospack = rospkg.RosPack()
    barrett_grasp_rep_path = rospack.get_path("automatic_barrett_grasp_replication")
    in_file = open(barrett_grasp_rep_path + "/grasp_data/" + filename, "r")
    grasp_csv = csv.DictReader(in_file, delimiter=",")
    
    out_grasp_dict = {}
    for line in grasp_csv:
        grasp_idx = line['Grasp No.'] 
        out_grasp_dict[grasp_idx] = line
        out_grasp_dict[grasp_idx]["Finger 1(rads)"] = float(out_grasp_dict[grasp_idx]["Finger 1(rads)"])
        out_grasp_dict[grasp_idx]["Finger 2(rads)"] = float(out_grasp_dict[grasp_idx]["Finger 2(rads)"])
        out_grasp_dict[grasp_idx]["Finger 3(rads)"] = float(out_grasp_dict[grasp_idx]["Finger 3(rads)"])
        out_grasp_dict[grasp_idx]["Spread (rads)"] = float(out_grasp_dict[grasp_idx]["Spread (rads)"])
        out_grasp_dict[grasp_idx]["J position1"] = (float(out_grasp_dict[grasp_idx]["J position1"])*math.pi)/180
        out_grasp_dict[grasp_idx]["J position2"] = (float(out_grasp_dict[grasp_idx]["J position2"])*math.pi)/180
        out_grasp_dict[grasp_idx]["J position3"] = (float(out_grasp_dict[grasp_idx]["J position3"])*math.pi)/180
        out_grasp_dict[grasp_idx]["J position4"] = (float(out_grasp_dict[grasp_idx]["J position4"])*math.pi)/180
        out_grasp_dict[grasp_idx]["J position5"] = (float(out_grasp_dict[grasp_idx]["J position5"])*math.pi)/180
        out_grasp_dict[grasp_idx]["J position6"] = (float(out_grasp_dict[grasp_idx]["J position6"])*math.pi)/180

    

    return out_grasp_dict

def send_command(params, joints):

	rospy.wait_for_service('send_robot_movements')
	try:
		send_cmd = rospy.ServiceProxy('send_robot_movements', robot_movement_command)
		temp = []
		for i in range(6):
			temp.append(joints[i])
		
		length = send_cmd(params[0], params[1], params[2], params[3], params[4], temp)
		print str(length) + " bytes sent"

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	print "ch1"
	
def send_adept_joints(pos):
	print "\tSending robot joint angles: ", pos
	last = []
	last = pos
		
	f_angles = array(pos, dtype=float32)

	cmd = []
	cmd.append(60)  #Message Length
	cmd.append(1)  #Message Type
	cmd.append(0)  #Message Command
	cmd.append(0)  #Message Reply 
	cmd.append(0)  #Discard bytes

	send_command(cmd, f_angles)

	
	return last

def send_parameters(params):
	rospy.wait_for_service('send_robot_parameters')
        try:
        	send_cmd = rospy.ServiceProxy('send_robot_parameters', robot_parameters_command)
                length = send_cmd(params[0], params[1], params[2], params[3], params[4])
  		
		print str(length) + " bytes sent"
        
  	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e
                #print "ch2"
 
def set_speed(spd):
 	cmd = []
        cmd.append(20)  #Message Length
        cmd.append(11)  #Message Type
        cmd.append(spd)  #Message Command
        cmd.append(0)  #Message Reply 
        cmd.append(0)  #Discard bytes
 
 	send_parameters(cmd)
 
def shake_home_pos():
 	pos = [0, -0.80, 2.49, 0, -0.45, 0]
	send_adept_joints(pos)
 
def shake_left():
 	pos = [-0.65, -0.80, 2.49, 0, -0.45, 0.55]
	send_adept_joints(pos)
 
def shake_right():
	pos = [0, -0.80, 2.49, -0.35, -0.10, -0.21]
        send_adept_joints(pos)

# Returns the number of trials the user wants to execute (int)
def get_trial_num():
	global grasp_num_dic
	for key in grasp_num_dic.keys():
		grasp_trial_num = raw_input("How many times do you want to test Grasps %s? "%key)
	
		#Validate input
		try:
			grasp_num_dic[key] = int(grasp_trial_num)
		except ValueError:
			print "Unrecongized trial entry, grasp for %s set to zero "%key
			grasp_num_dic[key] = 0
			continue

def arm_to_grasp_position(grasp):
	adept_joint_list = []
	adept_joint_list = [grasp["J position1"], grasp["J position2"], grasp["J position3"], grasp["J position4"], grasp["J position5"], grasp["J position6"]  ]
        send_adept_joints(adept_joint_list)

	return adept_joint_list

def get_user_hand_adj(grasp, command_pub):	
        raw_input("Press enter to grasp.")
        cur_hand_jts = [0, grasp["Finger 1(rads)"], grasp["Finger 2(rads)"], grasp["Finger 3(rads)"], 0, 0, grasp["Spread (rads)"], 0 ] 

	grasp_inc = "0"
	while grasp_inc != "q" and grasp_inc != "Q":
         	grasp_inc = float(grasp_inc)
          	cur_hand_jts[1] += grasp_inc
         	cur_hand_jts[2] += grasp_inc
         	cur_hand_jts[3] += grasp_inc
         	send_hand_position(command_pub, cur_hand_jts)	
	 	
		#Make sure input is correct and allow them to re-enter or quit
	 	while True:
	 		try:
         	 		grasp_inc = raw_input("Enter how much to increase closure by (change in radians, or q to quit):")
		 		if grasp_inc == "q" or  grasp_inc == "Q":
		 			break
		 		else:
					float(grasp_inc)
					break
			except ValueError:
		 		print "Invalid Entry, try again"
	return cur_hand_jts 

def get_hand_adj(grasp, command_pub, the_joint_data):

	time.sleep(15)	
	cur_hand_jts = [0, grasp["Finger 1(rads)"], grasp["Finger 2(rads)"], grasp["Finger 3(rads)"], 0, 0, grasp["Spread (rads)"], 0 ] 
	send_hand_position(command_pub, cur_hand_jts)
	time.sleep(0.3)

	# Find which fingers we're supposed to close
	close_list = []
	close_threshold = 0.05
	if grasp["Finger 1(rads)"] > close_threshold:
		close_list.append( (the_joint_data.get_velocity1, 0) )

	if grasp["Finger 2(rads)"] > close_threshold:
		close_list.append( (the_joint_data.get_velocity2, 1) )

	if grasp["Finger 3(rads)"] > close_threshold:
		close_list.append( (the_joint_data.get_velocity3, 2) )

	# Iterate over each finger closing them until each has hit an effort max
	grasp_inc = [0,0,0]
	while len(close_list) > 0:
		for f_vel, f_idx in close_list[:]:
			print "Current max for finger ", (f_idx + 1), ": ", f_vel()
			if f_vel() < .01 :
				close_list.remove( (f_vel, f_idx) )
				grasp_inc[f_idx] = 0
			else:	
				# The finger needs more closing
				grasp_inc[f_idx] = 0.02

		# Publish updated finger positions
		cur_hand_jts[1] += grasp_inc[0]
		cur_hand_jts[2] += grasp_inc[1]
		cur_hand_jts[3] += grasp_inc[2]
		send_hand_position(command_pub, cur_hand_jts)
		time.sleep(.1)
				
 	cur_hand_jts[1] += .07
	cur_hand_jts[2] += .07
	cur_hand_jts[3] += .07
	send_hand_position(command_pub, cur_hand_jts)	
	

	return cur_hand_jts

#def inc_f1(f_list, inc):
#	f_list[0] += inc

#def inc_f2(f_list, inc):
#	f_list[1] += inc

#def inc_f3(f_list, inc):
#	f_list[2] += inc

def automatic_hand_close(command_pub, hand_adj):
	time.sleep(15) #No feed back to know if arm is  in postion so we wait to grasp
        send_hand_position(command_pub, hand_adj )
	time.sleep(1.2) 

# Send arm up and winds up the object to be reset
def reset_arm_hand(grasp):
	send_hand_position(command_pub, [0,0,0,0,0,0,0,0])
	adept = []
	adept = [-.5,-1, grasp["J position3"], grasp["J position4"], grasp["J position5"], grasp["J position6"]  ] 
	set_speed(300)
	send_adept_joints(adept)
	set_speed(100)
	pi_start_up()


def check_grasp_name(grasp_num_name):
	name_list = ['A','B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q','R','S', 'T','U','V','W','X','Y','Z']
	if os.path.exists( "/home/roboticslab/AlphaTrialVideos/Grasp" + str(grasp_num_name)):
		for letter in name_list:
			if not os.path.exists("/home/roboticslab/AlphaTrialVideos/Grasp" + str(grasp_num_name)+ str(letter)):
				grasp_num_name = str(grasp_num_name) + str(letter)
				return grasp_num_name
				break
	else:
		return grasp_num_name

def ask_grasp_name(grasps):
	global  grasp_num_dic
    	while True:
    		#Only need to ask part under if at the beginning
		grasp_num = raw_input("Which grasp do you want for the hand (q to quit)? ")
	
		# Validate input
		if grasp_num.lower() == "q":
			break
		else:
			if grasp_num in grasps.keys():
				grasp_num_dic[grasp_num] = 0  #set number of trials for that grasp equal to zero	
				print "Got it! Follow prompt to enter in another grasp or quit"
				continue			
			else:
				print "Unrecognized grasp index."
				continue

	return grasp_num
	
if __name__ == "__main__":
    grasp_filename = "grasp_list.csv"  #File that contains the grasp to test
    rospy.init_node("run_grasp")
    command_pub = rospy.Publisher("bhand_node/command", JointState, queue_size=100) #Need this send messages to the barrett hand
    
    grasps = parse_grasp_file(grasp_filename) #change the file to a workable vairable 
    
    t1 = init_vid_thread() #Start up video recording thread
    the_joint_data = joint_data_feedback() #Start getting joint feed back
   
    #Keep Code runing as long as trials are not finished or system is not shutdown
    hand_adj = None
    while not rospy.is_shutdown():
	grasp_num  = ask_grasp_name(grasps) #Find the grasp they want to test and checks if all fingers are being used

	if grasp_num.lower() == "q" and len(grasp_num_dic) == 0:
		break

	user_choice = "y"#raw_input("Do you want grasp to close automatically? (Y or N) ")
	shake_test_bool = "y"#raw_input("Enter 'Y' to run the shake test along with grasps or 'N' not to: ")
	
	get_trial_num() #get the number of times they want to run each grasp
	
	#Keep code running as long as there are grasp to test
	while len(grasp_num_dic) > 0:
		grasp_num = grasp_num_dic.keys()[0] 
		grasp_num_name = check_grasp_name(grasp_num)
		grasp_trial_num = grasp_num_dic[grasp_num]
		print "Running Grasp %s time(s)" % (grasp_trial_num)
	
		grasp_trial_bool = True
		orig_trial_num = grasp_trial_num
		while grasp_trial_num > 0:
			excel_vid_list = []	#list to keep track of images for each trial
			current_trial_num = str(orig_trial_num - grasp_trial_num)
			#start_vid_record(grasp_num_name, current_trial_num)#Kinect Video service call
			grasp_pos = arm_to_grasp_position(grasps[grasp_num]) #Move the arm to grasp position


			# This is for if it is the first trial: Close hand and make small adjustments based on user input
			if grasp_trial_bool == True:
				if user_choice.lower() == "n":
					hand_adj = get_user_hand_adj(grasps[grasp_num], command_pub)
				else:
					hand_adj = get_hand_adj(grasps[grasp_num], command_pub, the_joint_data)
			else: #This is for if its not the first trial: Wait for two seconds then Close the hand
				automatic_hand_close(command_pub, hand_adj)
			
			print "staying alive" 
			#Kinect Picture Serice call    
			#ic = image_converter('_img0')
			#point_cloud_record('_pcl0' )
			#excel_vid_list.append(ic.img_name())

			if shake_test_bool.lower() == "y":
				set_speed(300) 
				shake_home_pos()
				time.sleep(1.2) #time for arm to get to home_pos()
				#ic1 = image_converter('_img1')
				#point_cloud_record('_pcl1')
				#excel_vid_list.append(ic1.img_name())
				for i in range(3):
					shake_left()
					shake_right()        
				set_speed(100)
				shake_home_pos()
				time.sleep(1.2)
				#ic2 = image_converter('_img2')
				#point_cloud_record('_pcl2')
				#excel_vid_list.append(ic2.img_name())
				
			send_adept_joints( [0, -0.87, 2.49, 0, -0.45, 0]) #Move Arm to drop object back down
			time.sleep(8)
			#ic3 = image_converter('_img3')
			#excel_vid_list.append(ic3.img_name())
		 
			reset_arm_hand(grasps[grasp_num]) #lets go of object and moves arm of of the way for object to reset 
			grasp_trial_num -= 1  
			grasp_trial_bool = False 
			
			#End the video for trial
			#stop_vid_record()
			grasp_trial_dic[current_trial_num] = excel_vid_list #make the current trial to it's images

		#xls_open_n_write(grasp_num_name, grasp_trial_dic) #At the end of all trials, makes an excel sheet
		del grasp_num_dic[grasp_num]		#gets ready for next grasp test if there is one
	send_adept_joints([0, 0, 0, 0, 0, 0])		#sends arm back to home postion once testing is done

    # Final cleanup
    rospy.signal_shutdown("Closing video thread.")
    start_vid_record(0,0)

