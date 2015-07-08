#! /usr/bin/env python
import rospy
import rospkg
import rosbag
from sensor_msgs.msg import JointState
from csv_file_writer import *
from kinect_vid import *
import math, struct
import time
from numpy import *
from osu_ros_adept.srv import *
from hand_control import send_hand_position
#from sensor_me
from sensor_msgs.msg import Image


#Global variable to keep track of trials
grasp_trial_num = 0
grasp_trial_bool = True


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
	while True:
		grasp_trial_num = raw_input("How many times do you want to run the trial? ")
	
		#Validate input
		try:
			return int(grasp_trial_num)
		except ValueError:
			print "Unrecongized trial entry"
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
		 		grasp_inc_Copy = 0
				if grasp_inc == "q" or  grasp_inc == "Q":
		 			break
		 		else:
					float(grasp_inc)
					grasp_inc_Copy = float(grasp_inc)
					break
			except ValueError:
		 		print "Invalid Entry, try again"
	return grasp_inc_Copy

def automatic_hand_close(grasp, command_pub, user_adj):
	time.sleep(15) #No feed back to know if arm is  in postion so we wait to grasp
        cur_hand_jts = [0, grasp["Finger 1(rads)"], grasp["Finger 2(rads)"], grasp["Finger 3(rads)"], 0, 0, grasp["Spread (rads)"], 0 ]
        cur_hand_jts[1] += user_adj
        cur_hand_jts[2] += user_adj
        cur_hand_jts[3] += user_adj
        send_hand_position(command_pub, cur_hand_jts)
	time.sleep(2) 

# Send arm and hand to home position
def reset_arm_hand():
	send_hand_position(command_pub, [0,0,0,0,0,0,0,0])
	time.sleep(3) #let go of object completly before arm moves
	send_adept_joints([0,0,0,0,0,0])
	time.sleep(15)


if __name__ == "__main__":
    grasp_filename = "grasp_list.csv"
    rospy.init_node("run_grasp")
    command_pub = rospy.Publisher("bhand_node/command", JointState, queue_size=100)
    
    grasps = parse_grasp_file(grasp_filename)
    
    # Start up video recording thread
    t1 = init_vid_thread()
   
    #Keep Code runing as long as trials are not finished or system is not shutdown
    user_adj = 0
    while not rospy.is_shutdown():
    	#Only need to ask part under if at the beginning
	grasp_num = raw_input("Which grasp do you want for the hand (q to quit)? ")
	
	# Validate input
	if grasp_num.lower() == "q":
		break
	else:
		if not grasp_num in grasps.keys():
			print "Unrecognized grasp index."
			continue
	
	#Ask how many trials they want to test
	grasp_trial_num = get_trial_num()
	print "Running Grasp %s time(s)" % (grasp_trial_num)
	
	grasp_trial_bool = True
	orig_trial_num = grasp_trial_num
	while grasp_trial_num > 0:
		#Kinect Video service call
		current_trial_num = str(orig_trial_num - grasp_trial_num)
		start_vid_record(grasp_num, current_trial_num)
		
		# Move the arm
		grasp_pos = arm_to_grasp_position(grasps[grasp_num])

		# This is for if it is the first trial: Close hand and make small adjustments based on user input
		if grasp_trial_bool == True:	
			user_adj = get_user_hand_adj(grasps[grasp_num], command_pub)
		else: #This is for if its not the first trial: Wait for two seconds then Close the hand
			automatic_hand_close(grasps[grasp_num], command_pub, user_adj)

		#Kinect Picture Serice call
		Image_record(grasp_num,'_img1')
		point_cloud_record(grasp_num,'_pcl1' )
		time.sleep(1.5)
	
		#Run a shake test
		if grasp_trial_bool == True:
			shake_test_bool = raw_input("Enter 'Y' to run the shake test along with grasps or 'N' not to: ")
		if shake_test_bool == "y" or shake_test_bool == "Y":
			set_speed(500) 
			shake_home_pos()
			Image_record(grasp_num,'_img2')
			point_cloud_record(grasp_num,'_pcl2')
			time.sleep(2)
			for i in range(3):
				shake_left()
				shake_right()        
			set_speed(100)
			shake_home_pos()
			Image_record(grasp_num,'_img3')
			point_cloud_record(grasp_num,'_pcl3')
			time.sleep(2)
	
		#Move Arm to set object back down
		send_adept_joints(grasp_pos)
		time.sleep(11)
		Image_record(grasp_num, '_img4')
		time.sleep(2)
	
		#sends arm to home postion. 
		reset_arm_hand()
		grasp_trial_num -= 1  
		grasp_trial_bool = False


		#End the video for trial
		stop_vid_record()
	
		#***** Raspberry Pi Service call to reel in object*******

    # Final cleanup
    rospy.signal_shutdown("Closing video thread.")
    start_vid_record(0,0)
