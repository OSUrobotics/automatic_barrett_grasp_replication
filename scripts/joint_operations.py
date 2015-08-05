 #! /usr/bin/env python
import rospy
from sensor_msgs.msg import JointState



class joint_data_feedback:

	def __init__(self):
		self.data = None	
		joints_listner = rospy.Subscriber('/joint_states', JointState, self.callback)
		
		

	def callback(self,data):
		self.data = data
		

	def joint_data_values(self):
		return self.data
	
	def joint_effort_finger_1(self):
		return self.data.effort[0]
	
	def joint_effort_finger_2(self):
		return self.data.effort[1]
	
	def joint_effort_finger_3(self):
		return self.data.effort[3]
	
	def joint_position_finger_1(self):
		return self.data.postion[0]

	def joint_position_finger_1(self):
		return self.data.postion[1]
	
	def joint_data_header(self):
		return self.data.header
	def joint_data_stamp(self):
		return self.data.header.stamp.to_sec() 

	def close_finger1(grasp):
		grasp_inc_1 = 0 #the amout of closer for each finger
		effort_rate1 = 0
		past_rate1  = 0
		effort_I1 = 0 # initial effort
		effort_V1 = 0 #final effort
		time_V = 0
		time_I = 0
		effort_bool1 = False
		if effort_rate1 - past_rate1 < .05 and grasp["Finger 1(rads)"] != 0.0:
			cur_hand_jts[1] += grasp_inc
			grasp_inc_1 += grasp_inc
		elif grasp["Finger 1(rads)"] == 0.0:
			effort_bool1 = True
			print "Finger 1 set true"
		else:
			print "Finger 1 set true"
			effort_bool1 = True

	def close_finger2(grasp):
		grasp_inc_2 = 0 #the amout of closer for each finger
		effort_rate2 = 0 
		past_rate2 = 0
		effort_I2 =0 # initial effort
		effort_V2 = 0 #final effort
		time_V = 0
		time_I = 0
		effort_bool2 = False
		 
		if effort_rate2 - past_rate2 < .05 and grasp["Finger 2(rads)"] != 0.0:
			cur_hand_jts[2] += grasp_inc
			grasp_inc_2 += grasp_inc
		elif grasp["Finger 2(rads)"] == 0.0:
			print "Finger 2 set true"
			effort_bool2 = True
		else:
			print "Finger 2 set true"
			effort_bool2 = True

	def close_finger3(grasp):
		grasp_inc_3 = 0 #the amout of closer for each finger
		effort_rate3 = 0 
		past_rate3 = 0
		effort_I3 =0 # initial effort
		effort_V3 = 0 #final effort
		time_V = 0
		time_I = 0
		effort_bool3 = False

		if effort_rate3 - past_rate3 < .05 and grasp["Finger 3(rads)"] != 0.0:
			cur_hand_jts[3] += grasp_inc
			grasp_inc_3 += grasp_inc
		elif grasp["Finger 3(rads)"] == 0.0:
			print "Finger 3 set true"
			effort_bool3 = True
		else:
			print "Finger 3 set true"
			effort_bool3 = True

