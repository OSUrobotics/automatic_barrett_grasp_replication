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
		return self.data.effort[3]
	
	def joint_effort_finger_2(self):
		return self.data.effort[0]
	
	def joint_effort_finger_3(self):
		return self.data.effort[1]
	
	def joint_position_finger_1(self):
		return self.data.postion[3]
