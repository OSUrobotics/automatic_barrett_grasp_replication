 #! /usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

class joint_data_feedback:

	def __init__(self):
		self.data = None	
		joints_listner = rospy.Subscriber('/joint_states', JointState, self.callback)
		
		self.effort_rate_maxes = [0,0,0]
		self.avg_window = 40 #samples
		self.velocity_average_data = [[0]*self.avg_window, [0]*self.avg_window, [0]*self.avg_window]
		self.velocity_idx = 0

	def callback(self,new_data):
		if self.data == None:
			self.data = new_data
			return


		time_change = new_data.header.stamp.to_sec() - self.data.header.stamp.to_sec() 

		#Calculate velocity
		finger1_velocity = (new_data.position[1]- self.data.position[1]) / time_change
		finger2_velocity = (new_data.position[2]- self.data.position[2]) / time_change
		finger3_velocity = (new_data.position[3]- self.data.position[3]) / time_change
		#self.velocity_list = [finger1_velocity, finger2_velocity, finger3_velocity]
		#print self.velocity_list
		
		self.velocity_idx = self.velocity_idx % self.avg_window
		self.velocity_average_data[0][self.velocity_idx] = finger1_velocity
		self.velocity_average_data[1][self.velocity_idx] = finger2_velocity
		self.velocity_average_data[2][self.velocity_idx] = finger3_velocity
		self.velocity_idx += 1
		
		#Calculate effort
		'''	
		finger1_effort = ( new_data.effort[0] - self.data.effort[0] ) / ( time_change)
		finger2_effort = ( new_data.effort[1] - self.data.effort[1] ) / ( time_change)
		finger3_effort = ( new_data.effort[3] - self.data.effort[3] ) / ( time_change)
		
		self.effort_rate_list = [finger1_effort, finger2_effort, finger3_effort] 
	
		# Update Maxes
		for idx in range(len(self.effort_rate_list)):
			if self.effort_rate_list[idx] > self.effort_rate_maxes[idx]:
				self.effort_rate_maxes[idx] = self.effort_rate_list[idx]
		'''	
		# Update previous data
		self.data = new_data

	def joint_data_values(self):
		return self.data
	

	def joint_position_finger_1(self):
		return self.data.position[1]

	def joint_position_finger_2(self):
		return self.data.position[2]

	def joint_position_finger_3(self):
		return self.data.position[3]

	def joint_data_stamp(self):
		return self.data.header.stamp.to_sec() 

	def get_effort1_max(self):
		return self.effort_rate_maxes[0]
	def get_effort2_max(self):
		return self.effort_rate_maxes[1]
	def get_effort3_max(self):
		return self.effort_rate_maxes[2]

	def get_velocity1(self):
		return sum(self.velocity_average_data[0]) / len(self.velocity_average_data[0])
	def get_velocity2(self):
		return sum(self.velocity_average_data[1]) / len(self.velocity_average_data[1])
	def get_velocity3(self):
		return sum(self.velocity_average_data[2]) / len(self.velocity_average_data[2])
