 #! /usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

its_spinning = True

def joint_data_listener():
	global its_spinning
	while its_spinning == True:
		joints_listner = rospy.Subscriber('/joint_states', JointState, callback)
	
	its_spinning = True

def callback(data):
	global its_spinning
	its_spinning = False
	return data

	
