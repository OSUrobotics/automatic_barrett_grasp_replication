#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from pyHand_api import *

def send_hand_position(command_pub, hand_joint_list):
	
	#hand = pyHand('/dev/pcan32')
	print "\tSending hand positions: ", hand_joint_list
	
	command = JointState()
	command.header.stamp = rospy.Time.now()
	command.name = ['bh_j23_joint', 'bh_j12_joint', 'bh_j22_joint', 'bh_j32_joint', 'bh_j33_joint', 'bh_j13_joint', 'bh_j11_joint', 'bh_j21_joint']
	command.position = hand_joint_list
	command.velocity = [0, 0, 0, 0, 0, 0, 0, 0]
	command.effort = [0, 0, 0, 0, 0, 0, 0, 0]
	command_pub.publish(command)

	#print hand.get_packed_position(FINGER1)
	#print hand.get_strain(FINGER1)
