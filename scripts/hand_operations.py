#! /usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import hand_control

if __name__ == "__main__":
	rospy.init_node("open_barrett_hand")
	command_pub = rospy.Publisher("bhand_node/command", JointState, queue_size=100, latch=True)

	while True:
		command = raw_input("Pleae enter a command (o - open, q - quit): ")
		if command.lower() == "o":
			hand_control.send_hand_position(command_pub, [0,0,0,0,0,0,0,0])
		elif command.lower() == "q":
			break
