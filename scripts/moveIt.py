#! /usr/bin/env python
import sys
import copy
import rospkg
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from osu_ros_adept.srv import *
from sensor_msgs.msg import JointState
import os
from numpy import *


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



if __name__ == "__main__":
	print "============ Starting tutorial setup"
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('moveIt',anonymous=True)
	robot = moveit_commander.RobotCommander() #interface to the robot
	scene = moveit_commander.PlanningSceneInterface() #interface to the world around the robot
	group = moveit_commander.MoveGroupCommander("adept_arm") #interface to one group of jonts
	print "here"
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)
	group.clear_pose_targets()
	group_variable_values = group.get_current_joint_values()
	print "============ Joint values: ", group_variable_values
	group_variable_values[0] = 0.5
	group.set_joint_value_target(group_variable_values)

	plan2 = group.plan()
	print "Joint state len: ", len(plan2.joint_trajectory.points)
	for i in range(10):
		print "Joint state points: ",plan2.joint_trajectory.points[i].positions
		#send_adept_joints(plan2.joint_trajectory.points[i].positions)
	#group.go(wait=True)
	moveit_commander.roscpp_shutdown()

