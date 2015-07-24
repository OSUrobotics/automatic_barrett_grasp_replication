#!/usr/bin/env python
from std_srvs.srv import Empty
import sys
import rospy

def windup(windup_srv):
        try:
                windup_srv()
        except:
                rospy.logerr("Service call failure for windup!")

def pi_start_up():
        rospy.loginfo("Windup_client node online!")
        rospy.wait_for_service('/windup')
	print("service connect")
        windup_srv = rospy.ServiceProxy('/windup', Empty)
	print("windup set")
        windup(windup_srv)      #call this to windup.

