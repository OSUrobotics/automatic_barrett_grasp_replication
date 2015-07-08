#! /usr/bin/env python
import rospy
import os
import rosbag
from sensor_msgs.msg import PointCloud2

	
bag = None
cnt = 3
def kinect_image_cb(msg):
        global bag, pcl_sub, cnt
        bag.write('camera/depth_registered/points', msg)
        cnt -= 1
        if cnt == 0:
                pcl_sub.unregister()
                print ("Finished")
                rospy.signal_shutdown("Program shutdown")

if __name__ == "__main__":
        rospy.init_node("bagger")
        bag = rosbag.Bag('pointcloud.bag', 'w', 'BZ2')
        pcl_sub = rospy.Subscriber("camera/depth_registered/points", PointCloud2, kinect_image_cb)


        while not rospy.is_shutdown():
                continue

        bag.close()

