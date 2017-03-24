#!/usr/bin/env python

import rospy
import message_filters

import pcl
import numpy as np

import sensor_msgs.point_cloud2 as pc2

class LidarConsumer:

    def __init__(self):
        # Init node
        rospy.init_node('lidar_consumer')

        # Standard Subscribers
        rospy.Subscriber('velodyne_points', PointCloud2, self.process_lidar)

        # Publishers
        #self.acc_pub = rospy.Publisher('', , queue_size=1)

    def process_lidar(self, pointcloud):
        # Do something with the lidar
        rospy.loginfo("Received LIDAR Point Cloud")


if __name__ == '__main__':
    try:
        LidarConsumer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start lidar consumer node.')
