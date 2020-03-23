#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
import utm
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from vn300.msg import gps
from vn300.msg import sensors
# from collections import deque

class gpsUtilities:
    def __init__(self):

        self.gps_sub = rospy.Subscriber('/vectornav/gps', gps, self.gps_callback)
        self.pose_pub = rospy.Publisher('/gps_pose', PoseStamped, queue_size=10)

        self.firstRun = True
        self.global_home = np.array([0, 0, 0])
        self.global_position = np.array([0, 0, 0])
        self.local_positionNED = np.array([0, 0, 0])
        self.local_positionENU = np.array([0, 0, 0])
        self.pose = PoseStamped()
        self.odom = Odometry()
        self.pcd_out = PointCloud2()
        self.initial_roll = 0
        self.initial_pitch = 0
        self.initial_yaw = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.init_rotn = np.identity(4)

    def gps_to_local(self):
        (easting_home, northing_home, zone_number_home, zone_letter_home) = utm.from_latlon(self.global_home[0], self.global_home[1])
        (easting, northing, zone_number, zone_letter) = utm.from_latlon(self.global_position[0], self.global_position[1])
        northing_local = northing - northing_home
        easting_local = easting - easting_home
        down_local = self.global_position[2] - self.global_home[2]
        self.local_positionNED = np.array([northing_local, easting_local, down_local])
        self.local_positionENU = np.array([northing_local, -easting_local, -down_local]);

    def gps_callback(self, data):
        self.global_position = np.array([data.LLA.x, data.LLA.y, 0])

        if self.firstRun:
            self.global_home = self.global_position		
            self.firstRun = False	
		
	quatn_init = tf.transformations.quaternion_from_euler(self.initial_roll, self.initial_pitch, self.initial_yaw)
	transformation_init = tf.transformations.quaternion_matrix(quatn_init)
        self.gps_to_local()
	transformation_ = tf.transformations.translation_matrix(self.local_positionENU)
	quatn_ = tf.transformations.quaternion_from_euler(0, 0, 0)
	rotation_ = tf.transformations.quaternion_matrix(quatn_ )
        transformation_current = np.dot(transformation_, rotation_)
        transformation_current = np.dot(np.linalg.inv(transformation_init), transformation_current)
        trans = tf.transformations.translation_from_matrix(transformation_current)
        quatn = tf.transformations.quaternion_from_matrix(transformation_current)

	self.pose.header.stamp = data.header.stamp
	self.pose.header.frame_id = "world"
	self.pose.pose.position.x = -self.local_positionENU[0]
	self.pose.pose.position.y = self.local_positionENU[1]
	self.pose.pose.position.z = self.local_positionENU[2]
	self.pose.pose.orientation.x = 0
	self.pose.pose.orientation.y = 0
	self.pose.pose.orientation.z = 0
	self.pose.pose.orientation.w = 1
	self.pose_pub.publish(self.pose) 
	  
if __name__ == '__main__':
    rospy.init_node('gps2xy', anonymous=True)
    gpsUtls = gpsUtilities()
    rospy.spin()
