#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
import utm
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from vn300.msg import ins
# from collections import deque

class gpsUtilities:
    def __init__(self):

        self.gps_sub = rospy.Subscriber('/vectornav/ins', ins, self.gps_callback)
        self.pose_pub = rospy.Publisher('/gps_pose', PoseStamped, queue_size=10)

        self.firstRun = True
        self.global_home = np.array([0, 0, 0])
        self.global_position = np.array([0, 0, 0])
        self.local_positionNED = np.array([0, 0, 0])
        self.local_positionENU = np.array([0, 0, 0])
        self.pose = PoseStamped()
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.init_rotn = np.identity(4)

    def gps_callback(self, data):

        lat = data.LLA.x
        long = data.LLA.y
        (easting, northing, zone_number, zone_letter) = utm.from_latlon(lat, long)
        print np.array([easting, northing])
        yaw = -data.RPY.z*math.pi/180
        orientation_yaw = tf.transformations.quaternion_from_euler(0, 0, yaw)
        rotation_yaw = tf.transformations.quaternion_matrix(orientation_yaw)
        translation = tf.transformations.translation_matrix(np.array([northing, -easting, 0]))
        self.transformation_curr = np.dot(translation, rotation_yaw)

        if self.firstRun:
            self.transformation_init = self.transformation_curr
            self.firstRun = False

        transformation_current_b = np.dot(np.linalg.inv(self.transformation_init), self.transformation_curr)
        trans = tf.transformations.translation_from_matrix(transformation_current_b)
        quatn = tf.transformations.quaternion_from_matrix(transformation_current_b)
        self.pose.header.stamp = data.header.stamp
        self.pose.header.frame_id = "world"
        self.pose.pose.position.x = trans[0]
        self.pose.pose.position.y = trans[1]
        self.pose.pose.position.z = trans[2]
        self.pose.pose.orientation.x = quatn[0]
        self.pose.pose.orientation.y = quatn[1]
        self.pose.pose.orientation.z = quatn[2]
        self.pose.pose.orientation.w = quatn[3]
        self.pose_pub.publish(self.pose)

if __name__ == '__main__':
    rospy.init_node('gps2xy', anonymous=True)
    gpsUtls = gpsUtilities()
    rospy.spin()
