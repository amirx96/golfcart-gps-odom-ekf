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
from vn300.msg import ins
from vn300.msg import sensors
# from collections import deque

class gpsUtilities:
    def __init__(self):

        self.gps_sub = rospy.Subscriber('/vectornav/ins', ins, self.gps_callback)
        self.pcd_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.pcd_callback)
        self.pose_pub = rospy.Publisher('/Pose', PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('/Odom', Odometry, queue_size=10)
        self.pcd_pub = rospy.Publisher('/PointCloud2', PointCloud2, queue_size=1)

        self.firstRun = True
        self.global_home = np.array([0, 0, 0])
        self.global_position = np.array([0, 0, 0])
        self.local_positionNED = np.array([0, 0, 0])
        self.local_positionENU = np.array([0, 0, 0])
        self.brGPS = tf.TransformBroadcaster()
        self.brLIDAR = tf.TransformBroadcaster()
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
        self.listener = tf.TransformListener()

    def gps_to_local(self):
        (easting_home, northing_home, zone_number_home, zone_letter_home) = utm.from_latlon(self.global_home[0], self.global_home[1])
        (easting, northing, zone_number, zone_letter) = utm.from_latlon(self.global_position[0], self.global_position[1])
        northing_local = northing - northing_home
        easting_local = easting - easting_home
        down_local = self.global_position[2] - self.global_home[2]
        self.local_positionNED = np.array([northing_local, easting_local, down_local])
        self.local_positionENU = np.array([northing_local, -easting_local, -down_local]);

    def gps_callback(self, data):
    	# path = Path()
        self.global_position = np.array([data.LLA.x, data.LLA.y, data.LLA.z])

        if self.firstRun:
            self.global_home = self.global_position
            self.initial_roll = data.RPY.x*math.pi/180
            self.initial_pitch = -data.RPY.y*math.pi/180
            self.initial_yaw = -data.RPY.z*math.pi/180

        quatn_init = tf.transformations.quaternion_from_euler(self.initial_roll, self.initial_pitch, self.initial_yaw)
        transformation_init = tf.transformations.quaternion_matrix(quatn_init)

        self.gps_to_local()
        # self.local_positionENU[2] = 0
        transformation_ = tf.transformations.translation_matrix(self.local_positionENU)
        self.roll = data.RPY.x*math.pi/180
        self.pitch = -data.RPY.y*math.pi/180
        self.yaw = -data.RPY.z*math.pi/180; 
        quatn_ = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
        rotation_ = tf.transformations.quaternion_matrix(quatn_ )
        transformation_current = np.dot(transformation_, rotation_)
        transformation_current = np.dot(np.linalg.inv(transformation_init), transformation_current)
        trans = tf.transformations.translation_from_matrix(transformation_current)
        quatn = tf.transformations.quaternion_from_matrix(transformation_current)

        self.brGPS.sendTransform((trans[0], trans[1], trans[2]), quatn, data.header.stamp, "/gps_link", "/map")
        quatn_ENU = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.brGPS.sendTransform((0, 0, 0), quatn_ENU, data.header.stamp, "/imu_link", "/gps_link")
        try:
        	(trans_imu_lidar, rot_imu_lidar) = self.listener.lookupTransform('/vectornav', '/velodyne', rospy.Time(0))

	        self.brLIDAR.sendTransform(trans_imu_lidar, rot_imu_lidar, data.header.stamp, "/lidar_link", "/gps_link");
	        self.pose.header.stamp = data.header.stamp
	        self.pose.header.frame_id = "map"
	        self.pose.pose.position.x = trans[0] 
	        self.pose.pose.position.y = trans[1] 
	        self.pose.pose.position.z = trans[2]
	        self.pose.pose.orientation.x = quatn[0]
	        self.pose.pose.orientation.y = quatn[1]
	        self.pose.pose.orientation.z = quatn[2]
	        self.pose.pose.orientation.w = quatn[3]
	        self.pose_pub.publish(self.pose)

	        self.odom.header.stamp = data.header.stamp
	        self.odom.header.frame_id = "map"
	        self.odom.child_frame_id = "gps_link"
	        self.odom.pose.pose = self.pose.pose
	        self.odom.twist.twist.linear.x = data.NedVel.x;
	        self.odom.twist.twist.linear.y = -data.NedVel.y;
	        self.odom.twist.twist.linear.z = -data.NedVel.z;

	        self.odom.pose.covariance[0] = data.PosUncertainty
	        self.odom.pose.covariance[7] = data.PosUncertainty
	        self.odom.pose.covariance[14] = data.PosUncertainty
	        self.odom.pose.covariance[21] = data.RollUncertainty
	        self.odom.pose.covariance[28] = data.PitchUncertainty
	        self.odom.pose.covariance[35] = data.YawUncertainty

	        self.odom.twist.covariance[0] = data.VelUncertainty
	        self.odom.twist.covariance[7] = data.VelUncertainty
	        self.odom.twist.covariance[14] = data.VelUncertainty
	        
	        self.odom_pub.publish(self.odom)

	        self.firstRun = False

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        	print('Cannot lookupTransform')
  
    def pcd_callback(self, data):
    	self.pcd_out = data
    	self.pcd_out.header.frame_id = "lidar_link"
    	self.pcd_pub.publish(self.pcd_out)

if __name__ == '__main__':
    rospy.init_node('gps2XY', anonymous=True)
    gpsUtls = gpsUtilities()
    rospy.spin()
