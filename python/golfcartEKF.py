#!/usr/bin/env python

import rospy
import math
import numpy as np
np.set_printoptions(precision=4)
np.core.arrayprint._line_width = 160

import message_filters
from math import pi
from geometry_msgs.msg import Point, Pose2D, Quaternion, Vector3, PoseStamped
from vn300.msg import gps, sensors
from pacmod_msgs.msg import VehicleSpeedRpt, SystemRptFloat
import tf
from nav_msgs.msg import Odometry, Path

class GolfCartEKF:

    def __init__(self):

        rospy.init_node('GolfCartEKF')
        # Parameters
        self.L = 3.4
        self.SR = 22

        # create subscribers
        self.vn300_imu_sub      = rospy.Subscriber('/vectornav/imu',    sensors,    self.predict_step_imu)
        self.vn300_gps_sub      = rospy.Subscriber('/vectornav/gps',    gps,        self.update_step_gps)

        self.pacmod_speed_sub   = message_filters.Subscriber('/pacmod/parsed_tx/vehicle_speed_rpt',     VehicleSpeedRpt)
        self.pacmod_steer_sub   = message_filters.Subscriber('/pacmod/parsed_tx/steer_rpt',             SystemRptFloat)

        ts = message_filters.ApproximateTimeSynchronizer([self.pacmod_speed_sub, self.pacmod_steer_sub], 10, 0.03)
        ts.registerCallback(self.update_step_pacmod)

        self.state_pub = rospy.Publisher('golfcartEKF/state', Odometry, queue_size=10)
        self.path_pub  = rospy.Publisher('golfcartEKF/path',  Path,     queue_size=10, latch=True)

        self.state_odom = Odometry()
        self.state_odom.header.frame_id = 'golfcart_frame'
        self.state_odom.child_frame_id = 'world'

        self.br = tf.TransformBroadcaster()

        self.accel_time = rospy.rostime.Time()
        self.accel_time_init = False
        self.pacmod_time = rospy.rostime.Time()
        self.pacmod_time_init = False
        self.last_gps_time = 0.0
        
        self.dE_prev  = 0.0
        self.dN_prev  = 0.0
        self.lon_home = 0.0
        self.lat_home = 0.0
        self.lon_lat_init = False

        # Index - Variable- Initial Value:
        #     0 - x       - 0
        #     1 - y       - 0
        #     2 - theta   - 0
        #     3 - vx      - 0
        #     4 - vy      - 0
        #     5 - omega   - 0


        self.state = np.zeros([6,1])
        self.state[2] = -pi/2

        self.Q = np.zeros([6,6])
        np.fill_diagonal( self.Q, [0.1, 0.1, 0.1, 0.01, 0.01, 0.01] )

        self.state_cov = self.Q
        
        self.R_pcm = np.zeros([3,3])
        np.fill_diagonal( self.R_pcm, [0.05, 0.07, 0.01] )

        self.R_gps = np.zeros([3,3])
        np.fill_diagonal( self.R_gps, [0.4, 0.4, 0.75] )

    def predict_step_imu(self, msg):
        if self.accel_time_init:
            dt = msg.header.stamp.to_sec() - self.accel_time
            F = np.identity(6)
            F[0,3] = dt 
            F[1,4] = dt 
            F[5,5] = 0.0

            B = np.zeros([6,3])
            B[0,0] =  (dt**2)/2*math.cos(self.state[2])
            B[0,1] = -(dt**2)/2*math.sin(self.state[2])
            B[1,0] =  (dt**2)/2*math.sin(self.state[2])
            B[1,1] =  (dt**2)/2*math.cos(self.state[2])
            B[2,2] =  dt 
            B[3,0] =  dt*math.cos(self.state[2])
            B[3,1] = -dt*math.sin(self.state[2])
            B[4,0] =  dt*math.sin(self.state[2])
            B[4,1] =  dt*math.cos(self.state[2])
            B[5,2] =  1.0

            U = np.array([[msg.Accel.x],[msg.Accel.y],[-msg.Gyro.z]])

            self.state = np.matmul(F, self.state) + np.matmul(B, U)
            
            # P_k|k-1 = F_k * P_k-1|k-1 * F_k^T + Q_k
            self.state_cov = np.matmul(F, np.matmul(self.state_cov, F.T)) + self.Q

        else: 
            self.accel_time_init = True
        
        self.accel_time = msg.header.stamp.to_sec()
        
    
    def update_step_pacmod(self, speed_msg, steer_msg):
        
        if self.pacmod_time_init:

            z = np.array([  [speed_msg.vehicle_speed * math.cos(self.state[2])] , \
                            [speed_msg.vehicle_speed * math.sin(self.state[2])] , \
                            [speed_msg.vehicle_speed / self.L * math.tan(steer_msg.output / self.SR) ] ])

            self.H_pcm = np.zeros([3,6])
            self.H_pcm[0,3] = 1.0
            self.H_pcm[1,4] = 1.0
            self.H_pcm[2,5] = 1.0

            # y_k       = z_k - h(xhat_k|k-1)
            self.y_pcm = z - self.state[3:6]
            # print(self.y_pcm.T)

            # # S_k       = H_k * P_k|k-1 * H_k^T + R_kError
            self.S_pcm = np.matmul(self.H_pcm, np.matmul(self.state_cov, self.H_pcm.T)) + self.R_pcm 
            # print(self.S_pcm)

            # # K_k       = P_k|k-1 * H_k^T * S_k^-1
            self.K_pcm = np.matmul(self.state_cov, np.matmul(self.H_pcm.T, np.linalg.inv(self.S_pcm)))
            # print(self.K_pcm)

            # # xhat_k|k  = xhat_k|k-1 + K_k * y_k
            self.state = self.state + np.matmul(self.K_pcm, self.y_pcm)

            # # P_k|k     = (I - K_k * H_k) * P_k|k-1
            self.state_cov = np.matmul((np.eye(6) - np.matmul(self.K_pcm, self.H_pcm)), self.state_cov)

        else: 
            self.pacmod_time_init = True

        self.pacmod_time = speed_msg.header.stamp.to_sec()
        

    def update_step_gps(self, msg):

        if not self.lon_lat_init:
            self.lon_lat_init = True
            self.lat_home = msg.LLA.x
            self.lon_home = msg.LLA.y

        elif not msg.UtcTime.millisecond == self.last_gps_time:
            
            # Ellipsoidal model calculation of Earth
            a = 6378137.0
            b = 6356752.3142
            R = math.sqrt(( math.pow( math.pow(a,2) * math.cos(msg.LLA.x*pi/180), 2)+ \
                            math.pow( math.pow(b,2) * math.sin(msg.LLA.x*pi/180), 2))/ \
                           (math.pow( a * math.cos(msg.LLA.x*pi/180), 2)+ \
                            math.pow( b * math.sin(msg.LLA.x*pi/180), 2))) + msg.LLA.z

            # Transform LLH to 2D position
            dlon = msg.LLA.y - self.lon_home    # degrees
            dlat = msg.LLA.x - self.lat_home    # degrees
            dE = R * math.radians(dlon)     # meters
            dN = R * math.radians(dlat)     # meters
            dT = math.atan2(dN - self.dN_prev, dE - self.dE_prev)
            self.dE_prev = dE
            self.dN_prev = dN

            # # Measurement in matrix format
            z = np.array([[dE],[dN],[dT]])

            self.H_gps = np.zeros([3,6])
            self.H_gps[0,0] = 1 
            self.H_gps[1,1] = 1 
            self.H_gps[2,2] = 1

            # # y_k       = z_k - h(xhat_k|k-1)
            self.y_gps = z - np.matmul(self.H_gps, self.state)

            # # S_k       = H_k * P_k|k-1 * H_k^T + R_kError
            self.S_gps = np.matmul(self.H_gps, np.matmul(self.state_cov, self.H_gps.T)) + self.R_gps

            # # K_k       = P_k|k-1 * H_k^T * S_k^-1
            self.K_gps = np.matmul(self.state_cov, np.matmul(self.H_gps.T, np.linalg.inv(self.S_gps)))

            # # xhat_k|k  = xhat_k|k-1 + K_k * y_k
            self.state = self.state + np.matmul(self.K_gps, self.y_gps)
            

            # # P_k|k     = (I - K_k * H_k) * P_k|k-1
            self.state_cov = np.matmul((np.eye(6) - np.matmul(self.K_gps, self.H_gps)), self.state_cov)

            self.last_gps_time = msg.UtcTime.millisecond  

        self.state_pacmod = self.state
        self.pacmod_time = msg.header.stamp.to_sec()
        self.accel_time  = msg.header.stamp.to_sec()


    def run(self):

        rate = rospy.Rate(50) # 10 Hz
        while(not rospy.is_shutdown()):
            self.state_odom.header.stamp = rospy.Time.now()
            self.state_odom.pose.pose.position = Point(self.state[0],self.state[1],0)
            self.state_odom.pose.pose.orientation = Quaternion(0, 0, math.sin(self.state[2]/2), math.cos(self.state[2]/2))
            self.state_odom.pose.covariance = self.state_cov[0:6,0:6].flatten()
            
            self.state_odom.twist.twist.linear = Vector3(self.state[3],self.state[4],0)
            self.state_odom.twist.twist.angular = Vector3(0,0,self.state[5])
            self.state_odom.twist.covariance = self.state_cov[0:6,0:6].flatten()

            self.state_pub.publish( self.state_odom )

            self.br.sendTransform(( 0, 0, 0),
                                    tf.transformations.quaternion_from_euler(0, 0, 0),
                                    rospy.Time.now(),
                                    self.state_odom.header.frame_id,
                                    "world")

            rate.sleep()

class OdomToPath:
    def __init__(self):
        self.path_pub = rospy.Publisher('/golfcartEKF/path', Path, latch=True, queue_size=10)
        self.odom_sub = rospy.Subscriber('/golfcartEKF/state', Odometry, self.odom_cb, queue_size=10)
        self.path = Path()

    def odom_cb(self, msg):
        cur_pose = PoseStamped()
        cur_pose.header = msg.header
        cur_pose.pose = msg.pose.pose
        self.path.header = msg.header
        self.path.poses.append(cur_pose)
        self.path_pub.publish(self.path)


if __name__ == '__main__':
    GolfCart_EKF = GolfCartEKF()
    odom_to_path = OdomToPath()
    GolfCart_EKF.run()
    