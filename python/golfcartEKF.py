#!/usr/bin/env python

import rospy
import math
import numpy as np
np.set_printoptions(precision=4)
np.core.arrayprint._line_width = 160

import message_filters
from math import pi
from geometry_msgs.msg import Point, Pose2D, Quaternion, Vector3
from vn300.msg import gps, sensors
from pacmod_msgs.msg import VehicleSpeedRpt, SystemRptFloat
import tf
from nav_msgs.msg import Odometry

# (24 -45)
# (79 -2)
class GolfCartEKF:

    def __init__(self):

        rospy.init_node('GolfCartEKF')
        # Parameters
        self.wheelbase = 3.4

        # create subscribers
        self.vn300_imu_sub      = rospy.Subscriber('/vectornav/imu',    sensors,    self.predict_step_imu)
        self.vn300_gps_sub      = rospy.Subscriber('/vectornav/gps',    gps,        self.update_step_gps)

        self.pacmod_speed_sub   = message_filters.Subscriber('/pacmod/parsed_tx/vehicle_speed_rpt',     VehicleSpeedRpt)
        self.pacmod_steer_sub   = message_filters.Subscriber('/pacmod/parsed_tx/steer_rpt',             SystemRptFloat)

        ts = message_filters.ApproximateTimeSynchronizer([self.pacmod_speed_sub, self.pacmod_steer_sub], 10, 0.03)
        ts.registerCallback(self.update_step_pacmod)


        self.state_pub = rospy.Publisher('golfcartEKF/state', Odometry, queue_size=10)
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

        # Index - State Variable - Initial Value:
        #     0 - x       - 0
        #     1 - y       - 0
        #     2 - theta   - 0
        #     3 - vx      - 0
        #     4 - vy      - 0
        #     5 - omega   - 0

        #     6 - IMU_x   - 2.5
        #     7 - IMU_y   - 0.5

        #     8 - SR      - 22

        #     9  - Maxy   - 0
        #     10 - Maxz   - 0
        #     11 - Mayx   - 0
        #     12 - Mayz   - 0
        #     13 - Mazx   - 0
        #     14 - Mazy   - 0
        #     15 - Sax    - 1
        #     16 - Say    - 1
        #     17 - Saz    - 1
        #     18 - Bax    - 0
        #     19 - Bay    - 0
        #     20 - Baz    - 0

        #     21 - Mgxy   - 0
        #     22 - Mgxz   - 0
        #     23 - Mgyx   - 0
        #     24 - Mgyz   - 0
        #     25 - Mgzx   - 0
        #     26 - Mgzy   - 0
        #     27 - Sgx    - 1
        #     28 - Sgy    - 1
        #     29 - Sgz    - 1
        #     30 - Bgx    - 0
        #     31 - Bgy    - 0
        #     32 - Bgz    - 0

        self.state = np.zeros([33,1])
        self.state[5]  = 0.001
        self.state[6]  = 2.5
        self.state[7]  = 0.5
        self.state[8]  = 22.0
        self.state[9]  = 1.0
        self.state[16] = 1.0
        self.state[17] = 1.0
        self.state[18] = 1.0
        self.state[28] = 1.0
        self.state[29] = 1.0
        self.state[30] = 1.0
        self.state_pacmod = self.state

        self.Q = np.zeros([33,33])
        np.fill_diagonal( self.Q, \
            np.concatenate(( \
                np.repeat(10,3), \
                np.repeat(10,3), \
                np.repeat(1,27))))

        self.state_cov = self.Q

        self.R_gps = np.identity(3) * 2.0
        
        self.R_pcm = np.zeros([7,7])
        np.fill_diagonal( self.R_pcm, 100 )


    def predict_step_imu(self, msg):
        if self.accel_time_init:
            dt = msg.header.stamp.to_sec() - self.accel_time
            F = np.identity(33)
            F[0,3] = dt 
            F[1,4] = dt 
            F[5,5] = 0.0

            B = np.zeros([33,3])
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

            U = np.array([[msg.Accel.x],[msg.Accel.y],[msg.Gyro.z]])
            
            self.state = np.matmul(F, self.state) + np.matmul(B, U)
            
            # P_k|k-1 = F_k * P_k-1|k-1 * F_k^T + Q_k
            self.state_cov = np.matmul(F, np.matmul(self.state_cov, F.T)) + self.Q

        else: 
            self.accel_time_init = True
        
        self.accel_time = msg.header.stamp.to_sec()
        
    
    def update_step_pacmod(self, speed_msg, steer_msg):
        
        if self.pacmod_time_init:
            # print('\nPacmod')
            dt = speed_msg.header.stamp.to_sec() - self.pacmod_time
            F = np.identity(7)
            F[0,3] = dt 
            F[1,4] = dt 
            F[2,5] = dt

            X = np.array( [ self.state_pacmod[0], \
                            self.state_pacmod[1], \
                            self.state_pacmod[2], \
                          [ speed_msg.vehicle_speed*math.cos(self.state_pacmod[2])], \
                          [ speed_msg.vehicle_speed*math.sin(self.state_pacmod[2])], \
                          [ speed_msg.vehicle_speed*math.tan(-steer_msg.output/self.state[8])/self.wheelbase], \
                            self.state[8]])

            v = math.sqrt(self.state[3]**2 + self.state[4]**2)

            # print(v)
            # print(speed_msg.vehicle_speed)
            self.H_pcm = np.zeros([7,33])
            self.H_pcm[0,0] = 1.0
            self.H_pcm[1,1] = 1.0
            self.H_pcm[2,2] = 1.0
            self.H_pcm[3,3] = 1.0
            self.H_pcm[4,4] = 1.0
            self.H_pcm[5,5] = 1.0
            self.H_pcm[6,8] = 1.0 

            
            if not v == 0:
                self.H_pcm[0,3] =  dt * math.cos(self.state_pacmod[2]) / v * self.state[3]
                self.H_pcm[0,4] =  dt * math.cos(self.state_pacmod[2]) / v * self.state[4]
                self.H_pcm[0,3] =  dt * math.sin(self.state_pacmod[2]) / v * self.state[3]
                self.H_pcm[0,4] =  dt * math.sin(self.state_pacmod[2]) / v * self.state[4]
                self.H_pcm[3,3] =       math.cos(self.state_pacmod[2]) / v * self.state[3]
                self.H_pcm[3,4] =       math.cos(self.state_pacmod[2]) / v * self.state[4]
                self.H_pcm[4,3] =       math.sin(self.state_pacmod[2]) / v * self.state[3]
                self.H_pcm[4,4] =       math.sin(self.state_pacmod[2]) / v * self.state[4]



            # y_k       = z_k - h(xhat_k|k-1)
            self.y_pcm = np.matmul( F, X ) - self.state[np.array([0,1,2,3,4,5,8])]
            print('\n')
            print(self.state[8])
            print(- steer_msg.output / math.atan2( self.wheelbase * self.state[5], speed_msg.vehicle_speed ))
            print(self.state[8] + steer_msg.output / math.atan2( self.wheelbase * self.state[5], speed_msg.vehicle_speed ))
            
            if abs(steer_msg.output) > 2.0 and not speed_msg.vehicle_speed == 0.0 and not self.state[5] == 0:
                self.H_pcm[6,3] = -steer_msg.output * self.wheelbase * self.state[5] * self.state[3] / ( math.atan2(self.wheelbase * self.state[5], v) * (self.wheelbase**2 * self.state[5]**2 + v**2) * v )
                self.H_pcm[6,4] = -steer_msg.output * self.wheelbase * self.state[5] * self.state[4] / ( math.atan2(self.wheelbase * self.state[5], v) * (self.wheelbase**2 * self.state[5]**2 + v**2) * v )
                self.H_pcm[6,5] =  steer_msg.output * self.wheelbase * self.state[5] *       v       / ( math.atan2(self.wheelbase * self.state[5], v) * (self.wheelbase**2 * self.state[5]**2 + v**2)     )
                self.y_pcm[6]   = -self.state[8] - steer_msg.output / math.atan2( self.wheelbase * self.state[5], speed_msg.vehicle_speed )


            # # S_k       = H_k * P_k|k-1 * H_k^T + R_kError
            self.S_pcm = np.matmul(self.H_pcm, np.matmul(self.state_cov, self.H_pcm.T)) + self.R_pcm 


            # # K_k       = P_k|k-1 * H_k^T * S_k^-1
            self.K_pcm = np.matmul(self.state_cov, np.matmul(self.H_pcm.T, np.linalg.inv(self.S_pcm)))


            # # xhat_k|k  = xhat_k|k-1 + K_k * y_k
            self.state = self.state + np.matmul(self.K_pcm, self.y_pcm)

            # # P_k|k     = (I - K_k * H_k) * P_k|k-1
            self.state_cov = np.matmul((np.eye(33) - np.matmul(self.K_pcm, self.H_pcm)), self.state_cov)
            

        else: 
            self.pacmod_time_init = True

        self.state_pacmod = self.state
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
            measurement = np.array([[dE],[dN],[dT]])

            self.H_gps = np.zeros([3,33])
            self.H_gps[0,0] = 1 
            self.H_gps[1,1] = 1 
            self.H_gps[2,2] = 1

            # # y_k       = z_k - h(xhat_k|k-1)
            self.y_gps = measurement - np.matmul(self.H_gps, self.state)

            # # S_k       = H_k * P_k|k-1 * H_k^T + R_kError
            self.S_gps = np.matmul(self.H_gps, np.matmul(self.state_cov, self.H_gps.T)) + self.R_gps

            # # K_k       = P_k|k-1 * H_k^T * S_k^-1
            self.K_gps = np.matmul(self.state_cov, np.matmul(self.H_gps.T, np.linalg.inv(self.S_gps)))

            # # xhat_k|k  = xhat_k|k-1 + K_k * y_k
            self.state = self.state + np.matmul(self.K_gps, self.y_gps)
            

            # # P_k|k     = (I - K_k * H_k) * P_k|k-1
            self.state_cov = np.matmul((np.eye(33) - np.matmul(self.K_gps, self.H_gps)), self.state_cov)

            self.last_gps_time = msg.UtcTime.millisecond  

        self.state_pacmod = self.state
        self.pacmod_time = msg.header.stamp.to_sec()
        self.accel_time  = msg.header.stamp.to_sec()


    def run(self):

        rate = rospy.Rate(50) # 10 Hz
        while(not rospy.is_shutdown()):

            self.state_odom.header.stamp = rospy.Time.now()
            self.state_odom.pose.pose.position = Point(self.state[0],self.state[1],0)
            self.state_odom.pose.pose.orientation = Quaternion(0,0,math.sin(self.state[2]/2),math.cos(self.state[2]/2))
            self.state_odom.pose.covariance = self.state_cov[0:6,0:6].flatten()
            
            self.state_odom.twist.twist.linear = Vector3(self.state[3],self.state[4],0)
            self.state_odom.twist.twist.angular = Vector3(0,0,self.state[5])
            self.state_odom.twist.covariance = self.state_cov[0:6,0:6].flatten()

            self.state_pub.publish( self.state_odom )

            self.br.sendTransform(( self.state[0], self.state[1], 0),
                                    tf.transformations.quaternion_from_euler(0, 0, 0),
                                    rospy.Time.now(),
                                    self.state_odom.header.frame_id,
                                    "world")

            rate.sleep()


if __name__ == '__main__':
    GolfCart_EKF = GolfCartEKF()
    GolfCart_EKF.run()