#include "ros/ros.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "vn300/sensors.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <sstream>

#include <geometry_msgs/PoseStamped.h>

class stateEstimator {
private:
    ros::NodeHandle nh;
    std::string node_name;
    ros::Time time_stamp;

    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;

    ros::Publisher pose_pub;
    ros::Publisher path_pub;

    double acc_std, gps_std;

    Eigen::Matrix3d Rotn;
    Eigen::Vector3d acc_curr;
    Eigen::Vector3d gyro_curr;

    double yaw;
    Eigen::Quaterniond  quat;
    double dt_imu;
    bool isYawSet;
    int count;

    double x, vx, ax;
    double y, vy, ay;
    Eigen::Vector4d stateVector;
    Eigen::Vector2d u;
    Eigen::Vector2d z;
    Eigen::Matrix4d A;
    Eigen::MatrixXd B;
    Eigen::Matrix4d P;
    Eigen::Matrix2d Q;
    Eigen::Matrix2d R;
    Eigen::MatrixXd H;
    Eigen::MatrixXd K;

    nav_msgs::Path path_out;

public:
    stateEstimator(ros::NodeHandle( n)) {
        nh = n;
        node_name = ros::this_node::getName();

        gps_sub = nh.subscribe("/gps_pose", 1, &stateEstimator::gpsCallback, this);
        imu_sub = nh.subscribe("/vectornav/imu", 1, &stateEstimator::imuCallback, this);
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/estimator_pose", 1);
        path_pub = nh.advertise<nav_msgs::Path>("/estimator_path", 1);

        acc_std = readParam<double>(nh, "acc_std");
        gps_std = readParam<double>(nh, "gps_std");

        Rotn = Eigen::Matrix3d::Identity();
        Rotn(1, 1) = -1;
        Rotn(2, 2) = -1;
        acc_curr = Eigen::Vector3d::Zero();
        gyro_curr = Eigen::Vector3d::Zero();
        yaw = 0;
        dt_imu = 0.02;

        isYawSet = false;
        count = 0;

        x = y = 0;
        vx = vy = 0;
        ax = ay = 0;

        A = Eigen::Matrix4d::Identity();
        B = Eigen::MatrixXd(4, 2);
        stateVector = Eigen::Vector4d::Zero();
        u = Eigen::Vector2d::Zero();
        P = Eigen::Matrix4d::Identity();
        Q = Eigen::Matrix2d::Identity();
        Q(0, 0) = Q(1, 1) = acc_std*acc_std;
        R = Eigen::Matrix2d::Identity();
        R(0, 0) = R(1, 1) = gps_std*gps_std;
        H = Eigen::MatrixXd(2, 4);
        H << 1, 0, 0, 0,
             0, 1, 0, 0;
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name) {
        T ans;
        if (n.getParam(name, ans))
        {
            ROS_INFO_STREAM("[ "<< node_name << " ]: " << " Loaded " << name << ": " << ans);
        }
        else
        {
            ROS_ERROR_STREAM("[ "<< node_name << " ]: " << " Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    void posePublisher() {
        x = stateVector(0);
        y = stateVector(1);
        geometry_msgs::PoseStamped pose_out;
        path_out.header.frame_id = pose_out.header.frame_id = "world";
        path_out.header.stamp = pose_out.header.stamp = time_stamp;
        pose_out.pose.position.x = x;
        pose_out.pose.position.y = y;
        pose_out.pose.position.z = 0;
        pose_out.pose.orientation.x = quat.x();
        pose_out.pose.orientation.y = quat.y();
        pose_out.pose.orientation.z = quat.z();
        pose_out.pose.orientation.w = quat.w();
        pose_pub.publish(pose_out);
        path_out.poses.push_back(pose_out);
        path_pub.publish(path_out);
        ROS_INFO_STREAM("State: " << stateVector.transpose());
    }

    void predict() {
        yaw = yaw + dt_imu*gyro_curr.z();
        quat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())*
                                  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())*
                                  Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        u = Eigen::Vector2d(acc_curr.x(), acc_curr.y());
//        ROS_INFO_STREAM("IMU Callback yaw: " << yaw);
        double ctheta = cos(yaw);
        double stheta = sin(yaw);

        A << 1, 0, dt_imu, 0,
             0, 1, 0, dt_imu,
             0, 0, 1, 0,
             0, 0, 0, 1;

        B << 0.5*dt_imu*dt_imu*ctheta, -0.5*dt_imu*dt_imu*stheta,
             0.5*dt_imu*dt_imu*stheta, 0.5*dt_imu*dt_imu*ctheta,
             dt_imu*ctheta, -dt_imu*stheta,
             dt_imu*stheta, dt_imu*ctheta;

        stateVector = A*stateVector + B*u;
        P = A*P*A.transpose() + B*Q*B.transpose();
        posePublisher();
    }

    void gpsCallback(const geometry_msgs::PoseStampedConstPtr &gps_msg) {
        if(count%5 == 0) {
            Eigen::Quaterniond quat = Eigen::Quaterniond(gps_msg->pose.orientation.w, gps_msg->pose.orientation.x,
                                                         gps_msg->pose.orientation.y, gps_msg->pose.orientation.z);
            quat = quat.normalized();
            Eigen::Vector3d euler123 = quat.toRotationMatrix().eulerAngles(0, 1, 2);
            yaw = atan2(sin(euler123.z()), cos(euler123.z()));
//            ROS_WARN_STREAM("GPS Callback yaw: " << yaw);
            isYawSet = true;
            if(count != 0) {
                z = Eigen::Vector2d(gps_msg->pose.position.x,
                                    gps_msg->pose.position.y);
                Eigen::Vector2d innov = z - H*stateVector;
                Eigen::Matrix2d S = H*P*H.transpose() + R;
                K = P*H.transpose()*S.inverse();
                stateVector = stateVector + K*innov;
                P = P - K*H*P;
                P = 0.5*(P+P.transpose());// To ensure P is symmetric
                ROS_WARN_STREAM("State: " << stateVector.transpose());
            }
        }
    }

    void imuCallback(const vn300::sensorsConstPtr &imu_msg) {
        if (isYawSet) {
            time_stamp = imu_msg->header.stamp;
            acc_curr = Rotn*Eigen::Vector3d(imu_msg->Accel.x,
                    imu_msg->Accel.y,
                    imu_msg->Accel.z);
            gyro_curr = Rotn*Eigen::Vector3d(imu_msg->Gyro.x, imu_msg->Gyro.y, imu_msg->Gyro.z);
            predict();
            count++;
        } else {
            ROS_INFO_STREAM("Yaw is not set");
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "state_estimator_node");
    ros::NodeHandle nh("~");
    stateEstimator sE(nh);
    ros::spin();
    return 0;
}
