#include "ros/ros.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "pacmod_msgs/VehicleSpeedRpt.h"
#include "pacmod_msgs/SystemRptFloat.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sstream>

typedef message_filters::sync_policies::ApproximateTime<
        pacmod_msgs::VehicleSpeedRpt,
        pacmod_msgs::SystemRptFloat> SyncPolicy;

class kalmanFilterNode {
private:
    ros::NodeHandle nh;

    message_filters::Subscriber<pacmod_msgs::VehicleSpeedRpt> *velocity_sub;
    message_filters::Subscriber<pacmod_msgs::SystemRptFloat> *steering_sub;
    message_filters::Synchronizer<SyncPolicy> *sync;

    std::string node_name;
    ros::Time time_stamp;

    ros::Subscriber gpsPose_sub;
    ros::Publisher pose_pub;
    ros::Publisher path_pub;

    geometry_msgs::PoseStamped pose;
    nav_msgs::Path path;

    // Jacobians
    Eigen::Matrix3d Ak;
    Eigen::MatrixXd Bk;
    Eigen::MatrixXd Hk;

    // Noise Params
    Eigen::Matrix2d Qk; // Process Error
    Eigen::Matrix2d Rk; // Measurement Error

    // parameters
    double L, dt;

    // states
    double x, y, theta;

    // state covariance
    Eigen::Matrix3d P;

    // Kalman Gain
    Eigen::MatrixXd K;

    // Noise Params
    double p00, p11;
    double q00, q11;
    double r00, r11;

    //counter
    int count;
public:
    kalmanFilterNode(ros::NodeHandle( n)) {
        nh = n;
        node_name = ros::this_node::getName();
        L = readParam<double>(nh, "L");
        dt = readParam<double>(nh, "dt");
        p00 = readParam<double>(nh, "p00");
        p11 = readParam<double>(nh, "p11");
        q00 = readParam<double>(nh, "q00");
        q11 = readParam<double>(nh, "q11");
        r00 = readParam<double>(nh, "r00");
        r11 = readParam<double>(nh, "r11");

        velocity_sub = new message_filters::Subscriber<pacmod_msgs::VehicleSpeedRpt>(nh, "/pacmod/parsed_tx/vehicle_speed_rpt", 1);
        steering_sub = new message_filters::Subscriber<pacmod_msgs::SystemRptFloat>(nh, "/pacmod/parsed_tx/steer_rpt", 1);
        sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *velocity_sub, *steering_sub);
        sync->registerCallback(boost::bind(&kalmanFilterNode::controlCallback, this, _1, _2));

        gpsPose_sub = nh.subscribe("/gps_pose", 1, &kalmanFilterNode::measurementCallback, this);

        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 1);
        path_pub = nh.advertise<nav_msgs::Path>("/estimated_path", 1);

        Qk = Eigen::Matrix2d::Identity();
        Qk(0, 0) = q00; // Tuning Paramater
        Qk(1, 1) = q11; // Tuning Parameter
        Rk = Eigen::Matrix2d::Identity();
        Rk(0, 0) = r00; // Tuning Parameter
        Rk(1, 1) = r11; // Tuning Parameter

        x = y = theta = 0;
        P = Eigen::Matrix3d::Identity();
        P(0, 0) = p00;
        P(1, 1) = p11;
        K = Eigen::MatrixXd(3, 2);

        count= 0;
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
        Eigen::AngleAxisd yawAngle(theta, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond quat_theta(yawAngle);

        pose.header.stamp = time_stamp;
        pose.header.frame_id = "world";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = quat_theta.x();
        pose.pose.orientation.y = quat_theta.y();
        pose.pose.orientation.z = quat_theta.z();
        pose.pose.orientation.w = quat_theta.w();

        pose_pub.publish(pose);
    }

    void pathPublisher() {
        path.header.frame_id = "world";
        path.header.stamp = time_stamp;
        path.poses.push_back(pose);
        path_pub.publish(path);
    }

    void controlCallback(const pacmod_msgs::VehicleSpeedRptConstPtr &velocity_msg,
                  const pacmod_msgs::SystemRptFloatConstPtr &steering_msg) {
        ROS_INFO_STREAM("Prediction");
        time_stamp = velocity_msg->header.stamp;
        pacmod_msgs::VehicleSpeedRpt velocity_data = *velocity_msg;
        pacmod_msgs::SystemRptFloat steering_data = *steering_msg;

        double vehicle_speed = velocity_data.vehicle_speed;
        double steering = steering_data.output;

        steering = atan2(sin(steering/20), cos(steering/20));

        x = x + vehicle_speed*cos(theta)*dt;
        y = y + vehicle_speed*sin(theta)*dt;
        theta = theta - (vehicle_speed/L)*tan(steering)*dt;
        theta = atan2(sin(theta), cos(theta));

        Ak = Eigen::Matrix3d::Identity();
        Ak(0, 2) = -vehicle_speed*sin(theta)*dt;
        Ak(1, 2) = vehicle_speed*cos(theta)*dt;

        Bk = Eigen::MatrixXd(3, 2);
        Bk << cos(theta)*dt, 0,
              sin(theta)*dt, 0,
              -tan(steering)*dt/L, -vehicle_speed*dt/(L*cos(steering)*cos(steering));

        P = Ak*P*Ak.transpose() + Bk*Qk*Bk.transpose();
        posePublisher();
        pathPublisher();
        count++;
    }

    void measurementCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg) {
        if(count == 5) {
            ROS_INFO_STREAM("Measurement");
            ROS_INFO_STREAM("trace(P) before incorporating measurement: " << P.trace());
            time_stamp = pose_msg->header.stamp;
            geometry_msgs::PoseStamped gpsPose_data = *pose_msg;
            Hk = Eigen::MatrixXd(2, 3);
            Hk << 1, 0, 0,
                    0, 1, 0;
            Eigen::MatrixXd Sk(2, 2);
            Sk = Hk*P*Hk.transpose() + Rk;
            K = P*Hk.transpose()*Sk.inverse();
            Eigen::Vector2d measurement = Eigen::Vector2d(gpsPose_data.pose.position.x, gpsPose_data.pose.position.y);
            Eigen::Vector3d predicted_state = Eigen::Vector3d(x, y, theta);
            Eigen::Vector2d innovation = measurement - Hk*predicted_state;
            Eigen::Vector3d estimated_state = predicted_state + K*innovation;
            P = P - K*Hk*P;
            P = 0.5*(P+P.transpose());// To ensure P is symmetric
            ROS_INFO_STREAM("trace(P) after incorporating measurement: " << P.trace());
            x = estimated_state(0);
            y = estimated_state(1);
            theta = estimated_state(2);
            posePublisher();
            pathPublisher();
            count = 0;
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "kalman_filter_node");
    ros::NodeHandle nh("~");
    kalmanFilterNode kFM(nh);
    ros::spin();
    return 0;
}