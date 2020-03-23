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

typedef message_filters::sync_policies::ApproximateTime<pacmod_msgs::VehicleSpeedRpt,
        pacmod_msgs::SystemRptFloat> SyncPolicy;

class kinematicModel {
private:
    ros::NodeHandle nh;

    message_filters::Subscriber<pacmod_msgs::VehicleSpeedRpt> *velocity_sub;
    message_filters::Subscriber<pacmod_msgs::SystemRptFloat> *steering_sub;
    message_filters::Synchronizer<SyncPolicy> *sync;

    ros::Publisher pose_pub;
    ros::Publisher path_pub;

    geometry_msgs::PoseStamped pose;
    nav_msgs::Path path;

    std::string node_name;
    ros::Time time_stamp;
    // parameters
    double L, dt;

    // states
    double x, y, theta;

public:
    kinematicModel(ros::NodeHandle( n)) {
        nh = n;
        node_name = ros::this_node::getName();
        L = readParam<double>(nh, "L");
        dt = readParam<double>(nh, "dt");

        velocity_sub = new message_filters::Subscriber<pacmod_msgs::VehicleSpeedRpt>(nh, "/pacmod/parsed_tx/vehicle_speed_rpt", 1);
        steering_sub = new message_filters::Subscriber<pacmod_msgs::SystemRptFloat>(nh, "/pacmod/parsed_tx/steer_rpt", 1);
        sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *velocity_sub, *steering_sub);
        sync->registerCallback(boost::bind(&kinematicModel::callback, this, _1, _2));

        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/kinematic_pose", 1);
        path_pub = nh.advertise<nav_msgs::Path>("/kinematic_path", 1);
        x = y = theta = 0;
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

    void callback(const pacmod_msgs::VehicleSpeedRptConstPtr &velocity_msg,
            const pacmod_msgs::SystemRptFloatConstPtr &steering_msg) {
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
        posePublisher();
        pathPublisher();
    }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "kinematic_model");
  ros::NodeHandle nh("~");
  kinematicModel kM(nh);
  ros::spin();
  return 0;
}
