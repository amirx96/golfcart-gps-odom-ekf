#include "ros/ros.h"

#include <eigen3/Eigen/Core>


#include "pacmod_msgs/VehicleSpeedRpt.h"
#include "geometry_msgs/PoseStamped.h"
#include "vn300/sensors.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <fstream>

#include <sstream>

typedef message_filters::sync_policies::ApproximateTime<
        pacmod_msgs::VehicleSpeedRpt,
        geometry_msgs::PoseStamped,
        vn300::sensors> SyncPolicy;

class estimateSensorNoise {
private:
    ros::NodeHandle nh;

    message_filters::Subscriber<pacmod_msgs::VehicleSpeedRpt> *velocity_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> *gps_sub;
    message_filters::Subscriber<vn300::sensors> *imu_sub;
    message_filters::Synchronizer<SyncPolicy> *sync;

    std::string node_name;
    double curr_velocity;
    double prev_velocity;

    std::vector<Eigen::Vector3d> acc_xyz_vec;
    std::vector<Eigen::Vector2d> gps_xy_vec;

public:
    estimateSensorNoise(ros::NodeHandle( n)) {
        nh = n;
        node_name = ros::this_node::getName();
        velocity_sub = new message_filters::Subscriber<pacmod_msgs::VehicleSpeedRpt>(nh, "/pacmod/parsed_tx/vehicle_speed_rpt", 1);
        gps_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, "/gps_pose", 1);
        imu_sub = new message_filters::Subscriber<vn300::sensors>(nh, "/vectornav/imu", 1);
        sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *velocity_sub, *gps_sub, *imu_sub);
        sync->registerCallback(boost::bind(&estimateSensorNoise::sensorCallback, this, _1, _2, _3));
        curr_velocity = 0;
        prev_velocity = 0;
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

    void sensorCallback(const pacmod_msgs::VehicleSpeedRptConstPtr &velocity_msg,
                        const geometry_msgs::PoseStampedConstPtr &gps_msg,
                        const vn300::sensorsConstPtr &sensor_msg) {
        curr_velocity = velocity_msg->vehicle_speed;
        if (curr_velocity == 0 && prev_velocity == 0) {
            Eigen::Vector3d acc_xyz = Eigen::Vector3d(sensor_msg->Accel.x,
                                                      sensor_msg->Accel.y,
                                                      sensor_msg->Accel.z);
            Eigen::Vector2d gps_xy = Eigen::Vector2d(gps_msg->pose.position.x,
                                                     gps_msg->pose.position.y);
            acc_xyz_vec.push_back(acc_xyz);
            gps_xy_vec.push_back(gps_xy);
        } else {
            ROS_ASSERT(acc_xyz_vec.size() == gps_xy_vec.size());
            int no_of_frames = acc_xyz_vec.size();
            if (no_of_frames > 300) {

                std::ofstream file;
                file.open("/home/usl/catkin_ws/src/robotperception/estimate_sensor_noise/results/accelgps.csv");
                double mean_acc_x = 0;
                double mean_acc_y = 0;
                double mean_acc_z = 0;
                double mean_gps_x = 0;
                double mean_gps_y = 0;
                for (int i = 0; i < no_of_frames; i++) {
                    mean_acc_x += acc_xyz_vec[i].x();
                    mean_acc_y += acc_xyz_vec[i].y();
                    mean_acc_z += acc_xyz_vec[i].z();
                    mean_gps_x += gps_xy_vec[i].x();
                    mean_gps_y += gps_xy_vec[i].y();
                    file << acc_xyz_vec[i].x() << ", "
                         << acc_xyz_vec[i].y() << ", "
                         << acc_xyz_vec[i].z() << ", "
                         << gps_xy_vec[i].x() << ", "
                         << gps_xy_vec[i].y() << std::endl;
                }
                file.close();
                ROS_WARN_STREAM("Wrote to file");
                mean_acc_x /= no_of_frames;
                mean_acc_y /= no_of_frames;
                mean_acc_z /= no_of_frames;
                mean_gps_x /= no_of_frames;
                mean_gps_y /= no_of_frames;

                double err_acc_x = 0;
                double err_acc_y = 0;
                double err_acc_z = 0;
                double err_gps_x = 0;
                double err_gps_y = 0;
                for (int i = 0; i < no_of_frames; i++) {
                    err_acc_x += (acc_xyz_vec[i].x() - mean_acc_x)*(acc_xyz_vec[i].x() - mean_acc_x);
                    err_acc_y += (acc_xyz_vec[i].y() - mean_acc_y)*(acc_xyz_vec[i].y() - mean_acc_y);
                    err_acc_z += (acc_xyz_vec[i].z() - mean_acc_z)*(acc_xyz_vec[i].z() - mean_acc_z);
                    err_gps_x += (gps_xy_vec[i].x() - mean_gps_x)*(gps_xy_vec[i].x() - mean_gps_x);
                    err_gps_y += (gps_xy_vec[i].y() - mean_gps_y)*(gps_xy_vec[i].y() - mean_gps_y);
                }
                double N = no_of_frames - 1;
                double std_acc_x = sqrt(err_acc_x/N);
                double std_acc_y = sqrt(err_acc_y/N);
                double std_acc_z = sqrt(err_acc_z/N);
                double std_gps_x = sqrt(err_gps_x/N);
                double std_gps_y = sqrt(err_gps_y/N);
                ROS_INFO_STREAM("std_acc_x = " << std_acc_x);
                ROS_INFO_STREAM("std_acc_y = " << std_acc_y);
                ROS_INFO_STREAM("std_acc_z = " << std_acc_z);
                ROS_INFO_STREAM("std_gps_x = " << std_gps_x);
                ROS_INFO_STREAM("std_gps_y = " << std_gps_y);
                ros::shutdown();
            }
            acc_xyz_vec.clear();
            gps_xy_vec.clear();
        }
        prev_velocity = curr_velocity;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "kalman_filter_node");
    ros::NodeHandle nh("~");
    estimateSensorNoise eSN(nh);
    ros::spin();
    return 0;
}