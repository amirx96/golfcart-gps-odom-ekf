#include "ros/ros.h"

#include "vn300/sensors.h"
#include "vn300/ins.h"

#include <sstream>
#include <iostream>
#include <fstream>

class bag2CSV {
private:
    ros::NodeHandle nh;
    std::string node_name;
    double start_time_stamp;
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;

    std::ofstream gps_file;
    std::ofstream accel_gyro_file;
    std::ofstream  orientation_file;
    std::ofstream velocity_file;

    double time_stamp;
    bool isYawSet;
    int count;
    bool isFirstFrame;
public:
    bag2CSV(ros::NodeHandle( n)) {
        nh = n;
        node_name = ros::this_node::getName();

        gps_sub = nh.subscribe("/vectornav/ins", 1, &bag2CSV::insCallback, this);
        imu_sub = nh.subscribe("/vectornav/imu", 1, &bag2CSV::imuCallback, this);

        start_time_stamp = 0;

        time_stamp = 0;
        isYawSet = false;
        isFirstFrame = true;
        count = 0;

        gps_file.open("/home/usl/catkin_ws/src/robotperception/bag2csv/result/gps.csv");
        accel_gyro_file.open("/home/usl/catkin_ws/src/robotperception/bag2csv/result/accel_gyro.csv");
        orientation_file.open("/home/usl/catkin_ws/src/robotperception/bag2csv/result/orentation.csv");
        velocity_file.open("/home/usl/catkin_ws/src/robotperception/bag2csv/result/velocity.csv");
        time_stamp = 0;
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name) {
        T ans;
        if (n.getParam(name, ans)) {
            ROS_INFO_STREAM("[ "<< node_name << " ]: " << " Loaded " << name << ": " << ans);
        }
        else {
            ROS_ERROR_STREAM("[ "<< node_name << " ]: " << " Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    void insCallback(const vn300::insConstPtr &ins_msg) {
        if(count%10 == 0) {
            isYawSet = true;
            if(!isFirstFrame) {
                time_stamp = ros::Time::now().toSec() - start_time_stamp;
            } else {
                isFirstFrame = false;
                start_time_stamp = ros::Time::now().toSec();
                time_stamp = 0;
            }
            double gps_lat, gps_long, gps_alt, r_x, r_y, r_z, v_n, v_e, v_d;
            gps_lat = ins_msg->LLA.x;
            gps_long = ins_msg->LLA.y;
            gps_alt = ins_msg->LLA.z;
            r_x = ins_msg->RPY.x;
            r_y = ins_msg->RPY.y;
            r_z = ins_msg->RPY.z;
            v_n = ins_msg->NedVel.x;
            v_e = ins_msg->NedVel.y;
            v_d = ins_msg->NedVel.z;
            gps_file << std::setprecision(16) << time_stamp << "," << gps_lat << "," << gps_long << "," << gps_alt << std::endl;
            orientation_file << std::setprecision(16) << time_stamp << "," << r_x << "," << r_y << "," << r_z << std::endl;
            velocity_file << std::setprecision(16) << time_stamp << "," << v_n << "," << v_e << "," << v_d << std::endl;
        }
    }

    void imuCallback(const vn300::sensorsConstPtr &imu_msg) {
        if (isYawSet) {
            if (!isFirstFrame) {
                time_stamp = ros::Time::now().toSec() - start_time_stamp;
            } else {
                isFirstFrame = false;
                start_time_stamp = ros::Time::now().toSec();
                time_stamp = 0;
            }
            double acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
            acc_x = imu_msg->Accel.x;
            acc_y = imu_msg->Accel.y;
            acc_z = imu_msg->Accel.z;
            gyro_x = imu_msg->Gyro.x;
            gyro_y = imu_msg->Gyro.y;
            gyro_z = imu_msg->Gyro.z;
            accel_gyro_file << std::setprecision(16) << time_stamp << ","
                            << acc_x << "," << acc_y << "," << acc_z << ","
                            << gyro_x << "," << gyro_y << "," << gyro_z << std::endl;
            count++;
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "bag2csv_node");
    ros::NodeHandle nh("~");
    bag2CSV bC(nh);
    ros::spin();
    return 0;
}
