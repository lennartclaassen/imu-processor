#ifndef IMU_DUMMY_H
#define IMU_DUMMY_H
 
#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
 
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
 
#include <stdio.h>
#include <fstream>
#include <iostream>
 
#include <yaml-cpp/yaml.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
 
class imu_dummy {
    private:
        ros::Publisher  imuPublisher;
        ros::Subscriber accSubscriber;
        ros::Subscriber yprSubscriber;
 
        ros::NodeHandle nh;

        void accCallback(const geometry_msgs::Vector3::ConstPtr &msg);
        void yprCallback(const geometry_msgs::Vector3::ConstPtr &msg);
 
    public:
        imu_dummy();
 
        sensor_msgs::Imu* imuMsg;
        geometry_msgs::Quaternion m_q;

        std::string frameID;

        double roll, pitch, yaw;
        double q_x, q_y, q_z, q_w;                   //orientation values
        double cov_or_x, cov_or_y, cov_or_z;         //covariance for orientation values     C_or = diag(cov_or_x, cov_or_y, cov_or_z)
        double vel_x, vel_y, vel_z;                        //angular velocity values
        double cov_vel_x, cov_vel_y, cov_vel_z;      //covariance for velocity values        C_vel = diag(cov_vel_x, cov_vel_y, cov_vel_z)
        double acc_x, acc_y, acc_z;                  //linear acceleration values
        double cov_acc_x, cov_acc_y, cov_acc_z;      //covariance vor acceleration values    C_acc = diag(cov_acc_x, cov_acc_y, cov_acc_z)

        bool useArduinoMsg;
        bool simMovement;

        void createDummyMsg();
        void simulateMovement();
        void loop();
};
 
#endif
