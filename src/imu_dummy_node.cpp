#include "imu_dummy/imu_dummy_node.h"
 
using namespace std;
using namespace angles;
 
/**
 * @brief Constructor
 */
imu_dummy::imu_dummy() {
 
    //initialize publisher for imu topic
    nh.param("/imu_dummy/roll", this->roll, 0.0);
    nh.param("/imu_dummy/pitch", this->pitch, 0.0);
    nh.param("/imu_dummy/yaw", this->yaw, 0.0);
    nh.param("/imu_dummy/cov_or_x", this->cov_or_x, 0.01);                  //standard deviation in degrees
    nh.param("/imu_dummy/cov_or_y", this->cov_or_y, 0.01);                  //standard deviation in degrees
    nh.param("/imu_dummy/cov_or_z", this->cov_or_z, 0.01);                  //standard deviation in degrees
    nh.param("/imu_dummy/vel_x", this->vel_x, 0.0);
    nh.param("/imu_dummy/vel_y", this->vel_y, 0.0);
    nh.param("/imu_dummy/vel_z", this->vel_z, 0.0);
    nh.param("/imu_dummy/cov_vel_x", this->cov_vel_x, 0.01);                  //standard deviation in degrees
    nh.param("/imu_dummy/cov_vel_y", this->cov_vel_y, 0.01);                  //standard deviation in degrees
    nh.param("/imu_dummy/cov_vel_z", this->cov_vel_z, 0.01);                  //standard deviation in degrees
    nh.param("/imu_dummy/acc_x", this->acc_x, 0.0);
    nh.param("/imu_dummy/acc_y", this->acc_y, 0.0);
    nh.param("/imu_dummy/acc_z", this->acc_z, 0.0);
    nh.param("/imu_dummy/cov_acc_x", this->cov_acc_x, 0.01);                  //standard deviation in degrees
    nh.param("/imu_dummy/cov_acc_y", this->cov_acc_y, 0.01);                  //standard deviation in degrees
    nh.param("/imu_dummy/cov_acc_z", this->cov_acc_z, 0.01);                  //standard deviation in degrees
    nh.param<std::string>("/imu_dummy/frameID", this->frameID, "/camera_link");
    nh.param<bool>("/imu_dummy/use_arduino_msg", this->useArduinoMsg, false);    //TODO
    nh.param<bool>("/imu_dummy/simulate_movement", this->simMovement, false);    //TODO

    accSubscriber = nh.subscribe("/acc", 1, &imu_dummy::accCallback, this);
    yprSubscriber = nh.subscribe("/ypr", 1, &imu_dummy::yprCallback, this);
    imuPublisher = nh.advertise<sensor_msgs::Imu>("/imu", 10, this);

    this->createDummyMsg();
}
 
void imu_dummy::createDummyMsg(){
    this->imuMsg = new sensor_msgs::Imu();

    ROS_INFO("%f  %f  %f -> %f  %f  %f", this->roll, this->pitch, this->yaw, from_degrees(this->roll), from_degrees(this->pitch), from_degrees(this->yaw));


    tf::Quaternion tfQ = tf::createQuaternionFromRPY(from_degrees(this->roll), from_degrees(this->pitch), from_degrees(this->yaw));
    geometry_msgs::Quaternion q;

    tf::quaternionTFToMsg(tfQ, q);

    ROS_INFO("%f  %f  %f  %f", tfQ.x(), tfQ.y(), tfQ.z(), tfQ.w());

    this->imuMsg->orientation = q;

    this->imuMsg->orientation_covariance.at(0) = pow(from_degrees(this->cov_or_x), 2);
    this->imuMsg->orientation_covariance.at(4) = pow(from_degrees(this->cov_or_y), 2);
    this->imuMsg->orientation_covariance.at(8) = pow(from_degrees(this->cov_or_z), 2);

    geometry_msgs::Vector3 vel_vec;
    vel_vec.x = this->vel_x;
    vel_vec.y = this->vel_y;
    vel_vec.z = this->vel_z;

    this->imuMsg->angular_velocity = vel_vec;

    this->imuMsg->angular_velocity_covariance.at(0) = this->cov_vel_x;
    this->imuMsg->angular_velocity_covariance.at(4) = this->cov_vel_y;
    this->imuMsg->angular_velocity_covariance.at(8) = this->cov_vel_z;

    geometry_msgs::Vector3 acc_vec;
    acc_vec.x = this->acc_x;
    acc_vec.y = this->acc_y;
    acc_vec.z = this->acc_z;

    this->imuMsg->linear_acceleration = acc_vec;

    this->imuMsg->linear_acceleration_covariance.at(0) = this->cov_acc_x;
    this->imuMsg->linear_acceleration_covariance.at(4) = this->cov_acc_y;
    this->imuMsg->linear_acceleration_covariance.at(8) = this->cov_acc_z;


}

void imu_dummy::simulateMovement(){
    //TODO
}

/**
 * @brief main loop - topic publishing
 */
void imu_dummy::loop(){
 
    ros::Rate rate(200);

    while(ros::ok()){
        this->imuMsg->header.stamp = ros::Time::now();
        this->imuMsg->header.frame_id = this->frameID;

        if(simMovement)
            simulateMovement();

        imuPublisher.publish(*imuMsg);
        ros::spinOnce();
        rate.sleep();
    }
}

void imu_dummy::accCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
//    geometry_msgs::Quaternion q = m_q;

//    float gravX = 2.0f * (q.y * q.w - q.x * q.z);
//    float gravY = 2.0f * (q.x * q.y + q.z * q.w);
//    float gravZ = q.x * q.x - q.y * q.y - q.z * q.z + q.w * q.w;

//    this->imuMsg->linear_acceleration.x = msg->x - gravX;
//    this->imuMsg->linear_acceleration.y = msg->y - gravY;
//    this->imuMsg->linear_acceleration.z = msg->z - gravZ;


    this->imuMsg->linear_acceleration.x = 0.0;
    this->imuMsg->linear_acceleration.y = 0.0;
    this->imuMsg->linear_acceleration.z = 0.0;

    if(fabs(msg->x) > 0.15){
        this->imuMsg->linear_acceleration.x = msg->x;
        ROS_INFO("IMU: x valid");
    }
    if(fabs(msg->y) > 0.15) {
        this->imuMsg->linear_acceleration.y = msg->y;
        ROS_INFO("IMU: y valid");
    }
    if(fabs(msg->z) > 0.15) {
        this->imuMsg->linear_acceleration.z = msg->z;
        ROS_INFO("IMU: z valid");
    }
}

void imu_dummy::yprCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    tf::Quaternion tfQ = tf::createQuaternionFromRPY(from_degrees(msg->z), from_degrees(msg->y), from_degrees(msg->x));
    geometry_msgs::Quaternion q;

    tf::quaternionTFToMsg(tfQ, q);

    m_q = q;
    this->imuMsg->orientation = m_q;
//    ROS_INFO("Orientation callback");
}



 
/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_dummy");

    imu_dummy dummy;
    dummy.loop();
}
