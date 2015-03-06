#include "imu_processor/imu_processor_node.h"
 
using namespace std;
using namespace angles;
 
/**
 * @brief Constructor
 */
imu_processor::imu_processor() {
 
    //get the ROS parameters
    nh.param("/imu_processor/roll", this->roll, 0.0);
    nh.param("/imu_processor/pitch", this->pitch, 0.0);
    nh.param("/imu_processor/yaw", this->yaw, 0.0);
    nh.param("/imu_processor/cov_or_x", this->cov_or_x, 0.01);                  //standard deviation in degrees
    nh.param("/imu_processor/cov_or_y", this->cov_or_y, 0.01);                  //standard deviation in degrees
    nh.param("/imu_processor/cov_or_z", this->cov_or_z, 0.01);                  //standard deviation in degrees
    nh.param("/imu_processor/vel_x", this->vel_x, 0.0);
    nh.param("/imu_processor/vel_y", this->vel_y, 0.0);
    nh.param("/imu_processor/vel_z", this->vel_z, 0.0);
    nh.param("/imu_processor/cov_vel_x", this->cov_vel_x, 0.01);                  //standard deviation in degrees
    nh.param("/imu_processor/cov_vel_y", this->cov_vel_y, 0.01);                  //standard deviation in degrees
    nh.param("/imu_processor/cov_vel_z", this->cov_vel_z, 0.01);                  //standard deviation in degrees
    nh.param("/imu_processor/acc_x", this->acc_x, 0.0);
    nh.param("/imu_processor/acc_y", this->acc_y, 0.0);
    nh.param("/imu_processor/acc_z", this->acc_z, 0.0);
    nh.param("/imu_processor/cov_acc_x", this->cov_acc_x, 0.01);                  //standard deviation in degrees
    nh.param("/imu_processor/cov_acc_y", this->cov_acc_y, 0.01);                  //standard deviation in degrees
    nh.param("/imu_processor/cov_acc_z", this->cov_acc_z, 0.01);                  //standard deviation in degrees
    nh.param<std::string>("/imu_processor/frameID", this->frameID, "/camera_link");

    //initialize subscribers for imu topic
    accSubscriber = nh.subscribe("/acc", 1, &imu_processor::accCallback, this);
    yprSubscriber = nh.subscribe("/ypr", 1, &imu_processor::yprCallback, this);

    //initialize publisher for imu topic
    imuPublisher = nh.advertise<sensor_msgs::Imu>("/imu", 10, this);

    this->createDummyMsg();
}

/**
 * @brief imu_processor::createDummyMsg     Can be used for creating a dummy message for debugging
 */
void imu_processor::createDummyMsg(){

//    ROS_INFO("%f  %f  %f -> %f  %f  %f", this->roll, this->pitch, this->yaw, from_degrees(this->roll), from_degrees(this->pitch), from_degrees(this->yaw));

    //create a quaternion representation of the orientation values
    tf::Quaternion tfQ = tf::createQuaternionFromRPY(from_degrees(this->roll), from_degrees(this->pitch), from_degrees(this->yaw));
    geometry_msgs::Quaternion q;
    tf::quaternionTFToMsg(tfQ, q);

//    ROS_INFO("%f  %f  %f  %f", tfQ.x(), tfQ.y(), tfQ.z(), tfQ.w());

    //set the orientation of the IMU message
    this->imuMsg = new sensor_msgs::Imu();
    this->imuMsg->orientation = q;
    this->imuMsg->orientation_covariance.at(0) = pow(from_degrees(this->cov_or_x), 2);
    this->imuMsg->orientation_covariance.at(4) = pow(from_degrees(this->cov_or_y), 2);
    this->imuMsg->orientation_covariance.at(8) = pow(from_degrees(this->cov_or_z), 2);

    //create a velocity message
    geometry_msgs::Vector3 vel_vec;
    vel_vec.x = this->vel_x;
    vel_vec.y = this->vel_y;
    vel_vec.z = this->vel_z;


    //set the velocity of the IMU message
    this->imuMsg->angular_velocity = vel_vec;
    this->imuMsg->angular_velocity_covariance.at(0) = this->cov_vel_x;
    this->imuMsg->angular_velocity_covariance.at(4) = this->cov_vel_y;
    this->imuMsg->angular_velocity_covariance.at(8) = this->cov_vel_z;

    //create an acceleration message
    geometry_msgs::Vector3 acc_vec;
    acc_vec.x = this->acc_x;
    acc_vec.y = this->acc_y;
    acc_vec.z = this->acc_z;

    //set the acceleration of the IMU message
    this->imuMsg->linear_acceleration = acc_vec;
    this->imuMsg->linear_acceleration_covariance.at(0) = this->cov_acc_x;
    this->imuMsg->linear_acceleration_covariance.at(4) = this->cov_acc_y;
    this->imuMsg->linear_acceleration_covariance.at(8) = this->cov_acc_z;
}

/**
 * @brief main loop - topic publishing
 */
void imu_processor::loop(){
 
    ros::Rate rate(50);

    while(ros::ok()){
        this->imuMsg->header.stamp = ros::Time::now();
        this->imuMsg->header.frame_id = this->frameID;

        imuPublisher.publish(*imuMsg);
        ros::spinOnce();
        rate.sleep();
    }
}

/**
 * @brief imu_processor::accCallback    callback for the acceleration values from the IMU/Arduino
 * @param msg
 */
void imu_processor::accCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    //set accelerations to 0
    this->imuMsg->linear_acceleration.x = 0.0;
    this->imuMsg->linear_acceleration.y = 0.0;
    this->imuMsg->linear_acceleration.z = 0.0;

    //publish real acceleration values only if they exceed a certain threshold
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

/**
 * @brief imu_processor::yprCallback    callback for the orientation values from the IMU/Arduino
 * @param msg
 */
void imu_processor::yprCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    //get the message values
    float yaw, pitch, roll;
    yaw = ((int)((msg->x)*10.0))/10.0;
    pitch = ((int)((msg->y)*10.0))/10.0;
    roll = ((int)((msg->z)*10.0))/10.0;

    //create a quaternion
    tf::Quaternion tfQ = tf::createQuaternionFromRPY(from_degrees(roll), from_degrees(pitch), from_degrees(yaw));
    geometry_msgs::Quaternion q;
    tf::quaternionTFToMsg(tfQ, q);

    //set the orientation of the IMU message
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
    ros::init(argc, argv, "imu_processor");

    imu_processor processor;
    processor.loop();
}
