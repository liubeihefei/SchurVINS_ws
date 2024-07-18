#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

sensor_msgs::Imu imu;
ros::Publisher imu_pub;

void convertImu(const sensor_msgs::ImuPtr& msg)
{
    imu.header = msg->header;

    imu.orientation.x = -msg->orientation.y;
    imu.orientation.y = msg->orientation.z;
    imu.orientation.z = msg->orientation.x;
    
    imu.orientation_covariance = msg->orientation_covariance;

    imu.angular_velocity.x = -msg->angular_velocity.y;
    imu.angular_velocity.y = msg->angular_velocity.z;
    imu.angular_velocity.z = msg->angular_velocity.x;

    imu.angular_velocity_covariance = msg->angular_velocity_covariance;

    imu.linear_acceleration.x = -msg->linear_acceleration.y;
    imu.linear_acceleration.y = msg->linear_acceleration.z;
    imu.linear_acceleration.z = msg->linear_acceleration.x;

    imu.linear_acceleration_covariance = msg->angular_velocity_covariance;

    imu_pub.publish(imu);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_converter");
    ros::NodeHandle nh;

    // 订阅imu数据
    ros::Subscriber imu_sub = nh.subscribe("/stereo_inertial_publisher/imu", 1, convertImu);

    imu_pub = nh.advertise<sensor_msgs::Imu>("/stereo_inertial_publisher/imu_con", 1);

    ros::spin();

    return 0;
}
