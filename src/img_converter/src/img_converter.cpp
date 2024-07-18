#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

ros::Publisher mono8_left_pub;
ros::Publisher mono8_right_pub;
sensor_msgs::ImagePtr output_left_msg;
sensor_msgs::ImagePtr output_right_msg;
ros::Time temp;
bool left_state = false;
bool right_state = false;

void depthLeftImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // 将 ROS 深度图像消息转换为 OpenCV 图像
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat depth_image = cv_ptr->image;

        // 归一化深度图像，将其转换为 0-255 的灰度值
        double min_val, max_val;
        cv::minMaxLoc(depth_image, &min_val, &max_val);
        cv::Mat depth_normalized;
        depth_image.convertTo(depth_normalized, CV_8UC1, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));

        // 发布新的深度图像
        output_left_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depth_normalized).toImageMsg();

        left_state = true;

        if(left_state == true && right_state == true)
        {
            temp = ros::Time::now();
            ROS_ERROR("cv_bridge exception: %f", temp);
            output_left_msg->header.stamp = temp;
            output_right_msg->header.stamp = temp;
            mono8_left_pub.publish(output_left_msg);
            mono8_right_pub.publish(output_right_msg);
            left_state = false;
            right_state = false;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void depthRightImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // 将 ROS 深度图像消息转换为 OpenCV 图像
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::Mat depth_image = cv_ptr->image;

        // 归一化深度图像，将其转换为 0-255 的灰度值
        double min_val, max_val;
        cv::minMaxLoc(depth_image, &min_val, &max_val);
        cv::Mat depth_normalized;
        depth_image.convertTo(depth_normalized, CV_8UC1, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));

        // 发布新的深度图像
        output_right_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depth_normalized).toImageMsg();

        right_state = true;

        if(left_state == true && right_state == true)
        {
            temp = ros::Time::now();
            ROS_ERROR("cv_bridge exception: %f", temp);
            output_left_msg->header.stamp = temp;
            output_right_msg->header.stamp = temp;
            mono8_left_pub.publish(output_left_msg);
            mono8_right_pub.publish(output_right_msg);
            left_state = false;
            right_state = false;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_to_mono8");
    ros::NodeHandle nh;

    // 订阅深度图像
    ros::Subscriber depth_left_sub = nh.subscribe("/camera/left/depth/image_raw", 1, depthLeftImageCallback);
    ros::Subscriber depth_right_sub = nh.subscribe("/camera/right/depth/image_raw", 1, depthRightImageCallback);

    // 发布 mono8 编码的图像
    mono8_left_pub = nh.advertise<sensor_msgs::Image>("/stereo_inertial_publisher/left/image_raw", 1);
    mono8_right_pub = nh.advertise<sensor_msgs::Image>("/stereo_inertial_publisher/right/image_raw", 1);

    ros::spin();

    return 0;
}
