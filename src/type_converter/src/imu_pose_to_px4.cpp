#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class converter{
private:
geometry_msgs::PoseStamped send;
ros::Subscriber state_sub;
ros::Publisher local_pos_pub;

public:
    converter()
    {
        ros::NodeHandle nh;

        //订阅。<>里面为模板参数，传入的是订阅的消息体类型，（）里面传入三个参数，分别是该消息体的位置、缓存大小（通常为1000）、回调函数
        state_sub = nh.subscribe("svo/pose_imu", 1, &converter::pose_imu_cb, this);
    
        //发布之前需要公告，并获取句柄，发布的消息体的类型为：geometry_msgs::PoseStamped
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1);

        ros::spin();
    }

    //订阅时的回调函数，接受到该消息体的内容时执行里面的内容，这里面的内容就是赋值
    void pose_imu_cb(const geometry_msgs::PoseWithCovarianceStampedPtr& msg){
        send.pose.position.y = msg->pose.pose.position.x;
        send.pose.position.x = -msg->pose.pose.position.y;
        send.pose.position.z = msg->pose.pose.position.z;

        send.pose.orientation.x = -msg->pose.pose.orientation.y;
        send.pose.orientation.y = msg->pose.pose.orientation.x;
        send.pose.orientation.z = msg->pose.pose.orientation.z;
        send.pose.orientation.w = msg->pose.pose.orientation.w;

        send.header.stamp = ros::Time::now();

        local_pos_pub.publish(send);
    }
};

int main(int argc, char **argv)
{
    // 初始化ROS
    ros::init(argc, argv, "imu_pose_to_px4_node");

    converter con;

    return 0;
}


