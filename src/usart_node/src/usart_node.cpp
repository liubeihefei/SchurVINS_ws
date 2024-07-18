#include <cstdlib>
#include <memory>
#include <chrono>
#include <functional>


#include "USART.h"

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


#include "std_msgs/msg/string.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.h>
#include "std_msgs/msg/bool.hpp"



using std::placeholders::_1;
using namespace Eigen;

// 用于ms的命名空间
using namespace std::chrono_literals;

// 外部数据订阅类
class ControlInput : public rclcpp::Node
{
public:
    ControlInput() : Node("control_input")
    {
        // 初始化参数
        initParameters();

        // 测试数据状态下不初始化串口
        if(!data_test)
        {
            RCLCPP_INFO(this->get_logger(),"start to open usart");
            // 串口相关的初始化
            usart_port.init("/dev/ttyUSB0", 115200, true, 0, 1);
            // 创建定时器，用于串口发送数据
            timer_ = this->create_wall_timer(
                33ms, std::bind(&ControlInput::timerCallback, this));
        }

        // 订阅键盘信息
        key_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "key_state", 10, std::bind(&ControlInput::keyStateCallback, this, _1));

        if(!usart_test)
        {
            // 订阅姿态信息
            pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "odometry", 10, std::bind(&ControlInput::poseCallback, this, _1));
        }

        // 发布vio重启的信息
        restart_vio_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/vins_restart", 10);


        // 调试模式下发布位置和角度信息
        if(show_msg)
        {
            position_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("position_topic", 10);
            euler_angle_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("euler_angle_topic", 10);
        }

    }


private:
    // 初始化参数
    void initParameters()
    {
        // 读取参数并初始化
        this->declare_parameter("usart_test", false);
        this->declare_parameter("show_msg", false);
        this->declare_parameter("data_test", true);

        this->get_parameter("usart_test", usart_test);
        this->get_parameter("show_msg",  show_msg);
        this->get_parameter("data_test", data_test);

        if(data_test && usart_test)
        {
            RCLCPP_ERROR(this->get_logger(), "You shouldn't open the usart_test and data_test at the same time");
            // 退出程序
            std::exit(EXIT_FAILURE); // EXIT_FAILURE 是标准的错误退出状态码
        }

        if(show_msg && usart_test)
        {
            RCLCPP_ERROR(this->get_logger(), "You shouldn't open the usart_test and show_msg at the same time");
            // 退出程序
            std::exit(EXIT_FAILURE); // EXIT_FAILURE 是标准的错误退出状态码
        }
    }

    // 键盘信息订阅回调
    void keyStateCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string data = msg->data;
        // 假设data的格式是"XY"，其中X是first_press，Y是key_state
        if (data.length() == 3) {
            first_press = (data[0] == '1');
            key_pressed = (data[1] == '1');
            restart_vio = (data[2] == '1');

            std::cout << "Received data: " << data << std::endl;
            std::cout << "First Press: " << first_press << ", Key State: " << key_pressed << "restart_vio: " << restart_vio << std::endl;
        }

        // 如果第一次按下则初始化位置和姿态
        if(key_pressed && first_press)
        {
            init_position = vins_position;
            init_pose = vins_pose;
        }

        // 按键如果按下，计算姿态并执行机械臂控制
        if(key_pressed && !first_press)
        {
            // 计算姿态变化量
            delta_position = vins_position - init_position;
            delta_pose = init_pose.inverse() * vins_pose;            
            // 将待发送数据存储到send_pack中
            armControl();
        }

        // 如果按下重启，则重启vio
        if(restart_vio)
        {   // 创建一个Bool消息对象
            auto restart_msg = std_msgs::msg::Bool();
            restart_msg.data = true; // 设置消息的值
            restart_vio_publisher_->publish(restart_msg);
        }

        // 未按下则将发送的包清零
        if(!key_pressed)
        {
            // 将数据存储到sendpack中
            send_pack.x = 0;
            send_pack.y = 0;
            send_pack.z = 0;
            send_pack.yaw = 0;
            send_pack.pitch = 0;
            send_pack.roll = 0;
            send_pack.work_flag = false;
        }
    }

    // 姿态信息订阅回调
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 从Odometry消息中解偶位置
        vins_position = Vector3d(msg->pose.pose.position.x,
                                 msg->pose.pose.position.y,
                                 msg->pose.pose.position.z);

        // 从Odometry消息中解偶姿态（四元数）
        vins_pose = Quaterniond(msg->pose.pose.orientation.w,
                                msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z);

        // 从Odometry消息中解偶速度
        vins_velocity = Vector3d(msg->twist.twist.linear.x,
                                 msg->twist.twist.linear.y,
                                 msg->twist.twist.linear.z);
    }

    // 串口定时器回调
    void timerCallback() {
        // 在这里通过串口发送信息
        RCLCPP_INFO(this->get_logger(), "Sending data via serial port at 30Hz...");
        // 串口发送代码...
        usart_port.Send(send_pack);
    }

    // 机械臂控制函数
    void armControl()
    {
        if(!usart_test)
        {
            // 将四元数转换为欧拉角,弧度（绕Z轴 Yaw，绕Y轴 Pitch，绕X轴 Roll）
            Eigen::Vector3d euler_angles = delta_pose.toRotationMatrix().eulerAngles(2, 1, 0);
            // 转换为角度并进行存储
            delta_euler_angles.z() = euler_angles.z() * 180.0 / M_PI; // yaw
            delta_euler_angles.y() = euler_angles.y() * 180.0 / M_PI; // pitch
            delta_euler_angles.x() = euler_angles.x() * 180.0 / M_PI; // roll

            if(abs(delta_euler_angles.x()) > 90.0)
            {
                delta_euler_angles.z() += 180.0;
                delta_euler_angles.y() = 180.0 - delta_euler_angles.y();
                delta_euler_angles.x() += 180;
            }
            if(delta_euler_angles.z() > 180.0)
            {
                delta_euler_angles.z() -= 360.0;
            }
            else if(delta_euler_angles.z() < -180)
            {
                delta_euler_angles.z() += 360.0;
            }

            if(delta_euler_angles.y() > 180.0)
            {
                delta_euler_angles.y() -= 360.0;
            }
            else if(delta_euler_angles.y() < -180)
            {
                delta_euler_angles.y() += 360.0;
            }

            if(delta_euler_angles.x() > 180.0)
            {
                delta_euler_angles.x() -= 360.0;
            }
            else if(delta_euler_angles.x() < -180)
            {
                delta_euler_angles.x() += 360.0;
            }

            // 将数据存储到sendpack中
            send_pack.x = delta_position.x();
            send_pack.y = delta_position.y();
            send_pack.z = delta_position.z();
            send_pack.yaw = delta_euler_angles.z();
            send_pack.pitch = delta_euler_angles.y();
            send_pack.roll = delta_euler_angles.x();
            send_pack.work_flag = true;

            // 将eigen的数据转换为msg，便于可视化
            auto const send_position = [this]{
            geometry_msgs::msg::Vector3 msg;
                msg.x = this->delta_position.x();
                msg.y = this->delta_position.y();
                msg.z = this->delta_position.z();
                return msg;
            }();
            
            // yaw pitch roll
            auto const send_pose = [this]{
                geometry_msgs::msg::Vector3 msg;
                msg.z = this->delta_euler_angles.z();
                msg.y = this->delta_euler_angles.y();
                msg.x = this->delta_euler_angles.x();
                return msg;
            }();

            if(show_msg)
            {
                position_publisher_->publish(send_position);
                euler_angle_publisher_->publish(send_pose);
            }

            RCLCPP_INFO(this->get_logger(),
                "Send Pose: Position (x: %f, y: %f, z: %f), Eular angles (yaw: %f, pitch: %f, roll: %f)",
                send_position.x,
                send_position.y,
                send_position.z,
                send_pose.z,
                send_pose.y,
                send_pose.x
                );
        }
        else
        {
            // 将数据存储到sendpack中
            send_pack.x = 10.0;
            send_pack.y = 10.0;
            send_pack.z = 10.0;
            send_pack.yaw = 90.0;
            send_pack.pitch = 90.0;
            send_pack.roll = 90.0;
            send_pack.work_flag = true;
        }
        
    }


    // 初始的位置和姿态
    Vector3d init_position;
    Quaterniond init_pose;
    // 新获取的位置、姿态、速度
    Vector3d vins_position;
    Quaterniond vins_pose;
    Vector3d vins_velocity;
    // 位置和姿态变化量
    Vector3d delta_position;
    Quaterniond delta_pose;
    // 将待变换的四元数转换为欧拉角
    Vector3d delta_euler_angles;

    // 订阅键盘和姿态数据的订阅者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription_;
    // 发布位置和欧拉角的变化量便于debug
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr euler_angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr restart_vio_publisher_;
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;


public:
    // 存储按键状态，可供外部调用
    bool first_press = false;
    bool key_pressed = false;
    bool restart_vio = false;

    // 串口
    USART usart_port;
    /// 经由串口发送给裁判系统的数据包
    USART::SendPack send_pack;

    // 模式设置
    bool usart_test = false; // 串口测试
    bool show_msg = false;   // 是否将发送给电控的数据发布出来
    bool data_test = false;  // 只测试数据，不初始化串口以及发送部分

};


int main(int argc, char **argv)
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);

    // 创建ControlInput节点的实例
    auto node = std::make_shared<ControlInput>();

    // 运行节点，等待回调函数的触发
    rclcpp::spin(node);

    // 关闭ROS 2
    rclcpp::shutdown();
    return 0;
}