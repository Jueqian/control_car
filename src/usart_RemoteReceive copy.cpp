#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include <string>

#include <serial/serial.h> // 要安装ros-noetic-serial库

// 全局变量用于串口通信
serial::Serial my_serial;
// std::string tty_device = "/dev/ttyUSB0";
std::string tty_device;
int baudrate = 57600;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "usart_RemoteReceive");
    ros::NodeHandle nh;

    ros::Publisher cmd_vel_Rec = nh.advertise<geometry_msgs::Twist>("/car/cmd_vel_Remote", 5);
    ros::Rate rate(10); // 设置发布频率为10Hz

    nh.getParam("usart_RemoteReceive/tty_device_Remote", tty_device);
    nh.getParam("usart_RemoteReceive/baudrate_Remote", baudrate);

    geometry_msgs::Twist twist_msg;
    // 先初始化一下
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;


    try
    {
        my_serial.setPort(tty_device);
        my_serial.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        my_serial.setTimeout(to);
        my_serial.open();
        ROS_INFO("[Receive] -> Success to open Receive serial port: %s, baudrate: %d", tty_device.c_str(), baudrate);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("[Receive] -> Failed to open Receive serial port: %s", e.what());
        // return -1;
    }

    while (ros::ok())
    {
        // if (!my_serial.isOpen())
        // {
        //     try
        //     {
        //         my_serial.setPort(tty_device);
        //         my_serial.setBaudrate(baudrate);
        //         my_serial.open();
        //         // 清空串口缓冲区
        //         // my_serial.flushInput();
        //     }
        //     catch (const std::exception &e)
        //     {
        //         ROS_ERROR("Failed to open Receive serial port: %s", e.what());
        //         // return -1;
        //         continue;
        //     }
        // }

        std::string serial_data;
        while (my_serial.available() > 0)
        {
            serial_data = my_serial.readline(); // 从串口读取最新一行数据
        }
        
        if (!serial_data.empty())
        {
            ROS_INFO("\033[1;32m[Receive] -> Received serial data: %s\033[0m", serial_data.c_str());
            // 解析串口数据，数据格式为 "Linear: 1.60833, Angular: 0"
            double linear, angular;
            int result = sscanf(serial_data.c_str(), "Linear: %lf, Angular: %lf", &linear, &angular);
            if (result == 2)
            {
                // std::cout << "[Receive] -> Linear: " << linear << ", Angular: " << angular << std::endl;
                // 创建 Twist 消息并发布到 "/car/cmd_vel" 话题上
                
                // static geometry_msgs::Twist twist_msg_last;
                twist_msg.linear.x = linear;
                twist_msg.angular.z = angular;
                // twist_msg_last = twist_msg;
                std::cout << "-----------" << std::endl;
                std::cout << twist_msg << std::endl;
                std::cout << "-----------" << std::endl;
                cmd_vel_Rec.publish(twist_msg);

                // if (twist_msg != twist_msg_last)
                // {
                //     cmd_vel_Rec.publish(twist_msg);
                //     twist_msg_last = twist_msg;
                //     ROS_INFO("Published cmd_vel message. Linear: %f, Angular: %f", linear, angular);
                // }
            }
            else
            {
                std::cout << "[Receive] -> Failed to parse the usart string." << std::endl;
                continue;
            }
        }
        ros::spinOnce();
    }

    my_serial.close(); // 关闭串口

    return 0;
}
