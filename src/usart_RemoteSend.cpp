#include <stdio.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include <string>
#include <std_msgs/String.h>


#include <serial/serial.h> // 要安装ros-noetic-serial库


ros::Publisher cmd_serial_send_pub; // only for debug_serial_send

// 全局变量用于串口通信
serial::Serial my_serial;
// std::string tty_device = "/dev/ttyUSB0";
std::string tty_device;
int baudrate = 57600;
int skip_data_frame = 0;

void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
    std::stringstream ss;
    ss << "Linear: " << msg->linear.x << ", Angular: " << msg->angular.z << "\r\n";
    std::string cmd_vel_str = ss.str();
    // std::cout << "send ss: " << cmd_vel_str << std::endl;
    // ROS_INFO("[Send] -> Send serial data: %s", cmd_vel_str.c_str());
    if (skip_data_frame < 12)
    {
        skip_data_frame++;
        std::cout << "Skip send frame times: " << skip_data_frame << std::endl;
        return;
    }
    
    ROS_INFO("\033[1;32m[Send] -> Send serial data: %s\033[0m", cmd_vel_str.c_str());
    


    ////////////////////////////////////////////    debug_serial_send
    // std_msgs::String serial_msg;
    // serial_msg.data = cmd_vel_str;
    // cmd_serial_send_pub.publish(serial_msg);
    ////////////////////////////////////////////

    try
    {
        my_serial.write(cmd_vel_str);   // 发送消息到串口
        my_serial.flush();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("[Send] -> The device of USART was disconnected: %s", tty_device.c_str());
        ROS_INFO("[Send] -> Retrying to connect device: %s", tty_device.c_str());
        try
        {
            my_serial.setPort(tty_device);
            my_serial.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(100);
            my_serial.setTimeout(to);
            my_serial.open();
            ROS_INFO("[Send] -> Success to Reconnect serial port: %s", tty_device.c_str());
        }
        catch(const std::exception& e)
        {
            static int retry_count=0;
            ++retry_count;
            ROS_ERROR("[Send] -> Failed to open serial port: %s, Attempts:%d", tty_device.c_str(), retry_count);
            return;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "usart_RemoteSend");
    ros::NodeHandle nh;

    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 5, cmdVelCallback);

    // cmd_serial_send_pub = nh.advertise<std_msgs::String>("cmd_serial_send", 5); // only for debug_serial_send


    nh.getParam("/usart_RemoteSend/tty_device_Remote", tty_device);
    nh.getParam("/usart_RemoteSend/baudrate_Remote", baudrate);

    // 打开串口
    try
    {
        my_serial.setPort(tty_device);
        my_serial.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        my_serial.setTimeout(to);
        my_serial.open();
        ROS_INFO("[Send] -> Success to open Send serial port: %s, baudrate: %d", tty_device.c_str(), baudrate);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("[Send] -> Failed to open serial port: %s", e.what());
        // return -1;
    }


    ros::spin(); // 阻塞，直到节点被关闭

    my_serial.close(); // 关闭串口

    return 0;
}
