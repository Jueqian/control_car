#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "canusb.hpp"
#include <iostream>
#include <sstream>
#include <algorithm>

#include <unistd.h>

double PI = 3.141592654;
double right_vel = 0.0;
double left_vel = 0.0;
double width_robot = 0.605;
double width_wheel = 0.19;
double gap_time = 0.5;
bool print_traffic = true;
std::string tty_device = "/dev/ttyUSB3";
int canusb_int_speed = 500000;
int br = 115200;

/*
这里采用闭环调速模式（详细请参考驱动控制pdf文档P61）
闭环调速或位置控制时，写入数值为电机目标转速(RPM --- 转每分钟)
命令    索引号（低字节在前）   子索引号       数据（4位）低字节在前（下面为输入1388）16进制，对应十进制为5000 RPM
2B      0120                  00          88 13 00 00
车的轮子对应编号为下：
    前面
601     604


602     603
    后面

轮子半径为： 0.15m

注意：如果出现轮子反转速度出现问题，请使用23发送4个字节数据，完整格式
*/
bool control_wheel()
{
    bool valid = true;
    std::string wheel_601 = "2B012000";
    std::string wheel_602 = "2B012000";
    std::string wheel_603 = "2B012000";
    std::string wheel_604 = "2B012000";

    int right_vel_rpm = right_vel * 60 / (2 * PI * width_wheel);
    int left_vel_rpm = left_vel * 60 / (2 * PI * width_wheel);

    std::stringstream stream_1;
    stream_1 << std::hex << right_vel_rpm;           // 将整数以十六进制格式写入流
    std::string shex_right_vel_rpm = stream_1.str(); // 从流中获取字符串

    std::stringstream stream_2;
    stream_2 << std::hex << left_vel_rpm;           // 将整数以十六进制格式写入流
    std::string shex_left_vel_rpm = stream_2.str(); // 从流中获取字符串

    std::string new_shex_right_vel_rpm;
    std::string new_shex_left_vel_rpm;

    // 首先判断正反
    if (left_vel_rpm >= 0)
    {
        std::cout << "+ shex_left_vel_rpm : " << shex_left_vel_rpm << std::endl;

        // 将速度转为低字节在前
        int s_len = shex_left_vel_rpm.length();
        if (s_len > 4)
        {
            std::cout << "The + speed is too fast, please check the problem! ! !" << std::endl;
            valid = false;
        }
        else if (s_len == 4)
            new_shex_left_vel_rpm = shex_left_vel_rpm;
        else if (s_len == 3)
            new_shex_left_vel_rpm = "0" + shex_left_vel_rpm;
        else if (s_len == 2)
            new_shex_left_vel_rpm = "00" + shex_left_vel_rpm;
        else if (s_len == 1)
            new_shex_left_vel_rpm = "000" + shex_left_vel_rpm;
        else if (s_len == 0)
            new_shex_left_vel_rpm = "0000";

        wheel_601.push_back(new_shex_left_vel_rpm[2]);
        wheel_601.push_back(new_shex_left_vel_rpm[3]);
        wheel_601.push_back(new_shex_left_vel_rpm[0]);
        wheel_601.push_back(new_shex_left_vel_rpm[1]);
        wheel_601.append("0000");

        wheel_602.push_back(new_shex_left_vel_rpm[2]);
        wheel_602.push_back(new_shex_left_vel_rpm[3]);
        wheel_602.push_back(new_shex_left_vel_rpm[0]);
        wheel_602.push_back(new_shex_left_vel_rpm[1]);
        wheel_602.append("0000");
    }
    else
    {
        std::cout << "- shex_left_vel_rpm : " << shex_left_vel_rpm << std::endl;
        // 将速度转为低字节在前
        if (shex_left_vel_rpm[0] != 'f' || shex_left_vel_rpm[1] != 'f' ||
            shex_left_vel_rpm[2] != 'f' || shex_left_vel_rpm[3] != 'f')
        {
            std::cout << "The - speed is too fast, please check the problem! ! !" << std::endl;
            valid = false;
        }

        wheel_601.push_back(shex_left_vel_rpm[2 + 4]);
        wheel_601.push_back(shex_left_vel_rpm[3 + 4]);
        wheel_601.push_back(shex_left_vel_rpm[0 + 4]);
        wheel_601.push_back(shex_left_vel_rpm[1 + 4]);
        wheel_601.append("0000");

        wheel_602.push_back(shex_left_vel_rpm[2 + 4]);
        wheel_602.push_back(shex_left_vel_rpm[3 + 4]);
        wheel_602.push_back(shex_left_vel_rpm[0 + 4]);
        wheel_602.push_back(shex_left_vel_rpm[1 + 4]);
        wheel_602.append("0000");
    }

    // 首先判断正反
    if (right_vel_rpm >= 0)
    {
        std::cout << "+ shex_right_vel_rpm : " << shex_right_vel_rpm << std::endl;

        // 将速度转为低字节在前
        int s_len = shex_right_vel_rpm.length();
        if (s_len > 4)
        {
            std::cout << "The + speed is too fast, please check the problem! ! !" << std::endl;
            valid = false;
        }
        else if (s_len == 4)
            new_shex_right_vel_rpm = shex_right_vel_rpm;
        else if (s_len == 3)
            new_shex_right_vel_rpm = "0" + shex_right_vel_rpm;
        else if (s_len == 2)
            new_shex_right_vel_rpm = "00" + shex_right_vel_rpm;
        else if (s_len == 1)
            new_shex_right_vel_rpm = "000" + shex_right_vel_rpm;
        else if (s_len == 0)
            new_shex_right_vel_rpm = "0000";

        wheel_603.push_back(new_shex_right_vel_rpm[2]);
        wheel_603.push_back(new_shex_right_vel_rpm[3]);
        wheel_603.push_back(new_shex_right_vel_rpm[0]);
        wheel_603.push_back(new_shex_right_vel_rpm[1]);
        wheel_603.append("0000");

        wheel_604.push_back(new_shex_right_vel_rpm[2]);
        wheel_604.push_back(new_shex_right_vel_rpm[3]);
        wheel_604.push_back(new_shex_right_vel_rpm[0]);
        wheel_604.push_back(new_shex_right_vel_rpm[1]);
        wheel_604.append("0000");
    }
    else
    {
        std::cout << "- shex_right_vel_rpm : " << shex_right_vel_rpm << std::endl;
        // 将速度转为低字节在前
        int s_len = shex_right_vel_rpm.length();
        if (shex_right_vel_rpm[0] != 'f' || shex_right_vel_rpm[1] != 'f' ||
            shex_right_vel_rpm[2] != 'f' || shex_right_vel_rpm[3] != 'f')
        {
            std::cout << "The - speed is too fast, please check the problem! ! !" << std::endl;
            valid = false;
        }

        wheel_603.push_back(shex_right_vel_rpm[2 + 4]);
        wheel_603.push_back(shex_right_vel_rpm[3 + 4]);
        wheel_603.push_back(shex_right_vel_rpm[0 + 4]);
        wheel_603.push_back(shex_right_vel_rpm[1 + 4]);
        wheel_603.append("0000");

        wheel_604.push_back(shex_right_vel_rpm[2 + 4]);
        wheel_604.push_back(shex_right_vel_rpm[3 + 4]);
        wheel_604.push_back(shex_right_vel_rpm[0 + 4]);
        wheel_604.push_back(shex_right_vel_rpm[1 + 4]);
        wheel_604.append("0000");
    }

    if (valid == true)
    {
        can_send_data(1, const_cast<char *>("601"), const_cast<char *>(wheel_601.c_str()));
        can_send_data(1, const_cast<char *>("602"), const_cast<char *>(wheel_602.c_str()));
        can_send_data(1, const_cast<char *>("603"), const_cast<char *>(wheel_603.c_str()));
        can_send_data(1, const_cast<char *>("604"), const_cast<char *>(wheel_604.c_str()));
        return true;
    }
    else
    {
        return false;
    }

    return true;
}

bool Init_car()
{
    set_can(print_traffic, const_cast<char *>(tty_device.c_str()), canusb_int_speed, br, gap_time, 2);
    can_send_data(2, const_cast<char *>("601"), const_cast<char *>("2f00200001000000"));
    can_send_data(2, const_cast<char *>("602"), const_cast<char *>("2f00200001000000"));
    can_send_data(2, const_cast<char *>("603"), const_cast<char *>("2f00200001000000"));
    can_send_data(2, const_cast<char *>("604"), const_cast<char *>("2f00200001000000"));
    std::cout << "Init_car" << std::endl;
    return true;
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg)
{
    double linear_x = cmd_vel_msg->linear.x;   // 米每秒（m/s）
    double angular_z = cmd_vel_msg->angular.z; // 弧度每秒（rad/s）

    // 在这里可以处理速度控制指令，例如，将指令传递给机器人底盘
    // 你的控制逻辑可以放在这里
    ROS_INFO("Received cmd_vel: linear_x=%.2f, angular_z=%.2f", linear_x, angular_z);
    // 这里将速度结算，然后通过usb转can发出来
    left_vel = linear_x - angular_z * width_robot / 2.0;
    right_vel = linear_x + angular_z * width_robot / 2.0;
    ROS_INFO("left_vel=%.4f, right_vel=%.4f", left_vel, right_vel);

    control_wheel();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_subscriber");
    ros::NodeHandle nh;

    nh.getParam("width_robot", width_robot);
    nh.getParam("width_wheel", width_wheel);
    nh.getParam("gap_time", gap_time);
    nh.getParam("print_traffic", print_traffic);
    nh.getParam("tty_device", tty_device);
    nh.getParam("canusb_int_speed", canusb_int_speed);
    nh.getParam("baudrate", br);
    Init_car();
    std::cout << "control_car begin" << std::endl;

    // ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel_Remote", 1, cmdVelCallback);
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, cmdVelCallback);

    ros::spin();

    for (int i = 0; i < 1; i++)
    {
        left_vel = 0;
        right_vel = 0;
        control_wheel();
    }
    
    return 0;
}
