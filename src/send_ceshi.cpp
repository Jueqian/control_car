#include "ros/ros.h"
#include "serial/serial.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_sender");
    ros::NodeHandle nh;

    serial::Serial ser;
    ser.setPort("/dev/ttyUSB0");
    ser.setBaudrate(57600);

    // 设置循环的频率为 5Hz
    ros::Rate rate(5);

    while (ros::ok())
    {
        // 检查串口是否打开，如果不打开，则尝试打开
        if (!ser.isOpen())
        {
            ROS_WARN("Serial port is not open. Trying to open...");
            try
            {
                ser.open();
            }
            catch (const std::exception &e)
            {
                ROS_ERROR("Failed to open serial port: %s", e.what());
                // 可以添加进一步的处理逻辑
                continue;  // 跳过本次循环，继续下一次
            }
        }

        try
        {
            // 尝试执行一些串口操作，比如读取一行数据
            std::string serial_data = ser.readline();

            // 如果能够成功执行到这里，说明串口是打开的
        }
        catch (const std::exception &e)
        {
            ROS_WARN("Error reading from serial port: %s", e.what());

            // 在这里可以添加相应的处理逻辑，比如重新打开串口等
            ser.close();
            continue;  // 跳过本次循环，继续下一次
        }

        // 发送单个字符 'a' 到串口
        ser.write("aabcdefg");

        // 打印信息到终端
        ROS_INFO("Sending 'aabcdefg' over serial");

        // 延时等待，以满足设置的循环频率
        rate.sleep();
    }

    // 关闭串口
    ser.close();

    return 0;
}
