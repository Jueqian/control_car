#pragma once

// 初始化can通信的一些参数
int set_can(bool print_traffic_, char *tty_device_, int canusb_int_speed_,
            int baudrate_, float gap_time_, int can_mode_);
// 发送数据
int can_send_data(int terminate_after_, char *ID_, char *data_);
