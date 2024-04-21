# Date: 2024/1/23 22:05
import serial
import time
from datetime import datetime

# Serial communication settings
serial_port = 'COM16'  # 请根据实际情况修改串口名称
baud_rate = 57600
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Data statistics variables
delay_times = []

# 初始化统计变量
max_delay = float('-inf')
min_delay = float('inf')
total_delay = 0
B_local_count = 0
count_diff = 0
first_read = True
lost_packets = 0


# Function to receive and process data at receiver B
def receive_data():
    global max_delay, min_delay, total_delay, count, count_diff, lost_packets, B_local_count, first_read
    while True:
        data_len = ser.inWaiting()
        if data_len > 0:
            # if receive_data is not None:
            received_data = ser.readline().decode().strip()
            # ser.flushInput()
            # 获取当前系统时间
            current_time = datetime.now()
            # print(current_time)

            # 字符串处理
            try:
                A_send_count = int(received_data.split('_')[1].split(':')[-1])
                A_send_timestamp = received_data.split('_')[-1].split('-')
                A_send_second = int(A_send_timestamp[1]) * 60 + int(A_send_timestamp[2]) + int(
                    A_send_timestamp[3]) / 1e6

            except:
                print('error!')
                lost_packets += 1
                continue


            B_local_seconds = current_time.minute * 60 + current_time.second + current_time.microsecond / 1e6
            print(f"Original timestamp: {received_data.split('_')[-1]}")
            print(f"Time in seconds: {A_send_second:.6f} seconds")
            print(f"Current time: {current_time.strftime('%H:%M:%S.%f')}")
            print(f"Time in seconds: {B_local_seconds:.6f} seconds")

            # 更新统计变量
            AtoB_delay = B_local_seconds - A_send_second
            max_delay = max(max_delay, AtoB_delay)
            min_delay = min(min_delay, AtoB_delay)
            total_delay += AtoB_delay
            B_local_count += 1

            data_length = len(received_data)
            print("received_data: ", received_data.strip())
            print("data_len: ", data_length)
            print("Once delay:", AtoB_delay)
            # print()

            if first_read is True:
                first_read = False
                count_diff = A_send_count - B_local_count

            if B_local_count + count_diff + lost_packets != A_send_count:
                lost_packets += 1
                print(f"Error: Lost Packets: {lost_packets}")

            # 每10次数据打印一次结果
            if B_local_count % 10 == 0:
                avg_delay = total_delay / B_local_count
                print()
                print(f"Max Delay: {max_delay:.6f} seconds | ")
                print(f"Min Delay: {min_delay:.6f} seconds | ")
                print(f"Avg Delay: {avg_delay:.6f} seconds | ")
                print(f"B_local_count: {B_local_count} | count_diff: {count_diff} | ")
                print(f"丢包次数 lost_packets: {lost_packets}")
                print()
            data_len = 0

        else:
            time.sleep(0.002)
            # print('hello?')


if __name__ == "__main__":
    # Run receiver B in a separate thread
    # import threading
    #
    # receiver_thread = threading.Thread(target=receive_data)
    # receiver_thread.start()

    receive_data()
    # while True:

        # try:
        #     pass
        # except KeyboardInterrupt:
        #     # Allow graceful exit on Ctrl+C
        #     # Close the serial port
        #     ser.close()
        #     break


