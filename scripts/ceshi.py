import serial
import time


serial_port = '/dev/ttyUSB0'  # 请根据实际情况修改串口名称
baud_rate = 57600
ser = serial.Serial(serial_port, baud_rate, timeout=1)


if ser.inWaiting() > 0:
    print(ser.in_waiting())
else:
    print('None')



linear_value = -10.02215
angular_value = 2.031523
message = f"Linear: {linear_value}, Angular: {angular_value}"

# 记录发送时刻
send_time = time.time()
ser.write(message.encode())
# 记录发送后的时间
end_time = time.time()
# 计算发送花费的时间
send_duration = end_time - send_time
print(f"Time spent on sending: {send_duration} seconds")

while True:
    if ser.inWaiting() > 0:
        receive_time = time.time()
        print(ser.inWaiting())
        received_data = ser.readline().decode()
        print("received_data:", received_data)

        # 计算发送到接收的时间间隔
        time_interval = receive_time - send_time
        print(f"Time interval: {time_interval} seconds")

        break
    else:
        pass

ser.close()
