from datetime import datetime

received_data = 'Linear: 9.570806, Angular: -15.558133_Count: 180_Timestamp: 21-08-16-097551'

# 按下划线分割字符串
A_send_count = int(received_data.split('_')[1].split(':')[-1])

A_send_timestamp = received_data.split('_')[-1].split('-')
A_send_second = int(A_send_timestamp[1])*60 + int(A_send_timestamp[2]) + int(A_send_timestamp[3]) / 1e6

print(A_send_count)
print(type(A_send_count))

print('A_send_second:', A_send_second)
print(type(A_send_second))

timestamp_str = received_data.split(":")[-1].strip()
received_timestamp = datetime.strptime(timestamp_str, "%H-%M-%S-%f")
received_time_in_seconds = received_timestamp.minute * 60 + received_timestamp.second + received_timestamp.microsecond / 1e6
print(received_time_in_seconds)
print(type(received_time_in_seconds))