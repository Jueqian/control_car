import serial
import time
import random
from datetime import datetime

# Serial communication settings
serial_port = '/dev/ttyUSB0'  # 请根据实际情况修改串口名称
baud_rate = 57600
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Function to generate random float within a given range
def generate_random_float():
    return round(random.uniform(-20.000000, 20.000000), 6)

# Function to send data from sender A
def send_data():
    send_count = 1
    while True:
        linear_value = generate_random_float()
        angular_value = generate_random_float()

        # Format the message
        message = f"Linear: {linear_value}, Angular: {angular_value}"
        print("len message: ", len(message))
        timestamp = datetime.now().strftime("%H-%M-%S-%f")
        message += f"_Count: {send_count}_Timestamp: {timestamp}\r\n"
        

        # Send the message
        ser.write(message.encode())
        print(message + '\n')
        send_count += 1
        time.sleep(1)  # 200ms interval

if __name__ == "__main__":
    send_data()
