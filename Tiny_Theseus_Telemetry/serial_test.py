import serial
import time

ser = serial.Serial('COM8', 9600, timeout=2.0)
print("opened, waiting for data...")
time.sleep(1)

for i in range(10):
    waiting = ser.in_waiting
    print(f"bytes waiting: {waiting}")
    if waiting > 0:
        data = ser.read(waiting)
        print(f"data: {repr(data)}")
    time.sleep(0.5)

ser.close()