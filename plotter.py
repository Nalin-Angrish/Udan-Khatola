import serial
import matplotlib.pyplot as plt
import numpy as np

roll_list = []
pitch_list = []
yaw_list = []

with serial.Serial("COM5", 115200) as s:
    while len(roll_list) < 100:
        l = s.readline().decode("utf-8")
        try:
            # order of pitch roll and yaw depends on how the sensor is mounted
            pitch, roll, yaw = map(float, l.split())
        except ValueError:
            continue
        roll_list.append(roll)
        pitch_list.append(pitch)
        yaw_list.append(yaw)
        print(pitch, roll, yaw)

plt.plot(np.arange(len(roll_list)), roll_list, label="roll")
plt.plot(np.arange(len(pitch_list)), pitch_list, label="pitch")
plt.plot(np.arange(len(yaw_list)), yaw_list, label="yaw")
plt.legend()
plt.show()
