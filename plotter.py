import serial
import matplotlib.pyplot as plt
import numpy as np

roll_list = []
pitch_list = []

with serial.Serial("COM5", 115200) as s:
    while len(roll_list) < 100:
        l = s.readline().decode("utf-8")
        try:
            # order of pitch and roll depends on how the sensor is mounted
            pitch, roll = map(float, l.split())
        except ValueError:
            print("ValueError:", l)
            continue
        roll_list.append(roll)
        pitch_list.append(pitch)
        print(pitch, roll)

plt.plot(np.arange(len(roll_list)), roll_list, label="roll")
plt.plot(np.arange(len(pitch_list)), pitch_list, label="pitch")
plt.ylim(-180, 180)
plt.legend()
plt.show()
