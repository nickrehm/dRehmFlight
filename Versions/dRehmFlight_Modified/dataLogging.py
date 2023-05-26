import serial
import pandas as pd
import numpy as np

portName = "/dev/ttyACM1"
baud = 500000

#!! REMEMBER TO CHANGE THIS !!#
fileName = "PID_tuning_kp_0.2_ki_0.0_kd_0.06"
serialComm = serial.Serial(portName, baud)
print("Connection established")
file = open(fileName, "w")

getData = serialComm.readline()
dataString = getData.decode('utf-8')

print(dataString)


