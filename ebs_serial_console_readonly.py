import serial
import threading
import time
		
ser = serial.Serial('COM9', 119200, timeout = 5)

while 1:
	while (ser.in_waiting != 0):
		sOutMsg = ser.read(ser.in_waiting)
		print('>>' + sOutMsg.hex() + '\n')
ser.close()