import serial

ser = serial.Serial('COM9', 119200, timeout = 1)
ser.write(b'23')
sMsg = ser.read(4)
print (sMsg)
	#print (" ".join(hex(ord(n)) for n in sMsg))