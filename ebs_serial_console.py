import serial
import threading
import time

class SerRead(threading.Thread):
	"""docstring for SerRead"""
	#exFlag = 0
	def __init__(self):
		super (SerRead, self).__init__()
		self.exFlag = 0;
	
	def run(self):
		while 1:
			if self.exFlag == 1:
				break
			if ser.in_waiting != 0:
				sOutMsg = ser.read(ser.in_waiting)
				print('\n>>'+sOutMsg.hex()+'\n<<')

	def breakLoop(self):
		self.exFlag = 1;

		
ser = serial.Serial('COM12', 119200, timeout = 5)
serRead = SerRead()
serRead.start()

while 1:
	time.sleep(1)
	sInMsg = input('<<')
	if sInMsg == 'exit':
		serRead.breakLoop()
		break
	ser.write(bytes.fromhex(sInMsg))
ser.close()