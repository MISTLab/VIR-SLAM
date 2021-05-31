#!/usr/bin/env python
# A driver for yc uwb devices, read data and parse to range information.
import serial
from serial import Serial
import time



class uwbdriver(Serial):
	"""docstring for uwbdriver"""
	def __init__(self, port, baudrate, goonMeas=False):
		super(uwbdriver, self).__init__(port=port, baudrate=baudrate, rtscts=True, timeout=1,dsrdtr=True)
		self.rawData = 0
		self.rangeRaw01 = 0
		self.rangeRaw02 = 0
		self.rangeTime = 0
		self.rangeCorrected = 0
		self.goonMeas = goonMeas
		self.rangeGot = False
		self.calibCoeff = 1.
		self.calibIntercept = 0.


	def getRange(self):
		while self.rangeGot == False or self.goonMeas:
			self.rawData = self.readline()
			self.rangeTime = time.time()
			#print(" raw ", self.rawData)
			self.decode()
			pass
		pass

	def decode(self):
		if self.rawData[0:6] == "mr 01 " or self.rawData[0:6] == "mr 03 ":
			rangestr = self.rawData[5:14]
			self.rangeRaw01 = int(rangestr,16)
			rangestr = self.rawData[14:23]
			self.rangeRaw02 = int(rangestr,16)
			self.rangeGot = True
			print("rangeRaw01 is ", self.rangeRaw01," rangeRaw02 is ", self.rangeRaw02 )
		pass

if __name__ == '__main__':
	try:
		port = "/dev/ttyACM0"
		baudrate = 115200
		uwbdevice = uwbdriver(port,baudrate)
		print("Connected with ", port)
		uwbdevice.getRange()
		print("Do ranging")
	except serial.SerialException:
		raise
