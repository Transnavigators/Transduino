import serial

ser = serial.Serial(port='/dev/ttyACM0',baudrate=115200,timeout=0)
while True:
	data = ser.readline()
	if data and data != "\n" and data != "\n\r" and data != "\r\n":
		print data
