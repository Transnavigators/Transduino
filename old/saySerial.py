import serial
import sys

ser = serial.Serial(port='/dev/ttyACM0',baudrate=115200,timeout=0)
ser.write(sys.argv[1])
