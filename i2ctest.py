import smbus
import time
# set up busù http://www.raspberry-projects.com/pi/programming-in-python/i2c-programming-in-python/using-the-i2c-interface-2
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04
move_cmd = 'm'

def writeNumber(value1,value2):
    bus.write_i2c_block_data(address, move_cmd, value1, value2, 0);
    # bus.write_byte_data(address, 0, value)
    return -1

def readNumber():
    number = bus.read_byte(address)
    # number = bus.read_byte_data(address, 1)
    return number

while True:
    s1 = input("Enter 1 - 256: ")
    s2 = input("Enter 1 - 256: ")

    writeNumber(s1,s2)
    print "RPI: Hi Arduino, I sent you ", var
    # sleep one second
    time.sleep(1)

    number = readNumber()
    print "Arduino: Hey RPI, I received a digit ", number