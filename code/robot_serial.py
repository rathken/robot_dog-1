import serial
from time import sleep

################################################################################
# functions
################################################################################

def pose1():
    ser.write("r1\nr5 250\n")
    print("pose1 - r1")
    sleep(0.25)    

def pose2():
    ser.write("r3\nr5 250\n")
    print("pose2 - r3")
    sleep(0.25)

def pose3():
    ser.write("r4 0 150 3 150 6 150 9 150\n")
    print("pose 3")
    sleep(0.25)
################################################################################
# main
################################################################################
# To find ports
# sudo python -m serial.tools.list_ports

ser = serial.Serial()
ser.baudrate = 115200
ser.port = '/dev/ttyACM0'
ser.timeout = 1

if (ser.is_open != True):
    print("Opening serial port")
    ser.open()




print("Reading serial port...")

need_reset = True
pos = 0


while(1):
    line = ser.readline()
    print(line)
    if line.startswith("ok"):
        if need_reset == True:
            print("Reset robot")
            ser.write("r0\n")
            need_reset = False
            pos = 1
        elif pos == 1:
            pose1()
            pos=2
        elif pos == 2:
            pose2()
            pos=1
        elif pos == 3:
            pose3()
            





