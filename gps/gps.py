# uses pynmea2 library
# https://github.com/Knio/pynmea2/tree/master

import serial
import time
import string
import pynmea2

# works if the sensor is under direct sky, won't work inside
# /dev/ttyTHS1 is the tx/rx ports on the gpio
# port = "/dev/ttyTHS1"
# for usb
port = "/dev/ttyACM0"

ser = serial.Serial(port,baudrate=9600,timeout=0.5)
dataout = pynmea2.NMEAStreamReader()

while True:
    newdata=ser.readline().decode('utf-8')
    if newdata[0:6]=="$GPRMC":
        newmsg=pynmea2.parse(newdata)
        lat=newmsg.latitude
        lng=newmsg.longitude
        gps="Latitude=" +str(lat) + " and Longitude=" +str(lng)
        print(gps)
