#!/usr/bin/env python3
from __future__ import print_function
import rospy
import serial
import serial.tools.list_ports
import time
import atexit

from motor_control.srv import Esp32

# Note this serial number is for the ESP32C6 plugged into UART
# We have two ESP32C6's
# Transmitter
# 38592ae6a0beed1195c3614371c9e8b5
# Reciever (The one used here)
# dc297ede9ebeed1195b45f4371c9e8b5

# Discover which port the ESP is connected to
def find_ESP32(port=None):
    if port is None:
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if p.serial_number is not None and ("dc297ede9ebeed1195b45f4371c9e8b5" or "38592ae6a0beed1195c3614371c9e8b5") in p.serial_number:
                port = p.device
    return port

# define function for initializing connection
def handshake_ESP32(ESP32, sleep_time=1):
    time.sleep(sleep_time)
    timeout = ESP32.timeout
    ESP32.timeout = 2
    ESP32.write(bytes([1]))
    while (ESP32.in_waiting < 0):
        pass
    _ = ESP32.read_until()
    ESP32.timeout = timeout

# Setup esp port
def open_ESP32(port, baudrate=115200, timeout=1):
    ESP32 = serial.Serial(port, baudrate=baudrate, timeout=timeout)
    ESP32.close()
    ESP32.open()
    handshake_ESP32(ESP32)
    return ESP32

def move_command(left, right):
    int_left = int(left*-1000)
    int_right = int(right*-1000)
    s_left = format(int_left, '05d')
    s_right = format(int_right, '05d')
    move_message= ('s'+s_left+s_right+'0'+'0').encode()
    ESP32.write(bytes(move_message))
    sensors = ESP32.readline()
    return sensors


def handle_move_command(req):
    print(req)
    return move_command(req.left, req.right)

def move_command_server():
    rospy.init_node('move_command_server')
    s = rospy.Service('motor_command', Esp32, handle_move_command)
    print("Ready to move")
    rospy.spin()

if __name__ == '__main__':
    port = find_ESP32()
    ESP32 = serial.Serial(port, baudrate=115200, timeout=1)
    # ensure port is clear after initial handshake
    ESP32.close()
    ESP32.open()
    # run handshape code to initialize the connection
    handshake_ESP32(ESP32)
    ESP32.close()
    with serial.Serial(port, baudrate=115200, timeout=1) as ESP32:
        handshake_ESP32(ESP32)
    ESP32 = open_ESP32(port)

    move_command_server()

