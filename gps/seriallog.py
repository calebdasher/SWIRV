import serial.tools.list_ports
import serial
import time
import atexit



def find_USB_device():
    myports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
    print(myports)
    usb_port_list = [p[0] for p in myports]
    
    return usb_port_list

list_ports = find_USB_device()

date = time.localtime()

file_name = str(date[1])+"_"+str(date[2])+"_"+str(date[0])+"_"+str(date[3])+"h_"+str(date[4])+"m_"+str(date[5])+"s.txt"

serial = serial.Serial("COM3", 9600, timeout=5)

file = open(file_name, "at", buffering=1)

while True:
    data = str(serial.readline().decode("utf-8")).rstrip('\r\n')
    file.write(data)
    file.write("\n")

atexit.register(file.close())
