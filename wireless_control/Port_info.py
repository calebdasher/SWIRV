import serial
import serial.tools.list_ports

ports = serial.tools.list_ports.comports()
print ([port.manufacturer for port in ports])
