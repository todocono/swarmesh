import time, serial
from serial.tools import list_ports

def start_serial():
    # use the following to see all the serial ports:
    # python -m serial.tools.list_ports

    # for my computer, that's:
    port = "/dev/tty.usbmodem14402"
    baud = 115200  # that is the default to talk with microbit

    s = serial.Serial(port)
    s.baudrate = baud
