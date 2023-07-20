"""
Script to run on the server at the same time that main.c is running
This file will receive the bytes over uart and decode them back into numbers
and save the raw data to a file
"""

import serial

# Probably change this later to just keep running until we are done sending data
BUFF_SIZE = 4096        # Number of 16-bit ints being sent over uart
NUM_BYTES = BUFF_SIZE * 2   # unnecessary?
FILENAME = "out"        # File type??

file = open(FILENAME, 'w')
buffer = ""

ser = serial.Serial('/dev/ttyUSB0')  # open serial port

for i in range(BUFF_SIZE):
    msb = int(ser.readline())        # I don't know if msb/lsb are swapped
    lsb = int(ser.readline())
    short = msb << 8 + lsb
    print(short)
    buffer += str(short) + '\n'

ser.close()
file.write(buffer)