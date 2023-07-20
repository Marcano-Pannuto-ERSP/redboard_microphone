"""
Script to run on the server at the same time that main.c is running
This file will receive the bytes over uart and decode them back into numbers
and save the raw data to a file
"""

import serial

# Probably change this later to just keep running until we are done sending data
BUFF_SIZE = 4096        # Number of 16-bit ints being sent over uart
FILENAME = "out.raw"

file = open(FILENAME, 'wb')
buffer = ""

ser = serial.Serial('/dev/ttyUSB0', baudrate=115300)  # open serial port

byteArr = ser.read(160000)
file.write(byteArr)
# for i in byteArr:
#     print(hex(i))
#     file.write(str(i))

ser.close()
file.close()

# for i in range(BUFF_SIZE):
#     msb = ser.readline()        # I don't know if msb/lsb are swapped
#     lsb = ser.readline()
#     # short = msb << 8 + lsb
#     # #print(short)
#     # buffer += str(short) + '\n'
#     buffer += msb + lsb

# ser.close()
# file.write(buffer)