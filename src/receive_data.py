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

ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)  # open serial port

NUM_CHUNKS = 30
for j in range(NUM_CHUNKS):     # keep recording data
    for i in range(BUFF_SIZE * 2):
        byte = ser.read(1)
        # print(hex(byte[0]))
        file.write(byte)
    print(f"{j} chunks / {100*j/NUM_CHUNKS}%")

# byteArr = ser.read(BUFF_SIZE * 2)
# file.write(byteArr)


# for i in byteArr:
#     print(hex(i))
#     file.write(str(i))

ser.close()
file.close()
