#!/usr/bin/python

# -------------------------------------------------------------------
# A script that redirects Arduino serial output to a file and 
# adds time stamps
# 
# David Vella, June 2020
# -------------------------------------------------------------------

import serial, time

serial_port = '/dev/ttyUSB0'
baud_rate = 9600
log_file = 'serial.log'
read_time = 5

reads = 0

ser = serial.Serial(serial_port, baud_rate)

with open(log_file, 'w') as f:
    flag = True
    start = time.time()

    while time.time() - start < read_time:
        line = ser.readline()
        
        try:
            line = line.decode('utf-8')

            if 'Serial connection' in line:
                continue

            if flag:
                start = time.time()
                flag = False
                continue

            line = line[:-1]

            f.write(f'{time.time() - start} {line}')

        except UnicodeDecodeError:
            print('Failed to decode line')

ser.close()

print('Finished')

with open(log_file, 'r') as f:
    for lines in f:
        reads += 1

print(f'Read frequency: {int(reads / read_time)} Hz')