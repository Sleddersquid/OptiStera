import serial
# import pandas as pd

SAMPLES = 5000

path = "MATLAB_code/"

csv_file = path + "noe.csv"

with open(csv_file, 'w'): pass

def readserial(comport, baudrate):

    ser = serial.Serial(comport, baudrate)

    current_sample = 0

    while current_sample < SAMPLES:
        # Reads line from Serial port, decodes from bytes to str and strips leading and trailing whitespace
        line = ser.readline().decode().strip()
        
        print(line)

        if not line: # If line empty, next iteration
            continue
        
        with open(csv_file, 'a') as f:
            f.write(line + '\n') # Since line is stripped for leading and trailing whitespace, need to add newline
            current_sample += 1 # Only increment sample count when there is a valid line
    
    ser.close()

if __name__ == '__main__':
    readserial('COM5', 115200)
