import serial
# import pandas as pd

SAMPLES = 5000

csv_file = "clean_reset.csv"

with open(csv_file, 'w'): pass

def readserial(comport, baudrate):

    ser = serial.Serial(comport, baudrate, timeout=0.1)

    current_sample = 0

    while current_sample < SAMPLES:
        line = ser.readline().decode().strip()
        
        print(line)

        if not line:
            continue
        
        with open(csv_file, 'a') as f:
            f.write(line + '\n') # Since line is stripped for leading and traling whitespace , need to add newline
            current_sample += 1
    
    ser.close()

if __name__ == '__main__':
    readserial('COM5', 115200)
