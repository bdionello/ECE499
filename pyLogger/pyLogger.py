import serial
import csv

serialPort = serial.Serial(
    port="COM5", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
)
serialString = ""  # Used to hold data coming over UART

with open('Logger.csv', 'a') as the_file:    
    while 1:
        # Read data out of the buffer until a carraige return / new line is found
        serialString = serialPort.readline()
        serialString = serialString.decode("utf-8")
        serialString = serialString.rstrip('\0')

        # Print the contents of the serial data
        try:
            print(serialString)
           # print(serialString.decode("Ascii"))
            the_file.write(serialString)
        except:
            pass