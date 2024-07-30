import serial
import csv
import atexit

def main():
    atexit.register(exit_handler)

    serialPort = serial.Serial(port="COM7", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
    serialString = ""  # Used to hold data coming over UART

    with open('./pyLogger/Logger.csv', 'a') as the_file:    
        while 1:
           # Read data out of the buffer until a carraige return / new line is found
            serialString = serialPort.readline()
            printString = serialString.decode("utf-8")
            printString = printString.rstrip('\0')
           # Print the contents of the serial data
            try:
                if(len(printString) > 0):
                    print(printString)
                    the_file.write(printString)
            except:        
                pass

def exit_handler():
    print ('My application is ending!')
main()