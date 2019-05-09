#from socket import *
import socket # for socket
import sys 
import pickle
import os

from multiprocessing import Process, Pipe

#Imports to communicate with Arduino
import serial
import RPi.GPIO as GPIO
import time
import datetime

#time.sleep(10)

#Commands for Arduino
ser=serial.Serial("/dev/ttyACM0",9600)  #change ACM number as found from ls /dev/tty/ACM*

def do_startcar():
     print ("Starting Car")
     ser.write('g'.encode('utf-8'))

def do_stopcar():
     print ("Stoping Car")
     ser.write('s'.encode('utf-8'))

def do_goleft():
     print ("Turning Left")
     ser.write('l'.encode('utf-8'))
                          
def do_goright():
     print ("Turning Right")
     ser.write('r'.encode('utf-8'))
     
def startStream():
    #Start up stream
    execfile("./Desktop/main/VideoStream/stream.py")
     
def sendInterupt(conn):
    import time
    import os
    import RPi.GPIO as GPIO
    pid  = os.getpid()
    run = 0
    prevMsg = ""
    conn.send(pid)
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT)
        GPIO.setup(22, GPIO.IN)
        GPIO.setup(9, GPIO.IN)
        while(GPIO.input(22) == True):
            while (GPIO.input(9) == True):                       #Wait for to start
                #print("WAiting to start")
                GPIO.output(18, GPIO.HIGH)
                GPIO.output(18, GPIO.LOW)
                time.sleep(.1)
                #print("Send interupt")
    finally:
        GPIO.cleanup()

#ip = '172.16.93.182'
ip = '192.168.4.1'
try: 
        s = socket.socket() 
        print ("Socket successfully created")
except socket.error as err: 
        print ("socket creation failed with error %s" %(err)) 

# default port for socket 
port = 22

# connecting to the server
pp = (ip,port)
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)

while True:
    try:
        s.bind(pp)
        break

    except socket.error as error:
        os.system('fuser -k 22/tcp')
        
        
##    s.bind(pp)


#f = open("./scriptLog.txt", "w")

p_conn, c_conn = Pipe()

#Set up video stream Process
#vp = Process(target = startStream)
interuptProcess = Process(target = sendInterupt, args=(c_conn,))

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)

GPIO.output(23, GPIO.HIGH)
interuptProcess.start()
try:
    while True:
        #listen mode
        s.listen(5) 
        print ("socket is listening")

        dataObj = bytes(8)

        #accept 
        conn,addr = s.accept()
        msg = "connected"
        print (msg)
        
        #For starting the stream set this to 1
        start = 1
        
        GPIO.output(25, GPIO.HIGH)
        interuptPID = p_conn.recv()
        #p_conn.send(1)
        while True:
            
                
            dataObj = conn.recv(4096)
            decodeData = dataObj#.decode(encoding='utf-8', errors='strict')
            if (not decodeData): break
            data = ''
            print (decodeData)
            if(len(decodeData) > 4):
               data = (decodeData[3:4]) + (decodeData[4:5])
               
            if (decodeData == b't\x00\x0211'):
                do_goright()
                msg = "Turning right"
                
            elif (decodeData == b't\x00\x0221'):
                do_goleft()
                msg = "Turning left"
                
            elif (decodeData == b't\x00\x0231'):
                do_startcar()
                msg = "Starting"
                
            elif (decodeData == b't\x00\x0230'):
                do_stopcar()
                msg = "Stopping"
            else:
                msg = "Nothing"
    ##        date = str(datetime.date.today())
    ##        if((date +": " + msg) != (date +": Nothing")):
    ##            f.write((date +": " + msg+"\n"))
            #conn.sendall(msg)
        
        GPIO.output(25, GPIO.LOW)    
        #os.system('kill -9 %d' %interuptPID)
        conn.close()
        do_stopcar()
        print("Disconnected")
    ##    f.write((date +": " + "Device disconnected\n"))
        
finally:
    GPIO.output(23, GPIO.LOW)
    GPIO.output(25, GPIO.LOW)    
    interuptProcess.join()
    GPIO.cleanup()