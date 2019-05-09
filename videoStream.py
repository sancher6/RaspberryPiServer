import os
from socket import *
import socket # for socket
import sys 
import pickle
#import testScript as testStream

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
     
def sendInterupt(conn):
    import time
    run = 0
    
    while True:                       #Wait for to start
        if(conn.recv == 1):           #Start clock
            run = 1
        elif(conn.recv == -1):        #Break process
            break
        while (run == 1):             #running clock
            if(conn.recv == 0):       #stop clock
                break
            time.sleep(.05)
            print("Send interupt")
            
     
def startStream(conn):
    #Start up stream
    #execfile("./Desktop/main/VideoStream/stream.py")
    import os
    pid  = os.getpid()
    conn.send(pid)
    runStream = 0
    while True:
        runStream = conn.recv()
        print("test")
        if(runStream == 1):
            from VideoStream import stream
            print("running stream")
        elif(runStream == -1):
            break

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

parent_conn, child_conn = Pipe()
p_conn, c_conn = Pipe()

while True:
    try:
        s.bind(pp)
        break

    except socket.error as error:
        os.system('fuser -k 22/tcp')
    
f = open("./scriptLog.txt", "w")


#Set up video stream Process
runStream = 0

pid = os.getpid()
print(pid)
vp = Process(target = startStream, args=(child_conn,))
interuptProcess = Process(target = sendInterupt, args=(c_conn,))

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
    
    vp.start()
    interuptProcess.start()
    cPid = parent_conn.recv()
    print("Child = {}".format(cPid))
    
    while True:
    
        parent_conn.send(1)
        p_conn.send(1)
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
        date = str(datetime.date.today())
        if((date +": " + msg) != (date +": Nothing")):
            f.write((date +": " + msg+"\n"))
        #conn.sendall(msg)
    
    parent_conn.send(-1)
    p_conn.send(0)
    p_conn.send(-1)
    print("Waiting for child")
    os.system('kill -9 %d' %cPid)
    os.system('[fuser -k 8000/tcp')
    vp.join()
    interuptProcess.join()
    conn.close()
    do_stopcar()
    print("Disconnected")
    f.write((date +": " + "Device disconnected\n"))
