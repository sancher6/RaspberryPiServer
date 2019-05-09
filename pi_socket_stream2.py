#from socket import *
import socket # for socket
import sys 
import pickle
import os

from multiprocessing import Process, Pipe
import http.client

#Imports to communicate with Arduino
import serial
import RPi.GPIO as GPIO
import time
import datetime

#time.sleep(10)

#make sure all ports are available
GPIO.cleanup()

#Commands for Arduino
ser=serial.Serial("/dev/ttyACM0",115200, timeout = .1)  #change ACM number as found from ls /dev/tty/ACM*
ser.setDTR(False)
ser.setRTS(False)

def do_startcar():
     #print ("Starting Car")
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
    
#def endStream():
    #print("Ending")
    ##conn = http.client.HTTPConnection("localhost:8000")
    #conn.request("QUIT", "/")
    #conn.getresponse()
    
def startStream(conn):
    import os
    pid  = os.getpid()
    conn.send(pid)
    runStream = 0
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(6, GPIO.IN)
        while True:
            runStream = conn.recv()
            print("test")
            if(runStream == 1):
                from VideoStream import stream
            elif(runStream == -1):
                break
            
    finally:
        GPIO.cleanup()
     
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
            while (GPIO.input(9) == True):   
                GPIO.output(18, GPIO.HIGH)
                time.sleep(.1)
                GPIO.output(18, GPIO.LOW)
    finally:
        GPIO.cleanup()
        
def phoneComm(connP, s):
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(2, GPIO.OUT)
    
    try:
        while True:
            s.listen(5) 
            print ("socket is listening")

            #accept 
            conn,addr = s.accept()
            msg = "Connect"
            print(msg)
            connP.send(msg)
            try:
                while True:
                    dataObj = conn.recv(4096)
                    
                    try:
                        print(dataObj.decode("utf-8"))
                    except:
                        print("Set to ' '")
                    
                    decodeData = dataObj#.decode(encoding='utf-8')
                    
                    GPIO.output(2, GPIO.LOW)
                    
                    if (decodeData != " "):
                        GPIO.output(2, GPIO.HIGH)
                        connP.send(decodeData)
                        connP.recv()
                    elif (decodeData == "disconnect"):
                        break
                    
                    
                connP.send("Disconnect")
            except socket.error as e:
                print(e)
            except IOError as e:
                if e.error == errno.EPIPE:
                    connP.send("Disconnect")
                else:
                    print(e)
    finally:
        GPIO.cleanup()
    #do phone comm here

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

p_conn, c_conn = Pipe()
pVS_conn, cVS_conn = Pipe()
pPhone_conn, cPhone_conn = Pipe()
print("Pipes created")
#Set up video stream Process
vp = Process(target = startStream, args=(cVS_conn,))
interuptProcess = Process(target = sendInterupt, args=(c_conn,))
phoneProcess = Process(target = phoneComm, args = (cPhone_conn, s))
print("Processes created")

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(3, GPIO.IN)
GPIO.output(23, GPIO.HIGH)
interuptProcess.start()
interuptPID = p_conn.recv()
vp.start()
vpPID = pVS_conn.recv()
phoneProcess.start()
try:
    while True:
        
        #For starting the stream set this to 1
        start = 1
        
        
        GPIO.output(12, GPIO.HIGH)
        pVS_conn.send(1)
        print("waitng...")
        while True:
            if(pPhone_conn.recv() == "Connect"):
                break
        print("RUN!")
        
        while True:
            msg = ""
            GPIO.output(19, GPIO.HIGH)
            GPIO.output(25, GPIO.HIGH)
            #TODO: move these two lines to a new process
            
    ##        date = str(datetime.date.today())
    ##        if((date +": " + msg) != (date +": Nothing")):
    ##            f.write((date +": " + msg+"\n"))
            #conn.sendall(msg)
            if(GPIO.input(3)==True):
                msg = pPhone_Conn.recv()
                print(msg)
                pPhone_conn.send("done")
                
            if(msg == "left"):
                do_goleft();
            elif(msg == "right"):
                do_goright();
            elif(msg == "forward"):
                do_startcar();
            elif(msg == "stop"):
                do_stopcar();
            elif(msg == "disconnect"):
                print(msg)
                break
            
            if(ser.in_waiting ):
                #print("HEre")
                #front_sens = ser.readline()
                #print("herr?")
                #back_sens = ser.readline()
                #left_sens = ser.readline()
                #right_sens = ser.readline()
                #print("Front: %s" %front_sense)
                #print("Back: %s" %back_sense)
                #print("Left: %s" %left_sense)
                #print("Right: %s" %right_sense)
                try:
                    data = ser.read(ser.inWaiting()).decode('utf-8')
                    if(data != "" or data != " " or data != None):
                        print( data )
                except:
                    print("Caught")
                    
        GPIO.output(25, GPIO.LOW)
        GPIO.output(12, GPIO.LOW)
        #endStream()
        #os.system('kill -9 %d' %interuptPID)
        conn.close()
        do_stopcar()
        print("Disconnected")
    ##    f.write((date +": " + "Device disconnected\n"))
        
finally:
    GPIO.output(23, GPIO.LOW)
    GPIO.output(25, GPIO.LOW)
    GPIO.output(12, GPIO.LOW)
    #endStream()
    os.system('kill -9 %d' %vpPID)
    vp.join()
    interuptProcess.join()
    phoneProcess.join()
    GPIO.cleanup()