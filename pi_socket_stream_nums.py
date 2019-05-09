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
ser=serial.Serial("/dev/ttyACM0",115200, timeout = .01)  #change ACM number as found from ls /dev/tty/ACM*
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

def do_adjleft():
     print ("Turning Left")
     ser.write('k'.encode('utf-8'))
                          
def do_adjright():
     print ("Turning Right")
     ser.write('q'.encode('utf-8'))
     
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
                time.sleep(.05)
                GPIO.output(18, GPIO.LOW)
                time.sleep(.05)
    finally:
        GPIO.cleanup()
        
def phoneComm(cPhone_conn, s):
    import os
    import RPi.GPIO as GPIO
    pid  = os.getpid()
    cPhone_conn.send(pid)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(2, GPIO.OUT)
    GPIO.output(2, GPIO.LOW)
    
    try:
        while True:
            s.listen(5) 
            print ("socket is listening")

            #accept 
            conn,addr = s.accept()
            msg = "Connect"
            print(msg)
            cPhone_conn.send(msg)
            try:
                response = ""
                while True:
                    
                    dataObj = conn.recv(4096)
                    response += dataObj.decode("utf-8", "ignore")
                    if not dataObj:
                        break; 
                    try:
                        if(dataObj != b''):
                            print(response)
                            #print(str(dataObj, 'UTF-8'))
                            
                            #print(b't\x00\x07forward'.decode("utf-8", "ignore"))
                            #print(b't\x00\x03endq\x00~\x00\x00q\x00~\x00\x01q\x00~\x00\x00q\x00~\x00\x01'.decode("utf-8", "ignore"))
                            if("forward" in response):
                                print("    forward")
                            elif("end" in response):
                                print("    end")
                            elif("right" in response):
                                print("    right")
                            elif("left" in response):
                                print("    left")

                    except:
                        dataObj = None
                    
                    decodeData = dataObj#.decode(encoding='utf-8')
                    
                    
                    
                    if (decodeData is not None and decodeData != b''):
                        GPIO.output(2, GPIO.HIGH)
                        cPhone_conn.send(decodeData)
                        GPIO.output(2, GPIO.LOW)
                        if(cPhone_conn.poll(.1)):
                            print(cPhone_conn.recv())
                        #conn.send("done")
                    elif (decodeData == "disconnect"):
                        break
                    
                    
                cPhone_conn.send("Disconnect")
            except socket.error as e:
                print(e)
            except IOError as e:
                if e.error == errno.EPIPE:
                    cPhone_conn.send("Disconnect")
                else:
                    print(e)
    finally:
        GPIO.cleanup()
    #do phone comm here


##This is the obstical avoidance stuff
# Conn is pipe, ser is serial input
def obstAvdAlg(conn, ser):
    import os
    import RPi.GPIO as GPIO
    pid  = os.getpid()
    conn.send(pid)
    fSense = 0
    bSense = 0
    lSense = 0
    rSense = 0
    while True:                #Fill out logic later for always run while car on
        while True:            #Fill out logic for always run while phone connected
            if(ser.in_waiting ):
                try:
                    data = ser.read(ser.inWaiting()).decode('utf-8')
                    if(data != "" or data != " " or data != None):
                        if("F" in data and "B" in data and "L" in data and "R" in data):
                            print(data.split())
                            fSense = data[1]
                            bSense = data[3]
                            lSense = data[5]
                            rSense = data[7]
                        
                except:
                    data = ""
            
            #Rest of alg can go here
            


## Terry this is where the control will go
# conn is the pipe if you need to run anything
def controlAlg(conn):
    import os
    import RPi.GPIO as GPIO
    pid  = os.getpid()
    conn.send(pid)
    
    while True:          #Always run while phone is on
        while True:      #Always run while phone connected
            control = True
            
            
            #Rest of alg can go here
            
            


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
pAvoid_conn, cAvoid_conn = Pipe()
pControl_conn, cControl_conn = Pipe()

print("Pipes created")
#Set up video stream Process
vp = Process(target = startStream, args=(cVS_conn,))
interuptProcess = Process(target = sendInterupt, args=(c_conn,))
phoneProcess = Process(target = phoneComm, args = (cPhone_conn, s))
obstacleProcess = Process(target = obstAvdAlg, args = (cAvoid_conn, ser))
controlProcess = Process(target = controlAlg, args = (cControl_conn, ))
print("Processes created")

print("".format())


GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(2, GPIO.OUT)
GPIO.setup(3, GPIO.IN)
GPIO.output(23, GPIO.HIGH)
interuptProcess.start()
ipPID = p_conn.recv()
vp.start()
vpPID = pVS_conn.recv()
phoneProcess.start()
ppPID = pPhone_conn.recv()
obstacleProcess.start()
oaPID = pAvoid_conn.recv()
controlProcess.start()
cpPID = pControl_conn.recv()

print("Video: {}".format(vpPID))
print("Interrupt: {}".format(ipPID))
print("Phone: {}".format(ppPID))
print("Obstacle: {}".format(oaPID))
print("Control: {}".format(cpPID))

try:
    while True:
        
        #For starting the stream set this to 1
        start = 1
        
        
        GPIO.output(12, GPIO.HIGH)
        pVS_conn.send(1)
        print("waiting...")
        while True:
            if(pPhone_conn.recv() == "Connect"):
                break
        print("RUN!")
        
        while True:
            msg = ""
            GPIO.output(19, GPIO.HIGH)
            GPIO.output(25, GPIO.HIGH)
            
            if(GPIO.input(3)==True):
                msg = pPhone_conn.recv()
                pPhone_conn.send("done")
                GPIO.output(2, GPIO.LOW) 
                
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
                        if("F" in data and "B" in data and "L" in data and "R" in data):
                            #print(data.split())
                            fSense = data[1]
                            bSense = data[3]
                            lSense = data[5]
                            rSense = data[7]
                            #print("Front: {},  Back: {},  Left: {},  Right: {}".format(fSense, bSense, lSense, rSense))
                        #print( data ) #F = lime 1, B = line 2, L = line 3, R = line 4
                        #if(int(data )<= 25 and int(data) > 10):
                        #    print("Warning!")
                        #elif(int(data) <= 10):
                        #    print("STOP")
                        #else:
                        #    print("Keep going")
                        
                except:
                    data = ""
                    
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
    #End all the processes
    os.system('kill -9 %d' %vpPID)
    os.system('kill -9 %d' %ipPID)
    os.system('kill -9 %d' %oaPID)
    os.system('kill -9 %d' %cpPID)
    os.system('kill -9 %d' %ppPID)
    
    #Clean up the processes
    vp.join()
    interuptProcess.join()
    phoneProcess.join()
    obstacleProcess.join()
    controlProcess.join()
    GPIO.cleanup()