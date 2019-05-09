#from socket import *
import socket # for socket
import sys 
import pickle
import os
import struct

from multiprocessing import Process, Pipe
import http.client

#Imports to communicate with Arduino
import serial
import RPi.GPIO as GPIO
import time
import datetime


def do_forward(ser):
     #print("F: {}".format(str(time.clock())))
     ser.write('f'.encode('utf-8'))
     # ("Move forward")#: {}".format(str(time.clock())))

def do_stopcar(ser):
     #pwairint("S: {}".format(str(time.clock())))
     ser.write('s'.encode('utf-8'))
     #print ("Stoping Car: {}".format(str(time.clock())))

def do_goleft(ser):
     #print("L: {}".format(str(time.clock())))
     ser.write('l'.encode('utf-8'))
     #print ("Turning Left: {}".format(str(time.clock())))
                          
def do_goright(ser):
     #print("R: {}".format(str(time.clock())))
     ser.write('r'.encode('utf-8'))
     #print ("Turning Right: {}".format(str(time.clock())))
                          
def do_goback(ser):
     #print("B: {}".format(str(time.clock())))
     ser.write('b'.encode('utf-8'))
     #print ("Beep!: {}".format(str(time.clock())))
    

def startStream(conn):
    import os
    pid  = os.getpid()
    conn.send(pid)
    runStream = 0
    try:
        #from VideoStream import stream
        #os.system("raspivid -t 0 -w 640 -h 480 -hf -fps 60 -o - | nc %s 
        execfile("./Desktop/main/VideoStream/stream.py")
            
    finally:
        print("Stopping stream")
        
def phoneComm(cPhone_conn, s, ser):
    import os
    import RPi.GPIO as GPIO
    pid  = os.getpid()
    cPhone_conn.send(pid)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(2, GPIO.OUT)
    GPIO.output(2, GPIO.LOW)
    GPIO.setup(11, GPIO.OUT)
    GPIO.output(11, GPIO.LOW)
    power = 1
    auto = False
    try:
        while True:
            s.listen(5) 
            print ("socket is listening here")
    
            #accept 
            conn,addr = s.accept()
            #cPhone_conn.send(msg)
            try:
                i=0
                dataString = ""
                while True:
                    #added print, each run the addr changes 
                    #print (str(conn))
                    #print("Run : " + str(i))
                    #print("Waiting for command")#: {}".format(str(time.clock())))
                    data = conn.recv(1024)
                    #print("Command received")#: {}".format(str(time.clock())))
                    
                    dataString = data.decode(('utf-8'),'replace')
                    dataString = dataString.rstrip("\r\n")
                    #print("Input: {}".format(dataString))
                    GPIO.output(2, GPIO.HIGH)
                    if not data:
                        #print('breaking')
                        conn.sendall("DC".encode("UTF-8"))
                        cPhone_conn.send("DC")
                        conn.close() 
                        break
                    if(dataString == "Off"):
                        #print('Client' + str(addr)+ ' sent: ' + dataString)
                        #print("Powering down...")
                        conn.sendall(bytes("Off\n", "UTF-8"))
                        cPhone_conn.send("Off")
                        power = 0
                        break
                    if(dataString == "DC"):
                        #print('Client' + str(addr)+ ' sent: ' + dataString)
                        #print("Successful Disconnection")
                        ser.write('s'.encode('utf-8'))
                        conn.sendall(bytes("DC\n", "UTF-8"))
                        cPhone_conn.send("DC")
                        break
                    elif(dataString == "C"):
                        #print('Client' + str(addr)+ 'sent: ' + dataString)
                        #print("Successful Connection")
                        cPhone_conn.send("C")
                        conn.sendall(bytes("c\n", "UTF-8"))
                        i=i+1
                    elif(dataString == "f"):
                        #print("Moving Forward")
                        #do_forward(ser)
                        #print("Forward received: {}".format(str(time.clock())))
                        ser.write('f'.encode('utf-8'))
                        conn.sendall(bytes("F\n", "UTF-8"))
                        #print("Forward processed: {}".format(str(time.clock())))
                    elif(dataString == "l"):
                        #print("Left")
                        #do_goleft(ser)
                        ser.write('l'.encode('utf-8'))
                        conn.sendall(bytes("L\n", "UTF-8"))
                    elif(dataString == "r"): 
                        #print("Right")
                        #do_goright(ser)
                        ser.write('r'.encode('utf-8'))
                        conn.sendall(bytes("R\n", "UTF-8"))
                    elif(dataString == "b"):
                        #print("back")
                        #do_goback(ser)
                        ser.write('b'.encode('utf-8'))
                        conn.sendall("B\n".encode("UTF-8"))
                    elif(dataString == "end"):
##                        print("ending")
                        #do_stopcar(ser)
                        #print("end received: {}".format(str(time.clock())))
                        ser.write('s'.encode('utf-8'))
                        conn.sendall(bytes("End\n", "UTF-8"))
                        #print("end p: {}".format(str(time.clock())))
                    elif(dataString == "s"):
                        #print("STOP")
                        #do_stopcar(ser)
                        ser.write('s'.encode('utf-8'))
                        cPhone_conn.send("s")
                        GPIO.output(11, GPIO.HIGH)
                        conn.sendall(bytes("Stop\n", "UTF-8"))
                    elif(dataString == "m"):
                        #print("m")
                        cPhone_conn.send("m")
                        conn.sendall(bytes("m\n", "UTF-8"))
                    elif(dataString == "a"):
                        #print("a")
                        auto = True
                        cPhone_conn.send("a")
                        conn.sendall(bytes("a\n", "UTF-8"))
                    elif(dataString == "done"):
                        #print("Done")
                        conn.sendall(bytes("Done\n", "UTF-8"))
                    else:
                        if auto:
                            cPhone_conn.send(dataString)
                            conn.sendall(bytes("received\n", "UTF-8"))
                        else:
                            print("UC")
                            conn.sendall(bytes("UC\n", "UTF-8"))
                    
                print('closed')
                conn.close() 
                
                if(power == 0):
                    break
            except socket.error as e:
                print(e)
            except IOError as e:
                if e.error == errno.EPIPE:
                    cPhone_conn.send("Disconnect")
                else:
                    print(e)
            except:
                print("a FATAL error has occured: ", sys.exec_info()[0])
    finally:
        s.close()
        GPIO.cleanup()
            
def processInstr(conn, ser):
    import os
    import RPi.GPIO as GPIO
    pid  = os.getpid()
    conn.send(pid)
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(8, GPIO.IN)
        while True:
            msg = conn.recv()
            conn.send("done")
            instructions = msg.split(" ")
            #time.sleep(1)
            print(msg)
            while (len(instructions) >1):
                
                if(GPIO.input(8) == 1):
                    GPIO.output(11, GPIO.LOW)
                    break
                
                print(instructions[0])
                print(instructions[1])
                
                if(instructions[0] == "Forward" or instructions[0] == "f"):                    
                    ser.write('f'.encode('utf-8'))
                    #do_forward(ser)
                elif(instructions[0] == "Back" or instructions[0] == "b"):
                    ser.write('b'.encode('utf-8'))
                    #do_goback(ser)
                elif(instructions[0] == "Left" or instructions[0] == "l"):
                    ser.write('l'.encode('utf-8'))
                    #do_goleft(ser)
                elif(instructions[0] == "Right" or instructions[0] == "r"):
                    ser.write('r'.encode('utf-8'))
                    #do_goright(ser)
                elif(instructions[0] == "Stop" or instructions[0] == "s"):
                    ser.write('s'.encode('utf-8'))
                    #do_stopcar(ser)
                    
                time.sleep(int(instructions[1]))
                
                instructions.pop(0)
                instructions.pop(0)
            do_stopcar(ser)
                
    
    finally:
        GPIO.cleanup()
        
#make sure all ports are available
GPIO.cleanup()
GPIO.setwarnings(False)

#Commands for Arduino
ser=serial.Serial("/dev/ttyACM0",9600)  #change ACM number as found from ls /dev/tty/ACM*
ser.setDTR(False)
ser.setRTS(False)

ip = '192.168.4.1'
try: 
        s = socket.socket() 
        print ("Socket successfully created")
except socket.error as err: 
        print ("socket creation failed with error %s" %(err)) 

# default port for socket
port = 4957

# connecting to the server
pp = (ip,port)
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)

while True:
    try:
        s.bind(pp)
        break

    except socket.error as error:
        os.system('fuser -k 4957/tcp')

#Set up pipes
#print("Creating Pipes")    
    
#p_conn, c_conn = Pipe()
pVS_conn, cVS_conn = Pipe()
pPhone_conn, cPhone_conn = Pipe()
#pControl_conn, cControl_conn = Pipe()
pInstr_conn, cInstr_conn = Pipe()

#print("Pipes created")

#Set up Processes
vp = Process(target = startStream, args=(cVS_conn,))
#interuptProcess = Process(target = sendInterupt, args=(c_conn,))
phoneProcess = Process(target = phoneComm, args = (cPhone_conn, s, ser))
#controlProcess = Process(target = controlAlg, args = (cControl_conn, ser))
instrProcess = Process(target = processInstr, args = (cInstr_conn, ser))

#print("Processes created")

#print("".format())

#Set up GPIO for interrupts
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(2, GPIO.OUT)
GPIO.setup(3, GPIO.IN)
GPIO.output(23, GPIO.HIGH)

#Start processes and get the PIDs
#interuptProcess.start()
#ipPID = p_conn.recv()
vp.start()
vpPID = pVS_conn.recv()
phoneProcess.start()
ppPID = pPhone_conn.recv()
#controlProcess.start()
#cpPID = pControl_conn.recv()
instrProcess.start()
irPID = pInstr_conn.recv()

#print("Video: {}".format(vpPID))
#print("Interrupt: {}".format(ipPID))
#print("Phone: {}".format(ppPID))
#print("Control: {}".format(cpPID))
#print("Instructions: {}".format(irPID))

manual = 0
auto = 0
power = 1

try:
    while(power == 1):
        
        GPIO.output(25, GPIO.LOW)
        #For starting the stream set this to 1
        start = 0
        
        GPIO.output(12, GPIO.HIGH)
        pVS_conn.send(1)
        print("waiting...")
        while True:
            if(pPhone_conn.recv() == "C"):
                print("WE HAVE CONNECTED")
                break
        
        while True:
            msg = ""
            GPIO.output(19, GPIO.HIGH)
            GPIO.output(25, GPIO.HIGH)
            
            if(GPIO.input(3)==True):
                msg = pPhone_conn.recv()
                pPhone_conn.send("done")
                GPIO.output(2, GPIO.LOW)
                
                if(msg == "Off"):
                    power = 0
                    break
                
                #Switch to manual or auto
                if(manual == 0 and auto ==0):
                    if(msg == "m"):
                        print("Switching to manual override")
                        manual = 1
                    elif(msg == "a"):
                        auto = 1
                        
                #Manual conditions
                elif(manual == 1):
                    #if(msg == "f"):
                    #    do_forward(ser)
                    #elif(msg == "l"):
                    #    do_goleft(ser)
                    #elif(msg == "r"):
                    #    do_goright(ser)
                    #elif(msg == "b"):
                    #    do_goback(ser)
                    #elif(msg == "s" or msg == "DC"):
                    #    do_stopcar(ser)
                    if(msg == "a"):
                        manual = 0
                        auto = 1
                        
                elif(auto == 1):
                    #print("Here")
                    if(msg == "m"):
                        #GPIO.output(11, GPIO.HIGH)
                        pInstr_conn.send("m")
                        manual = 1
                        auto = 0
                    else:
                        #print("Send ir")
                        pInstr_conn.send(msg)
                        pInstr_conn.recv()
                        
            
                    
        GPIO.output(25, GPIO.LOW)
        GPIO.output(12, GPIO.LOW)
        do_stopcar(ser)
        print("Disconnected")
        
finally:
    GPIO.output(23, GPIO.LOW)
    GPIO.output(25, GPIO.LOW)
    GPIO.output(12, GPIO.LOW)
    
    #End all the processes
    os.system('kill -9 %d' %vpPID)
    #os.system('kill -9 %d' %ipPID)
    #os.system('kill -9 %d' %cpPID)
    os.system('kill -9 %d' %ppPID)
    os.system('kill -9 %d' %irPID)
    
    #Clean up the processes
    vp.join()
    #interuptProcess.join()
    phoneProcess.join()
    #controlProcess.join()
    instrProcess.join()
    GPIO.cleanup()
    do_stopcar(ser)
    #os.system('sudo shutdown -h now')

