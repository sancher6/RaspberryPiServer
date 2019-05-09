# PI SPECIFIC
from threading import Condition, Thread
import time,random

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

IP = '192.168.4.1'
PORT = 22
PORT2 = 5900 # YOUDOSUIOAGKDSLAJG CHANGE THIS ALKSDJGIASODJGA 
MAX_COMM = 5

# FOR ARDUINO COMMANDS
ser=serial.Serial("/dev/ttyACM0",115200, timeout = .75)
# FOR ARDUINO STATES
# ser=serial.Serial("/dev/ttyACM0",9600)

# CONDITION VARIABLES  
given_instr = Condition() 
given_state = Condition()
c = Condition()             #This should be shared to all threads so that
running = 0                 #in the case of phone disconnect, everything gets shut off
threads = []

dataObj = bytes(8)

def initGPIO():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(23, GPIO.OUT)
    GPIO.setup(25, GPIO.OUT)
    GPIO.setup(12, GPIO.OUT)
    GPIO.setup(19, GPIO.OUT)

class phone_Thread(Thread):
    
    def __init__(self, name):
        Thread.__init__(self)
        self.name = name
        
    def run(self):
        global running
        while True:
            createSocket()
            listen_tophone()
    
    # SEND CAR STATE TO PHONE 
    def msg_tophone(state): 
        print("Message is being sent\n")
        # SEND ITTTT
        try: 
            t = socket.socket() 
            print("Socket successfully created")
        except socket.error as err: 
                print ("socket creation failed with error %s" %(err)) 
        # BINDS SOCKET 
        t.bind((IP,PORT2))
        print("Socket successfully Binded") 
        t.send(state.encode('utf-8'))

    # SEND INSTRUCTIONS TO CAR 
    def msg_tocar(data): 
        print("Message is being sent\n")
        if (data == "11"):
            do_goright()
            while not [given_state]:
                given_state.wait() 
            msg = "Turning right"
        elif (data == "21"):
            do_goleft()
            while not [given_state]:
                given_state.wait() 
            msg = "Turning left"
        elif (data == "31"):
            do_startcar()
            while not [given_state]:
                given_state.wait() 
            msg = "Starting"
        elif (data == "30"):
            do_stopcar()
            while not [given_state]:
                given_state.wait() 
            msg = "Stopping"
        else:
            msg = "Nothing"

    # LISTENS FOR PHONE INSTRUCTIONS- shouldnt this run while phone is connected!?
    # as soon as there are no instructions it breaks, we dont want to re run this all the time
    def listen_tophone():
        given_instr.acquire() 
        while not [given_instr]:
            given_instr.wait()

        while(True): 
            # SOCKET LISTEN MODE 
            s.listen(MAX_COMM)  
            print("Listening for Instructions\n")
            conn, addr = s.accept() 
            dataObj = conn.recv(4096)
            if(not dataObj):
                running = 0
                break

            else:
                data = (dataObj[3:4]) + (dataObj[4:5])
                msg_tocar(data)
                given_instr.wait() 


    # listens for state from the car 
    def listen_tocar(): 
        print("Listening for Car State\n")
        #state update given 
        given_state.acquire() 
        while not [given_state]:
            given_state.wait()
        # SEND STATE TO PHONE 
        # msg_to_phone(state)
        given_state.notify()

    # CREATES SOCKET FOR INSTRUCTION SENDING 
    def create_socket():
        try: 
            s = socket.socket() 
            print("Socket successfully created")
            running = 1
            
        except socket.error as err: 
                print ("socket creation failed with error %s" %(err)) 
        
        # BINDS SOCKET 
        s.bind((IP,PORT))
        print("Socket successfully Binded") 

        # ACCEPT
        conn, addr = s.accept()
        print("Connected") 


# CAR INSTRUCTIONSSSSSSSSSSSSSSSSSSSSS
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
     
#Control Stream
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
     
#Send signal to Arduino to send data and main fucntion to collect data
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
                GPIO.output(18, GPIO.LOW)
                time.sleep(.1)
                print("FML")
    finally:
        GPIO.cleanup()
#main function to call threads 
if __name__ == '__main__':
    
    #Start socket and GPIO pins
    initGPIO()
    while True:
        create_socket()
        GPIO.output(25, GPIO.HIGH)
        GPIO.output(12, GPIO.HIGH)
        #pVS_conn.send(1)
        while running:  
            GPIO.output(19, GPIO.HIGH)
    
    # INITIALIZE CONDITION VARIABLES 

    # START THREADS 