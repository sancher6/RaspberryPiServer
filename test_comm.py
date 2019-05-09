
import socket # for socket

ip = '192.168.4.1'
try: 
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print ("Socket successfully created")
except socket.error as err: 
        print ("socket creation failed with error %s" %(err)) 

# default port for socket 
port = 22

# connecting to the server
pp = (ip,port)
s.connect(pp)
try:
    while True:
        s.connect(pp)
        s.sendall("Connected".encode("UTF-8"))
        print(s.recv(1024).decode("UTF-8"))


        s.sendall("Disconnect".encode("UTF-8"))
        print(s.recv(1024).decode("UTF-8"))
finally:
    s.close()