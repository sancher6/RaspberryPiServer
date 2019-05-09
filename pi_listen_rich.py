import socket
import binascii

HOST = '192.168.4.1'
PORT = 22
    
#with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST,PORT))
s.listen(1)
conn,addr = s.accept()
i=0
dataString = ""
while True:
    #added print, each run the addr changes 
    #print (str(conn))
    print("Run : " + str(i))
    data = conn.recv(1024)
    if not data:
        print('breaking')
        conn.close() 
        break
    dataString = data.decode(('utf-8'),'replace')
    print(dataString)
    if(dataString == "Disconnect"):
        print('Client' + str(addr)+ ' sent: ' + dataString)
        print("Successful Disconnection")
        #conn.sendto("Goodbye Friend".encode('utf-8'),addr)
        #conn.close() 
        break
    elif(dataString == "Connected"):
        print('Client' + str(addr)+ 'sent: ' + dataString)
        print("Successful Connection")
        #conn.sendto("Hello Friend".encode("UTF-8"),addr)
        #conn.close() 
        i=i+1
        continue
    else:
        print("Unknown command")
        break
    
print('closed')
conn.close() 
s.close()
    #conn.close()
