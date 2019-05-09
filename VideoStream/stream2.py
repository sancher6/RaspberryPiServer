import socket
import time
import picamera

with picamera.PiCamera() as camera:
    camera.resolution = (640,480)
    camera.framerate = 24
    
    s = socket.socket()
    s.bind(('0.0.0.0',8000))
    s.listen(0)
    
    conn,addr = s.accept()
    
    