import socket
import time
# SOCKET DEFINITIONS
IP2 = "192.168.137.249"
IP = "192.168.137.1"
PORT = 3333
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UD
# sock.bind((IP2, PORT))    
bufferSize = 1024


def sendMessage(mes):
    message = bytes(mes, 'utf-8')
    sock.sendto(message, (IP2, 3333))
    msgFromServer = sock.recvfrom(bufferSize)
    msg = msgFromServer[0].decode()
    if(msg == "Done\n"):
        print("So far so good\n")
    else:
        while(msg != "Done\n"):
            message = bytes("Waiting\n", 'utf-8')
            sock.sendto(message, (IP2, 3333))
            msgFromServer = sock.recvfrom(bufferSize)
            msg = msgFromServer[0].decode()
            print(msg)



while True:

    numberStations = 0
    
    inputString = input("Enter motor controls:\n"
    "Forward : \"f <PWM>\"\n"
    "Backward: \"b <PWM>\"\n"
    "Stop:\"s\"\n"
    "Distance:\"d <distance>\"\n"
    "PID:\"P <P>\"\n"
    "PWM:\"I <I>\"\n"
    "PWM:\"D <D>\"\n")

    sendMessage(inputString)

