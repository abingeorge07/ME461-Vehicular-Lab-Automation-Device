import socket
import time
# SOCKET DEFINITIONS
IP2 = "192.168.137.137"
IP = "192.168.137.1"
PORT = 3333
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UD
# sock.bind((IP2, PORT))

bufferSize = 1024

while True:

    message = "Hi ESP32, I'm Python, the UDP client. Nice to meet you!"
    message = bytes(message, 'utf-8')
    sock.sendto(message, (IP2, 3333))
    print(message)
    msgFromServer = sock.recvfrom(bufferSize)
    msg = msgFromServer[0].decode()
    ind1 = msg.index("Tx:")
    ind2 = msg.index("Ty:")
    Tx = float(msg[ind1+4:ind2-2])
    print(Tx)
