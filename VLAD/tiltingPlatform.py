import socket
import time
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import math
from datetime import datetime
from matplotlib import pyplot
from matplotlib.animation import FuncAnimation


def getTiltingValues():
    # SOCKET DEFINITIONS
    IP2 = "192.168.137.23"
    IP = "192.168.137.1"
    PORT = 3333
    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UD
    # sock.bind((IP2, PORT))

    bufferSize = 1024

    angle = []
    time = []
    initVal = 0


    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    plt.grid(False)
    plt.clf()

    for i in range(1,1000):
        message = "Data"
        message = bytes(message, 'utf-8')
        sock.sendto(message, (IP2, 3333))
        print(message)
        msgFromServer = sock.recvfrom(bufferSize)
        msg = msgFromServer[0].decode()
        ind1 = msg.index("Ty:")
        ind2 = msg.index("Tz:")
        Ty = float(msg[ind1+4:ind2-2])
        angle.append(round(Ty))
        time.append(initVal)
        initVal = initVal + 10
        print(Ty)
        # Ploting graph
        plt.clf()
        plt.axis('on')
        plt.plot(time, angle, color = 'green', linewidth=5)
        plt.draw()
        plt.pause(0.00002)
    plt.draw()       
    return plt


getTiltingValues()