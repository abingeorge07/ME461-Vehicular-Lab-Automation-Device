import PySimpleGUI as sg
import socket
import calVisuals
import tiltingPlatform
import threading
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, 
NavigationToolbar2Tk)
import threading
from matplotlib import pyplot
from matplotlib.animation import FuncAnimation
from matplotlib import pyplot as plt
import matplotlib.animation as animation


GUI = 1
# sg.theme_previewer()

# SOCKET DEFINITIONS
IP2 = "192.168.137.254"
IP = "192.168.137.1"
PORT = 3333
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UD
# sock.bind((IP2, PORT))
bufferSize = 1024
global seqList


# UI paramters
fontName = "Helvitica "
regSize = "20"
bigSize = "35" #titles
button_size = (6,3)
sg.theme("DarkGrey13")
sg.set_options(font= fontName+regSize)


# Global Variables
calibrated = 0
sequenceExists = 0
global messageGlobal
global calibrationBegin
global seqList
global seqExists
global loop
angleZ = []
angleY = []
pos = []


# Sending message to VLAD
def sendMessage():
    global messageGlobal
    mes = messageGlobal
    if(messageGlobal[0:3] == 'cal'):
        calibrationBegin = 1
    else:
        calibrationBegin = 0
    message = bytes(mes, 'utf-8')
    sock.sendto(message, (IP2, 3333))
    msgFromServer = sock.recvfrom(bufferSize)
    msg = msgFromServer[0].decode()
    print(msg)

    while(msg != "Done\n" or msg != "Done"):
        print(msg)
        mes = "Listening\n"
        message = bytes(mes, 'utf-8')
        sock.sendto(message, (IP2, 3333))
        msgFromServer = sock.recvfrom(bufferSize)
        msg = msgFromServer[0].decode()
        # if(calibrationBegin == 1 and msg != "Done\n"):
        #     ind1 = msg.index("Z")
        #     ind2 = msg.index("Y")
        #     ind3 = msg.index("Pos")
        #     ind4 = msg.index("X")
        #     # print(ind1)
        #     # Will need to change the below for the future
        #     pos.append(float(msg[ind3+5 : ind4-2]))
        #     angleZ.append(float(msg[ind1+3:len(msg)]))
        #     angleY.append(float(msg[ind2+3:ind1-2]))

        calibrationBegin = 0


def moveStation(seqList): 
    global seqExists
    global loop
    # global seqList
    stationNum = 0
    while seqExists:
        global messageGlobal
        messageGlobal = "move2 "+str(seqList[stationNum])+"\n"
        sendMessage()
        stationNum = stationNum + 1
        if(stationNum >= int(numStations)):
            stationNum = 0
            if (loop == 0):
                seqExists = 0


def errorFun(msg):
    layoutU = [[[sg.Text(msg, 
    auto_size_text = True, font=fontName+"30", justification ="center", expand_x = True, expand_y = True
    , pad = (10,30))]], 
    [sg.Button('I apologize', key="apologyAccepted")]]
    return sg.Window(title="VLAD Error", layout= layoutU, size = [350,350])




layoutU = [[sg.Text('Vehicular Laboratory Autonomous Device', 
    auto_size_text = True, font=fontName+bigSize, justification ="center", expand_x = True, expand_y = True
    , pad = (10,30))],
    [sg.Button('Exit', key="-EXIT-")],
    [sg.Text('Number of Stations: ', size =(18, 1)), sg.InputText(size=(8,1), do_not_clear=False), sg.Button('Calibrate', key='cal')], 
    [sg.Checkbox('Keep looping', default=False, size=(12,3),key="-LOOP-")],
    [sg.Text('Sequence: ', size =(18, 1)), sg.InputText(size=(8,1), do_not_clear=False), sg.Button('Start sequence', key='seq')],
    [sg.Button('Refresh', key="-STOPLOOPING-")],             
    [sg.Canvas(size=(500,500), key='canvas')]]


window = sg.Window(title="VLAD", layout= layoutU, size = [850,450])

event, values = window.read()

numStations = 0

loop = 0

while True:

    if event == sg.WIN_CLOSED:
        break

    if event == 'cal':
        # values[0])>=11 or int(values[0])<=0 or
        print(type(values[0]))
        if(not values[0].isdigit()):
            print("ERROR\n")
            window2 = errorFun("Bad Input")
            event, values = window2.read()
            if(event == "apologyAccepted"):
                window2.close()
        elif(int(values[0]) >120 or int(values[0])<0):
            print("ERROR\n")
            window2 = errorFun("Bad Input")
            event, values = window2.read()
            if(event == "apologyAccepted"):
                window2.close()
        else:
            print("Cal"+ str(values[0]))
            numStations = int(values[0])
            calibrationBegin = 1
            messageGlobal = 'cal '+str(values[0])
            UDP_thread = threading.Thread(target=sendMessage, args=())
            UDP_thread.start()

    if event == "seq":
        seq = values[1]
        seqList = seq.split(",")
        error = 0
        for i in range(0,len(seqList)):
            temp = seqList[i]

            if(not temp.isdigit()):
                print("ERROR\n")
                # this is not working for some reason
                window2 = errorFun("Bad Input")
                event, values = window2.read()
                if(event == "apologyAccepted"):
                    window2.close()
                error = 1
                break

        if (error == 0):
            seqExists = 1
            if(values['-LOOP-'] == True):
                loop = 1
            UDP_thread = threading.Thread(target=moveStation, args=())
            UDP_thread.start()
            # moveStation(seqList)
            print(values[1])

    if values["-LOOP-"] == True:
        print("loop")
        loop = 1
    else:
        loop = 0

    if event == "-EXIT-":
        break

    # if event == "-STOPLOOPING-":
    #     loop = 0


    event, values = window.read()


window.close()
