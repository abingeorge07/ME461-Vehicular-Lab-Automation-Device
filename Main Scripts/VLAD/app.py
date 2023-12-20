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
# Preview all the available themes
# sg.theme_previewer()

# SOCKET DEFINITIONS
IP2 = "192.168.137.143"
IP = "192.168.137.1"
PORT = 3333
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UD
# sock.bind((IP2, PORT))
bufferSize = 1024


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
angleZ = []
angleY = []
pos = []


fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
plt.grid(False)
plt.clf()



# Sending message to VLAD
def sendMessage():
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

    while(msg != "Done\n"):
        print(msg)
        mes = "Listening\n"
        message = bytes(mes, 'utf-8')
        sock.sendto(message, (IP2, 3333))
        msgFromServer = sock.recvfrom(bufferSize)
        msg = msgFromServer[0].decode()
        if(calibrationBegin == 1 and msg != "Done\n"):
            # ind1 = msg.index("X")
            ind2 = msg.index("Y")
            ind3 = msg.index("Pos")
            # print(ind1)
            # Will need to change the below for the future
            pos.append(float(msg[ind3+5 : ind3+10]))
        print(msg)

    # fig = plt.figure()
    # plt.clf()    
    # plt.plot(pos, color = 'green', linewidth=5)  
    # plt.draw()  
    calibrationBegin = 0
    # print(pos)
    # return plt.gcf()

# Start Window
def startWindow():
    layoutU = [[sg.Push()],[sg.Text('Vehicular Laboratory Autonomous Device', 
    auto_size_text = True, font=fontName+bigSize, justification ="center", expand_x = True, expand_y = True
    , pad = (10,30))],
    [sg.Button("Run Calibration", expand_x = True, pad = (10,10), key='go2cal')]]

    return sg.Window(title="VLAD", layout= layoutU, size = [850,450])



# Calibration Window
def calWindow():
    layoutU = [[sg.Push()],[sg.Text('Calibration', 
    auto_size_text = True, font=fontName+bigSize, justification ="center", expand_x = True, expand_y = True
    , pad = (10,30))],
    [sg.Button("Tilting Platform", expand_x = True, pad = (10,10))],
    [sg.Button("Start", expand_x = True, pad = (10,10))],
    [sg.Button("Go Back", expand_x = True, pad = (10,10))]]

    return sg.Window(title="VLAD", layout= layoutU, size = [850,450])

# Calibration Window
def tiltingPlatformWindow():
    layoutU = [[sg.Push()],[sg.Text('Tilting Platform', 
    auto_size_text = True, font=fontName+bigSize, justification ="center", expand_x = True, expand_y = True
    , pad = (10,30))],
    [sg.Button("Set PID", expand_x = True, pad = (10,10))],
    [sg.Button("Start", expand_x = True, pad = (10,10))],
    [sg.Button("Go Back", expand_x = True, pad = (10,10))]]
    return sg.Window(title="VLAD", layout= layoutU, size = [850,450])

# After hitting the calibration button
def startCal():
    layoutU = [[sg.Push()],[sg.Text('Calibration Starting', 
    auto_size_text = True, font=fontName+bigSize, justification ="center", expand_x = True, expand_y = True
    , pad = (10,30))],
    [sg.Canvas(size=(500,200), key='canvas')],
    [sg.Button("Go Back", expand_x = True, pad = (10,10), key='go2cal')]]
    return sg.Window(title="VLAD", layout= layoutU, size = [850,450])


# unused function, not in the final implementation
def draw_figure(canvas, figure, loc=(0, 0)):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg




# Creating window
if (GUI == 1):
    window = startWindow()
    event, values = window.read()


    while True:
        if event == sg.WIN_CLOSED:
            break

        if event == "go2cal":
            window.close()
            window = calWindow()
            
            event, values = window.read()
            if event == "Start":
                # plt = calVisuals.calibrate()
                # plt.draw()
                # Start Calibration
                window.close()
                window = startCal()
                numStations = 3
                calibrationBegin = 1
                messageGlobal = 'cal '+str(numStations)
                sendMessage()
                # UDP_thread = threading.Thread(target=sendMessage, args=())
                # UDP_thread.start()
                plt.clf()
                plt.axis('on')  
                plt.plot(pos, color = 'green', linewidth=5)  
                # plt.draw() 
                plt.show()
                # window.Refresh()
                # event, values = window.read()

            if event == "Tilting Platform":
                window.close()
                window = tiltingPlatformWindow
                tiltingPlatform.getTiltingValues()
                
            if event == "Go Back":
                window.close()
                window = startWindow()
                event, values = window.read()
                # msgFromServer = sock.recvfrom(bufferSize)

    window.close()