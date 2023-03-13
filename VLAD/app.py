import PySimpleGUI as sg
import socket
import calVisuals
import tiltingPlatform
import threading
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, 
NavigationToolbar2Tk)

GUI = 1
# Preview all the available themes
# sg.theme_previewer()

# SOCKET DEFINITIONS
IP2 = "192.168.137.137"
IP = "192.168.137.1"
PORT = 3333
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) 

# UI paramters
fontName = "Helvitica "
regSize = "20"
bigSize = "35" #titles
button_size = (6,3)
sg.theme("DarkGrey13")
sg.set_options(font= fontName+regSize)



# Start Window
def startWindow():
    layoutU = [[sg.Push()],[sg.Text('Vehicular Laboratory Autonomous Device', 
    auto_size_text = True, font=fontName+bigSize, justification ="center", expand_x = True, expand_y = True
    , pad = (10,30))],
    [sg.Button("Run Calibration", expand_x = True, pad = (10,10))]]

    return sg.Window(title="VLAD", layout= layoutU, size = [850,350])



#Calibration Window
def calWindow():
    layoutU = [[sg.Push()],[sg.Text('Calibration', 
    auto_size_text = True, font=fontName+bigSize, justification ="center", expand_x = True, expand_y = True
    , pad = (10,30))],
    [sg.Button("Tilting Platform", expand_x = True, pad = (10,10))],
    [sg.Button("Start", expand_x = True, pad = (10,10))],
    [sg.Button("Go Back", expand_x = True, pad = (10,10))]]

    return sg.Window(title="VLAD", layout= layoutU, size = [850,350])

#Calibration Window
def tiltingPlatformWindow():
    layoutU = [[sg.Push()],[sg.Text('Tilting Platform', 
    auto_size_text = True, font=fontName+bigSize, justification ="center", expand_x = True, expand_y = True
    , pad = (10,30))],
    [sg.Button("Set PID", expand_x = True, pad = (10,10))],
    [sg.Button("Start", expand_x = True, pad = (10,10))],
    [sg.Button("Go Back", expand_x = True, pad = (10,10))]]

    return sg.Window(title="VLAD", layout= layoutU, size = [850,350])




#creating window
if (GUI == 1):
    window = startWindow()
    event, values = window.read()


    while True:
        if event == sg.WIN_CLOSED:
            break

        if event == "Run Calibration":
            window.close()
            window = calWindow()
            
            event, values = window.read()
            while event == "Start":
                plt = calVisuals.calibrate()
                plt.draw()
                event, values = window.read()

            if event == "Tilting Platform":
                window.close()
                window = tiltingPlatformWindow
                tiltingPlatform.getTiltingValues()
                
            if event == "Go Back":
                window.close()
                window = startWindow()
                event, values = window.read()
                msgFromServer = sock.recvfrom(bufferSize)

        



    window.close()