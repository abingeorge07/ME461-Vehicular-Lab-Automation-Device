import socket
import time
# SOCKET DEFINITIONS
IP2 = "192.168.137.168"
IP = "192.168.137.1"
PORT = 3333
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) # UD
# sock.bind((IP2, PORT))
bufferSize = 1024

calibrated = 0
sequenceExists = 0

def sendMessage(mes):
    message = bytes(mes, 'utf-8')
    sock.sendto(message, (IP2, 3333))
    msgFromServer = sock.recvfrom(bufferSize)
    msg = msgFromServer[0].decode()
    print(msg)
    if(msg == "OK"):
        print("So far so good\n")
    while(msg != "Done\n"):
        mes = "Listening\n"
        message = bytes(mes, 'utf-8')
        sock.sendto(message, (IP2, 3333))
        msgFromServer = sock.recvfrom(bufferSize)
        msg = msgFromServer[0].decode()
        print(msg)

    



while True:

    inputString = input("Enter C for calibration\n"
                        "Enter S for sequence\n")

    if inputString == 'C' or inputString == 'c':
        numStations = input("N: ")
        sendMessage("cal "+str(numStations))
        if(int(numStations) > 0):
            calibrated = 1

    elif inputString == 'S' or inputString == 's':
        # if calibrated:
        numStations = 3
        print("There are "+ str(numStations) + " different stations\n")
        seq = input("What sequence would you like? (eg if 5 stations: 3,1,2,4,5)")
        timeout = input("How long of a timeout do you want between each station in seconds?\n")
        sendMessage("time: "+str(timeout))
        # sendMessage("Ord: "+seq)
        seqList = seq.split(",")
        print(seqList)
        sequenceExists = 1
        stationNum = 0
        break
        # else: 
        print("Please calibrate first\n")

while sequenceExists:
    sendMessage("move2 "+str(seqList[stationNum])+"\n")
    stationNum = stationNum + 1
    if(stationNum >= int(numStations)):
        stationNum = 0
    # time.sleep(int(timeout))


    # message = "Hi ESP32, I'm Python, the UDP client. Nice to meet you!"
    # message = bytes(message, 'utf-8')
    # sock.sendto(message, (IP2, 3333))
    # print(message)
    # msgFromServer = sock.recvfrom(bufferSize)
    # msg = msgFromServer[0].decode()
    # ind1 = msg.index("Tx:")
    # ind2 = msg.index("Ty:")
    # Tx = float(msg[ind1+4:ind2-2])
    # print(Tx)
