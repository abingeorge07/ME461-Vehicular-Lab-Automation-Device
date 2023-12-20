from matplotlib import pyplot as plt
import matplotlib.animation as animation
import math
from datetime import datetime
from matplotlib import pyplot
from matplotlib.animation import FuncAnimation
from random import randrange


def calibrate():

    dist = []
    angle = []

    dist.append(0)
    angle.append(0)

    # creating fake values
    for i in range(1,5):
        for j in range(1,10):
            dist.append(dist[len(dist)-1] + 1)
            angle.append(angle[len(angle)-1])

        for j in range(1,19):
            dist.append(dist[len(dist)-1] + 1)
            angle.append(angle[len(angle)-1] + 5)

    currentDist = 0
    x_pt = 0
    y_pt = 0
    x = []
    y = []

    x.append(x_pt)
    y.append(y_pt)

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    plt.grid(False)
    plt.clf()
    wait = '1'

    for i in range(len(dist)):

        # plt.draw()
        # if wait == '1':
        #     wait = input("Enter")
        x_pt = x_pt + (dist[i] - currentDist)*math.cos((90+angle[i])*3.14152/180)
        y_pt = y_pt + (dist[i] - currentDist)*math.sin((90+angle[i])*3.14152/180)

        currentDist = dist[i]
        x.append(x_pt)
        y.append(y_pt)
        
        # Ploting graph
        plt.clf()
        plt.axis('off')
        plt.plot(x, y, color = 'green', linewidth=5)
        plt.plot(x[len(x)-1], y[len(y)-1], color = 'red',  marker ='x', markersize=20)
        plt.draw()
        plt.pause(0.2)
    # plt.draw()       
    # return plt


# calibrate()