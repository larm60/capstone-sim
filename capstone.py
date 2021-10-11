import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import argparse
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
import imageio
from PIL import Image 
import PIL 


#linear motion

x0 = [0,0] #initial position
x0e = [0,0] #intial position 
xf = [0,0] #final position
xfe = [0,0] #final position with error
v = 1 #velocity (ft/sec)
ve = 1 #velocity with error
a = 0 #acceleration (ft/sec^2)
ae = 0 #acceleration with error
t = 0.1 #time step (sec)
theta = 3 #angle

expectedFinalPositionX = []
expectedFinalPositionY = []
actualFinalPositionX = []
actualFinalPositionY = []
expectedFinalAccelerationX = []
expectedFinalAccelerationY = []
actualFinalAccelerationX = []
actualFinalAccelerationY = []



errorV = int(input("Enter your error for velocity(+/-%): " ))*0.01
errorA = int(input("Enter your error for acceleration(+/-%): " ))*0.01

#shows propagation of error over total time

#constant velcity scenario
print("Constant Velocity Scenario")
totalTime = 10 #in sec
for i in range(totalTime): 
    #expected final position
    xf[0] = x0[0] + v*t + 0.5*a*t**2
    xf[1] = x0[1] + v*t + 0.5*a*t**2
    x0[0] = xf[0]
    x0[1] = xf[1]

    print("Expected final position after time ", i+1, ": ", xf)
    expectedFinalPositionX.append(xf[0])
    expectedFinalPositionY.append(xf[1])

    #final position with error
    xfe[0] = x0e[0] + v*t*(1+errorV) + (1+errorA)*0.5*a*t**2
    xfe[1] = x0e[1] + v*t*(1+errorV) + (1+errorA)*0.5*a*t**2
    x0e[0] = xfe[0]
    x0e[1] = xfe[1]
    print("Actual final position with error after time ", i+1, ": ", xfe)
    actualFinalPositionX.append(xfe[0])
    actualFinalPositionY.append(xfe[1])

#Accelerating scenario (a!=0)
print("Accelerating scenario (a!=0)")

x0 = [0,0] #initial position
x0e = [0,0] #intial position 
xf = [0,0] #final position
xfe = [0,0] #final position with error
v = 1 #velocity (ft/sec)
ve = 1 #velocity with error
a = 0 #acceleration (ft/sec^2)
ae = 0 #acceleration with error
t = 0.1 #time step (sec)
theta = 3 #angle

a = int(input("Acceleration value (ft/sec^2): "))
ae = a
totalTime = 10 #in sec
for i in range(totalTime): 
    v += a*t
    #expected final position
    xf[0] = x0[0] + v*t + 0.5*a*t**2
    xf[1] = x0[1] + v*t + 0.5*a*t**2
    x0[0] = xf[0]
    x0[1] = xf[1]
    print("Expected final position after time ", i+1, ": ", xf)
    expectedFinalAccelerationX.append(xf[0])
    expectedFinalAccelerationY.append(xf[1])

    #final position with error
    ve += (ae*t * (1+errorA)) * (1+errorV)
    xfe[0] = x0e[0] + ve*t + 0.5*a*t**2
    xfe[1] = x0e[1] + ve*t + 0.5*a*t**2
    x0e[0] = xfe[0]
    x0e[1] = xfe[1]
    print("Actual final position with error after time ", i+1, ": ", xfe)
    actualFinalAccelerationX.append(xf[0])
    actualFinalAccelerationY.append(xf[1])

imageCounter = 0

fig, ax = plt.subplots()
x = expectedFinalPositionX
y = expectedFinalPositionY
line, = ax.plot(x, y, color='k')

for n in range(len(expectedFinalPositionX)):
    line.set_data(x[:n], y[:n])
    ax.axis([0, 5, 0, 5])
    fig.canvas.draw()
    fig.savefig('Frame%03d.png' %imageCounter)
    imageCounter = imageCounter + 1

x = actualFinalPositionX
y = actualFinalPositionY
line, = ax.plot(x, y, color='b')

for n in range(len(expectedFinalPositionX)):
    line.set_data(x[:n], y[:n])
    ax.axis([0, 5, 0, 5])
    fig.canvas.draw()
    fig.savefig('Frame%03d.png' %imageCounter)
    imageCounter = imageCounter + 1

x = expectedFinalAccelerationX
y = expectedFinalAccelerationY
line, = ax.plot(x, y, color='g')

for n in range(len(expectedFinalPositionX)):
    line.set_data(x[:n], y[:n])
    ax.axis([0, 5, 0, 5])
    fig.canvas.draw()
    fig.savefig('Frame%03d.png' %imageCounter)
    imageCounter = imageCounter + 1

x = actualFinalAccelerationX
y = actualFinalAccelerationY
line, = ax.plot(x, y, color='r')

for n in range(len(expectedFinalPositionX)):
    line.set_data(x[:n], y[:n])
    ax.axis([0, 50, 0, 50])
    fig.canvas.draw()
    fig.savefig('Frame%03d.png' %imageCounter)
    imageCounter = imageCounter + 1





print("test")