from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
my_drive = odrive.find_any()

# -----------------------------------------------------------------------
# Calibrate motor and wait for it to finish
print("starting calibration...")
my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while my_drive.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# ------------------------------------------------------------------------

# Calibrate motor and wait for it to finish
print("starting calibration...")
my_drive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while my_drive.axis1.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
# ------------------------------------------------------------------------

ticksPerRev = 200
radius = 120 #in mm
length = 500 #in mm
toRadPerSec = 1000000 * (math.pi * 2) / ticksPerRev
positionComputeInterval = 1000 #in microseconds 

#Global variables
wl = 0
wr = 0
prevWheelComputeTime = 0
leftTicksPrev = 0
rightTicksPrev = 0
prevIntegrationTime = 0
radius = 120 # mm
length = 500 # mm
xc = 0
yc = 0
theta = 0
referenceTime = 0
currentState = "Idle"

def computeAngVel():
    global prevWheelComputeTime
    global leftTicksPrev
    global rightTicksPrev
    global wl
    global wr

    dt_omega = time.time()*1000 - prevWheelComputeTime

    leftTicks = my_drive.axis1.encoder.shadow_count
    rightTicks = my_drive.axis0.encoder.shadow_count

    changeLeftTicks = leftTicks - leftTicksPrev
    changeRightTicks = rightTicks - rightTicksPrev

    wl = changeLeftTicks / dt_omega * toRadPerSec
    wr = changeRightTicks / dt_omega * toRadPerSec

    leftTicksPrev = leftTicks
    rightTicksPrev = rightTicks

    prevWheelComputeTime = time.time()*1000

def computePosition():
    global w1
    global wr
    global positionComputeInterval
    global prevIntegrationTime
    global radius
    global length
    global xc
    global yc
    global theta
    if time.time()*1000 - prevIntegrationTime > positionComputeInterval:
        computeAngVel()
        # Time elapsed after the previous position has been integrated.
        # change in time is defined as previous - current to prevent round off error.
        dt_integration = time.time()*1000 - prevIntegrationTime

        dt = dt_integration / 1000000.0; # convert to seconds

        # Dead reckoning equations

        Vl = wl * radius #linear velocity left wheel
        Vr = wr * radius #linear velocity right wheel
        v = (Vr + Vl) / 2.0 #average velocity
        w = (Vr - Vl) / length #angular velocity
        # Uses 4th order Runge-Kutta to integrate numerically to find position.
        xNext = xc + dt * v*(2 + math.cos(dt*w / 2))*math.cos(theta + dt * w / 2) / 3
        yNext = yc + dt * v*(2 + math.cos(dt*w / 2))*math.sin(theta + dt * w / 2) / 3
        thetaNext = theta + dt * w

        xc = xNext
        yc = yNext
        theta = thetaNext

        toRPM = 30 / math.pi
        dist = math.sqrt(xc*xc + yc * yc)

    prevIntegrationTime = time.time()*1000

def stopWheels():
	global referenceTime
	global currentState
	global previousState
	my_drive.axis0.controller.input_vel = 0
	my_drive.axis1.controller.input_vel = 0
	referenceTime = time.time()
	time.sleep(5)
	currentState = "Stop"
	if (currentState == "Rotate"):
		previousState = "Straight"
	else:
		previousState = "Rotate"

def rotateNinety():
	global referenceTime
	global currentState
	global previousState
	my_drive.axis1.controller.input_vel = 20
	referenceTime = time.time()
	time.sleep(5)
	currentState = "Rotate"
	previousState = "Stop"

def moveStraight():
	global referenceTime
	global currentState
	global previousState
	my_drive.axis0.controller.input_vel = 50
	my_drive.axis1.controller.input_vel = -50
	referenceTime = time.time()
	time.sleep(5)
	currentState = "Straight"
	previousState = "Rotate"


#Has robot move in straight 
my_drive.axis0.controller.input_vel = 50
my_drive.axis1.controller.input_vel = -50
value = input("Press r for rectangle: ")
initialTime = time.time()
currentState = "Straight"
previousState = "Stop"


while(value == "r"):
	# Rectangle or Path: Stop -> Straight -> Stop -> Rotate -> Stop -> Straight -> Stop -> Rotate -> Stop -> Straight -> Stop -> Rotate -> Stop -> Straight
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	time.sleep(1)

	# Move Straight
	if (currentState == "Stop" and previousState == "Rotate"):
		moveStraight()
	# Stopping
	if ((currentState == "Rotate" or currentState == "Straight") and previousState == "Stop"):
		stopWheels()
	# Rotate 90
	if (currentState == "Stop" and previousState == "Straight"):
		rotateNinety()


	#if (time.time()-initialTime > 10): # checks for if 10 seconds have passed (for purposes of testing only to see if we can have robot move and stop while getting position)
		#stopWheels()

	#time.sleep(5)

	#if (time.time()-referenceTime > 5): # checks if wheel has finally stopped moving)
		#rotateNinety()
		#stopWheels()

	#time.sleep(5)

	#if (time.time() - referenceTime > 5):
		#moveStraight()