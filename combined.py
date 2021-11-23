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

ticksPerRev = 1286
radius = 120 #in mm
length = 500 #in mm
toRadPerSec = 1000000.0 * (math.pi * 2) / ticksPerRev
positionComputeInterval = ticksPerRev * 1000000.0 #in microseconds 

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
prevPositionComputeTime = 0
prevSendTime = 0
POSITION_COMPUTE_INTERVAL = 50 # milliseconds
SEND_INTERVAL = 100 # milliseconds

def computeAngVel():
    global prevWheelComputeTime
    global leftTicksPrev
    global rightTicksPrev
    global wl
    global wr

    dt_omega = time.time()*1000.0 - prevWheelComputeTime

    leftTicks = -1 * my_drive.axis1.encoder.shadow_count
    rightTicks = my_drive.axis0.encoder.shadow_count

    changeLeftTicks = leftTicks - leftTicksPrev
    changeRightTicks = rightTicks - rightTicksPrev

    #wl = changeLeftTicks / dt_omega * toRadPerSec
    #wr = changeRightTicks / dt_omega * toRadPerSec
    wl = (changeLeftTicks * (2 * math.pi) / dt_omega)
    wr = (changeRightTicks * (2 * math.pi) / dt_omega)
	



    leftTicksPrev = leftTicks
    rightTicksPrev = rightTicks

    prevWheelComputeTime = time.time()*1000.0

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
    print("Subtraction: ", time.time()*1000.0 - prevIntegrationTime)
    print("positionComputeInterval: ", positionComputeInterval)
    #if time.time()*1000.0 - prevIntegrationTime > positionComputeInterval: # get rid of conditional since we sample in the while loop at the bottom
    computeAngVel()
    print("Made it past if")
    # Time elapsed after the previous position has been integrated.
    # change in time is defined as previous - current to prevent round off error.
    dt_integration = time.time()*1000000.0 - prevIntegrationTime

    dt = dt_integration / 1000000.0; # convert to seconds

    # Dead reckoning equations

    Vl = wl * radius #linear velocity left wheel
    Vr = wr * radius #linear velocity right wheel
    print("left linear: ", Vl)
    print("right linear: ", Vr)
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

    prevIntegrationTime = time.time()*1000.0

def stopAfterRotate():
	global xc
	global yc
	print("Stop After Rotate")
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	global referenceTime
	global currentState
	global previousState
	my_drive.axis0.controller.input_vel = 0
	my_drive.axis1.controller.input_vel = 0
	referenceTime = time.time()
	time.sleep(8)
	currentState = "Stop After Rotate"
	previousState = "Rotate"

def stopAfterStraight():
	global xc
	global yc
	print("Stop After Straight")
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	global referenceTime
	global currentState
	global previousState
	my_drive.axis0.controller.input_vel = 0
	my_drive.axis1.controller.input_vel = 0
	referenceTime = time.time()
	time.sleep(8)
	currentState = "Stop After Straight"
	previousState = "Straight"

def rotateNinety():
	global xc
	global yc
	print("Rotating")
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	global referenceTime
	global currentState
	global previousState
	my_drive.axis1.controller.input_vel = 20
	my_drive.axis0.controller.input_vel = 0
	referenceTime = time.time()
	time.sleep(8.5)
	currentState = "Rotate"
	previousState = "Stop After Straight"

def moveStraight():
	global xc
	global yc
	global wl
	global wr
	print("Going Straight")
	global prevPositionComputeTime
	global prevSendTime
	global POSITION_COMPUTE_INTERVAL
	global SEND_INTERVAL
	if (time.time()*1000.0 - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL):
		print("Computing Position")
		computePosition()
		prevPositionComputeTime = time.time()*1000
	if (time.time()*1000.0 - prevSendTime > SEND_INTERVAL):
		print("Printing values")
		print("x: ", xc)
		print("y: ", yc)
		print("wl: ", wl)
		print("wr: ", wr)
		prevSendTime = time.time()*1000.0
	#computePosition()
	#print("x: ", xc)
	#print("y: ", yc)
	global referenceTime
	global currentState
	global previousState
	my_drive.axis0.controller.input_vel = 100
	my_drive.axis1.controller.input_vel = -100
	referenceTime = time.time()
	time.sleep(3)
	currentState = "Straight"
	previousState = "Stop After Rotate"


#Has robot move in straight 
my_drive.axis0.controller.input_vel = 100
my_drive.axis1.controller.input_vel = -100
value = input("Enter state: ")
initialTime = time.time()
currentState = "Straight"
previousState = "Stop After Rotate"


while(value == "r"):
	# Rectangle or Path: Stop -> Straight -> Stop -> Rotate -> Stop -> Straight -> Stop -> Rotate -> Stop -> Straight -> Stop -> Rotate -> Stop -> Straight
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	time.sleep(1)

	# Move Straight
	if (currentState == "Stop After Rotate" and previousState == "Rotate"):
		moveStraight()
	# Stop after straight
	if (currentState == "Straight" and previousState == "Stop After Rotate"):
		stopAfterStraight()
	# Stop after rotate
	if (currentState == "Rotate" and previousState == "Stop After Straight"):
		stopAfterRotate()
	# Rotate 90
	if (currentState == "Stop After Straight" and previousState == "Straight"):
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

while (value == "s"):
	my_drive.axis0.controller.input_vel = 100
	my_drive.axis1.controller.input_vel = -100
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Shadow Left: ", my_drive.axis1.encoder.shadow_count)
	print("Shadow Right: ", my_drive.axis0.encoder.shadow_count)
	time.sleep(1)