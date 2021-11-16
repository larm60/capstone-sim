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

my_drive.axis0.controller.input_vel = -20
my_drive.axis1.controller.input_vel = 20


ticksPerRev = 200
radius = 120 #in mm
length = 500 #in mm
toRadPerSec = 1000000 * TWO_PI / ticksPerRev
positionComputeInterval = 100 #in microseconds 

#variables
dt_omega = 0
changeLeftTicks = 0
changeRightTicks = 0
wl = 0
wr = 0

def computeAngVel():
    dt_omega = time.time()*1000 - prevWheelComputeTime

    leftTicks = shadowcount
    rightTicks = shadowcount

    changeLeftTicks = leftTicks - leftTicksPrev
	changeRightTicks = rightTicks - rightTicksPrev

	wl = changeLeftTicks / dt_omega * toRadPerSec
	wr = changeRightTicks / dt_omega * toRadPerSec

    leftTicksPrev = leftTicks
	rightTicksPrev = rightTicks

	prevWheelComputeTime = time.time()*1000

def computePosition():
    if micros() - prevIntegrationTime > positionComputeInterval:
		computeAngularVelocities()
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

		toRPM = 30 / PI
		dist = math.sqrt(xc*xc + yc * yc)

		prevIntegrationTime = time.time()*1000

	#loop to keep calculating positions and distances
	while(1):
		computePosition()
		print(xc, yc, dist)
		temp = True
		#turn attempt (only does it once)
		if temp:
			my_drive.axis0.controller.input_vel = 0
			my_drive.axis1.controller.input_vel = 20
			time.sleep(1) #waits for 1 second

			my_drive.axis0.controller.input_vel = 0
			my_drive.axis1.controller.input_vel = 0
			temp = False
