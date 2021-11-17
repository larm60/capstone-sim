#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

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

ninetyDegreeTurn = 800
initialShadowCountBoot = my_drive.axis1.encoder.shadow_count # measure initial pos

my_drive.axis0.controller.input_vel = 50
my_drive.axis1.controller.input_vel = -50
currentShadowCount = my_drive.axis1.encoder.shadow_count
print(abs(currentShadowCount - initialShadowCountBoot))

if (abs(currentShadowCount - initialShadowCountBoot) > 800):
	my_drive.axis0.controller.input_vel = 0
	my_drive.axis1.controller.input_vel = 0
	# currentShadowCount = my_drive.axis1.encoder.shadow_count
	while(1):
		initialShadowCount = my_drive.axis1.encoder.shadow_count # measure initial pos
		print(my_drive.axis0.encoder.shadow_count)
		print(my_drive.axis1.encoder.shadow_count)
		my_drive.axis1.controller.input_vel = 20
		currentShadowCount = my_drive.axis1.encoder.shadow_count
		if (abs(currentShadowCount - initialShadowCount) > ninetyDegreeTurn):
			my_drive.axis1.controller.input_vel = 0

#--------------------------------------------------------------------------
# Rectangle

# Initialize
currentShadowCount = my_drive.axis1.encoder.shadow_count # shadow count boot-up

# Move straight
my_drive.axis0.controller.input_vel = 50
my_drive.axis1.controller.input_vel = -50

# Stop moving
# First, see if shadow count has reached a certain limit
updateShadowCount()
if (abs(currentShadowCount - newShadowCount) > 800): # if it has reached a threshold
	my_drive.axis0.controller.input_vel = 0
	my_drive.axis1.controller.input_vel = 0
currentShadowCount = newShadowCount

# Rotate 90-degrees
# First, see if the robot is ready to rotate 90 degrees
# This means that robot has completely stopped moving (no change in shadow count)
updateShadowCount()
if (abs(currentShadowCount - newShadowCount) < 5): # Robot has stopped moving
	my_drive.axis1.controller.input_vel = 20 # move only 1 wheel
currentShadowCount = newShadowCount

# now, stop the wheel after a full 90 degree rotation
newShadowCount()
if (abs(currentShadowCount - newShadowCount) > 800): # if it has reached a threshold
	my_drive.axis0.controller.input_vel = 0
	my_drive.axis1.controller.input_vel = 0
updateShadowCount()

# Stop moving
# First, see if shadow count has reached a certain limit
updateShadowCount()
if (abs(currentShadowCount - newShadowCount) > 800): # if it has reached a threshold
	my_drive.axis0.controller.input_vel = 0
	my_drive.axis1.controller.input_vel = 0
currentShadowCount = newShadowCount
#---------------------------------------------------------------------------
	


def updateShadow():
	newShadowCount = my_drive.axis1.encoder.shadow_count # measure current pos
	