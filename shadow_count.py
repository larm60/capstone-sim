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
	