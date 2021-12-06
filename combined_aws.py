from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math
import boto3

#Values to access AWS
AWS_ACCESS_KEY="[insert credentials here]" 
AWS_SECRET_ACCESS_KEY="[insert credentials here]"
AWS_REGION="us-east-1"
DYNAMODB_TABLE='deadreckoning'

# accesses dynamodb table on aws   
dynamodb = boto3.resource('dynamodb', aws_access_key_id=AWS_ACCESS_KEY,
                          aws_secret_access_key=AWS_SECRET_ACCESS_KEY,
                          region_name =AWS_REGION)

dynamodb_client = boto3.client('dynamodb', aws_access_key_id=AWS_ACCESS_KEY,
                          aws_secret_access_key=AWS_SECRET_ACCESS_KEY,
                          region_name =AWS_REGION)

existing_tables = dynamodb_client.list_tables()['TableNames']

time.sleep(5) #waits for table to delete

response = dynamodb_client.create_table(
        AttributeDefinitions=[
            {
                'AttributeName': 'timestamp (s)',
                'AttributeType': 'N',
            }
        ],
        KeySchema=[
            {
                'AttributeName': 'timestamp (s)',
                'KeyType': 'HASH',
            }
        ],
        ProvisionedThroughput={
            'ReadCapacityUnits': 1,
            'WriteCapacityUnits': 1
        },
        TableName=DYNAMODB_TABLE
    )

time.sleep(10) #waits for table to create

#Sends deadreckoning position to dynamodb on AWS
table = dynamodb.Table(DYNAMODB_TABLE)

#Creates new table for each run
if DYNAMODB_TABLE in existing_tables:
    table = dynamodb.Table(DYNAMODB_TABLE)
    table.delete()
    print("EXISTS")

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
toRadPerSec = 1000000.0 * (math.pi * 2) / ticksPerRev
positionComputeInterval = 1 #in seconds 

#Global variables
wl = 0
wr = 0
prevWheelComputeTime = 0
leftTicksPrev = 0
rightTicksPrev = 0
prevIntegrationTime = 0
radius = 120 / 1000.0 # m
length = 500 / 1000.0 # m
xc = 0
yc = 0
xRef = 0
yRef = 0
theta = 0
initial = 0
currentState = "Idle"
prevPositionComputeTime = 0
prevSendTime = 0
POSITION_COMPUTE_INTERVAL = 1 # seconds
SEND_INTERVAL = 1 #seconds

def computeAngVel():
    global prevWheelComputeTime
    global leftTicksPrev
    global rightTicksPrev
    global wl
    global wr

    dt_omega = time.time() - prevWheelComputeTime

    leftTicks = -1 * my_drive.axis1.encoder.shadow_count
    rightTicks = my_drive.axis0.encoder.shadow_count

    changeLeftTicks = leftTicks - leftTicksPrev
    changeRightTicks = rightTicks - rightTicksPrev

    #wl = changeLeftTicks / dt_omega * toRadPerSec
    #wr = changeRightTicks / dt_omega * toRadPerSec
    wl = (changeLeftTicks * (2 * math.pi) / dt_omega) / ticksPerRev
    wr = (changeRightTicks * (2 * math.pi) / dt_omega) / ticksPerRev
	
    leftTicksPrev = leftTicks
    rightTicksPrev = rightTicks

    prevWheelComputeTime = time.time()

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
    #if time.time()*1000.0 - prevIntegrationTime > positionComputeInterval: # get rid of conditional since we sample in the while loop at the bottom
    computeAngVel()
    # Time elapsed after the previous position has been integrated.
    # change in time is defined as previous - current to prevent round off error.
    dt_integration = time.time() - prevIntegrationTime

    dt = dt_integration # convert to seconds

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

    prevIntegrationTime = time.time()

def stopAfterRotate():
	global xc
	global yc
	global wl
	global wr
	global theta
	print("Stop After Rotate")
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	table.put_item(
    Item={
        "timestamp (s)": time.time() - initial,  #sends current time as timestamp
        "x (m)": str(xc),           
        "y (m)": str(yc),
		"wl (radians/sec)": str(wl),
		"wr (radians/sec)": str(wr),
		"theta (radians)": str(theta)
        }
    )
	global currentState
	global previousState
	my_drive.axis0.controller.input_vel = 0
	my_drive.axis1.controller.input_vel = 0
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	table.put_item(
    Item={
        "timestamp (s)": time.time() - initial,  #sends current time as timestamp
        "x (m)": str(xc),           
        "y (m)": str(yc),
		"wl (radians/sec)": str(wl),
		"wr (radians/sec)": str(wr),
		"theta (radians)": str(theta)
        }
    )
	time.sleep(4)
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	table.put_item(
    Item={
        "timestamp (s)": time.time() - initial,  #sends current time as timestamp
        "x (m)": str(xc),           
        "y (m)": str(yc),
		"wl (radians/sec)": str(wl),
		"wr (radians/sec)": str(wr),
		"theta (radians)": str(theta)
        }
    )
	time.sleep(4)
	currentState = "Stop After Rotate"
	previousState = "Rotate"

def stopAfterStraight():
	global xc
	global yc
	global wl
	global wr
	global theta
	print("Stop After Straight")
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	table.put_item(
    Item={
        "timestamp (s)": time.time() - initial,  #sends current time as timestamp
        "x (m)": str(xc),           
        "y (m)": str(yc),
		"wl (radians/sec)": str(wl),
		"wr (radians/sec)": str(wr),
		"theta (radians)": str(theta)
        }
    )
	global currentState
	global previousState
	my_drive.axis0.controller.input_vel = 0
	my_drive.axis1.controller.input_vel = 0
	time.sleep(4)
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	table.put_item(
    Item={
        "timestamp (s)": time.time() - initial,  #sends current time as timestamp
        "x (m)": str(xc),           
        "y (m)": str(yc),
		"wl (radians/sec)": str(wl),
		"wr (radians/sec)": str(wr),
		"theta (radians)": str(theta)
        }
    )
	time.sleep(4)
	currentState = "Stop After Straight"
	previousState = "Straight"

def rotateNinety():
	global xc
	global yc
	global wl
	global wr
	global theta
	global currentState
	global previousState
	print("Rotating")
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	table.put_item(
    Item={
        "timestamp (s)": time.time() - initial,  #sends current time as timestamp
        "x (m)": str(xc),           
        "y (m)": str(yc),
		"wl (radians/sec)": str(wl),
		"wr (radians/sec)": str(wr),
		"theta (radians)": str(theta)
        }
    )
	my_drive.axis1.controller.input_vel = 20
	my_drive.axis0.controller.input_vel = 0
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	table.put_item(
    Item={
        "timestamp (s)": time.time() - initial,  #sends current time as timestamp
        "x (m)": str(xc),           
        "y (m)": str(yc),
		"wl (radians/sec)": str(wl),
		"wr (radians/sec)": str(wr),
		"theta (radians)": str(theta)
        }
    )
	time.sleep(4.25)
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	table.put_item(
    Item={
        "timestamp (s)": time.time() - initial,  #sends current time as timestamp
        "x (m)": str(xc),           
        "y (m)": str(yc),
		"wl (radians/sec)": str(wl),
		"wr (radians/sec)": str(wr),
		"theta (radians)": str(theta)
        }
    )
	time.sleep(4.25)
	currentState = "Rotate"
	previousState = "Stop After Straight"

def moveStraight():
	global xc
	global xRef
	global yc
	global yRef
	global wl
	global wr
	global theta
	global currentState
	global previousState
	reachedThreshold = False
	print("Going Straight")
	print("Computing Position")
	my_drive.axis0.controller.input_vel = 100
	my_drive.axis1.controller.input_vel = -100
	computePosition()
	xRef = xc
	yRef = yc
	while(not reachedThreshold):
		computePosition()
		print("xRef: ", xRef)
		print("xc: ", xc)
		print("yRef: ", yRef)
		print("yc: ", yc)
		print("Difference in X: ", abs(xRef - xc))
		print("Difference in Y: ", abs(yRef - yc))
		if (abs(xRef - xc) > 1.5 or abs(yRef - yc) > 1.5):
			reachedThreshold = True
			print("-------------------------------------")
			print("x: ", xc)
			print("y: ", yc)
			print("wl: ", wl)
			print("wr: ", wr)
			print("Theta: ", theta)
			print("-------------------------------------")
		else:
			print("x: ", xc)
			print("y: ", yc)
			print("wl: ", wl)
			print("wr: ", wr)
			print("Theta: ", theta)
			print("-------------------------------------")
			time.sleep(1)
	table.put_item(
    Item={
        "timestamp (s)": time.time() - initial,  #sends current time as timestamp
        "x (m)": str(xc),           
        "y (m)": str(yc),
		"wl (radians/sec)": str(wl),
		"wr (radians/sec)": str(wr),
		"theta (radians)": str(theta)
        }
    )
	print("Reached Threshold")
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
	time.sleep(1)
	print("STATES")
	print(currentState)
	print(previousState)
	initial = time.time()

	# Move Straight
	if (currentState == "Stop After Rotate" and previousState == "Rotate"):
		moveStraight()
		print("DONE MOVING STRAIGHT")
	# Stop after straight
	if (currentState == "Straight" and previousState == "Stop After Rotate"):
		stopAfterStraight()
		print("DONE STOPPING AFTER STRAIGHT")
	# Stop after rotate
	if (currentState == "Rotate" and previousState == "Stop After Straight"):
		stopAfterRotate()
		print("DONE STOPPING AFTER ROTATE")
	# Rotate 90
	if (currentState == "Stop After Straight" and previousState == "Straight"):
		rotateNinety()
		print("DONE ROTATING")

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