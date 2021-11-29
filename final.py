from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math
import boto3

#Values to access AWS
AWS_ACCESS_KEY=""
AWS_SECRET_ACCESS_KEY=""
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

if DYNAMODB_TABLE in existing_tables:
    table = dynamodb.Table(DYNAMODB_TABLE)
    table.delete()
    print("EXISTS")

time.sleep(5) #waits for table to delete

response = dynamodb_client.create_table(
        AttributeDefinitions=[
            {
                'AttributeName': 'timestamp',
                'AttributeType': 'N',
            }
        ],
        KeySchema=[
            {
                'AttributeName': 'timestamp',
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
theta = 0
referenceTime = 0
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
    #print("left linear: ", Vl)
    #print("right linear: ", Vr)
    v = (Vr + Vl) / 2.0 #average velocity
    w = (Vr - Vl) / length #angular velocity
    # Uses 4th order Runge-Kutta to integrate numerically to find position.
    xNext = xc + dt * v*(2 + math.cos(dt*w / 2))*math.cos(theta + dt * w / 2) / 3
    yNext = yc + dt * v*(2 + math.cos(dt*w / 2))*math.sin(theta + dt * w / 2) / 3
    #xNext = xc + dt * v * math.cos(theta + dt*w)
    #yNext = yc + dt * v * math.sin(theta + dt*w)
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
	global table
	print("Stop After Rotate")
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	global referenceTime
	global currentState
	global previousState
	my_drive.axis0.controller.input_vel = 0
	my_drive.axis1.controller.input_vel = 0
	referenceTime = time.time()
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	table.put_item(
    Item={
        "timestamp": int(time.time()),  #sends current time as timestamp
        "x": str(xc),           #sends sonar reading from mbed through serial to dynamodb
        "y": str(yc),
		"wl": str(wl),
		"wr": str(wr),
		"theta": str(theta)         #sends threshold value from mbed to dynamodb
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
        "timestamp": int(time.time()),  #sends current time as timestamp
        "x": str(xc),           #sends sonar reading from mbed through serial to dynamodb
        "y": str(yc),
		"wl": str(wl),
		"wr": str(wr),
		"theta": str(theta)         #sends threshold value from mbed to dynamodb
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
	global table
	print("Stop After Straight")
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	table.put_item(
    Item={
        "timestamp": int(time.time()),  #sends current time as timestamp
        "x": str(xc),           #sends sonar reading from mbed through serial to dynamodb
        "y": str(yc),
		"wl": str(wl),
		"wr": str(wr),
		"theta": str(theta)         #sends threshold value from mbed to dynamodb
        }
    )
	global referenceTime
	global currentState
	global previousState
	my_drive.axis0.controller.input_vel = 0
	my_drive.axis1.controller.input_vel = 0
	referenceTime = time.time()
	time.sleep(4)
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	time.sleep(4)
	currentState = "Stop After Straight"
	previousState = "Straight"

def rotateNinety():
	global xc
	global yc
	global wl
	global wr
	global theta
	global table
	print("Rotating")
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	table.put_item(
    Item={
        "timestamp": int(time.time()),  #sends current time as timestamp
        "x": str(xc),           #sends sonar reading from mbed through serial to dynamodb
        "y": str(yc),
		"wl": str(wl),
		"wr": str(wr),
		"theta": str(theta)         #sends threshold value from mbed to dynamodb
        }
    )
	global referenceTime
	global currentState
	global previousState
	my_drive.axis1.controller.input_vel = 20
	my_drive.axis0.controller.input_vel = 0
	referenceTime = time.time()
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	table.put_item(
    Item={
        "timestamp": int(time.time()),  #sends current time as timestamp
        "x": str(xc),           #sends sonar reading from mbed through serial to dynamodb
        "y": str(yc),
		"wl": str(wl),
		"wr": str(wr),
		"theta": str(theta)         #sends threshold value from mbed to dynamodb
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
        "timestamp": int(time.time()),  #sends current time as timestamp
        "x": str(xc),           #sends sonar reading from mbed through serial to dynamodb
        "y": str(yc),
		"wl": str(wl),
		"wr": str(wr),
		"theta": str(theta)         #sends threshold value from mbed to dynamodb
        }
    )
	time.sleep(4.25)
	currentState = "Rotate"
	previousState = "Stop After Straight"

def moveStraight():
	global xc
	global yc
	global wl
	global wr
	global theta
	global table
	print("Going Straight")
	global prevPositionComputeTime
	global prevSendTime
	global POSITION_COMPUTE_INTERVAL
	global SEND_INTERVAL
	print("Computing Position")
	computePosition()
	prevPositionComputeTime = time.time()
	print("Printing values")
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	table.put_item(
    Item={
        "timestamp": int(time.time()),  #sends current time as timestamp
        "x": str(xc),           #sends sonar reading from mbed through serial to dynamodb
        "y": str(yc),
		"wl": str(wl),
		"wr": str(wr),
		"theta": str(theta)         #sends threshold value from mbed to dynamodb
        }
    )
	prevSendTime = time.time()
	#computePosition()
	#print("x: ", xc)
	#print("y: ", yc)
	global referenceTime
	global currentState
	global previousState
	my_drive.axis0.controller.input_vel = 100
	my_drive.axis1.controller.input_vel = -100
	referenceTime = time.time()
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	table.put_item(
    Item={
        "timestamp": int(time.time()),  #sends current time as timestamp
        "x": str(xc),           #sends sonar reading from mbed through serial to dynamodb
        "y": str(yc),
		"wl": str(wl),
		"wr": str(wr),
		"theta": str(theta)         #sends threshold value from mbed to dynamodb
        }
    )
	time.sleep(1.5)
	computePosition()
	print("x: ", xc)
	print("y: ", yc)
	print("wl: ", wl)
	print("wr: ", wr)
	print("Theta: ", theta)
	table.put_item(
    Item={
        "timestamp": int(time.time()),  #sends current time as timestamp
        "x": str(xc),           #sends sonar reading from mbed through serial to dynamodb
        "y": str(yc),
		"wl": str(wl),
		"wr": str(wr),
		"theta": str(theta)         #sends threshold value from mbed to dynamodb
        }
    )
	time.sleep(1.5)
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
	table.put_item(
    Item={
        "timestamp": int(time.time()),  #sends current time as timestamp
        "x": str(xc),           #sends sonar reading from mbed through serial to dynamodb
        "y": str(yc),
		"wl": str(wl),
		"wr": str(wr),
		"theta": str(theta)         #sends threshold value from mbed to dynamodb
        }
    )
	time.sleep(1)