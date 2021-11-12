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
    dt_omega = micros() - prevWheelComputeTime

    leftTicks = shadowcount
    rightTicks = shadowcount

    changeLeftTicks = leftTicks - leftTicksPrev
	changeRightTicks = rightTicks - rightTicksPrev

	wl = changeLeftTicks / dt_omega * toRadPerSec
	wr = changeRightTicks / dt_omega * toRadPerSec

    leftTicksPrev = leftTicks
	rightTicksPrev = rightTicks

	prevWheelComputeTime = micros()

def computePosition():
    if micros() - prevIntegrationTime > positionComputeInterval:
		computeAngularVelocities()
		# Time elapsed after the previous position has been integrated.
		# change in time is defined as previous - current to prevent round off error.
		dt_integration = micros() - prevIntegrationTime

		dt = dt_integration / 1000000.0; # convert to seconds

		# Dead reckoning equations

		Vl = wl * radius #linear velocity left wheel
		Vr = wr * radius #linear velocity right wheel
		v = (Vr + Vl) / 2.0 #average velocity
		w = (Vr - Vl) / length #angular velocity
		# Uses 4th order Runge-Kutta to integrate numerically to find position.
		xNext = xc + dt * v*(2 + cos(dt*w / 2))*cos(theta + dt * w / 2) / 3
		yNext = yc + dt * v*(2 + cos(dt*w / 2))*sin(theta + dt * w / 2) / 3
		thetaNext = theta + dt * w

		xc = xNext
		yc = yNext
		theta = thetaNext

		toRPM = 30 / PI
		dist = sqrt(xc*xc + yc * yc)

		prevIntegrationTime = micros()