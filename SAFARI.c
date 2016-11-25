#include "NXTServo-lib-UW.c"

typedef struct
{
	float m_p;   // P coefficient
	float m_i;   // I coefficient
	float m_d;   // D coefficient

	float m_desiredValue; // Desired value
	float m_previousValue; // Value at last call
	float m_errorSum; // Sum of previous errors (for I calcfulation)
	float m_errorIncrement; // Max increment to error sum each call
	float m_errorEpsilon; // Allowable error in determining when done

	bool m_firstCycle; // Flag for first cycle
	float m_maxOutput; // Ceiling on calculation output

	int m_minCycleCount; // Minimum number of cycles in epsilon range to be done
	int m_cycleCount; // Current number of cycles in epsilon range
}PIDStruct;

void PIDinit(PIDStruct &pid)
{
	pid.m_desiredValue = 0; // Default to 0
	pid.m_firstCycle = true;
	pid.m_maxOutput = 100.0; // Default to full range of nxt motor
	pid.m_errorIncrement = 1;

	pid.m_cycleCount = 0;
	pid.m_minCycleCount = 10; // Default
}

void setMax(PIDStruct &pid, float max)
{
	pid.m_maxOutput = max;
}

void setConstants(PIDStruct &pid, float p, float i, float d)
{
	pid.m_p = p;
	pid.m_i = i;
	pid.m_d = d;
}

void setErrorEpsilon(PIDStruct &pid, float epsilon)
{
	pid.m_errorEpsilon = epsilon;
}

void setErrorIncrement(PIDStruct &pid, float inc)
{
	pid.m_errorIncrement = inc;
}

void setDesiredValue(PIDStruct &pid, float val)
{
	pid.m_desiredValue = val;
}

float calcPID(PIDStruct &pid, float currentValue)
{
	// Initialize all components to 0.0 to start.
	float pVal = 0.0;
	float iVal = 0.0;
	float dVal = 0.0;

	// Don't apply D the first time through.
	if(pid.m_firstCycle)
	{
		pid.m_previousValue = currentValue;  // Effective velocity of 0
		pid.m_firstCycle = false;
	}

	// Calculate P Component.
	float error = pid.m_desiredValue - currentValue;
	pVal = pid.m_p * (float)error;

	// Calculate I Component.
	// Error is positive and outside the epsilon band.
	if(error >= pid.m_errorEpsilon)
	{
		// Check if epsilon was pushing in the wrong direction.
		if(pid.m_errorSum < 0)
		{
			// If we are fighting away from the point, reset the error.
			pid.m_errorSum = 0;
		}
		if(error < pid.m_errorIncrement)
		{
			// If the error is smaller than the max increment amount, add it.
			pid.m_errorSum += error;
		}
		else
		{
			// Otherwise, add the maximum increment per cycle.
			pid.m_errorSum += pid.m_errorIncrement;
		}
	}
	// Error is negative and outside the epsilon band.
	else if(error <= -pid.m_errorEpsilon)
	{
		if(pid.m_errorSum > 0)
		{
			// If we are fighting away from the point, reset the error.
			pid.m_errorSum = 0;
		}
		// error is small than max contribution -> just subtract error amount
		if(error > -pid.m_errorIncrement)
		{
			// If the error is smaller than the max increment amount, add it.
			pid.m_errorSum += error; // Error is negative
		}
		else
		{
			// Otherwise, subtract the maximum increment per cycle.
			pid.m_errorSum -= pid.m_errorIncrement;
		}
	}
	// Error is inside the epsilon band.
	else
	{
		pid.m_errorSum = 0;
	}
	iVal = pid.m_i * (float)pid.m_errorSum;

	// Calculate D Component.
	float velocity = currentValue - pid.m_previousValue;
	dVal = pid.m_d * (float)velocity;

	// Calculate and limit the ouput: Output = P + I - D
	float output = pVal + iVal - dVal;
	if(output > pid.m_maxOutput)
	{
		output = pid.m_maxOutput;
	}
	else if(output < -pid.m_maxOutput)
	{
		output = -pid.m_maxOutput;
	}

	// Save the current value for next cycle's D calculation.
	pid.m_previousValue = currentValue;

	return output;
}

bool isDone(PIDStruct &pid)
{
	if (pid.m_previousValue <= pid.m_desiredValue + pid.m_errorEpsilon
			&& pid.m_previousValue >= pid.m_desiredValue - pid.m_errorEpsilon
			&& !pid.m_firstCycle)
	{
		if(pid.m_cycleCount >= pid.m_minCycleCount)
		{
			pid.m_cycleCount = 0;
			return true;
		}
		else
		{
			pid.m_cycleCount++;
		}
	}
	return false;
}

bool followLine(int aisle, bool isAisle, int distance) // Sumeet - tested by following a path made by a black line and obseriving if it is able to follow it
{
	int direction = -1;

	const float BLACK = 31.0;
	const float WHITE = 60.0;
	const float MAX = 50.0;
	const float MIN = 35.0;
	const float SPEED = direction*50;
	float gain = 0.0;

	if(SensorValue[S3] <= 30)
	{
			motor[motorA] = 0;
			motor[motorC] = 0;
			//displayString(0,"%f",nMotorEncoder[motorA]);
			//displayString(1,"%f",distance);
	}
	else if (isAisle)
	{
		if(nMotorEncoder[motorA] <= distance)
			return true;
	}
	else if(SensorValue[S2] == aisle)
			return true;

	if(MIN < SensorValue[S1] && SensorValue[S1] < MAX)
	{
		motor[motorA] = SPEED;
		motor[motorC] = SPEED;
	}
	else if (MIN > SensorValue[S1])
	{
		gain = (MIN - SensorValue[S1])/BLACK;
		motor[motorA] = SPEED; motor[motorC] = SPEED*gain;
		while(MIN > SensorValue[S1])
		{}
	}
	else if(SensorValue[S1] > MAX)
	{
		gain = (SensorValue[S1] - MAX)/WHITE;
		motor[motorA] = SPEED*gain; motor[motorC] = SPEED;
		while(MAX < SensorValue[S1])
		{}
	}
	return false;
}

void motorDrive(PIDStruct &motorPID, int val)
{
	int output = 0;
	setDesiredValue(motorPID, val);
	nMotorEncoder[motorA] = 0;
	while(!isDone(motorPID))
	{
		switch(motorPID.m_maxOutput)
		{
			case 100:
					output = calcPID(motorPID, nMotorEncoder[motorA]);
					motor[motorA] = motor[motorC] = output;
					break;
			case 20:
					output = calcPID(motorPID, nMotorEncoder[motorB]);
					//displayString(0,"%d",output);
					motor[motorB] = output;
					break;
			case 75:
					output = calcPID(motorPID, nMotorEncoder[motorA]);
					motor[motorA] = output; motor[motorC] = -output;
					break;
		}
	}
	motor[motorA] = motor[motorC] = motor[motorB] =  0;
}

/*void drive(float distance, PIDStruct &drivePID) // Saksham - tested by running this function with a distance of 2000ticks and measuring to see if it is withen the error epsilon
{
	int output = 0;
	setErrorEpsilon(drivePID, 80);
	setDesiredValue(drivePID, distance);
	nMotorEncoder[motorA] = 0;
	while(!isDone(drivePID))
	{
		output = calcPID(drivePID, nMotorEncoder[motorA]);
		motor[motorA] = motor[motorC] = output;
	}
	motor[motorA] = motor[motorC] = 0;
}

void elevDrive(PIDStruct &elevPID, int distance) // Saksham - tested by running this function with a distance of 1000ticks and measuring to see if it is withen the error epsilon
{
	int output = 0;
	setErrorEpsilon(elevPID,10);
	setDesiredValue(elevPID, distance);
	while(!isDone(elevPID))
	{
		output = calcPID(elevPID, nMotorEncoder[motorB]);
		motor[motorB] = output;
	}
	motor[motorB] = 0;
}

void turn(float distance, PIDStruct &turnPID) // Turn - tested by running this function with a distance of 425ticks(approx 90 degrees) and measuring to see if it is withen the error epsilon
{
	int output = 0;
	setErrorEpsilon(turnPID, 10);
	setDesiredValue(turnPID, distance);
	nMotorEncoder[motorA] = 0;
	while(!isDone(turnPID))
	{
		output = calcPID(turnPID, nMotorEncoder[motorA]);
		motor[motorA] = output; motor[motorC] = -output;
	}
	motor[motorA] = motor[motorC] = 0;
}*/

void claw(int pos) // Saksham - tested by passin in a position for the claw and measuring if it is correct
{
	setGripperPosition(S4,5,pos);
}

void elevator(PIDStruct &elevPID, int state) // jas - running the function for state 1 ad 0 and obersriving if the elevator goes up and down
{
	switch(state)
	{
		case 1:
			motorDrive(elevPID,-770);
			break;

		case 0:
			motorDrive(elevPID,0);
			break;
	}
}

void newItemPickUp(PIDStruct &drivePID, PIDStruct &turnPID, PIDStruct &elevPID, int pos) // Jas - running the function and obersriving if it is able to pick up an item
{
	claw(70);
	motorDrive(drivePID, 400);
	wait1Msec(1000);
	claw(pos);
	wait1Msec(1000);
	motorDrive(drivePID, -400);
}

void itemPickUp(PIDStruct &drivePID, PIDStruct &elevPID, int elevState, int pos) // Adian - running the function and obersriving if it is able to pick up an item
{
	claw(70);
	elevator(elevPID, elevState);
	wait1Msec(1000);
	motorDrive(drivePID, 445);
	wait1Msec(1000);
	claw(pos);
	wait1Msec(1000);
	motorDrive(drivePID, -545);
	wait1Msec(1000);
	elevator(elevPID, 0);
}

void itemStore(PIDStruct &drivePID, PIDStruct &elevPID, int elevState, int pos)
{
	elevator(elevPID, elevState);
	if(nMotorEncoder[motorB] == 0)
		motorDrive(elevPID, -100);
	wait1Msec(1000);
	motorDrive(drivePID, 445);
	wait1Msec(1000);
	claw(70);
	wait1Msec(1000);
	motorDrive(drivePID, -545);
	wait1Msec(1000);
	elevator(elevPID, 0);
	resetGripper(S4,5);
}

void itemDropOff(PIDStruct &drivePID, PIDStruct &elevPID, PIDStruct &turnPID) // Adian - running the function and obersriving if it is able to drop of an item
{
		motorDrive(turnPID, 390*2);
		motorDrive(drivePID, 445);
		claw(70);
		wait1Msec(1000);
		motorDrive(drivePID, -440);
		wait1Msec(1000);
		resetGripper(S4,5);
}

bool pickUpStateMachine(PIDStruct &drivePID, PIDStruct &turnPID, PIDStruct &elevPID, int aisle, int distance, int elevState, int &state, int pos) // Sumeet - running the function and obersriving it is able to complete a full pick up cycle
{
	const int angleTicks = 390;
	bool isDone = false;
	switch(state)
	{
		case 1:
		  while(!followLine(aisle, false, 0)){}
			state++;
			break;
		case 2:
			motorDrive(turnPID, -angleTicks);
			wait1Msec(500);
			nMotorEncoder[motorA] = 0;
			while(!followLine(aisle, true, -distance)){}
		  state++;
		  break;
		case 3:
			wait1Msec(1000);
		  motorDrive(turnPID, angleTicks);
			itemPickUp(drivePID, elevPID, elevState, pos);
		  motorDrive(turnPID, angleTicks);
		  nMotorEncoder[motorA] = 0;
		  while(!followLine(aisle, true, -distance+600)){}
		  motor[motorA] = motor[motorC] = 0;
		  time1[T1] = 0;
		  motor[motorA] = motor[motorC] = -50;
		  while(time1[T1] < 2000) {}
			motorDrive(turnPID, angleTicks-100);
		  state++;
		  break;
		case 4:
			while(!followLine(5, false, 0)){}
			state++;
			break;
		case 5:
			itemDropOff(drivePID, elevPID, turnPID);
		  state++;
		  break;
		case 6:
		  isDone = true;
		  break;
	}
	return isDone;
}

bool dropOffStateMachine(PIDStruct &drivePID, PIDStruct &turnPID, PIDStruct &elevPID, int aisle, int distance, int elevState, int &state, int pos) // Sumeet - running the functions and oberserving if it is able to complete a full drop off cycle
{
	const int angleTicks = 390;
	bool isDone =  false;
	switch(state)
	{
		case 1:
			newItemPickUp(drivePID,turnPID,elevPID,pos);
			state++;
			break;
		case 2:
			while(!followLine(aisle, false, 0)){}
			state++;
			break;
		case 3:
			motorDrive(turnPID, -angleTicks);
			wait1Msec(500);
			nMotorEncoder[motorA] = 0;
			while(!followLine(aisle, true, -distance)){}
		  state++;
		  break;
		case 4:
			wait1Msec(1000);
		  motorDrive(turnPID, angleTicks);
			itemStore(drivePID, elevPID, elevState, pos);
		  motorDrive(turnPID, angleTicks);
		  nMotorEncoder[motorA] = 0;
		  while(!followLine(aisle, true, -distance+600)){}
		  motor[motorA] = motor[motorC] = 0;
		  time1[T1] = 0;
		  motor[motorA] = motor[motorC] = -50;
		  while(time1[T1] < 2000) {}
			motorDrive(turnPID, angleTicks-100);
		  state++;
			break;
		case 5:
			while(!followLine(5, false, 0)){}
			state++;
			break;
		case 6:
		  motorDrive(turnPID, 390*2-90);
		  state++;
		  break;
		case 7:
		  isDone = true;
			break;
	}
	return isDone;
}

task main() // Sumeet
{
	short distance = 0, aisle = 1, item = 0;
	int elevState = 0, dropOrPickMsg = 0, aisleMsg = 0, itemMsg = 0, state1 = 1, state2 = 0, pos = 0;
	bool isDone = false;

  //sets initial PID variables that will not change
	PIDStruct PIDdrive;
	PIDStruct PIDturn;
	PIDStruct PIDelev;

	setBluetoothOn();
	ClearMessage();

	PIDinit(PIDdrive);
	PIDinit(PIDturn);
	PIDinit(PIDelev);

	setConstants(PIDdrive, 0.3, 0, 2);
	setConstants(PIDturn, 4, 0, 4);
	setConstants(PIDelev, 0.3, 1, 5);

	setMax(PIDdrive,100);
	setMax(PIDturn,75);
	setMax(PIDelev,20);

	setErrorEpsilon(PIDdrive, 40);
	setErrorEpsilon(PIDturn, 0);
	setErrorEpsilon(PIDelev, 10);

	setErrorIncrement(PIDdrive, 0.25);
	setErrorIncrement(PIDturn, 0.25);
	setErrorIncrement(PIDelev, 0.25);

	nMotorEncoder[motorB] = 0;

	// init sensors
	SensorType[S1] = sensorLightActive;
	SensorType[S2] = sensorColorNxtFULL;
	SensorType[S3] = sensorSONAR;
	SensorType[S4] = sensorI2CCustom9V;

	while(!isDone)
	{
		switch(state2)
		{
			case 0:
				dropOrPickMsg = messageParm[0];
				aisleMsg = messageParm[1];
				itemMsg = messageParm[2];
				displayString(0,"Waiting for");
				displayString(1,"command...");

				if(dropOrPickMsg !=0 && aisleMsg != 0 && itemMsg !=0)
				{
					aisle = aisleMsg + 1;
					item = itemMsg;
				  state2++;
				}
				break;
			case 1:
			 if(item <= 5)
				{
					distance = (12 + 496 + (615 * (item-1)));
					elevState = 0;
				}
				else
				{
					distance = (12 + 496 + (615 * (item-6)));
					elevState = 1;
				}
				if(aisle == 2 || aisle == 4)
					pos = 29;
				else
					pos = 48;
				state2 = dropOrPickMsg;
				break;
			case 2:
			  isDone = dropOffStateMachine(PIDdrive, PIDturn, PIDelev, aisle, distance, elevState, state1, pos);
				break;
			case 3:
			  isDone = pickUpStateMachine(PIDdrive, PIDturn, PIDelev, aisle, distance, elevState, state1, pos);
				break;
		}
	}

	motor[motorA] = motor[motorC] = 0;
	while(nNxtButtonPressed == -1) {}
	while(nNxtButtonPressed != -1) {}
}
