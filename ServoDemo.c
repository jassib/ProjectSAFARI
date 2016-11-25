/**********************************************************************
Filename: ServoDemo.c
Date: June 6, 2015
File Version: 1.0

Demonstrate the operation of the Tetrix Prime motors (continous and
standard servo motors, including the gripper).

The standard servo can be used to position for positioning (from -90 to 90
degrees).

The gripper can be set for positions between 0 and 70.
  CAUTION:  The value to close the gripper is close to or at zero.  It
  differs based on the slight variations in gripper assembly.  If you
  hear the motor running while in the closed position, increase the
  commanded value slightly.

The continuous servos can be set to values between -100 and +100.
There is a deadband for the continous servos, which can be determined
using the program.  To determine the deadband offsets, run the program and
gradually increase the reverse speed by pushing the left button to
subtract from the speed setting.  Record the value just before the
motors start to move.  Similarly, use the right button to increase
speed and determine the value for the positive direction.  For example,
if the servo is stationary at -35 and moving at -36, the reverse offset is -35.
These negative and positive values can be sent to the setServoSpeed function.

History
Ver  Date       Comment
1.0  Jun  6/15  original release

**********************************************************************/
#include "NXTServo-lib-UW.c"

// Motor locations on interface board
const int SV_CSERVO1 = 1;
const int SV_CSERVO2 = 2;
const int SV_STANDARD = 3;
const int SV_GRIPPER = 4;

// Demo types
const int CONTINUOUS = 2;
const int GRIPPER = 3;
const int STANDARD = 1;

int getButton()
{
	while (nNxtButtonPressed == -1)
	{}
	int button = nNxtButtonPressed;
	while (nNxtButtonPressed != -1)
	{}
	return button;
}

void setDemoMotor(int motorChoice, int motorSetting)
{
		if (motorChoice == CONTINUOUS)
		{
			// set servo motors with desired speed
			setServoSpeed(S1,SV_CSERVO1, motorSetting);
			setServoSpeed(S1,SV_CSERVO2, motorSetting);
		}
		else if (motorChoice == STANDARD)
		{
			// set servo motor with desired position
			setServoPosition(S1,SV_STANDARD,motorSetting);
		}
		else
		{
			// set gripper with desired position
			setGripperPosition(S1,SV_GRIPPER,motorSetting);
		}
}

task main()
{
	// configure servo controller port
	SensorType[S1] = sensorI2CCustom9V;

	// display information on the screen
	displayString(0,"NXT S1: control");
	displayString(1,"SV%d: C servo 1",SV_CSERVO1);
	displayString(2,"SV%d: C servo 2",SV_CSERVO2);
	displayString(3,"SV%d: S servo",SV_STANDARD);
	displayString(4,"SV%d: Gripper",SV_GRIPPER);
	displayString(5,"SV8: Battery");
	displayString(7,"Confirm (button)");
	getButton();

	// select demo
	eraseDisplay();
	displayString(0,"Push button to");
	displayString(1,"select demo");
	displayString(3,"< : continuous");
	displayString(4,"orange : gripper");
	displayString(5,"> : standard");
	int choice = getButton();

	// menu
	eraseDisplay();
	if (choice == CONTINUOUS)
		displayString(0,"Continuous Servo");
	else if (choice == STANDARD)
		displayString(0,"Standard Servo");
	else
		displayString(0,"Gripper");
	displayString(2,"Orange to end");
	displayString(3,"< decrease");
	displayString(4,"> increase");

	// initial setting for motors
	int motorSetting = 0;
	if (choice == GRIPPER)
		motorSetting = 40;

	// keep looping until the orange button is pressed
	while (nNxtButtonPressed != 3)
	{
		// adjust setting up or down
		if (nNxtButtonPressed == 1)
		{
			motorSetting += 1;
			setDemoMotor(choice,motorSetting);
		}
		else if (nNxtButtonPressed == 2)
		{
			motorSetting -= 1;
			setDemoMotor(choice,motorSetting);
		}
		displayString(6, "setting: %d      ", motorSetting);
		wait1Msec(300);
	}

	eraseDisplay();
	displayString(0,"Done");

	// move motors to stop/neutral positions
	setServoSpeed(S1,SV_CSERVO1, 0);
	setServoSpeed(S1,SV_CSERVO2, 0);
	setServoPosition(S1,SV_STANDARD,0);
	resetGripper(S1,SV_GRIPPER);
	wait1Msec(10000);
}
/**********************************************************************
Copyright(c) 2014-2015 D. Lau, M. Stachowsky, and C.C.W. Hulls, P.Eng.
Students, staff, and faculty members at the University of Waterloo
are granted a non-exclusive right to copy, modify, or use this
software for non-commercial teaching, learning, and research purposes
provided the author(s) are acknowledged except for as noted below.

GENE 121 and ME 101 students can copy, modify, or use this software
in their current term course assignments without acknowledging
the source.
**********************************************************************/
