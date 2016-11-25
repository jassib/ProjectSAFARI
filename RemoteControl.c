task main() // Sumeet
{
	int i = 1, item = 0, aisle = 0;
	int button = 0;
	int isDropOff = 0;

	setBluetoothOn();
	btConnect(1,"NXT42");
	ClearMessage();

	displayString(0, "Left for pickup");
	displayString(1, "Right for dropoff");

	while(nNxtButtonPressed == -1) {}
	if(nNxtButtonPressed == 1)
			isDropOff = 2;
	else if(nNxtButtonPressed == 2)
			isDropOff = 3;
	while(nNxtButtonPressed != -1) {}

	while(button != 3)
	{
		displayString(3, "Aisle: %d", i);
		while(nNxtButtonPressed == -1) {}
		if(nNxtButtonPressed == 3)
		{
			aisle = i;
			button = 3;
		}
		if(nNxtButtonPressed == 1)
			if((i+1) <= 4)
				i++;
		if(nNxtButtonPressed == 2)
			if((i-1) >= 1)
				i--;
		while(nNxtButtonPressed != -1) {}
	}

  button = 0;

	while(button != 3)
	{
		displayString(4, "Item: %d", i);
		while(nNxtButtonPressed == -1) {}
		if(nNxtButtonPressed == 3)
		{
			item = i;
			button = 3;
		}
		if(nNxtButtonPressed == 1)
			if((i+1) <= 10)
				i++;
		if(nNxtButtonPressed == 2)
			if((i-1)>= 1)
				i--;
		while(nNxtButtonPressed != -1) {}
	}

	sendMessageWithParm(isDropOff,aisle,item);


	displayString(5, "%d %d %d", isDropOff, aisle, item);
	wait1Msec(45667);
}
