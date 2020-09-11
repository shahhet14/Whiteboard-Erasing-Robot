const int eraserMove=87;
const int eraserPower=40;
const int eraserRotate=80;

int optionSelect(){
	displayString(6,"up-blue, down-red,right-black,left-all");
	displayString(8,"Press center for shutdown");
	while(!getButtonPress(ANY_BUTTON)){}

	if(getButtonPress(UP_BUTTON)){
		//blue
		while(getButtonPress(UP_BUTTON)){}
		return 2;
	}
	else if(getButtonPress(DOWN_BUTTON)){
		//red
		while(getButtonPress(DOWN_BUTTON)){}
		return 5;
	}
	else if(getButtonPress(RIGHT_BUTTON)){
		//black
		while(getButtonPress(RIGHT_BUTTON)){}
		return 1;
	}
	else{
		while(getButtonPress(ENTER_BUTTON)){}
		return 0;
	}

}


void erase(int distance, int colorSelect, bool motorTypeA)
{
	wait1Msec(50);
	if(motorTypeA){
		if((SensorValue[S1] == colorSelect || SensorValue[S2] == colorSelect) && 
			(nMotorEncoder[motorA] < distance))
		{
			if(nMotorEncoder[motorD] != -eraserMove)
			{
				motor[motorD] = -eraserPower;
				while(nMotorEncoder[motorD] > -eraserMove)
				{}
				motor[motorD] = 0;
				motor[motorC] = eraserRotate;
			}
		}

		else
		{
			motor[motorC] = 0;
			if(nMotorEncoder[motorD] != 0)
			{
				motor[motorD] = eraserPower;
				while(nMotorEncoder[motorD] < 0)
				{}
				motor[motorD] = 0;
			}
		}
	}
	else{
		if((SensorValue[S1] == colorSelect || SensorValue[S2] == colorSelect) && 
			(nMotorEncoder[motorB] < distance))
		{
			if(nMotorEncoder[motorD] != -eraserMove)
			{
				motor[motorD] = -eraserPower;
				while(nMotorEncoder[motorD] > -eraserMove)
				{}
				motor[motorD] = 0;
				motor[motorC] = eraserRotate;
			}
		}

		else
		{
			motor[motorC] = 0;
			if(nMotorEncoder[motorD] != 0)
			{
				motor[motorD] = eraserPower;
				while(nMotorEncoder[motorD] < 0)
				{}
				motor[motorD] = 0;
			}
		}
	}
}


void resetDist(int speed, int dist)
{
	nMotorEncoder[motorB] = 0;
	motor[motorA]=motor[motorB]= speed;
	while(nMotorEncoder[motorB] > dist){}
	motor[motorA]=motor[motorB]= 0;
}

void driveNoAccelerate(int speed)
{
	motor[motorA] = motor[motorB] = speed;
	while(SensorValue[S3] == 0){}
	motor[motorA] = motor[motorB] = 0;
}

void turnCW(long turnAngle)
{
	if(turnAngle > 0)
	{
		long current = getGyroDegrees(S4);
		long partway =  current + turnAngle/3;
		while(getGyroRate(S4) != 0){}
		for(int i=1; i<=10; i++)
		{
			motor[motorA] += 1;
			motor[motorB] += 5;
			while(getGyroDegrees(S4)-current < i*3){}
		}
		while(getGyroDegrees(S4)-current < turnAngle-60){}
		for(int i=1; i<=9; i++)
		{
			motor[motorA] -= 1;
			motor[motorB] -= 3;
			while(getGyroDegrees(S4)-partway<i*6){}
		}
		while(getGyroDegrees(S4)<current+turnAngle){}
		motor[motorA]=motor[motorB]=0;
	}
}

void obstacle()
{
	displayString(2,"please remove obstacle");
	displayString(4,"reset robot and press a button");
	setSoundVolume(75);
	while(!getButtonPress(buttonAny))
	{
		playSoundFile("Boing");
	}
	setSoundVolume(0);
	eraseDisplay();
	//call shutdown function
}

void accelerate(int dist, int targetPower, int colour){
	if(targetPower<100 && targetPower>0 && dist>0)
	{
		int incDist = dist/targetPower;
		nMotorEncoder[motorB] = 0;
		for(int mpow = 1; mpow<targetPower; mpow++)
		{
			motor[motorA] = motor[motorB] = mpow;
			while(nMotorEncoder[motorB] < mpow*incDist)
			{
				if(mpow>1)
					erase(mpow*incDist, colour, true);
			}
		}
	}
	else if(targetPower<0 && targetPower>-100 && dist>0)
	{
		int incDist = dist/(-targetPower);
		nMotorEncoder[motorB] = 0;
		for(int mpow = -1; mpow>targetPower ; mpow--)
		{
			motor[motorA] = motor[motorB] = mpow;
			while(nMotorEncoder[motorB] > (-mpow)*incDist)
			{
				erase((-mpow)*incDist, colour, true);
			}
		}
	}
	else
		displayString(2,"invalid acceleration parameters");
}

// Gradual Speed Decrease Function
void gradualStop(int motorPower, int stopDistance)
{
	int slowDist=(stopDistance/motorPower);
	if(-100<motorPower<100)
	{
		for(int i=motorPower; i>=3; i--)
		{
			int intitialMotor=nMotorEncoder[motorA];
			motor[motorA] = motor[motorB] = i;
			while((nMotorEncoder[motorA]-intitialMotor)<slowDist){}
		}
		motor[motorA] = motor[motorB] = 0;
	}
	else
	{
		displayString(1, "Invalid Deceleration Parameters");
	}
}

bool driveFWD(int distance, int colour){
	nMotorEncoder[motorA] = 0;
	accelerate(300,40,colour);
	while(nMotorEncoder[motorA] < distance-100)
	{
		erase(distance-100, colour, false);
		if(SensorValue(S3) == 1)
		{
			gradualStop(40,100);
			return false;
		}
	}

	gradualStop(40,100);
	//motor[motorA]=motor[motorB]=0;
	return true;
}

float returnDist(bool direction)
{
	nMotorEncoder[motorA] = 0;
	if(direction)
	{
		accelerate(300,40, 4);
	}
	else
	{
		accelerate(300,-40, 4);
	}
	while(SensorValue[S3] == 0)
	{}
	motor[motorA] = motor[motorB] = 0;
	return nMotorEncoder[motorA];
}



void calibrate(int & lengthX, int & lengthY)
{
	int length1X=0;
	int length1Y=0;
	int length2X=0;
	int length2Y=0;

	length1X = returnDist(true);
	resetDist(-15,-200);
	turnCW(90);
	driveNoAccelerate(-5);


	length1Y = returnDist(true);
	resetDist(-15,-200);
	turnCW(90);
	driveNoAccelerate(-5);

	length2X = returnDist(true);
	resetDist(-15,-200);
	turnCW(90);
	driveNoAccelerate(-5);

	length2Y = returnDist(true);
	resetDist(-15,-200);
	turnCW(90);
	driveNoAccelerate(-5);

	lengthX = (length1X+length2X)/2;
	lengthY = (length1Y+length2Y)/2;
}

void origin(){
	int angle = getGyroDegrees(S4);
	for(int i = 0; angle != i-90; i += 90)
	{
		if (angle >= i-3 && angle <= i+3)
		{
			angle = i;
		}
	}
	if(angle % 360 == 0 || angle == 0){
	resetDist(-10,-200);
	turnCW(180);
		driveNoAccelerate(20);
		resetDist(-10,-200);
		turnCW(90);
		driveNoAccelerate(20);
		resetDist(-10,-200);
		turnCW(90);
	}
	else if(angle % 270 == 0){
		turnCW(270);
		driveNoAccelerate(20);
		resetDist(-10,-200);
		turnCW(90);
		driveNoAccelerate(20);
		resetDist(-10,-200);
		turnCW(90);
	}
	else if(angle % 180 == 0){
		driveNoAccelerate(20);
		resetDist(-10,-200);
		turnCW(90);
		driveNoAccelerate(20);
		resetDist(-10,-200);
		turnCW(90);
	}
	else if(angle % 90 == 0){
		turnCW(90);
		driveNoAccelerate(20);
		resetDist(-10,-200);
		turnCW(90);
		driveNoAccelerate(20);
		resetDist(-10,-200);
		turnCW(90);
	}
}

bool zamboniLoop(int length,int width, int color)
{
	int i = 0;
	int halfw = width/2;
	int lessw = halfw-50;
	bool drive = true;
	while(i<=halfw)
	{
		drive = driveFWD(length,color);
		if(!drive){
			return drive;
		}
		else
		{
			resetDist(-10,-200);
			turnCW(90);
			resetDist(-10,-50);
		}
		drive = driveFWD(halfw,color);
		if(!drive)
		{
			return drive;
		}
		else
		{
			turnCW(90);
			driveNoAccelerate(-10);
		}
		drive = driveFWD(length,color);
		if(!drive){
			return drive;
		}
		else
		{
			resetDist(-10,-200);
			turnCW(90);
			resetDist(-10,-50);
		}
		drive = driveFWD(lessw,color);
		if(!drive)
		{
			return drive;
		}
		else{
			resetDist(-10,-200);
			turnCW(90);
			driveNoAccelerate(-10);
			i+=50;
		}
	}

	return true;
}

task main()
{
	SensorType[S1] = sensorEV3_Color;
	SensorType[S2] = sensorEV3_Color;
	SensorType[S4] = sensorEV3_Gyro;
	SensorType[S3] = sensorEV3_Touch;
	wait1Msec(50);
	SensorMode[S1] = modeEV3Color_Color;
	SensorMode[S2] = modeEV3Color_Color;
	SensorMode[S4] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);
	resetGyro(S4);
	nMotorEncoder[motorA]=0;
	nMotorEncoder[motorD]=0;
	int lengthX = 0;
	int lengthY = 0;
	bool Path = false;


	displayString(3,"press any button to start");
	while(!getButtonPress(buttonAny)){}
	while(getButtonPress(buttonAny)){}
	eraseDisplay();

	calibrate(lengthX,lengthY);
	int option = optionSelect();
	eraseDisplay();

	while (option != 0)
	{
		Path = zamboniLoop(lengthX,lengthY, option);

		if(Path)
		{
			origin();
			option = optionSelect();
			eraseDisplay();
		}
		else
		{
			obstacle();
			option = optionSelect();
			eraseDisplay();
		}
	}
	
	displayString(3,"Robot shutting down");

	wait1Msec(5000);
}
