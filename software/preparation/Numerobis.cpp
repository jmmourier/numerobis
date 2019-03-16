#include "Numerobis.h"
//#include <Wire.h>
#include <I2C.h>
#include <VL53L0X.h>

Numerobis::Numerobis()
{
	EncoderLeftValue = 0;
	EncoderRightValue = 0;
	VL53L0X Tof1();
	VL53L0X Tof2();
	VL53L0X Tof3();
	valueTof1 = 0;
	valueTof2 = 0;
	valueTof3 = 0;
	valueSharp1 = 0;
	valueSharp2 = 0;
	valueSharp3 = 0;
	valueQTR1A1 = 0;
	valueQTR1A2 = 0;
	valueInterface = 0;
	asservEnabled = false;
	leftWheelTarget = 0;
	rightWheelTarget = 0;	
	sumErrorLeft = 0;
	sumErrorRight = 0;
	targetDistance = 0;
	targerOrientation = 0;
	oldRightEncoderValue = 0;
	oldLeftEncoderValue = 0;
	enableSideTof = true;
	targetReached = false;
	timeCommandAsserv = 0;
}

void Numerobis::init()
{
	Serial.begin(115200);

	Serial.println("initTof");
	this->initTof();
	Serial.println("initSharp");
	this->initSharp();
	Serial.println("initQTR");
	this->initQTR1A();
	Serial.println("initInterface");
	this->initInterface();
	Serial.println("initEncoder");
	this->initEncoder();
	Serial.println("initLed");
	this->initLed();
	return;
}

void Numerobis::initTof()
{
	pinMode(PIN_TOF2_XSHUT, OUTPUT);
	pinMode(PIN_TOF3_XSHUT, OUTPUT);

	I2c.begin();
	
	// hold Tof2 and Tof3
	digitalWrite(PIN_TOF2_XSHUT,HIGH);
	digitalWrite(PIN_TOF3_XSHUT,HIGH);
	delay(10);

	// change Tof1 adress
	this->Tof1.init();
	this->Tof1.setTimeout(500);
	this->Tof1.setAddress(30);
	delay(10);

	// free Tof2
	digitalWrite(PIN_TOF2_XSHUT,LOW);
	delay(10);
	this->Tof2.init();
	this->Tof2.setTimeout(500);
	this->Tof2.setAddress(31);
	delay(10);


	//free Tof3
	digitalWrite(PIN_TOF3_XSHUT,LOW);
	delay(10);
	this->Tof3.init();
	this->Tof3.setTimeout(500);
	delay(10);

	Tof1.startContinuous();
	Tof2.startContinuous();
	Tof3.startContinuous();
}

void Numerobis::initSharp()
{
	pinMode(PIN_SHARP1, INPUT);
	pinMode(PIN_SHARP2, INPUT);
	pinMode(PIN_SHARP3, INPUT);
}

void Numerobis::initQTR1A()
{
	pinMode(PIN_QTR1A_1, INPUT);
	pinMode(PIN_QTR1A_2, INPUT);
}

void Numerobis::initInterface()
{
	pinMode(PIN_INTERFACE_1, INPUT);
}

void Numerobis::initEncoder()
{
	pinMode(PIN_ENCODER_L_A, INPUT);
	pinMode(PIN_ENCODER_L_B, INPUT);
	pinMode(PIN_ENCODER_R_A, INPUT);
	pinMode(PIN_ENCODER_R_B, INPUT);
	
	attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_A), handleEncoderLeft, CHANGE );
	attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_A), handleEncoderRight, CHANGE );
}

void Numerobis::initLed()
{
	pinMode(PIN_LED, OUTPUT);
}

int Numerobis::getSensorValue(Componants componantID)
{
	int valueToReturn = 0;
	switch(componantID)
	{
		case TofLeft : valueToReturn = valueTof3;
			break;
		case TofMiddle : valueToReturn = valueTof1;
			break;
		case TofRight : valueToReturn = valueTof2;
			break;
		case SharpLeft : valueToReturn = valueSharp1;
			break;
		case SharpMiddle : valueToReturn = valueSharp2;
			break;
		case SharpRight : valueToReturn = valueSharp3;
			break;
		case QTR1ALeft : valueToReturn = valueQTR1A2;
			break;
		case QTR1ARight : valueToReturn = valueQTR1A1;
			break;
		case EncoderLeft : valueToReturn = EncoderLeftValue;
			break;
		case EncoderRight : valueToReturn = EncoderRightValue;
			break;
		case Interface : valueToReturn = valueInterface;
			break;
		default : 
			break;
	}
	return valueToReturn;
}


void Numerobis::debug()
{
	/*
	// Raw debug
	Serial.print("\t ");
	Serial.print(getSensorValue(TofLeft));
	Serial.print("\t ");
	Serial.print(getSensorValue(TofMiddle));
	Serial.print("\t ");
	Serial.print(getSensorValue(TofRight));
	Serial.print("\t ");
	Serial.print(getSensorValue(SharpLeft));
	Serial.print("\t ");
	Serial.print(getSensorValue(SharpMiddle));
	Serial.print("\t ");
	Serial.print(getSensorValue(SharpRight));
	Serial.print("\t ");
	Serial.print(getSensorValue(QTR1ALeft));
	Serial.print("\t ");
	Serial.print(getSensorValue(QTR1ARight));
	Serial.print("\t ");
	Serial.print(getSensorValue(EncoderLeft));
	Serial.print("\t ");
	Serial.print(getSensorValue(EncoderRight));
	Serial.print("\t ");
	Serial.print(getSensorValue(Interface));
	*/

	
	// interpreted debug

	Serial.print("\t ToF L ");
	Serial.print(getSensorValue(TofLeft));
	Serial.print("\t ToF M ");
	Serial.print(getSensorValue(TofMiddle));
	Serial.print("\t ToF R ");
	Serial.print(getSensorValue(TofRight));
	Serial.print("\t Shp L ");
	Serial.print(getSensorValue(SharpLeft)>512?0:1);
	Serial.print("\t Shp M ");
	Serial.print(getSensorValue(SharpMiddle)>512?0:1);
	Serial.print("\t Shp R ");
	Serial.print(getSensorValue(SharpRight)>512?0:1);
	Serial.print("\t QTR L ");
	Serial.print(getSensorValue(QTR1ALeft));
	Serial.print("\t QTR R ");
	Serial.print(getSensorValue(QTR1ARight));
	Serial.print("\t Enc L ");
	Serial.print(getSensorValue(EncoderLeft));
	Serial.print("\t Enc R ");
	Serial.print(getSensorValue(EncoderRight));
	Serial.print("\t Butt ");
	Serial.print(getSensorValue(Interface)>512?0:1);

	/*
	// encoder debug
	Serial.print("EncoL ");
	Serial.print(EncoderLeftValue);
	Serial.print("\tEncoR ");
	Serial.print(EncoderRightValue);
	
	Serial.print("\t ");
	Serial.print(digitalRead(PIN_ENCODER_L_A));
	Serial.print(" ");
	Serial.print(digitalRead(PIN_ENCODER_L_B));
	Serial.print(" ");
	Serial.print(digitalRead(PIN_ENCODER_R_A));
	Serial.print(" ");
	Serial.print(digitalRead(PIN_ENCODER_R_B));
	*/

	Serial.print("\r\n");
//	Serial.println("");
}


void Numerobis::updateSensor()
{
	// tof
	// problem, those are blocking function, 
	// need some rework on the wire librairy
	valueTof1 = Tof1.readRangeContinuousMillimeters();
	if (Tof1.timeoutOccurred()) 
		{ timeoutTof1 = true; } 
	else 
		{timeoutTof1 = false;}
	
	if(enableSideTof)
	{
		valueTof2 = Tof2.readRangeContinuousMillimeters();
		if (Tof2.timeoutOccurred()) 
			{ timeoutTof2 = true; } 
		else 
			{timeoutTof2 = false;}
	}

	if(enableSideTof)
	{
		valueTof3 = Tof3.readRangeContinuousMillimeters();
		if (Tof3.timeoutOccurred()) 
			{ timeoutTof3 = true;} 
		else 
			{timeoutTof3 = false;}	
	}
	
	// sharp
	valueSharp1 = analogRead(PIN_SHARP1);
	valueSharp2 = analogRead(PIN_SHARP2);
	valueSharp3 = analogRead(PIN_SHARP3);

	// QTR
	valueQTR1A1 = analogRead(PIN_QTR1A_1);
	valueQTR1A2 = analogRead(PIN_QTR1A_2);

	// interface
	valueInterface = analogRead(PIN_INTERFACE_1);
}

void Numerobis::setLed(bool flag)
{
	if(flag)
		digitalWrite(PIN_LED,HIGH);
	else
		digitalWrite(PIN_LED,LOW);
}


void Numerobis::setSpeed(int vMotRight, int vMotLeft)
{
	vMotRight = max(min(vMotRight,255),-255);
	vMotLeft = max(min(vMotLeft,255),-255);

	if(vMotRight > -25 && vMotRight < 25)
	{
		vMotRight = 0;
	}
	if(vMotLeft > -25 && vMotLeft < 25)
	{
		vMotLeft = 0;
	}

	if(vMotRight > 0)
	{
		analogWrite(PIN_MOTOR_R_1,vMotRight);
		analogWrite(PIN_MOTOR_R_2,0);
	}
	else
	{
		analogWrite(PIN_MOTOR_R_2,-vMotRight);
		analogWrite(PIN_MOTOR_R_1,0);
	}	
	if(vMotLeft > 0)
	{
		analogWrite(PIN_MOTOR_L_1,0);
		analogWrite(PIN_MOTOR_L_2,vMotLeft);
	}
	else
	{
		analogWrite(PIN_MOTOR_L_1,-vMotLeft);
		analogWrite(PIN_MOTOR_L_2,0);
	}	
}

int Numerobis::hasTofTimeout(Componants componantID)
{
	bool valueToReturn = -1;
	switch(componantID)
	{
		case TofLeft : 
			if(timeoutTof3) valueToReturn = 1; else valueToReturn=0;
			break;
		case TofMiddle : 			
			if(timeoutTof1) valueToReturn = 1; else valueToReturn=0;
			break;
		case TofRight : 
			if(timeoutTof2) valueToReturn = 1; else valueToReturn=0;
			break;
		default : 
			break;
	}
	return valueToReturn;
}

void Numerobis::enableAsserv(bool flag)
{
	asservEnabled = flag;
}

void Numerobis::setTargetEncoder(int setLeftWheelTarget, int setRightWheelTarget)
{
	leftWheelTarget = setLeftWheelTarget;
	rightWheelTarget = setRightWheelTarget;

	sumErrorLeft = 0;
	sumErrorRight = 0;
}

void Numerobis::setZeroAsserv()
{
	EncoderLeftValue = 0;
	EncoderRightValue = 0;
}	

void Numerobis::processAsserv()
{
	if(!asservEnabled) return;

	double K = 0.25;
	double Ki = 0.01;
	// Proportional corrector
	int ErrorLeft = leftWheelTarget - EncoderLeftValue;
	sumErrorLeft += ErrorLeft;
	int VmoteurLeft = 0;
	if(abs(ErrorLeft) >15)
	{
		VmoteurLeft = K * ErrorLeft + Ki * sumErrorLeft;
	}

	// Proportional corrector
	int ErrorRight = rightWheelTarget - EncoderRightValue;
	sumErrorRight += ErrorRight;
	int VmoteurRight = 0;
	if(abs(ErrorRight) > 15)
	{
		VmoteurRight = K * ErrorRight + Ki * sumErrorRight;
	}
	
	setSpeed(max(min(VMAX,VmoteurRight),-VMAX), max(min(VMAX,VmoteurLeft),-VMAX));

	Serial.print(leftWheelTarget);
	Serial.print("\t");
	Serial.print(EncoderLeftValue);
	Serial.print("\t");
	Serial.print(min(VMAX,VmoteurLeft));
	Serial.print("\t");
	Serial.print(rightWheelTarget);
	Serial.print("\t");
	Serial.print(EncoderRightValue);
	Serial.print("\t");
	Serial.print(min(VMAX,VmoteurRight));
	Serial.println();

	// (voir https://www.rcva.fr/10-ans-dexperience/9/)
}

void Numerobis::setOrientation(int orientation)
{
	targerOrientation = orientation ;
	targetReached = false;
	timeCommandAsserv = millis();
}

void Numerobis::setDistance(int distance)
{
	targetDistance = distance;
	targetReached = false;
	timeCommandAsserv = millis();
}

void Numerobis::setTargetAsserv(int distance, int orientation)
{
	setOrientation(orientation);
	setDistance(distance);
	setZeroAsserv();
}

void Numerobis::processAsservEnhanced()
{
	if(!asservEnabled) return;

	double Kpd = 0.35;
	double Kdd = 0.01;
	double Kpo = 0.35;
	double Kdo = 0.01;

	int differenceTimeSinceLastEnhancedAsserv = millis() - timeLastEnhancedAsserv;
	timeLastEnhancedAsserv = millis();

	if(differenceTimeSinceLastEnhancedAsserv < 1) return;

	int speedRight = ( EncoderRightValue - oldRightEncoderValue)*1000 / differenceTimeSinceLastEnhancedAsserv ;
	int speedLeft = ( EncoderLeftValue - oldLeftEncoderValue)*1000 / differenceTimeSinceLastEnhancedAsserv; 
	oldRightEncoderValue = EncoderRightValue;
	oldLeftEncoderValue = EncoderLeftValue;

//distance = (roue_d + roue_g) /2
	int distanceDone = (EncoderLeftValue + EncoderRightValue ) / 2;
//vitesse= (vitesse_roue_d + vitesse_roue_g)/2
	int speedDone = (speedRight + speedLeft ) / 2;
//ecart = consigne_distance – distance
	int diff = targetDistance - distanceDone;
//commande= ecart * GAIN_PROPORTIONNEL_DISTANCE
	int commandD = 0;
	if(abs(diff) > 15) 
		{commandD = diff * Kpd;} 
//commande_distance = commande – GAIN_DERIVE_DISTANCE*vitesse
	int commandDistance = commandD - Kdd * speedDone;
//orientation = roue_D – roue_G
	int orientation = EncoderRightValue - EncoderLeftValue;
//vitesse_orientation=vitesse_roue_D – vitesse_roue_G
	int speedOrient = speedRight - speedLeft; 
//ecart = consigne_orientation – orientation
	int diffOrient = targerOrientation - orientation;
//commande = ecart * GAIN_PROPORTIONNEL_ROTATION
	int commandO = 0;
	if(abs(diffOrient) > 15)
		{commandO = diffOrient * Kpo;}
//commande_rotation = commande – GAIN_DERIVE_ROTATION*vitesse_orientation
	int commandOrientation = commandO - Kdo * speedOrient;
//commande_roue_D= commande_distance + commande_rotation
	int commandRight = commandDistance + commandOrientation;
//commande_roue_G= commande_distance – commande_rotation
	int commandLeft = commandDistance - commandOrientation;

	if(commandO == 0 && commandD == 0)
	{
		targetReached = true;
	}
	if(millis() - timeCommandAsserv > 1200)
	{
		targetReached = true;
	}
	setSpeed(max(min(VMAX,commandRight),-VMAX), max(min(VMAX,commandLeft),-VMAX));

/*
	Serial.print("speedRight ");
	Serial.print(speedRight);
	Serial.print("\t");
	Serial.print("speedLeft ");
	Serial.print(speedLeft);
	Serial.print("\t");
	Serial.print("distanceDone ");
	Serial.print(distanceDone);
	Serial.print("\t");
	Serial.print("speedDone ");
	Serial.print(speedDone);
	Serial.print("\t");
	Serial.print("diff ");
	Serial.print(diff);
	Serial.print("\t");
	Serial.print("commandD ");
	Serial.print(commandD);
	Serial.print("\t");
	Serial.print("commandDistance ");
	Serial.print(commandDistance);
	Serial.print("\t");
	Serial.print("orientation ");
	Serial.print(orientation);
	Serial.print("\t");
	Serial.print("speedOrient ");
	Serial.print(speedOrient);
	Serial.print("\t");
	Serial.print("diffOrient ");
	Serial.print(diffOrient);
	Serial.print("\t");
	Serial.print("commandO ");
	Serial.print(commandO);
	Serial.print("\t");
	Serial.print("commandOrientation ");
	Serial.print(commandOrientation);
	Serial.print("\t");
	Serial.print("commandRight ");
	Serial.print(commandRight);
	Serial.print("\t");
	Serial.print("commandLeft ");
	Serial.print(commandLeft);
	Serial.print("\t");

	Serial.println("");
	*/
}

void Numerobis::stopSideTof()
{
	Tof2.stopContinuous();
	Tof3.stopContinuous();
	enableSideTof = false;
}

bool Numerobis::isTargetReached()
{
	return targetReached;
}