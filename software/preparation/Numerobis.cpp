#include "Numerobis.h"
#include <Wire.h>
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
}

void Numerobis::init()
{
	Serial.begin(115200);

	this->initTof();
	this->initSharp();
	this->initQTR1A();
	this->initInterface();
	this->initEncoder();
	this->initLed();
	return;
}

void Numerobis::initTof()
{
	pinMode(PIN_TOF2_XSHUT, OUTPUT);
	pinMode(PIN_TOF3_XSHUT, OUTPUT);

	Wire.begin();
	
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
	
	valueTof2 = Tof2.readRangeContinuousMillimeters();
	if (Tof2.timeoutOccurred()) 
		{ timeoutTof2 = true; } 
	else 
		{timeoutTof2 = false;}
	
	valueTof3 = Tof3.readRangeContinuousMillimeters();
	if (Tof3.timeoutOccurred()) 
		{ timeoutTof3 = true; } 
	else 
		{timeoutTof3 = false;}

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


