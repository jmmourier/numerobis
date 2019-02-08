#include <VL53L0X.h>
#include <Wire.h>

VL53L0X TOF1;
VL53L0X TOF2;
VL53L0X TOF3;

//pin numbers
//input
const int buttonPin = A7;    
const int pinSharp1 = A2;
const int pinSharp2 = A3;
const int pinSharp3 = A6;
const int groundSensor1 = A0;
const int groundSensor2 = A1;
const int encoderLA = 2;
const int encoderLB = 7;
const int encoderRA = 3;
const int encoderRB = 8;
//output
const int ledPin =  13;     
const int groundEmitter = 4;
const int xshut2 = 12;
const int xshut3 = 9;
const int IN1 = 5;
const int IN2 = 10;
const int IN3 = 6;
const int IN4 = 11;

// variables 
int buttonState = 0;    
int valSharp1 = 0;
int valSharp2 = 0;
int valSharp3 = 0;
int valGroundSensor1on = 0;
int valGroundSensor2on = 0;
int valGroundSensor1off = 0;
int valGroundSensor2off = 0;
int valTOF1 = 0;
int valTOF2 = 0;
int valTOF3 = 0;
int timeoutTOF1 = 0 ;
int timeoutTOF2 = 0 ;
int timeoutTOF3 = 0 ;
long valueLeft = 0;
long valueRight = 0;
int speedMotor = 255;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println("begin init");

  // output
  pinMode(ledPin, OUTPUT);
  pinMode(groundEmitter,OUTPUT);
  pinMode(xshut2,OUTPUT);
  pinMode(xshut3,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  // input
  pinMode(buttonPin, INPUT);
  pinMode(pinSharp1, INPUT);
  pinMode(pinSharp2, INPUT);
  pinMode(pinSharp3, INPUT);
  pinMode(groundSensor1, INPUT);
  pinMode(groundSensor2, INPUT);

  // ToF
  digitalWrite(xshut2,HIGH);
  digitalWrite(xshut3,HIGH);
  delay(10);

  TOF1.init();
  TOF1.setTimeout(500);
  TOF1.setAddress(30);

  delay(10);  
  digitalWrite(xshut2,LOW);
  delay(10);
  
  TOF2.init();
  TOF2.setTimeout(500);
  TOF2.setAddress(31);

  delay(10);
  digitalWrite(xshut3,LOW);
  delay(10);
  
  TOF3.init();
  TOF3.setTimeout(500);
  
  TOF1.startContinuous();
  TOF2.startContinuous();
  TOF3.startContinuous();

  // interuption
  attachInterrupt(digitalPinToInterrupt(encoderLA), encoderLeft, CHANGE );
  attachInterrupt(digitalPinToInterrupt(encoderRA), encoderRight, CHANGE );
  
  Serial.println("init done");
}

void loop() {
  // read sensors
  buttonState = analogRead(buttonPin);

  valSharp1 = analogRead(pinSharp1);
  valSharp2 = analogRead(pinSharp2);
  valSharp3 = analogRead(pinSharp3);

  digitalWrite(groundEmitter,HIGH); 
  delay(1);
  valGroundSensor1on = analogRead(groundSensor1);
  valGroundSensor2on = analogRead(groundSensor2);

  digitalWrite(groundEmitter,LOW);
  delay(1);
  valGroundSensor1off = analogRead(groundSensor1);
  valGroundSensor2off = analogRead(groundSensor2);

  valTOF1 = TOF1.readRangeContinuousMillimeters();
  if (TOF1.timeoutOccurred()) { timeoutTOF1 = 1; } else {timeoutTOF1 = 0;}
  valTOF2 = TOF2.readRangeContinuousMillimeters();
  if (TOF2.timeoutOccurred()) { timeoutTOF2 = 1; } else {timeoutTOF2 = 0;}
  valTOF3 = TOF3.readRangeContinuousMillimeters();
  if (TOF3.timeoutOccurred()) { timeoutTOF3 = 1; } else {timeoutTOF3 = 0;}

  // Actions
  if (buttonState > 512) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
    analogWrite(IN1,0);
    analogWrite(IN2,speedMotor);
    analogWrite(IN3,0);
    analogWrite(IN4,speedMotor);
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
    analogWrite(IN1,speedMotor);
    analogWrite(IN2,0);
    analogWrite(IN3,speedMotor);
    analogWrite(IN4,0);
  }
  printStatus();
}


void printStatus()
{
Serial.print("button ");
Serial.print(buttonState);

Serial.print("\tsharp 1 ");
Serial.print(valSharp1);
Serial.print("\tsharp 2 ");
Serial.print(valSharp2);
Serial.print("\tsharp 3 ");
Serial.print(valSharp3);

Serial.print("\tground 1 ");
Serial.print(valGroundSensor1on);
Serial.print("\tground 2 ");
Serial.print(valGroundSensor2on);
Serial.print("");
Serial.print("\tground 1 ");
Serial.print(valGroundSensor1off);
Serial.print("\tground 2 ");
Serial.print(valGroundSensor2off);

Serial.print("\tTOF1 ");
if(timeoutTOF1) Serial.print("TIMEOUT"); else Serial.print(valTOF1);
Serial.print("\tTOF2 ");
if(timeoutTOF2) Serial.print("TIMEOUT"); else Serial.print(valTOF2);
Serial.print("\tTOF3 ");
if(timeoutTOF3) Serial.print("TIMEOUT"); else Serial.print(valTOF3);

Serial.print("\tEncoder Left ");
Serial.print(valueLeft);
Serial.print("\tEncoder Right ");
Serial.print(valueRight);

Serial.println("");
}

// count encoder
void encoderLeft()
{
  if(digitalRead(encoderLA) == 1)
  {
    if(digitalRead(encoderLB) == 1)
    valueLeft--;
    else
    valueLeft++;
  }
  else // LA = 0
  {
    if(digitalRead(encoderLB) == 1)
    valueLeft++;
    else
    valueLeft--;
  }
}

// count encoder
void encoderRight()
{
{
  if(digitalRead(encoderRA) == 1)
  {
    if(digitalRead(encoderRB) == 1)
    valueRight--;
    else
    valueRight++;
  }
  else // RA = 0
  {
    if(digitalRead(encoderRB) == 1)
    valueRight++;
    else
    valueRight--;
  }
}
}


