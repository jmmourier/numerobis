

/*
Right and Left are defined as the robot point of view

Config to set on the board :
jumper for interface 1 : button or microstart
jumper for RX or button 2
jumper for TX or LED 2
jumper for QTR sensor emitter from arduino or 5V, should be 5V
*/

#include <VL53L0X.h>

// define pins 
#define PIN_TOF2_XSHUT 12
#define PIN_TOF3_XSHUT 9
//#define PIN_SDA A4 //for info
//#define PIN_SCL A5 //for info
#define PIN_SHARP1 A2
#define PIN_SHARP2 A3
#define PIN_SHARP3 A6
#define PIN_QTR1A_1 A0
#define PIN_QTR1A_2 A1
// #define PIN_QTR1A_EMITTER 4 // unused
#define PIN_MOTOR_L_1 5
#define PIN_MOTOR_L_2 10
#define PIN_MOTOR_R_1 6
#define PIN_MOTOR_R_2 11
#define PIN_ENCODER_L_A 2
#define PIN_ENCODER_L_B 7
#define PIN_ENCODER_R_A 3
#define PIN_ENCODER_R_B 8
#define PIN_INTERFACE_1 A7
#define PIN_MICROSTART A7  
#define PIN_BUTTON A7    
#define PIN_LED 13     

#define VMAX 90

enum Componants
{
	TofLeft,
	TofMiddle,
	TofRight,

	SharpLeft,
	SharpMiddle,
	SharpRight,
	
	QTR1ALeft,
	QTR1ARight,
	
	MotorLeft,
	MotorRight,
	
	EncoderLeft,
	EncoderRight,
	
	Interface,
	
	Led
};

class Numerobis
{

public:
	Numerobis();
	void init();
	void debug();

	void updateSensor();
	void setLed(bool flag);


	/* values
	for interface button 1024 = normal, 0 = pushed
	for QTR sensor blask ~= 900 white ~= 100
	*/
	int getSensorValue(Componants componantID);

	void setSpeed(int vMotRight, int vMotLeft);

	int hasTofTimeout(Componants componantID);

	void enableAsserv(bool flag);
	void setTargetEncoder(int setLeftWheelTarget, int setRightWheelTarget);
	void setZeroAsserv();
	void processAsserv();

	void setTargetAsserv(int distance, int orientation);
	void processAsservEnhanced();
	
	// send true when target is reached or timeouted
	bool isTargetReached();

	void stopSideTof();

/*
	TODO

	void setPosition(position);
	void enableAsserv(flag);
	void setAsservParameters(Kp Ki Kd Freq Tolerance)

	int isObstacle();
	int getSensorValue(Componants componantID);

	bool lineHasBeenDetected();
	bool leftlineHasBeenDetected();
	bool rightlineHasBeenDetected();
	void setinterfaceAsButton()
	void setinterfaceAsMicrostart()
	bool interfaceHasbeenpressed()
	bool interfaceisactive()
*/


private:
	void initTof();
	void initSharp();
	void initQTR1A();
	void initInterface();
	void initEncoder();
	void initLed();

	VL53L0X Tof1;
	VL53L0X Tof2;
	VL53L0X Tof3;

	int valueTof1;
	int valueTof2;
	int valueTof3;
	int valueSharp1; // value should be 0 or 1, but this is analog ...
	int valueSharp2; // ... and the sensor can be changed by a real ...
	int valueSharp3; // ... analogic value
	int valueQTR1A1;
	int valueQTR1A2;
	int valueInterface;

	bool timeoutTof1;
	bool timeoutTof2;
	bool timeoutTof3;

	bool asservEnabled;
	int leftWheelTarget;
	int rightWheelTarget;

	int sumErrorLeft;
	int sumErrorRight;

	int targetDistance;
	int targerOrientation;

	void setOrientation(int orientation);
	void setDistance(int distance);
	unsigned long timeLastEnhancedAsserv; 
	long oldRightEncoderValue;
	long oldLeftEncoderValue;

	bool enableSideTof;
	bool targetReached;
	unsigned long timeCommandAsserv;
};




/*
Handling encoder 

These function are not part of the class as the handle fucntion for interupt
can not be part of a class without beeing static, and as they are changing 
variables, those variables should be static as weel.
A proposed solution could be here : http://www.gammon.com.au/forum/?id=12983
but this look more elegant to me.

Encoder left and right count is opposite as they are inversed on the robot.
when the robot is going forward, one motor is going CW, while the other is CCW
*/

static volatile long EncoderLeftValue = 0;
static volatile long EncoderRightValue = 0;

static void handleEncoderLeft()
{
	if(digitalRead(PIN_ENCODER_L_A) == 1)
	{
		if(digitalRead(PIN_ENCODER_L_B) == 1)
			EncoderLeftValue++;
		else
			EncoderLeftValue--;
	}
	else // PIN_ENCODER_L_A = 0
	{
		if(digitalRead(PIN_ENCODER_L_B) == 1)
			EncoderLeftValue--;
		else
			EncoderLeftValue++;
	}
}

static void handleEncoderRight()
{
	if(digitalRead(PIN_ENCODER_R_A) == 1)
	{
		if(digitalRead(PIN_ENCODER_R_B) == 1)
			EncoderRightValue--;
		else
			EncoderRightValue++;
	}
	else // PIN_ENCODER_L_A = 0
	{
		if(digitalRead(PIN_ENCODER_R_B) == 1)
			EncoderRightValue++;
		else
			EncoderRightValue--;
	}
}