#include "Numerobis.h"

#include <avr/wdt.h>

#define BLINK_TIMING blinkTiming
// states 
enum state
{
	ENTRY_STATE_MACHINE,
	WAITING_START ,
	BEGIN_FIGHT ,
	FIRST_PUNCH ,
	SEARCHING_ENEMY ,
	ENEMY_DETECTED ,
	PUSHING_ENEMY ,
	STOP_AND_ROTATE ,
	BATTLE_OVER,
	DEBUG_MODE,
	ESCAPE_LINE_LEFT,
	ESCAPE_LINE_RIGHT,
	BACK_CENTER
};

enum enemyPos
{
	enemyOnLeft = 1,
	enemyOnRight,
	enemyOnBack
};

state currentState = 0;
Numerobis MyRobot;
bool buttonHasBeenPushed = false;
bool beginFightFlag = false;
bool valLed = true;
unsigned long timeButtonHasBeenPushed = 0;
unsigned long timeLastBlink = 0;
bool tofHaveBeenStopped = false;
int EnemyInitPos = 0;
int blinkTiming = 500;


void setup() {
	// put your setup code here, to run once:
	Serial.begin(115200);
	Serial.println("begin init");
	MyRobot.init();
	Serial.println("init done");

	//MyRobot.setTargetEncoder(500,500);
	//MyRobot.setTargetAsserv(0,0);
	MyRobot.enableAsserv(true);
}

bool lineHasBennDetected = false;

void loop() {
	
	MyRobot.updateSensor();
	//MyRobot.debug();
	//MyRobot.processAsserv();
	MyRobot.processAsservEnhanced();
	blink();

	// Condition prior to the strategy
	
	// TODO if there is a stop condition > for nimes ! 
	// if(beginFightFlag && condition to stop)
	// { currentState = BATTLE_OVER; }

	// TODO if there is a line detected
	if(MyRobot.getSensorValue(QTR1ALeft) < 400 && buttonHasBeenPushed)
		{
			//MyRobot.setSpeed(0,0);
			currentState = ESCAPE_LINE_LEFT;
			MyRobot.setTargetAsserv(-2000,0);	
		}
	if(MyRobot.getSensorValue(QTR1ARight) < 400 && buttonHasBeenPushed)
		{
			//MyRobot.setSpeed(0,0);
			currentState = ESCAPE_LINE_RIGHT;
			MyRobot.setTargetAsserv(-2000,0);
		}

	switch(currentState)
	{
		// general entry of the state machine,
		// come back to this point if anything is wrong or the robot lost
		case ENTRY_STATE_MACHINE:
			Serial.print("Begin state machine. ");
			// default option
			currentState = WAITING_START;
			break;

		// listening to the start condition until it has been triggered
		// variable should be set when fight has begun
		// this should also be setup depending of the begin signal button or remote
		case WAITING_START: 
			
			if(!buttonHasBeenPushed)
			{
				if(MyRobot.getSensorValue(Interface) < 512) 
					{
						buttonHasBeenPushed = true;
						timeButtonHasBeenPushed = millis();
						Serial.print("Button Pushed. ");
						blinkTiming = 50;
					}
			}

			if(buttonHasBeenPushed && !beginFightFlag)
			{
				// check time
				if(millis() - timeButtonHasBeenPushed > 4000 && !tofHaveBeenStopped)
				{
					MyRobot.stopSideTof();
					tofHaveBeenStopped = true;
					Serial.print("Stopping ToF. ");
				}

				if(millis() - timeButtonHasBeenPushed > 5000)
					{
						beginFightFlag = true;
					}
			}

			if(beginFightFlag)
			{
				// Record last tof value
				// stop TOF side
				int leftDistance = MyRobot.getSensorValue(TofLeft);
				int rightDistance = MyRobot.getSensorValue(TofRight);
				Serial.print(leftDistance);
				Serial.print(" ");
				Serial.print(rightDistance);
				Serial.print(" ");
				
				if(leftDistance < 300)
				{
					EnemyInitPos = enemyOnLeft;
					MyRobot.setTargetAsserv(0,1000);
					Serial.print("Enemy on left. ");
				}
				else if(rightDistance < 300)
				{
					EnemyInitPos = enemyOnRight;
					MyRobot.setTargetAsserv(0,-1000);
					Serial.print("Enemy on right. ");
				}
				else
				{
					EnemyInitPos = enemyOnBack;
					MyRobot.setTargetAsserv(0,2000);
					Serial.print("Enemy on back. ");
				}
				// change state from state machine
				currentState = BEGIN_FIGHT;
				blinkTiming = 200;
			}
			break;

		case BEGIN_FIGHT: 
			if(MyRobot.isTargetReached())
			{
				MyRobot.setTargetAsserv(5000,0);
				currentState = FIRST_PUNCH;
				Serial.print("Fight. ");
			
			}

			/*if(millis() - timeButtonHasBeenPushed > 6500)
			{
				MyRobot.setTargetAsserv(5000,0);
				currentState = FIRST_PUNCH;
			}*/

			break;

		// depending of the current position of the enemy rush for punch
		// either right, left or back,
		// if not detected on the side, Uturn and Punch !
		case FIRST_PUNCH :
			if(MyRobot.getSensorValue(TofMiddle) > 200)
			{
				currentState = SEARCHING_ENEMY;
			}
			else
			{
				currentState = PUSHING_ENEMY;
			}
			break;
		
		// rotating until detection
		case SEARCHING_ENEMY :
			if(MyRobot.getSensorValue(TofMiddle) > 200)
			{
				MyRobot.setSpeed(50,-50);
			}
			else
			{
				currentState = PUSHING_ENEMY;
			}
			break;

		// if enemy detected in front go push, if on the side, rotate and then push
		case ENEMY_DETECTED :
			break;
		
		// pushing enemy as long as the line is not detected
		// or enemy not detecting anymore
		case PUSHING_ENEMY :
			if(MyRobot.getSensorValue(TofMiddle) > 300)
			{
				currentState = SEARCHING_ENEMY;
			}
			else
			{
				MyRobot.setSpeed(250,250);
			}
			break;
		
		// when a line is detected this state should be set
		// the robot should do go backward, Uturn and go back to the middle
		// this should also depend of with detector has seen the line
		case STOP_AND_ROTATE :
			break;
	
		case ESCAPE_LINE_LEFT : 
			// MyRobot.setTargetAsserv(-200,200);	
			if(MyRobot.isTargetReached())
			{
				currentState = BACK_CENTER;
				//MyRobot.setTargetAsserv(1000,0);
			}	
			break;

		case ESCAPE_LINE_RIGHT :
			// MyRobot.setTargetAsserv(-200,-200);
			if(MyRobot.isTargetReached())
			{
				currentState = BACK_CENTER;
				//MyRobot.setTargetAsserv(1000,0);
			}

			break;

		case BACK_CENTER : 

				currentState = SEARCHING_ENEMY;
				MyRobot.setSpeed(50,-50);

			break;
	
		// when stop command has been received
		// this should be blocking unitl reset
		case BATTLE_OVER :
			while(1) {}
			break;

		// out of the strategy, only used to test stuffs
		case DEBUG_MODE :

			break;

				
		// should never be called, but just in case
		default : 
			currentState = ENTRY_STATE_MACHINE;
			break;
	}

	//Serial.print("State : ");
	//Serial.print(currentState);
	//Serial.println("");

	if(Serial.available())
	{
		char cmd = Serial.read();

		switch(cmd)
		{
		case 'a' :
			MyRobot.setTargetAsserv(500,0);
			break;
		case 'z' :
			MyRobot.setTargetAsserv(0,500);
			break;
		case 'e' :
			MyRobot.setTargetAsserv(0,-500);
			break;
		case 'r' :
			MyRobot.setTargetAsserv(-500,0);
			break;
		case 'd' :
			MyRobot.debug();
			break;
		case 's' :
			Serial.print("state is : ");
			Serial.println(currentState);
			break;
		default:
			break;
		}
	}



}



void blink()
{
	if(millis()-timeLastBlink > BLINK_TIMING)
	{
		MyRobot.setLed(valLed);
		valLed = !valLed;
		timeLastBlink = millis();
	}
}

