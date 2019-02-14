#include "Numerobis.h"

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
	BATTLE_OVER
};

state currentState = 0;
Numerobis MyRobot;
bool beginFightFlag = false;


void setup() {
	// put your setup code here, to run once:

	MyRobot.init();
}

bool lineHasBennDetected = false;

void loop() {
	
	MyRobot.updateSensor();
	//MyRobot.debug();

	// Condition prior to the strategy
	
	// TODO if there is a stop condition 
	// if(beginFightFlag && condition to stop)
	// { currentState = BATTLE_OVER; }

	// TODO if there is a line detected
	// currentState = STOP_AND_ROTATE;


	switch(currentState)
	{
		// general entry of the state machine,
		// come back to this point if anything is wrong or the robot lost
		case ENTRY_STATE_MACHINE:
			// default option
			currentState = WAITING_START;
			break;

		// listening to the start condition until it has been triggered
		// variable should be set when fight has begun
		// this should also be setup depending of the begin signal button or remote
		case WAITING_START: 
			// if(...)
			// currentState = BEGIN_FIGHT;
			// beginFightFlag = true;
			break;

		case BEGIN_FIGHT: 
			break;

		// depending of the current position of the enemy rush for punch
		// either right, left or back,
		// if not detected on the side, Uturn and Punch !
		case FIRST_PUNCH :
			break;
		
		// rotating until detection
		case SEARCHING_ENEMY :
			break;

		// if enemy detected in front go push, if on the side, rotate and then push
		case ENEMY_DETECTED :
			break;
		
		// pushing enemy as long as the line is not detected
		// or enemy not detecting anymore
		case PUSHING_ENEMY :
			break;
		
		// when a line is detected this state should be set
		// the robot should do go backward, Uturn and go back to the middle
		// this should also depend of with detector has seen the line
		case STOP_AND_ROTATE :
			break;

		// when stop command has been received
		// this should be blocking unitl reset
		case BATTLE_OVER :
			while(1) {}
			break;

		// should never be called, but just in case
		default : 
			currentState = ENTRY_STATE_MACHINE;
			break;
	}

}
