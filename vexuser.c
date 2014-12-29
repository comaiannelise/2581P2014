/*******************************************************************************
 * @mainpage
 * This is the program for the BCAMSC Robotics Team 2581P 2014 competition
 * robot.
 *
 * @details
 * This program is the primary file for the BCAMSC Robotics Team 2581P program
 * for the 2014-2015 school year.
 *
 * This program is based on a template by James Pearman in his "C on Vex"
 * library, V 1.00.
 *
 * @author James Pearman
 * @author Ethan Ruffing <ruffinge@mail.gvsu.edu>
 *
 * @since 2014-12-22
 ******************************************************************************/
#include <stdlib.h>

#include "ch.h"  		// needs for all ChibiOS programs
#include "hal.h" 		// hardware abstraction layer header
#include "vex.h"		// vex library header

#define motBackRight         kVexMotor_1
#define motFrontRight        kVexMotor_2
#define motLiftOne           kVexMotor_3
#define motLiftTwo           kVexMotor_4
#define motLiftThree         kVexMotor_5
#define motLiftFour          kVexMotor_6
#define motFrontLeft         kVexMotor_7
#define motBackLeft          kVexMotor_8
#define motRotateOne         kVexMotor_9
#define motRotateTwo         kVexMotor_10

#define firstJumper		  kVexDigital_1
#define secondJumper	  kVexDigital_2
#define tenthJumper		  kVexDigital_10

 #define PI  3.14




// Digi IO configuration
static  vexDigiCfg  dConfig[kVexDigital_Num] = {

        { kVexDigital_1,    kVexSensorDigitalInput,  kVexConfigOutput,      0 },
        { kVexDigital_2,    kVexSensorDigitalInput,  kVexConfigOutput,      0 },
        { kVexDigital_3,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_4,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_5,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_6,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_7,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_8,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_9,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_10,   kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_11,   kVexSensorDigitalInput,  kVexConfigQuadEnc1,    kVexQuadEncoder_1 },  //Annelise - Why are these configured for encoders?   
        { kVexDigital_12,   kVexSensorDigitalInput,  kVexConfigQuadEnc2,    kVexQuadEncoder_1 }   //I'm sure there's a reason, but why? 
};

static  vexMotorCfg mConfig[kVexMotorNum] = {
        { kVexMotor_1,      kVexMotor393T,           kVexMotorReversed,     kVexSensorIME,         kImeChannel_3 },
        { kVexMotor_2,      kVexMotor393T,            kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_3,      kVexMotor393T,           kVexMotorNormal,     kVexSensorIME,         kImeChannel_1 },
        { kVexMotor_4,      kVexMotor393T,           kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_5,      kVexMotor393T,           kVexMotorReversed,     kVexSensorIME,         kImeChannel_2 },
        { kVexMotor_6,      kVexMotor393T,            kVexMotorReversed,       kVexSensorNone,        0 },
        { kVexMotor_7,      kVexMotor393T,           kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_8,      kVexMotor393T,      kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_9,      kVexMotor393T,           kVexMotorReversed,     kVexSensorIME,         kImeChannel_4 },
        { kVexMotor_10,     kVexMotor393T,      	 kVexMotorNormal,       kVexSensorNone,        0 },
};

/**
 * @brief Functions
 * @details
 * All user function's are stored here
 */

/*
 *This function uses analog joysticks to drive the robot. 
 *
 *@author Alex Miller <alexmiller965@gmail.com>
 *@since 2014-12-21
 *
 *@param[in] ch1 
 *	Controls strafing on the x-axis
 *@param[in] ch2
 *	Controls forward and backward movement; y axis
 *@param[in] ch4
 *	Controls rotation around the z-axis. 
 */
void moveFunc(int ch1, int ch2, int ch4) 
{
	vexMotorSet(motBackRight,   ch1 - ch2 + ch4);	
	vexMotorSet(motFrontRight, -ch1 - ch2 + ch4);	
	vexMotorSet(motBackLeft,   -ch1 - ch2 - ch4);	
	vexMotorSet(motFrontLeft,   ch1 - ch2 - ch4);	
}

/*
 *This stops all motors on the robot. 
 *@since 2014-12-21
 
void stopMotors(void)	
{
	vexMotorSet(motClaw, 0);
	vexMotorSet(motExtender, 0);
	vexMotorSet(motForeArm, 0);
	vexMotorSet(motRotateTwo, 0);
	vexMotorSet(motRotateOne, 0);
	vexMotorSet(motFrontRight, 0);
	vexMotorSet(motFrontLeft, 0);
	vexMotorSet(motBackRight, 0);
	vexMotorSet(motBackLeft, 0);
}
*/c
/*
 *This controls the extension and retraction of our arm. 
 *
 *@author Alex Miller <alexmiller965@gmail.com>
 *@since 2014-12-21
 *
 *@param[in] extend 
 *	Extends the arm while assigned button is depressed
 *@param[in] retract
 *	Retracts the arm while assigned button is depressed
 */
void
extenderControl(int extend,int retract)
{
	vexMotorSet(motLiftOne, 96 * (extend - retract));
    vexMotorSet(motLiftTwo, 96 * (extend - retract));
    vexMotorSet(motLiftThree, 96 * (extend - retract));
    vexMotorSet(motLiftFour, 96 * (extend - retract));
}

/*
 *This function controls the opening and closing of the claw.
 *
 *@author Annelise Comai <email@here>
 *@since 2014-12-21
 *
 *@param[in] open
 *	Opens up the claw when button is pressed
 *@param[in] close
 *	Closes the claw when button is pressed
 */

 /*
void clawControl(int open,int close)
{
 if(open == 1)
 	{
 	vexMotorSet(motClaw,127);
	}
 else if(close == 1)
 	{
 	vexMotorSet(motClaw,-127);
	}
else {
	vexMotorSet(motClaw,0);
	}
wait(100);
}
*/

/*
 *This function controls the extending  and retracting  of the foot.
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2014-12-21
 *
 *@param[in] extend 
 *	Extends the foot when assigned button is pressed
 *@param[in] retract
 *	Retracts the foot when assigned button is pressed
 */
void footControl(int extend, int retract )
{
if(extend == 1)	
	{
	vexDigitalPinSet(1, kVexDigitalHigh);
	}
else if(retract == 1)	
	{
	vexDigitalPinSet(1, kVexDigitalLow);
	}
}
/**
 * @brief User setup
 * @details
 *  The digital and motor ports can (should) be configured here.
 */
void vexUserSetup()
{
	vexDigitalConfigure( dConfig, DIG_CONFIG_SIZE( dConfig ) );
	vexMotorConfigure( mConfig, MOT_CONFIG_SIZE( mConfig ) );
}

/**
 *  @brief User initialize
 *  @details
 *  This function is called after all setup is complete and communication has
 *  been established with the master processor.
 *  Start other tasks and initialize user variables here
 */
void vexUserInit()
{

}

/**
 * @brief Autonomous
 * @details
 *  This thread is started when the autonomous period is started
 */
int driveConstant = (627)/(2 * PI * 2);      //number of encoder counts per inch (current calculated value shown)
int turnConstant = 7;   					 //number of encoder counts needed to turn 90 degrees 
int liftConstant = 3;  		  				   //number of encoder counts needed to lift the lift from one position to another

/*
 *This function moves the robot forward.
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2014-12-21
 *
 *@param[in] inches
 *	This is the number of inches the robot is supposed to move forward
 @param driveConstant
 	This is the number of encoder counts the encoder measures when the robot goes one inch
 */
void driveForward(float inches)	
{
		vexMotorSet(motFrontLeft, -96);
		vexMotorSet(motBackLeft, -96);
		vexMotorSet(motFrontRight, -96);
		vexMotorSet(motBackRight, -96);

	while(vexMotorPositionGet(motBackRight) < inches * driveConstant || vexMotorPositionGet(motBackRight) > -inches * driveConstant)	
	{
		vexSleep( 25 );
		/*if (vexControllerGet(Btn8D) == 1)	
		{
		stopMotors();
		}*/
	}


		vexMotorSet(motFrontLeft, 0);
		vexMotorSet(motBackLeft, 0);
		vexMotorSet(motFrontRight, 0);
		vexMotorSet(motBackRight, 0);

	vexMotorPositionSet(motFrontRight, 0);
}
	
/*
 *This function also moves the robot forward.
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2014-12-24
 *
 *@param[in] inches
 *	This is the number of inches the robot is supposed to move forward
 @param driveConstant
 	This is the number of encoder counts the encoder measures when the robot goes one inch
 */

void driveForwardInWhileLoop(float inches)
{
	while(vexMotorPositionGet(motBackRight) < inches * driveConstant || vexMotorPositionGet(motBackRight) > -inches * driveConstant)
	{
		vexMotorSet(motFrontLeft, -96);
		vexMotorSet(motBackLeft, -96);
		vexMotorSet(motFrontRight, -96);
		vexMotorSet(motBackRight, -96);
	}
	vexMotorPositionSet(motBackRight, 0);
}
/*
 *This function does a point turn to the left. A point
 *turn keeps the robot in one place while it turns.
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2014-12-21
 */
void pointTurnLeft(void)	
{
	while(vexMotorPositionGet(motFrontRight) < turnConstant)	
	{
		vexMotorSet(motFrontLeft, -96);
		vexMotorSet(motBackLeft, -96);
		vexMotorSet(motFrontRight, 96);
		vexMotorSet(motBackRight, 96);
	}

	vexMotorPositionSet(motFrontRight, 0);
}

/*
 *This function does a point turn to the right. A point
 *turn keeps the robot in one place while it turns.
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2014-12-21
 */
void pointTurnRight(void)	
{
	while(vexMotorPositionGet(motFrontLeft) < turnConstant)	
	{
		vexMotorSet(motFrontLeft, 96);
		vexMotorSet(motBackLeft, 96);
		vexMotorSet(motFrontRight, -96);
		vexMotorSet(motBackRight, -96);
	}
		//if this doesn't work, remove while statement and replace with line below with proper wait time
		// wait(1000);
	vexMotorPositionSet(motFrontLeft, 0);
}


/*
 *This opens the claw when called. 
 *@since 2014-12-21
 
void openClaw(void)	
{
	vexMotorSet(motClaw, 63);
	wait(100);
}
*/
/*
 *This closes the claw when called. 
 *@since 2014-12-21
 
void closeClaw(void)	
{
	vexMotorSet(motClaw, -63);
	wait(100);
}
	*/
/*
 *This untested function was for raising the first arm we built.  With the new chain lift, this function becomes irrelephant.
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2014-12-21
 *
 *@param[in] position
 *	This is the desired position of the lift, either 1, 2, 2.5, or 
 *@param[in] current
 *	This is a variable; it has a value.
 */
 /*
void raiseArm(int position, int current) 
{
	if((position == 2) && (current == 0))	
	{
		while(vexMotorPositionGet(motRotateTwo) < liftConstant)	
		{
			vexMotorSet(motRotateTwo, 96);
			vexMotorSet(motRotateOne, 96);
		}
	}

	else if((position == 3) && (current == 0))	
	{
		while((vexImeGetCount(motRotateTwo)) < (2 * liftConstant))	
		{
			vexMotorSet(motRotateTwo, 96);
			vexMotorSet(motRotateOne, 96);
		}
	}

	else if((position == 3) && (current == 2))	
	{
		while(vexImeGetCount(motRotateTwo) < liftConstant)	
		{
			vexMotorSet(motRotateTwo, 96);
			vexMotorSet(motRotateOne, 96);
		}
	}

	else if(position == 2.5 && current == 2)	
	{
		while(vexImeGetCount(motRotateTwo) < .5 * liftConstant)	
		{
			vexMotorSet(motRotateTwo, 96);
			vexMotorSet(motRotateOne, 96);
		}
	}

	vexMotorPositionSet(motRotateTwo, 0);
}*/

/*
 *This function was for the lowering of the first arm we built.  With the chain lift, this function becomes irrelevant.
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2014-12-21
 *
 *@param[in] position
 *	This is a variable; it has a value.
 *@param[in] current
 *	This is a variable; it has a value.
 */
 /*
void lowerArm(int position, int current)	
{
	if(position == 0 && current == 2)	
	{
		while(vexImeGetCount(motRotateTwo) < -liftConstant)	
		{
			vexMotorSet(motRotateTwo, -96);
			vexMotorSet(motRotateOne, -96);
		}
	}

	else if(position == 0 && current == 3)	
	{
		while(vexImeGetCount(motRotateTwo) < -liftConstant * 2)	
		{
			vexMotorSet(motRotateTwo, -96);
			vexMotorSet(motRotateOne, -96);
		}
	}

	else if(position == 2 && current == 3)	
	{
		while (vexImeGetCount(motRotateTwo) < -liftConstant)	
		{
			vexMotorSet(motRotateTwo, -96);
			vexMotorSet(motRotateOne, -96);
		}
	}

	else if(position == 0 && current == 2.5)	
	{
		while (vexImeGetCount(motRotateTwo) < -1.5 * liftConstant)	
		{
			vexMotorSet(motRotateTwo, -96);
			vexMotorSet(motRotateOne, -96);
		}
	}

	vexImeSetCount(motRotateTwo, 0);
}

*/
msg_t vexAutonomous( void *arg )
{
    (void)arg;

    // Must call this
    vexTaskRegister("auton");

    while(1)
        {
        // Don't hog cpu
        vexSleep( 25 );
        }

    return (msg_t)0;
}

/**
 * @brief Driver Control
 *
 * @details
 *  This thread is started when the driver control period is started
 */
msg_t
vexOperator( void *arg )
{
	int16_t		blink = 0;

	(void)arg;

	// Must call this
	vexTaskRegister("operator");

	// Run until asked to terminate
	while(!chThdShouldTerminate())
		{

		int precision = 1 - vexControllerGet(Btn6U) * 0.25 - vexControllerGet(Btn6D) * 0.5;
		

		//Modifies drive base speed if certain buttons are pressed. Allows for more pewcise movement
		moveFunc( 		vexControllerGet(Ch1) * precision,	//Strafing
						vexControllerGet(Ch2) * precision,	//Forward and backward movement
						vexControllerGet(Ch4) * precision); //Rotational and turning movement
		//Other Movement
		extenderControl(vexControllerGet(Btn5U),	//Extends the arm
						vexControllerGet(Btn5D)	);	//Retracts the arm
		/*
        clawControl(	vexControllerGet(Btn7R),	//Opens the claw
						vexControllerGet(Btn7L)	);	//Closes the claw
		footControl(	vexControllerGet(Btn8L),	//Extends the foot
						vexControllerGet(Btn8R) );	//Retracts the foot
        */

		// Don't hog cpu
		vexSleep( 25 );
		}

	return (msg_t)0;
}
