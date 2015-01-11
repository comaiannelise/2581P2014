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
 * @author Alex Miller <alexmiller965@gmail.com>
 * @author Annelise Comai <anneliesecomai@gmail.com>
 * @author Ethan Ruffing <ruffinge@mail.gvsu.edu>
 *
 *
 * @since 2014-12-22
 ******************************************************************************/
#include <stdlib.h>

#include "ch.h"  		// needs for all ChibiOS programs
#include "hal.h" 		// hardware abstraction layer header
#include "vex.h"		// vex library header

#define motBackRight       kVexMotor_1      //Weirdly enough, this IME counts negative
#define motFrontRight      kVexMotor_2
#define motLiftOne         kVexMotor_3      //Assuming that this has the only IME on the lift
#define motLiftTwo         kVexMotor_4
#define motLiftThree       kVexMotor_5
#define motLiftFour        kVexMotor_10
#define motFrontLeft       kVexMotor_6
#define motClaw            kVexMotor_8
#define motBackLeft        kVexMotor_7

#define firstJumper		  kVexDigital_1
#define secondJumper	  kVexDigital_2

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
        { kVexDigital_11,   kVexSensorDigitalInput,  kVexConfigQuadEnc1,    kVexQuadEncoder_1 },  
        { kVexDigital_12,   kVexSensorDigitalInput,  kVexConfigQuadEnc2,    kVexQuadEncoder_1 } 
};

static  vexMotorCfg mConfig[kVexMotorNum] = {
        { kVexMotor_1,      kVexMotor393T,           kVexMotorReversed,     kVexSensorIME,         kImeChannel_1 },
        { kVexMotor_2,      kVexMotor393T,           kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_3,      kVexMotor393T,           kVexMotorNormal,       kVexSensorIME,         kImeChannel_4 },
        { kVexMotor_4,      kVexMotor393T,           kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_5,      kVexMotor393T,           kVexMotorReversed,     kVexSensorNone,        0 },
        { kVexMotor_6,      kVexMotor393T,           kVexMotorReversed,     kVexSensorIME,         kImeChannel_3 },
        { kVexMotor_7,      kVexMotor393T,           kVexMotorReversed,     kVexSensorIME,         kImeChannel_2 },
        { kVexMotor_8,      kVexMotor393T,           kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_9,      kVexMotor393T,           kVexMotorReversed,     kVexSensorNone,        0 },
        { kVexMotor_10,     kVexMotor393T,      	 kVexMotorNormal,       kVexSensorNone,        0 },
};

/**
 * @brief Functions
 * @details
 * All user functions are stored here
 */

 //Autonomous Functions

int driveConstant = (627)/(2 * PI * 2) - 10;      //number of encoder counts per inch (current calculated value shown)
int turnConstant = -696;                        //number of encoder counts needed to turn 90 degrees  - left, at least
int liftConstant = 3;                        //number of encoder counts needed to lift the lift from one position to the next

/*
 *This function moves the robot forward.
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2014-12-21
 *
 *@param[in] inches
 *  This is the number of inches the robot is supposed to move forward
 @param driveConstant
    This is the number of encoder counts the encoder measures when the robot goes one inch
 */
void driveForward(float inches) 
{
        vexMotorSet(motFrontLeft, 96);
        vexMotorSet(motBackLeft, 96);
        vexMotorSet(motFrontRight, 96);
        vexMotorSet(motBackRight, 96);

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
 *  This is the number of inches the robot is supposed to move forward
 @param driveConstant
    This is the number of encoder counts the encoder measures when the robot goes one inch
 */

void driveForwardInWhileLoop(float inches)
{
    vexMotorPositionSet(motBackRight, 0);
    while(vexMotorPositionGet(motBackRight) < inches * driveConstant)
    {
        vexMotorSet(motFrontLeft, -96);
        vexMotorSet(motBackLeft, -96);
        vexMotorSet(motFrontRight, -96);
        vexMotorSet(motBackRight, -96);
    }
        vexMotorSet(motFrontLeft, 0);
        vexMotorSet(motBackLeft, 0);
        vexMotorSet(motFrontRight, 0);
        vexMotorSet(motBackRight, 0);
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
    while(vexMotorPositionGet(motBackRight) < turnConstant)    
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
 */
 
void openClaw(void) 
{
    vexMotorSet(motClaw, 63);
   // wait(100);
}

/*
 *This closes the claw when called. 
 *@since 2014-12-21
*/ 

void closeClaw(void)    
{
    vexMotorSet(motClaw, -63);
   // wait(100);
}



/*
*This function will somehow raise the lift.
*
*@author Annelise Comai <anneliesecomai@gmail.com>
*@since 2014-12-29
*
*@param[in] middle
*   Set to 1 if lift is passing through or landing at the middle lift position
*@param[in] high
    Set to 1 if lift is landing at the high lift position

*/

void raiseLift(int middle, int high)
{

        while(vexMotorPositionGet(motLiftOne) < liftConstant * (middle + high))
        {
            vexMotorSet(motLiftOne,   127);
            vexMotorSet(motLiftTwo,   127);
            vexMotorSet(motLiftThree, 127);
            vexMotorSet(motLiftFour,  127);
    }
}

/*
*This function will somehow lower the lift.
*
*@author Annelise Comai <anneliesecomai@gmail.com>
*@since 2014-12-29
*
*@param[in] middle
*   Set to 1 if lift is passing through or landing at the middle lift position
*@param[in] low
    Set to 1 if lift is landing at the low lift position

*/

void lowerLift(int middle, int low)
{

        while(vexMotorPositionGet(motLiftOne) > -1 * liftConstant * (middle + low))
        {
            vexMotorSet(motLiftOne,   -127);
            vexMotorSet(motLiftTwo,   -127);
            vexMotorSet(motLiftThree, -127);
            vexMotorSet(motLiftFour,  -127);
    }
}


/********************


//Driver Period Functions



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
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2014-12-21
 */
void stopMotors(void)	
{
	vexMotorSet(motClaw, 0);
	vexMotorSet(motFrontRight, 0);
	vexMotorSet(motBackRight, 0);
	vexMotorSet(motFrontLeft, 0);
	vexMotorSet(motFrontRight, 0);
	vexMotorSet(motLiftOne, 0);
	vexMotorSet(motLiftTwo, 0);
	vexMotorSet(motLiftThree, 0);
	vexMotorSet(motLiftFour, 0);
}

/*
 *This controls the raising and lowering of the chain lift. 
 *
 *@author Alex Miller <alexmiller965@gmail.com>
 *@since 2014-12-29
 *
 *@param[in] raises 
 *	Raises the lift while assigned button is pressed
 *@param[in] lower
 *	Lowers the lift while assigned button is pressed
 */
void
liftControl(int raises, int lower)
{
	vexMotorSet(motLiftOne, 80 * (raises - lower));
    vexMotorSet(motLiftTwo, 80 * (raises - lower));
    vexMotorSet(motLiftThree, 80 * (raises - lower));
    vexMotorSet(motLiftFour, 80 * (raises - lower));
}

/*
 *This function controls the opening and closing of the claw.
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2014-12-21
 *
 *@param[in] open
 *	Opens up the claw when button is pressed
 *@param[in] close
 *	Closes the claw when button is pressed
 */

 
void clawControl(int open,int close)
{
vexMotorSet(motClaw, 127 * (open - close));
//wait(100);   //May not need to be here (except possibly for auton) if previous line works
}


/*
 *This function controls the extending and retracting  of the foot.
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
     vexMotorPositionSet(motBackRight, 0);
}

/**
 * @brief Autonomous
 * @details
 *  This thread is started when the autonomous period is started
 */





msg_t vexAutonomous( void *arg )
{
    (void)arg;

    // Must call this
    vexTaskRegister("auton");

        driveForwardInWhileLoop(24);
        openClaw();
        
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

	(void)arg;

	// Must call this
	vexTaskRegister("operator");

    pointTurnLeft();

	// Run until asked to terminate
	while(!chThdShouldTerminate())
		{

		int precision = 1 - vexControllerGet(Btn6U) * 0.25 - vexControllerGet(Btn6D) * 0.5;
		

		//Modifies drive base speed if certain buttons are pressed. Allows for more pewcise movement
		moveFunc( 		vexControllerGet(Ch1) * precision,	//Strafing
						vexControllerGet(Ch2) * precision,	//Forward and backward movement
						vexControllerGet(Ch4) * precision); //Rotational and turning movement
		//Other Movement
		liftControl(    vexControllerGet(Btn5U),	    //Lifts the lift
						vexControllerGet(Btn5D)	);	    //Lowers the lift
		
        clawControl(	vexControllerGet(Btn8R),	//Opens the claw
						vexControllerGet(Btn8L)	);	//Closes the claw

        /*
		footControl(	vexControllerGet(Btn8L),	//Extends the foot
						vexControllerGet(Btn8R) );	//Retracts the foot
        */

		// Don't hog cpu
		vexSleep( 25 );
		}

	return (msg_t)0;
}
