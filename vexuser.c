/*******************************************************************************
 * @mainpage
 * This is the program for the BCAMSC Robotics Team 2581B 2014 competition
 * robot.
 *
 * @details
 * This program is the primary file for the BCAMSC Robotics Team 2581B program
 * for the 2014-2015 school year.
 *
 * This program is based on a template by James Pearman in his "C on Vex"
 * library, V 1.00.
 *
 * @author James Pearman
 * @author Michel Momeyer <strihawk1213@gmail.com>
 * @author Ethan Ruffing <ruffinge@mail.gvsu.edu>
 *
 * @since 2014-12-22
 ******************************************************************************/
#include <stdlib.h>
#include <math.h>

#include "ch.h"  		// needs for all ChibiOS programs
#include "hal.h" 		// hardware abstraction layer header
#include "vex.h"		// vex library header
#include "vexuser.h"    // function declarations
#include "apollo.h"     // the library that includes the apollo debug window in screen
#include "smartmotor.h" // the library for smart motors

//Motors Declaration

#define BASE_NW  kVexMotor_2  //Front Left Drive Motor
#define BASE_NE  kVexMotor_3  //Front Right Drive Motor
#define BASE_SE  kVexMotor_4  //Back Right Drive Motor
#define BASE_SW  kVexMotor_1  //Back Left Drive Motor

#define LIFT_1   kVexMotor_7  //Front Left Lift Motor
#define LIFT_2   kVexMotor_8  //Front Right Lift Motor
#define LIFT_3   kVexMotor_9 //Upper Lift Motor


#define SHUTTLE  kVexMotor_5  //controls claw shuttle
#define CLAW     kVexMotor_6  //actuates claw

//Other Preprocessor Declaration

 #define PI (3.14159265359)
 #define wheelConstant (1)

// Digi IO configuration
static  vexDigiCfg  dConfig[kVexDigital_Num] = {
        { kVexDigital_1,    kVexSensorQuadEncoder,   kVexConfigQuadEnc1,    kVexQuadEncoder_1 },
        { kVexDigital_2,    kVexSensorQuadEncoder,   kVexConfigQuadEnc2,    kVexQuadEncoder_1 },
        { kVexDigital_3,    kVexSensorQuadEncoder,   kVexConfigQuadEnc1,    kVexQuadEncoder_2 },
        { kVexDigital_4,    kVexSensorQuadEncoder,   kVexConfigQuadEnc2,    kVexQuadEncoder_2 },
        { kVexDigital_5,    kVexSensorQuadEncoder,   kVexConfigQuadEnc1,    kVexQuadEncoder_3 },
        { kVexDigital_6,    kVexSensorQuadEncoder,   kVexConfigQuadEnc2,    kVexQuadEncoder_3 },
        { kVexDigital_7,    kVexSensorQuadEncoder,   kVexConfigQuadEnc1,    kVexQuadEncoder_4 },
        { kVexDigital_8,    kVexSensorQuadEncoder,   kVexConfigQuadEnc2,    kVexQuadEncoder_4 },
        { kVexDigital_9,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_10,   kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_11,   kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_12,   kVexSensorDigitalInput,  kVexConfigInput,       0 }
};

static  vexMotorCfg mConfig[kVexMotorNum] = {
        { BASE_SW,      kVexMotor393T,     kVexMotorNormal,       kVexSensorQuadEncoder,        kVexQuadEncoder_4 },
        { BASE_NW,      kVexMotor393T,     kVexMotorNormal,       kVexSensorQuadEncoder,        kVexQuadEncoder_1 },
        { BASE_NE,      kVexMotor393T,     kVexMotorNormal,       kVexSensorQuadEncoder,        kVexQuadEncoder_2 },
        { BASE_SE,      kVexMotor393T,     kVexMotorReversed,     kVexSensorQuadEncoder,        kVexQuadEncoder_3 },
        { SHUTTLE,      kVexMotor393T,     kVexMotorNormal,       kVexSensorIME,                kImeChannel_5 },
        { CLAW,         kVexMotor393T,     kVexMotorNormal,       kVexSensorNone,               0 },
        { LIFT_1,       kVexMotor393T,     kVexMotorReversed,     kVexSensorIME,                kImeChannel_3 },
        { LIFT_2,       kVexMotor393T,     kVexMotorNormal,       kVexSensorIME,                kImeChannel_4 },
        { LIFT_3,       kVexMotor393T,     kVexMotorNormal,       kVexSensorIME,                kImeChannel_1 },
        { kVexMotor_10, kVexMotor393T,     kVexMotorNormal,       kVexSensorIME,                kImeChannel_6 }
};

// Input Variables

int vertical;
int horizontal;
int spin;
int liftSpeed;
int clawPosition;
int shuttleSpeed = 127;

//Limits and Acceleration djustments

int deadZone = 15;
int maxAVert = 5;
int maxAHoriz = 1;
int maxASpin = 3;

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
 * @brief User initialize
 * @details
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
msg_t vexOperator( void *arg )
{
	(void)arg;

	// Must call this
	vexTaskRegister("operator");

	// Run until asked to terminate
	while(!chThdShouldTerminate())
	{

        updateInput();
        setMotors();

        //Temporary Shuttle Code
        if(vexControllerGet(Btn6U))
        {
            vexMotorSet(SHUTTLE,shuttleSpeed);
        }
        else if(vexControllerGet(Btn6D))
        {
            vexMotorSet(SHUTTLE,-shuttleSpeed);
        }
        else
        {
            vexMotorSet(SHUTTLE,0);
        }

		// Don't hog cpu
		vexSleep( 25 );
	}

	return (msg_t)0;
}

/*
 * This function determines and returns an acceptable increment to the given 
 *     power level based on user input.
 */
int getPowerIncrement(int oldValue, int input, int deadZone, int maxAcceleration)
{
    if(input > deadZone || input < -deadZone)
    {
        if(input > oldValue + maxAcceleration) 
        {
            return maxAcceleration;
        }
        else if(input < oldValue - maxAcceleration) 
        {
            return -maxAcceleration;
        }
        return input;
    }
    return -oldValue;
}

/*
 * This function takes the controller input and modify values for use.
 */
 void updateInput(void)
 {
    //Movement input adjusted for deadzone and maximum acceleration
    vertical += getPowerIncrement(vertical,vexControllerGet(Ch3),deadZone,maxAVert);
    horizontal += getPowerIncrement(horizontal,vexControllerGet(Ch4),deadZone,maxAHoriz);
    spin += getPowerIncrement(spin,vexControllerGet(Ch1),deadZone,maxASpin);
    
    //Lift input adjusted for deadzone FIX
    //liftSpeed += getPowerIncrement(liftSpeed,vexControllerGet(Ch2),deadZone,liftSpeed - vexControllerGet(Ch2));

    if(vexControllerGet(Ch2) > deadZone || vexControllerGet(Ch2) < -deadZone)
    {
        liftSpeed = vexControllerGet(Ch2);
    }
    else
    {
        liftSpeed = 0;
    }


    //Add Claw and Shuttle input modifying code goes here...

 }

/*
 * This function sets all the motors for driver control based.
 * 
 */
void setMotors(void)
{
    //Setting Base Drive Motors
    vexMotorSet(BASE_NW, spin + vertical + horizontal);
    vexMotorSet(BASE_NE, spin - vertical + horizontal);
    vexMotorSet(BASE_SE, spin - vertical - horizontal);
    vexMotorSet(BASE_SW, spin + vertical - horizontal);

    //Setting Lift Motors
    vexMotorSet(LIFT_1,liftSpeed);
    vexMotorSet(LIFT_2,liftSpeed);
    vexMotorSet(LIFT_3,liftSpeed);

    //Additional lift motors go here

    //Add Claw and Shuttle motor code here...

}

/*
 * This function drives forward a certain distance autonomously.
 */
void setAutonMotor(int motor, int dist, int speed)
 {
    //Convert distance from feet to encoder counts
    dist *= wheelConstant;

    //Set motor to speed
    vexMotorSet(motor,speed);

    //Checks which direction the motor is going
    if(vexMotorDirectionGet(motor))
    {
        //Determine when to stop
        if(vexMotorPositionGet(motor) > dist)
        {
            //Stop the motor
            vexMotorSet(motor,0); 
        }
    }
    else
    {
        //Determine when to stop
        if(vexMotorPositionGet(motor) < -dist)
        {
            //Stop the motor
            vexMotorSet(motor,0);
        }
    }
 }