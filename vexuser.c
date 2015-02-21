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
 * @author Alex Miller <amm@albion.edu>
 * @author Annelise Comai <anneliesecomai@gmail.com>
 * @author Ethan Ruffing <ruffinge@mail.gvsu.edu>
 *
 * @since 2014-12-22
 ******************************************************************************/
#include <stdlib.h>
#include "ch.h"         // needs for all ChibiOS programs
#include "hal.h"        // hardware abstraction layer header
#include "vex.h"        // vex library header
#include "robotc_glue.h"

// These alias the motor ports to what the motor does on the robot.
// Drive Base Motors
#define MOT_BACK_RIGHT       kVexMotor_1      
#define MOT_FRONT_RIGHT      kVexMotor_2
#define MOT_FRONT_LEFT       kVexMotor_6
#define MOT_BACK_LEFT        kVexMotor_7
// Chain Lift Motors
#define MOT_LIFT_ONE         kVexMotor_3      
#define MOT_LIFT_TWO         kVexMotor_4
#define MOT_LIFT_THREE       kVexMotor_5
#define MOT_LIFT_FOUR        kVexMotor_10
// Claw motor and the lift the claw is chained to.
#define MOT_CLAW             kVexMotor_8
#define MOT_CLAW_LIFT        kVexMotor_9

// Autonmous Jumpers
#define JUMPER_ONE        kVexDigital_7
#define JUMPER_TWO        kVexDigital_8

// Ultrasonic sensor used for preauton positioning
#define SONAR_LEFT          kVexSonar_1
#define SONAR_RIGHT         kVexSonar_2

// Digi IO configuration
static  vexDigiCfg  dConfig[kVexDigital_Num] = {

        { kVexDigital_1,    kVexSensorSonarCm,       kVexConfigSonarIn,     kVexSonar_1 },
        { kVexDigital_2,    kVexSensorSonarCm,       kVexConfigSonarOut,    kVexSonar_1 },
        { kVexDigital_3,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_4,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_5,    kVexSensorSonarCm,       kVexConfigSonarIn,     kVexSonar_2 },
        { kVexDigital_6,    kVexSensorSonarCm,       kVexConfigSonarOut,    kVexSonar_2 },
        { kVexDigital_7,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_8,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_9,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_10,   kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_11,   kVexSensorDigitalInput,  kVexConfigQuadEnc1,    kVexQuadEncoder_1 },  
        { kVexDigital_12,   kVexSensorDigitalInput,  kVexConfigQuadEnc2,    kVexQuadEncoder_1 } 
};
/*
 * General Guide for how reversing is set up.
 * Positive values go forward on a drive base
 * Up on a lift system.
 * Open on a claw or intake mechanism.
 * Motors are reversed to match this idea.
 */

static  vexMotorCfg mConfig[kVexMotorNum] = {
        { kVexMotor_1,      kVexMotor393T,           kVexMotorNormal,       kVexSensorIME,         kImeChannel_1 },
        { kVexMotor_2,      kVexMotor393T,           kVexMotorReversed,     kVexSensorNone,        0 },
        { kVexMotor_3,      kVexMotor393T,           kVexMotorReversed,     kVexSensorIME,         kImeChannel_3 },
        { kVexMotor_4,      kVexMotor393T,           kVexMotorReversed,     kVexSensorNone,        0 },
        { kVexMotor_5,      kVexMotor393T,           kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_6,      kVexMotor393T,           kVexMotorNormal,       kVexSensorIME,         0 },
        { kVexMotor_7,      kVexMotor393T,           kVexMotorNormal,       kVexSensorIME,         kImeChannel_2 },
        { kVexMotor_8,      kVexMotor393T,           kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_9,      kVexMotor393T,           kVexMotorNormal,       kVexSensorNone,        0 },
        { kVexMotor_10,     kVexMotor393T,           kVexMotorNormal,       kVexSensorNone,        0 },
};

// Autonmous Constants for unit conversion.
const int DRIVE_CONSTANT = 39.5;   //Encoder to inch
const int TURN_CONSTANT = 675;     //Encoder to degree
const int LIFT_CONSTANT = 300;     //Encoder to pseudounits

bool clawOpen = false;
float liftSetpoint = 0;

// Look Up Table for Motor Values
// Used for the purpose of linearzing motor speed.
const unsigned int TRUESPEED[128] = {
	0, 0, 0, 0, 0, 15,16,17,18,19,
	20,21,21,21,22,22,22,23,24,24,
	25,25,25,25,26,27,27,28,28,28,
	28,29,30,30,30,31,31,32,32,32,
	33,33,34,34,35,35,35,36,36,37,
	37,37,37,38,38,39,39,39,40,40,
	41,41,42,42,43,44,44,45,45,46,
	46,47,47,48,48,49,50,50,51,52,
	52,53,54,55,56,57,57,58,59,60,
	61,62,63,64,65,66,67,67,68,70,
	71,72,72,73,74,76,77,78,79,79,
	80,81,83,84,84,86,86,87,87,88,
	88,89,89,90,127,127,127
};

// Important Utility Functions

/**
 *This function returns the sign of any value.
 *
 *@author Alex Miller <amm@albion.edu>
 *@since 2015-02-10
 *
 *@param[in] value
 *   The value from which the sign will be taken
 */

int signOf(int value)
{
	if (value >= 0) return 1;
	else return -1;
}

/**
 *This function takes the motor input and linearizes it according to the TRUESPEED look up table
 *
 *@author Alex Miller <amm@albion.edu>
 *@since 2015-02-10
 *
 *@param[in] motor
 *   The function will set this motor to the power.
 *@param[in] power
 *   The linearized power to set the motor to.
 */

void motorSet(int motor,int power)
{
	//makes sure that the motors are never being set to more or less than +-127
	if (power > 127) power = 127;
	if (power < -127) power = -127;

	//Abs values are so that the function/motor speed retains its sign (since TRUESPEED values are all positive)
	//The "- 1" is to match the code to the array
	vexMotorSet(motor,signOf(power) * TRUESPEED[abs(power) - 1]);
}

/**
 *This function caps off the integral error so that the error will never be more than maxValue,
 *preventing robot from trying to compensate for impossible amounts of error
 *
 *@author Alex Miller <amm@albion.edu>
 *@since 2015-02-10
 *
 *@param[in] value
 *   The current, unadjusted error value
 *@param[in] maxValue
 *   The maximum value that the error will never go over
 */

float cap(float value, float maxValue){
	if (abs(value) > abs(maxValue)) return signOf(value) * abs(maxValue);
	else return value;
}



/**
 * @file vexuser.c
 * @brief Functions
 * @details
 * All user functions are stored here
 */



/**
 *This function is placed as a condition in every while loop to allow the user to escape an infinite loop.
 *
 *@author Alex Miller <amm@albion.edu>
 *@since 2015-02-03
 */

bool escapeTime(void)
{

    if ((vexControllerGet(Btn7U) == 1) &&
        (vexControllerGet(Btn7D) == 1) &&
        (vexControllerGet(Btn8D) == 1) &&
        (vexControllerGet(Btn5U) == 1) &&
        (vexControllerGet(Btn6U) == 1))
    {
        return true;
    }
    else
	{
        return false;
	}
}

/**
 *This function sets all base motors to 0.
 *
 *@author Alex Miller <amm@albion.edu>
 *@since 2015-01-29
 */

void stopBase(void)
{
    vexMotorSet(MOT_FRONT_LEFT, 0);
    vexMotorSet(MOT_BACK_LEFT, 0);
    vexMotorSet(MOT_FRONT_RIGHT, 0);
    vexMotorSet(MOT_BACK_RIGHT, 0);
}

/**
 *This function sets all the lift motors to a certain speed.  Used in raiseLift and 
 *lowerLift functions, as well as the vexLiftThread task.
 *
 *@author Alex Miller <amm@albion.edu>
 *@since 2015-02-03
 *
 *@param[in] power
 *   The power to set all of the lift motors to.
 */

void liftMotorSet(int power)
{
    vexMotorSet(MOT_LIFT_ONE,   power);
    vexMotorSet(MOT_LIFT_TWO,   power);
    vexMotorSet(MOT_LIFT_THREE, power);
    vexMotorSet(MOT_LIFT_FOUR,  power);
}
    
/**
 *This function moves the robot forward or backward using PID.
 *
 *@author Alex Miller <amm@albion.edu>
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2014-12-24
 *
 *@param[in] inches
 *  The number of inches the robot should move forward or backward
 */

void autonDrive(float inches)
{
	//Zeroing the encoders prevent prior movement from interfering.	
    vexMotorPositionSet(MOT_BACK_RIGHT, 0);
    vexMotorPositionSet(MOT_BACK_LEFT, 0);

	const float pValue = 0.55;                              //These values converts the error into workable units
	const float iValue = 0.9; 
	const float dValue = 3;
	float error, speed,iError = 0;                          //variable delarations
	float wasHere = 0;

	int i = 1;                                              //i is the number of times the drive has been in position/number 
                                                            //of times the robot has hit the correct location
	bool hitTarget = false;		                            //true or false depending on if robot has hit target destination
	float targetDistance = abs(DRIVE_CONSTANT * inches);	//converts inches into encoder counts
	int rightEncoder; 						               	//declares right and left encoders
	
	
    while(!hitTarget && !(escapeTime()) )	//while robot has not hit target and kill switch is not activated
    {

		rightEncoder = abs(vexMotorPositionGet(MOT_BACK_RIGHT) + 1); 
		error = targetDistance - rightEncoder;				//How far off the right encoder's count is from the
                											//targetDistance count
		double dError = rightEncoder - wasHere;
		speed = error * pValue + iError * iValue + dError *dValue;

		if (abs(speed) <= 123) iError += error *0.025;		//iError is the area under error curve (the "current"
		else speed = 123;

		speed *= signOf(inches);							//If desired number of inches is negative, then speed
															//of motors will be (-) for going backward

		//Setting Motors (With Corrections for Straightening)
        motorSet(MOT_FRONT_LEFT, speed + 4 );
        motorSet(MOT_BACK_LEFT,  speed + 4 );
        motorSet(MOT_FRONT_RIGHT,speed + 4 );
        motorSet(MOT_BACK_RIGHT, speed + 4 );

		if(rightEncoder <=  targetDistance + 6 && rightEncoder >= targetDistance -6){	
			i += 1;											//Each time it hits target, 1 is added to i
			if (i > 4) hitTarget = true;					
		}
		else i = 1;											//If robot does not hit target 7 times in a row, i goes

		wasHere = rightEncoder;
		vexSleep(25);
    }

	stopBase();												//Stops base after target is reached
}

/**
 *This basic function makes the robot go forward without PID.
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2015-02-20
 *
 *@param[in] inch
 *   The number of inches the robot should move forward or backward
 */

void goForward(float inch)
{
    //Reset Encoder
    vexMotorPositionSet(MOT_BACK_LEFT, 0);
    
    //Set speed
    int direction = signOf(inch);
    int speed = 60 * direction;

    while(abs(vexMotorPositionGet(MOT_BACK_LEFT)) < abs(inch) )  // * DRIVE_CONSTANT)
    {
        vexMotorSet(MOT_BACK_RIGHT,   speed);   
        vexMotorSet(MOT_FRONT_RIGHT,  speed);   
        vexMotorSet(MOT_BACK_LEFT,    speed);   
        vexMotorSet(MOT_FRONT_LEFT,   speed);   
    }

    vexMotorPositionSet(MOT_BACK_RIGHT, 0);
}

/**
 *This function caps off the integral error so that the error will never be more than maxValue,
 *preventing robot from trying to compensate for impossible amounts of error
 *
 *@author Alex Miller <amm@albion.edu>
 *@since 2015-02-10
 *
 *@param[in] rotation
 *   The amount of rotation the robot should turn
 */

void pointTurnRight(float rotation)
{
    //Reset Encoder
    vexMotorPositionSet(MOT_BACK_LEFT, 0);

    //Set speed
    int direction = signOf(rotation);
    int speed = 96 * direction;

    //Turn until point reached
    while((abs(vexMotorPositionGet(MOT_BACK_LEFT)) < abs(rotation)) && !(escapeTime()) )  
    {
        vexMotorSet(MOT_FRONT_LEFT,  speed);
        vexMotorSet(MOT_BACK_LEFT,   speed);
        vexMotorSet(MOT_FRONT_RIGHT, -speed);
        vexMotorSet(MOT_BACK_RIGHT,  -speed);
    }
}

/**
 *This function does a point turn to the right. A point turn keeps the robot in 
 *one place while it turns.
 *
 *@author Alex Miller <amm@albion.edu>
 *@since 2015-02-12
 *
 *@param[in] degrees
 *  Approximates the degrees turned by the robot
 */

void pointTurnRightAuton(float degrees)    
{
    vexMotorPositionSet(MOT_BACK_RIGHT, 0);				//Resets MOT_BACK_RIGHT encoder

	
	float targetTurn = TURN_CONSTANT * degrees / 90;
	float error = targetTurn - abs(vexMotorPositionGet(MOT_BACK_RIGHT));
	float pValue = 0.6;
    float dir = signOf(degrees);

    while((abs(vexMotorPositionGet(MOT_BACK_RIGHT)) > abs(targetTurn)) && !(escapeTime()) )    
    {
        //int dir = degrees/abs(degrees);
        vexMotorSet(MOT_FRONT_LEFT, pValue * error * dir);
        vexMotorSet(MOT_BACK_LEFT, pValue * error * dir);
        vexMotorSet(MOT_FRONT_RIGHT, -pValue * error * dir);
        vexMotorSet(MOT_BACK_RIGHT, -pValue * error * dir);
    }

    vexMotorPositionSet(MOT_BACK_RIGHT, 0);
}

/**
 *This function will perform a point turn to either the left or right. A point
 *turn keeps the robot in one place while it turns.
 *
 *@author Alex Miller <amm@albion.edu>
 *@since 2015-02-12
 *
 *param[in] inches
 *  Approximates the distance turned in inches
 */

void autonTurn(float inches)
{
	//Zeroing the encoders prevent prior movement from interfering.	
    vexMotorPositionSet(MOT_BACK_RIGHT, 0);
    vexMotorPositionSet(MOT_BACK_LEFT, 0);

	const float pValue = 0.55;                 //These values converts the error into workable units
	const float iValue = 0.9; 
	const float dValue = 3;
	float error, speed, iError = 0;             //variable delarations
	float wasHere = 0;

	int i = 1;                                 //i is the number of times the drive has been in position/number of 
                                               //times the robot has hit the correct location
	bool hitTarget = false;		               //true or false depending on if robot has hit target destination
	float targetDistance = abs(inches);	       //converts inches into encoder counts
	int  rightEncoder; 						   //declares right and left encoders
	
	
    while(!hitTarget && !(escapeTime()) )	   //while robot has not hit target and kill switch is not activated
    {

		rightEncoder = abs(vexMotorPositionGet(MOT_BACK_RIGHT) + 1); 
		error = targetDistance - rightEncoder;				//How far off the right encoder's count is from the
															//targetDistance count
		double dError = rightEncoder - wasHere;
		speed = error * pValue + iError * iValue + dError *dValue + 4;

		if (abs(speed) <= 123) iError += error *0.02;		//iError is the area under error curve (the "current"
		else speed = 123;

		speed *= signOf(inches);							//If desired number of inches is negative, then speed
															//of motors will be (-) for going backward
		
		//Setting Motors (With Corrections for Straightening)
        motorSet(MOT_FRONT_LEFT,  speed  );
        motorSet(MOT_BACK_LEFT,   speed  );
        motorSet(MOT_FRONT_RIGHT,-speed  );
        motorSet(MOT_BACK_RIGHT, -speed  );

		if(rightEncoder <=  targetDistance + 4  && rightEncoder >= targetDistance -4)
        {	
			i += 1;											//Each time it hits target, 1 is added to i
			if (i > 4) hitTarget = true;					
		}
		else i = 1;											//If robot does not hit target 7 times in a row, i goes

		wasHere = rightEncoder;
		vexSleep(20);
    }
	stopBase();												//Stops base after target is reached
}

/**
 *This function sets the lift position while the vexClawThread is running.
 *
 *@author Alex Miller <amm@albion.edu>
 *@since 2015-2-19
 *
 *@param[in] spot
 *  The desired location of the lift
 */

void autonLift(float spot)
{
	//while(liftSetpoint != 0 && !escapeTime()){ //Until the lift isn't running wait
	//vexSleep(25);
	//} 
	liftSetpoint = spot;
}

/**
 *This function makes the thread sleep for a period of time.
 *
 *@author Alex Miller <amm@albion.edu>
 *@since 2014-12-21
 *
 *@param[in] time
 *  Amount of time in milliseconds the thread should sleep
 */

 void wait(float time)
 {
    chThdSleepMilliseconds(time);
 }

/**
 *This opens the claw when called using a wait statement.
 *
 *@author Alex Miller <amm@albion.edu>
 *@since 2014-12-21
 *
 */
 
void openClaw(void) 
{
    vexMotorSet(MOT_CLAW, 127);
    wait(300);
    vexMotorSet(MOT_CLAW, 0);
}

/**
*This closes the claw when called using a wait statement. 
*
*@author Alex Miller <amm@albion.edu>
*@since 2014-12-21
*
*/

void closeClaw(void)    
{
    vexMotorSet(MOT_CLAW, -96);
    wait(250);
    vexMotorSet(MOT_CLAW, 0);
}

/**
 *This function will somehow raise the lift.
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2014-12-29
 *
 *@param[in] high
 *   Set to 1 if lift is landing at the high lift position
 */

void raiseLift(float high)
{
    vexMotorPositionSet(MOT_LIFT_ONE, 0);
    while((abs(vexMotorPositionGet(MOT_LIFT_ONE)) < abs(LIFT_CONSTANT * (high))) && !(escapeTime()) )
    {
		liftMotorSet(127);
    }

	liftMotorSet(0);
}

/**
 *This function will somehow lower the lift.
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2014-12-29
 *
 *@param[in] low
 *   Set to 1 if lift is landing at the low lift position
 */

void lowerLift(float low)
{
    vexMotorPositionSet(MOT_LIFT_ONE, 0);

    while((vexMotorPositionGet(MOT_LIFT_ONE) > LIFT_CONSTANT * -low) && !escapeTime()) 
    {
	   liftMotorSet(-127);
    }

	liftMotorSet(0);
}

/**
 *This function will move the robot sideways to the left.
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2015-01-17
 *
 *@param[in] inch
 *   number of inches left the robot should move.
 */

void strafe(float inch)
{
    vexMotorPositionSet(MOT_BACK_RIGHT, 0);
	
    while(vexMotorPositionGet(MOT_BACK_RIGHT) < inch * 14 / 16 * DRIVE_CONSTANT)
    {
        vexMotorSet(MOT_BACK_RIGHT,  -127);   
        vexMotorSet(MOT_FRONT_RIGHT,  127);   
        vexMotorSet(MOT_BACK_LEFT,    127);   
        vexMotorSet(MOT_FRONT_LEFT,  -127);   
    }

    vexMotorPositionSet(MOT_BACK_RIGHT, 0);
}

//////////////////////////////////////////////
//Driver Period Functions					//
//////////////////////////////////////////////


/**
 *This function uses analog joysticks to drive the robot. 
 *
 *@author Alex Miller <amm@albion.edu>
 *@since 2014-12-21
 *
 *@param[in] ch1 
 *  Controls strafing on the x-axis
 *@param[in] ch2
 *  Controls forward and backward movement; y axis
 *@param[in] ch4
 *  Controls rotation around the z-axis. 
 */
void moveFunc(int ch1, int ch2, int ch4) 
{
    vexMotorSet(motBackRight,   ch1 - ch2 + ch4);   
    vexMotorSet(motFrontRight, -ch1 - ch2 + ch4);   
    vexMotorSet(motBackLeft,   -ch1 - ch2 - ch4);   
    vexMotorSet(motFrontLeft,   ch1 - ch2 - ch4);   
}

/**
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

/**
 *This controls the raising and lowering of the chain lift. 
 *
 *@author Alex Miller <alexmiller965@gmail.com>
 *@since 2014-12-29
 *
 *@param[in] raises 
 *  Raises the lift while assigned button is pressed
 *@param[in] lower
 *  Lowers the lift while assigned button is pressed
 */
void liftControl(int raises, int lower)
{
    vexMotorSet(motLiftOne, 80 * (raises - lower));
    vexMotorSet(motLiftTwo, 80 * (raises - lower));
    vexMotorSet(motLiftThree, 80 * (raises - lower));
    vexMotorSet(motLiftFour, 80 * (raises - lower));
}

/**
 *This function controls the opening and closing of the claw.
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2014-12-21
 *
 *@param[in] open
 *  Opens up the claw when button is pressed
 *@param[in] close
 *  Closes the claw when button is pressed
 */

 
void clawControl(int open,int close)
{
vexMotorSet(motClaw, 127 * (open - close));
}


/**
 *This function controls the extending and retracting  of the foot.
 *
 *@author Annelise Comai <anneliesecomai@gmail.com>
 *@since 2014-12-21
 *
 *@param[in] extend 
 *  Extends the foot when assigned button is pressed
 *@param[in] retract
 *  Retracts the foot when assigned button is pressed
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
             //Sonar values: 72 for sonar2, 13 for sonar
                //closeClaw();
                //puts cube on skyrise
                closeClaw();
                raiseLift(1, 0);
                lowerLift(0, 1);
                driveBackward(6);
            /*  raiseLift(1, 0);
              pointTurnLeft(40);
              driveForward(6);
              lowerLiftEasy();
              driveBackward(8);
                //moves to skyrise piece
                pointTurnLeft(60);
                driveForward(15.5);
                pointTurnLeft(135);
                driveForward(4);
                closeClaw();
                raiseLift(1, 0);
                pointTurnLeft(115);
                driveForward(3);
                lowerLiftEasy();
                openClaw();
                //

              //  auton 2 
              //place robot on tile next to small post


                closeClaw();
                raiseLift(1, 0);
                driveForward(2);
                lowerLift(0, 1);
                openClaw();

                */

        
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

        vexMotorPositionSet(motBackRight, 0);
        vexMotorPositionSet(motFrontLeft, 0);
        vexMotorPositionSet(motLiftOne, 0);
        vexSonarStartAll();

    
    // Run until asked to terminate
    while(!chThdShouldTerminate())
        {

        //int precision = 1 - vexControllerGet(Btn6U) * 0.25 - vexControllerGet(Btn6D) * 0.5;
        

        //Modifies drive base speed if certain buttons are pressed. Allows for more precise movement
        moveFunc(      -vexControllerGet(Ch1),      //Strafing
                        vexControllerGet(Ch2),      //Forward and backward movement
                        vexControllerGet(Ch4));     //Rotational and turning movement
        //Other Movement
        liftControl(    vexControllerGet(Btn5U),        //Lifts the lift
                        vexControllerGet(Btn5D) );      //Lowers the lift
        
        clawControl(    vexControllerGet(Btn8R),    //Opens the claw
                        vexControllerGet(Btn8L) );  //Closes the claw
    
       if(vexControllerGet(Btn8U) == 1) 
          {
            raiseLift(1, 0);
            pointTurnLeft(40);
            driveForward(6);
            lowerLiftEasy();
            driveBackward(8);
            }


        vexLcdCode();


        /*
        footControl(    vexControllerGet(Btn8L),    //Extends the foot
                        vexControllerGet(Btn8R) );  //Retracts the foot
        */

        // Don't hog cpu
        vexSleep( 25 );
        }

    return (msg_t)0;
}