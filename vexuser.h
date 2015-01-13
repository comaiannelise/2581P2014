#ifndef VEXUSER_H
#define VEXUSER_H

/**
 * This function determines and returns an acceptable change to the given power level based
 *     base on user input.
 * 
 * @author Liam Bohl <liambohl@gmail.com>
 * @author Michel Momeyer <strihawk1213@gmail.com>
 * @since 2014-12-29
 *
 * @param oldValue[in]
 *     the current value to be changed
 * @param channelInput[in]
 *     the user input from the controller
 * @param deadZone[in]
 *     the minimum input threshold that must be reached before changes are considered
 * @param maxAcceleration[in]
 *     the maximum change in power that can be added at an instant
 * @return 
 *     the acceptable change to the given power level
 */
int getPowerIncrement(int oldValue, int input, int deadZone, int maxAcceleration);

/**
 * This function takes the controller input and modify values for use.
 * 
 * @author Michel Momeyer <strihawk1213@gmail.com>
 * @since 2014-12-29
 */
void updateInput(void);

/**
 * This function sets all the motors for driver control based.
 * 
 * @author Michel Momeyer <strihawk1213@gmail.com>
 * @since 2014-12-29
 */
void setMotors(void);

/**
 * This function drives forward a certain distance autonomously.
 *
 * @author Michel Momeyer <strihawk1213@gmail.com>
 * @since 2015-01-10
 * @param dist[in]
 *     the distance to travel in feet
 * @param dist[in]
 *     the distance to travel in feet
 */
 void setAutonMotor(int motor, int dist, int speed);


#endif // VEXUSER_H