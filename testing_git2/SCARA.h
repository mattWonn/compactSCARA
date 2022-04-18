/*
 * SCARA.h
 *
 *  Created on: Apr. 12, 2022
 *      Author: Jamie Boyd/Matthew Wonneberg
 *      declaring global variables in a single place
 *
 *
 *  using P1.5 as PWM1
 *  using P2.0 as PWM2
 *
 *  using P3.0 as INA1
 *  using P3.1 as INB1
 *
 *  using P3.3 as INA2
 *  using P3.4 as INB2
 *
 *  Using       as Z-axis direction
 *  Using       as Z-axis step
 *  Using       as output for software ESTOP
 *  Using       as input from E-stop button and limit switches
 *
 *  Using       as output to E-stop activated LED
 *
 */

#ifndef SCARA_H_
#define SCARA_H_


extern volatile signed int gPosCountL1;         // encoder count of L1
extern volatile signed int gPosCountL2;         // encoder count of L2
extern volatile signed int gPWML1;              // motor power and direction for L1
extern volatile signed int gPWML2;              // motor power and direction for L2
extern volatile signed int gZPos;               // position of Z axis - in steps from user-set 0
extern volatile unsigned int gToolData;         // whatever data your tool stores
extern volatile unsigned char gIsMoving;        // set when robot is performing a move
extern volatile unsigned char gSTOP =0;         // 0 is normal, non-zero means stopped


void SCARA_failure (void);      // blinks Estop light if unrecoverable failure happens
unsigned char SCARA_getState (unsigned char * inputData, unsigned char * outputResults);

#endif /* SCARA_H_ */
