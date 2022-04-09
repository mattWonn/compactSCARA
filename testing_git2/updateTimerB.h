/*
 * updateTimerB.h
 *
 *  Created on: Nov. 8, 2021
 *      Author: Rinz
 */

#ifndef UPDATETIMERB_H_
#define UPDATETIMERB_H_

//------- loop and position variables---------

#define MAX_PWM 90
#define MAX_VELOCITY 99
#define MIN_VELOCITY 12


#define SLOPE 50 // 100% conversion from 180deg to PWM is 0.55, this number is 0.25 multiplied for integer math
#define DEG_PER_PUL1 0.128571//N = 44
#define DEG_PER_PUL 0.105388//N = 71.165, 3415.92 countable events on O/P shaft


volatile signed int updateIndex;
volatile int startMoveJ;

volatile signed int velCount; // units pulses/updatetime
volatile signed int velCount2; // units pulses/updatetime
volatile signed int prevPosCount;
volatile signed int prevPosCount2;

volatile double kP;
volatile double kI;
volatile double kD;

volatile signed int prevError1;
volatile signed int prevError2;
volatile signed int posError1Sum;
volatile signed int posError2Sum;



void updateTimerBInit();
void updateTimer();

#endif /* UPDATETIMERB_H_ */
