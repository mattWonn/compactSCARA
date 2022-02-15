/*
 * UartPwmTimerA0.h
 *
 *  Created on: Feb. 20, 2021
 *      Author: Rinz
 */

#ifndef UARTPWMTIMERA0_H_
#define UARTPWMTIMERA0_H_

void timerA0Init(unsigned int pwmFreq);
char timerA0PwmFreqSet(unsigned int pwmFreq);
char timerA0DutyCycleSet(unsigned char dutyCycle);
char timerA02DutyCycleSet(unsigned char dutyCycle);



#endif /* UARTPWMTIMERA0_H_ */
