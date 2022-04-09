/*
 * UartPwmTimerA0.h
 *
 *  Created on: Feb. 20, 2021
 *      Author: Rinz
 */

#ifndef UARTPWMTIMERA0_H_
#define UARTPWMTIMERA0_H_

//------- PWM conditions-----------------------------

#define PWMFREQMAX 20000     //18.935?  20kHz?
#define PWMFREQMIN 100       // 18.9 Hz
#define PWMFREQ 10000 // current pwm frequency

#define DUTYCYCLEMIN 0
#define DUTYCYCLEMAX 90
#define DUTY_INC 100
#define DUTY_RAMP_MIN 99

void timerA0Init(unsigned int pwmFreq);
char timerA0PwmFreqSet(unsigned int pwmFreq);
char timerA0DutyCycleSet(unsigned char dutyCycle);
char timerA0DutyCycleSet2(unsigned char dutyCycle);



#endif /* UARTPWMTIMERA0_H_ */
