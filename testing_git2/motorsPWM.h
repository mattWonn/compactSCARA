/*
 * UartPwmTimerA0.h
 *
 *  Created on: Feb. 20, 2021
 *      Author: Rinz
 */

#ifndef UARTPWMTIMERA0_H_
#define UARTPWMTIMERA0_H_

// H-bridge selection bits
#define     CTRLPORT    P3OUT
#define     INA_L1      BIT0    // P3.0
#define     INB_L1      BIT1    // P3.1
#define     INA_L2      BIT3    // P3.3
#define     INB_L2      BIT4    // P3.4


void motorsPWMinit(void);
void motorsPWMset (signed int pwmL1, singed int pwmL2);

#endif /* UARTPWMTIMERA0_H_ */

CTRLPORT = ((CTRLMASK2 & CTRLPORT)+ctrl) ;   //send ctrl to output
