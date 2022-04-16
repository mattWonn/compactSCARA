/*
 * Uartvnh7070hpi.h
 *
 *  Created on: Feb. 20, 2021
 *      Author: Rinz
 */

#ifndef MDD_DRIVER_H_
#define MDD_DRIVER_H_

//--------------- motor driver -----------------------
unsigned int counting1;
unsigned int counting2;

#define CTRLPORT P3OUT

#define CTRLMASK (BIT0 + BIT1 +BIT2 )
#define ZEROVAL 0x00
#define INA BIT0 // P3.0
#define INB BIT1 // P3.1
#define SEL BIT2 // not used
#define CTRLOUT (P3DIR |= 0x7); (P3SEL |= 0x7)

#define CTRLMASK2 (BIT3 + BIT4 +BIT5)
#define INA2 BIT3 // P3.3
#define INB2 BIT4 // P3.4
#define SEL2 BIT5 // not used
#define CTRLOUT2 (P3DIR |= BIT3+BIT4+BIT5); (P3SEL |= BIT3+BIT4+BIT5)

#define CTRLCW (CTRLMASK & (INA & ~INB & ~SEL))
#define CTRLCCW (CTRLMASK & (INB & ~INA & ~SEL))
#define CTRLBRAKE (CTRLMASK & (~INA & ~INB & ~SEL))

#define CTRLCW2 (CTRLMASK2 & (INA2 & ~INB2 & ~SEL2))
#define CTRLCCW2 (CTRLMASK2 & (INB2 & ~INA2 & ~SEL2))
#define CTRLBRAKE2 (CTRLMASK2 & (~INA2 & ~INB2 & ~SEL2))


//---- CW and CCW / brake global variabless--------------

void displayPos();
char mddInputCtrl(unsigned char ctrl);


void displayPos2();
char mddInputCtrl2(unsigned char ctrl);


#endif /* MDD_DRIVER_H_ */
