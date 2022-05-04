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

#define CTRLMASK (BIT0 + BIT1)
#define ZEROVAL 0x00
#define INA BIT0 // P3.0
#define INB BIT1 // P3.1
#define CTRLOUT (P3DIR |= 0x3);

#define CTRLMASK2 (BIT4 + BIT2)
#define INA2 BIT4 // P3.3
#define INB2 BIT2 // P3.2
#define CTRLOUT2 (P3DIR |= BIT4+BIT2); (P3SEL |= BIT4+BIT2)

#define CTRLCW (CTRLMASK & (INA & ~INB))
#define CTRLCCW (CTRLMASK & (INB & ~INA))
#define CTRLBRAKE (CTRLMASK & (~INA & ~INB))

#define CTRLCW2 (CTRLMASK2 & (INA2 & ~INB2))
#define CTRLCCW2 (CTRLMASK2 & (INB2 & ~INA2))
#define CTRLBRAKE2 (CTRLMASK2 & (~INA2 & ~INB2))


//---- CW and CCW / brake global variabless--------------

void displayPos();
char mddInputCtrl(unsigned char ctrl);


void displayPos2();
char mddInputCtrl2(unsigned char ctrl);


#endif /* MDD_DRIVER_H_ */
