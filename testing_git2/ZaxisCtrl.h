/*
 * ZaxisCtrl.h
 *
 *  Created on: Apr. 17, 2022
 *      Author: Jamie Boyd / Matthew Wonneberg
 */

#ifndef ZAXISCTRL_H_
#define ZAXISCTRL_H_

#define     ZAXIS_INIT_RES              100         // steps/mm
#define     ZAXIS_INIT_SPEED            100         // steps/second = 1 mm/second
#define     ZAXIS_STEP                  BIT0        // timer A 1 on P2.0
#define     ZAXIS_DIR                   BIT2        // P2.2
#define     ZAXIS_ENABLE                BIT4        // P7.4

typedef struct zAxis{
    signed int curPos;
    signed int upperLimit;
    signed int lowerLimit;
    signed int curSpeed;
} zAxis, * zAxisPtr;

void zAxisInit (void);
void zAxisMeasRes (void);
void zAxisZero(void);
void zAxisSetUpper (signed int limit);
void zAxisSetUpperHere(void);
void zAxisSetLower (signed int limit);
void zAxisSetLowerHere(void);
unsigned char zAxisSetSpeed (unsigned int speed);  // sets speed for upcoming moves, not direction
void zAxisGoToPos (signed int pos);
void zAxisGo (signed int speed);        // speed AND direction at which to go until stopped, or a limit is hit
void zAxisStop (void);                  // stops movement on Z-axis

#endif /* ZAXISCTRL_H_ */
