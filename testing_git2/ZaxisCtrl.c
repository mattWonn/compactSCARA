/*
 * ZaxisCtrl.c
 *
 *  Created on: Apr. 17, 2022
 *      Author: Jamie Boyd / Matthew Wonneberg
 */
#include <msp430.h>
#include "SCARA.h"
#include "ZaxisCtrl.h"

zAxisController zControl;       // stores information about z-control
volatile signed int gZPos = 0;      // easily accessed global set from interrupt

// initializes timerA 1 for output and sets pins for direction and enable
void zAxisInit (void){
    P2DIR |= ZAXIS_STEP;    // set output pin for steps. pin 2.0  TA1 CCR1 output pin
    P2SEL |= ZAXIS_STEP;
    TA1CTL = (TASSEL__SMCLK +  MC__STOP + TAIE);     // SMCLK, no division, timer off  to start , set MC__UP to move
    TA1CCTL1 = OUTMOD_7;        // Reset/set mode

    P3DIR |= (ZAXIS_DIR + ZAXIS_ENABLE);    // set up direction and selection output pins
    P3OUT &= ~ZAXIS_ENABLE;
    zControl.jogSpeed = 0;            // calculated speed in steps/second when "jogging" Positive goes down, Negative goes up
    zControl.curDir =0;             // stopped +1 when moving down, -1 when moving up
    zControl.lowerLimit= 32767;     // no limit -  better set one
    zControl.upperLimit = -32768;   // no limit - better set one
    zControl.resolution = ZAXIS_RES; // as calculated, but can be set by user
    zControl.movSpeed = zAxisSetSpeed (ZAXIS_MAX_SPEED/5);  // sets CCR0 and CCR1 for desired speed
}

/* ***************************************
 * timerA0PwmFreqSet(unsigned int pwmFreq)
 *
 * Finds closest set of divisors to given frequency, given that always set CCR0 to 65535 for max control
 * We always use 20Mhz SMCLK for clock source - could go from 109 Hz down to 5 Hz using 32768 Hz ACLK.
 * If we wanted to make a crusade out of making an exact frequency of square wave we would get nearest clock divisor with CCR0 = 100
 * then adjust CCR0 up or down to make the exact frequency we wanted - but for PWM the exact frequency of the carrier is seldom of interest as
 * long as it is no too fast for driver to output, and is faster than time constant of the motor. pretty much always 10kHz is a good choice
20E6/
make sure you have timer stopped before calling this function
 *
****************************************/
unsigned int zAxisSetSpeed(unsigned int pwmFreq) {
    unsigned int rVal;
    TA1CTL &= ~ID__8;           // clear the bits before setting the bits
    if (pwmFreq > 305){        // no division at all, use 20Mhz and adjust CCR0
        TA1EX0 = TAIDEX_0;
        TA1CCR0 = (20000000/pwmFreq)-1;
        rVal = 20000000/(TA1CCR0 +1);
    }else{
        if (pwmFreq > 152){    // divide by 2
           TA1CTL |= ID__2;
           TA1EX0 = TAIDEX_0;
           TA1CCR0 = (10000000/pwmFreq)-1;
           rVal = 10000000/(TA1CCR0 + 1);
        }else{
            if (pwmFreq > 101){  // divide by 3
                TA1EX0 = TAIDEX_2;
                TA1CCR0 = (6666666/pwmFreq)-1;
                rVal = 6666666/(TA1CCR0 + 1);
            }else{
                if (pwmFreq > 76){    // divide by 4
                   TA1CTL |= ID__4;
                   TA1EX0 = TAIDEX_0;
                   TA1CCR0 = (5000000/pwmFreq)-1;
                   rVal = 5000000/(TA1CCR0 +1);
                }else{
                    if (pwmFreq > 61){    // divide by 5
                       TA1EX0 = TAIDEX_4;
                       TA1CCR0 = (4000000/pwmFreq)-1;
                       rVal = 4000000/(TA1CCR0 + 1);
                    } else{
                        if (pwmFreq > 50){    // divide by 6
                            TA1CTL &= ~ID__8;
                            TA1EX0 = TAIDEX_5;
                            TA1CCR0 = (3333333/pwmFreq)-1;
                            rVal = 3333333/(TA1CCR0 +1);
                        }else{
                            if (pwmFreq > 43){    // divide by 7
                               TA1CTL &= ~ID__8;
                               TA1EX0 = TAIDEX_6;
                               TA1CCR0 = (2857143/pwmFreq) -1;
                               rVal = 2857143/(TA1CCR0 + 1);
                            } else{
                                if (pwmFreq > 38){    // divide by 8
                                  TA1CTL |= ID__8;
                                  TA1EX0 = TAIDEX_0;
                                  TA1CCR0 = (2500000/pwmFreq) -1 ;
                                  rVal = 2500000/(TA1CCR0 + 1);
                                }else{
                                    if (pwmFreq > 30){    // divide by 10 (2 X 5)
                                      TA1CTL |= ID__2;
                                      TA1EX0 = TAIDEX_4;
                                      TA1CCR0 = (2000000/pwmFreq)-1;
                                      rVal =  2000000/(TA1CCR0 + 1);
                                    }else{
                                        if (pwmFreq > 25){     // divide by 12 (2 X 6)
                                             TA1CTL |= ID__2;
                                             TA1EX0 = TAIDEX_5;
                                             TA1CCR0 =  (1666667/pwmFreq)-1;
                                             rVal = 1666667/(TA1CCR0 +1);
                                        }else{
                                            if (pwmFreq > 21){     // divide by 14 (2 X 7)
                                                TA1CTL |= ID__2;
                                                TA1EX0 = TAIDEX_6;
                                                TA1CCR0 =  (1428571/pwmFreq)-1;
                                                rVal = 1428571/(TA1CCR0 +1);
                                            }else{
                                                if (pwmFreq > 19){     // divide by 16 (2 X 8)
                                                    TA1CTL |= ID__2;
                                                    TA1EX0 = TAIDEX_7;
                                                    TA1CCR0 =  (1250000/pwmFreq)-1;
                                                    rVal = 1250000/(TA1CCR0 + 1);

                                                }else{
                                                    if (pwmFreq > 15){     // divide by 20 (4 X 5)
                                                        TA1CTL |= ID__4;
                                                        TA1EX0 = TAIDEX_4;
                                                        TA1CCR0 = (1000000/pwmFreq)-1;
                                                        rVal = 1000000/(TA1CCR0 +1);
                                                    } else{
                                                        if (pwmFreq > 12){     // divide by 24 (4 X 6)
                                                            TA1CTL |= ID__4;
                                                            TA1EX0 = TAIDEX_5;
                                                            TA1CCR0 = (833333/pwmFreq)-1;
                                                            rVal = 833333/(TA1CCR0 + 1);
                                                        }else{
                                                            if (pwmFreq > 10){ // divide by 28 (4 x 7)
                                                                TA1CTL |= ID__4;
                                                                TA1EX0 = TAIDEX_6;
                                                                TA1CCR0 = (714286/pwmFreq)-1;
                                                                rVal = 714286/(TA1CCR0 +1);
                                                            }else{
                                                                if (pwmFreq > 9){ // divide by 32 (4 x 8)
                                                                    TA1CTL |= ID__4;
                                                                    TA1EX0 = TAIDEX_7;
                                                                    TA1CCR0 = (625000/pwmFreq)-1;
                                                                    rVal = 625000/(TA1CCR0 + 1);
                                                                }else{
                                                                    if (pwmFreq > 7){ // divide by 40 (8 x 5)
                                                                        TA1CTL |= ID__8;
                                                                        TA1EX0 = TAIDEX_4;
                                                                        TA1CCR0 = (500000/pwmFreq)-1;
                                                                        rVal =500000/(TA1CCR0 + 1);
                                                                    }else{
                                                                        if (pwmFreq > 6){ // divide by 48 (8 x 6)
                                                                            TA1CTL |= ID__8;
                                                                            TA1EX0 = TAIDEX_5;
                                                                            TA1CCR0 = (416667/pwmFreq)-1;
                                                                            rVal = 416667/(TA1CCR0 +1);
                                                                        }else{
                                                                            if (pwmFreq > 5){     // divide by 56 (8 x 7)
                                                                                TA1CTL |= ID__8;
                                                                                TA1EX0 = TAIDEX_6;
                                                                                TA1CCR0 = (357143/pwmFreq)-1;
                                                                                rVal = 357143/(TA1CCR0 +1);
                                                                            }else{                  // divide by 64 (8 x8)
                                                                                TA1CTL |= ID__8;
                                                                                TA1EX0 = TAIDEX_7;
                                                                                TA1CCR0 = (312500/pwmFreq)-1;
                                                                                rVal = 312500/(TA1CCR0 + 1);
                                                                            }
                                                                        }
                                                                    }
                                                                }

                                                            }
                                                        }
                                                    }
                                                }
                                            }

                                        }
                                    }
                                }
                            }

                        }
                    }
                }
            }
        }
    }
    TA1CCR1 = (TA1CCR0 + 1)/2;
    TA1CTL |= TACLR;         // to cleanse the discriminating timer palate
    return rVal;
}

/*************************** when TAR counts from value in CCR0 to 0 ***************************************/
#pragma vector = TIMER1_A0_VECTOR
__interrupt void doStep(void) {
    if (!(gSTOP)){
        if ((gZpos > zControl.lowerLimit) || (gZpos < zControl.upperLimit)){
            eStopSoftware ();
        }else{
            if (zControl.curDir){
                gZPos +=1;
                if (gZPos >= zControl.movTarget){
                    TA1CTL &= ~MC__UPDOWN;
                    zControl.jogSpeed = 0;
                    P3OUT &= ~ZAXIS_ENABLE;
                }
            }else{
                gZPos -=1;
                if (gZPos <= zControl.movTarget){
                    TA1CTL &= ~MC__UPDOWN;
                    zControl.jogSpeed = 0;
                    P3OUT &= ~ZAXIS_ENABLE;
                }
            }
        }
    }else{
        TA1CTL &= ~MC__UPDOWN;
        zControl.jogSpeed = 0;
        P3OUT &= ~ZAXIS_ENABLE;
    }
}


void zAxisZero(void){
    gZPos = 0;
}

void zAxisSetUpper (signed int limit){
    zControl.upperLimit = limit;
}

void zAxisSetLower (signed int limit){
    zControl.lowerLimit = limit;
}

void zAxisSetUpperHere(void){
    zControl.upperLimit = gZpos;
}
void zAxisSetLowerHere(void){
    zControl.lowerLimit = gZpos;
}

void zAxisGoToPos (signed int movPos){
    zControl.movTarget = movPos;
    if (movPos > gZpos){
        zControl.curDir = 1;
        P3OUT |= ZAXIS_DIR;
    }else{
        zControl.curDir = 0;
        P3OUT &= ~ZAXIS_DIR;
    }
    P3OUT |= ZAXIS_ENABLE;
    TA1CTL |= MC__UP;  // to activate timer
}


void zAxisJog (signed int speed){
   zControl.jogSpeed = speed;
    if (speed > 0){
       zControl.curDir = 1;
       P3OUT |= ZAXIS_DIR;
       zControl.jogSpeed = zAxisSetSpeed((unsigned int)speed)
       zControl.movTarget = zControl.lowerLimit;
    }else{
        zControl.curDir = 0;
        P3OUT &= ~ZAXIS_DIR;
        zControl.jogSpeed = zAxisSetSpeed((unsigned int)(-1 * speed));
        zControl.movTarget = zControl.upperLimit;
    }
    P3OUT |= ZAXIS_ENABLE;
    TA1CTL |= MC__UP;  // to activate timer
}


void zAxisStop (void){
   TA1CTL &= ~MC__UPDOWN;
   zControl.curSpeed = 0;
   P3OUT &= ~ZAXIS_ENABLE;
}

