/*
 * ZaxisCtrl.c
 *
 *  Created on: Apr. 17, 2022
 *      Author: Jamie Boyd / Matthew Wonneberg
 */

#include "ZaxisCtrl.h"
#include <msp430.h>

volatile signed int gPosCountZ;

// initializes timer A 1 on P2.0
void zAxisInit (void){
    unsigned int timerSpeed;     // speed in steps/second = Hz
    P2DIR |= ZAXIS_BIT;    // output pin 2.0    TA1 CCR1 output pin
    P1SEL |= ZAXIS_BIT;
    TA1CCR0 = 1999; // assign CCR0 value that the timer will count up to
    TA1CCR1 = 1000;  // capture compare register 1 initialized to 50%, because frequency is the thing here, not duty cycle
    TA1CTL = (TASSEL__SMCLK | MC_1); // Timer_A0 control register, SMCLK, , Up mode(timer counts up to CCR0)
    TA1CCTL1 = OUTMOD_0
    TA1CCTL1 |= (OUTMOD_7); // reset set mode (output is reset when the timer counts to the TAxCCRn value, it is set when the time counts from the TAxCCR0 value to 0
    timerSpeed
}
    /**************************************
     * Function: void timerA0Init()
     *
     *purpose: Initialize timerA0 to the correct settings
     *purpose: also configure the port settings
     *
     *returns PWM frequency chosen closest to desired
     **************************************/
    unsigned int timerA0Init(unsigned int pwmFreq){
        unsigned int retVal;
        P1DIR |= BIT2;    // output pin 1.2    TA0.1 pin
        P1SEL |= BIT2;    // pin 1.2

        TA0CCR0 = 99; // assign CCR0 value that the timer will count up to
        TA0CCR1 = 0;  // capture compare register 1 initialized to 0%
        TA0CCTL1 |= (OUTMOD_7); // reset set mode (output is reset when the timer counts to the TAxCCRn value, it is set when the time counts to the TAxCCR0 value

        TA0CTL = (TASSEL__SMCLK | MC_1); // Timer_A0 control register, SMCLK, , Up mode(timer counts up to CCR0)
        retVal = timerA0PwmFreqSet(pwmFreq);

        P1DIR |= BIT2;    // output pin 1.2    TA0.1 pin
        P1SEL |= BIT2;    // pin 1.2
        return retVal;


    }
      /*  P1DIR |= BIT3;    // output pin 1.3    TA0.2 pin
        P1SEL |= BIT3;    // pin 1.3
        P1OUT &= ~BIT0;   // Reset pin
    */


    /* ***************************************
     * timerA0PwmFreqSet(unsigned int pwmFreq)
     *
     * Finds closest set of divisors to given frequency, given that always set CCR0 to 65535 for max control
     * We always use 20Mhz SMCLK for clock source - could go from 109 Hz down to 5 Hz using 32768 Hz ACLK.
     * If we wanted to make a crusade out of making an exact frequency of square wave we would get nearest clock divisor with CCR0 = 100
     * then adjust CCR0 up or down to make the exact frequency we wanted - but for PWM the exact frequency of the carrier is seldom of interest as
     * long as it is no too fast for driver to output, and is faster than time constant of the motor. pretty much always 10kHz is a good choice
    20E6/

     *
****************************************/
unsigned int zAxisSetSpeed(unsigned int pwmFreq) {
    unsigned int rVal;
    TA1CTL &= ~ID__8;   // clear the bits before setting the bits
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
    return rVal;
}



void zAxisZero(void);
void zAxisSetUpper (signed int);
void zAxisSetLower (signed int);
void zAxisGoToPos (signed int);
void zAxisGo (signed char);
void zAxisStop (void);

