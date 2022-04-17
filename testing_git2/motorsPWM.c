/*
 * motorsPWM.c
 *
 *  Created on: Feb. 20, 2021
 *      Author: Matthew Wonneberg and Jamie Boyd
 */
#include <msp430.h>
#include <motorsPWM.h>

unsigned int gPWML1 =0;
unsigned int gPWML2=0;


/**************************************
 * Function: void timerA0Init()
 *
 *purpose: Initialize timerA0 to the correct settings
 *purpose: also configure the port settings
 *
 *returns nothing
 * Motor Init
PWM done with timer A0.
L1 on CCR3 on P1.4
L2 on CCR4 on P1.5
 **************************************/
void motorsPWMinit(void){
    TA0CCR3 = 0;// capture compare registers for PWM initialized to 0%
    pwmL1 = 0;
    TA0CCR0 = 999;      // pwm based on 1000
    TA0CTL = (TASSEL__SMCLK | ID__4 | MC__UP); // Timer_A0 control register, SMCLK, SMCLK/1 , Up mode(timer counts up to CCR0
    TA0EX0 = TAIDEX_4;     // divide by 5
    TA0CCTL3 = OUTMOD_7; // reset set mode (output is reset when the timer counts to the TAxCCRn value, it is set when the time counts to the TAxCCR0 value
    TA0CCTL4 = OUTMOD_7;    // reset set
    P1OUT &= ~(BIT4 + BIT5);   // Reset pin
    P1SEL |= (BIT4 + BIT5);    // pin 1.2
    P1DIR |= (BIT4 + BIT5;    // output pin 1.2    TA0.1 pin
    // motor driver INA, INB, outputs for L1 and L2
    P3DIR |= (INA_L1 + INB_L1 +INA_L2 + INB_L2); // pins set as output direction
    P3OUT &= (~INA_L1 & ~INB_L1 & ~INA_L2 & ~INB_L2); //
}


// positive directions mean counter clockwise direction
void motorsPWMset (signed int pwmL1, signed int pwmL2){
    unsigned char ctrl = CTRLPORT ;
    if ((pwmL1 > -1000) && (pwmL1 < 1000)){
        if ((pwmL1 < 0) && (gPWML1 >= 0)){
            ctrl |= INA_L1;
            ctrl &= ~INB_L1;
        }else{
            if ((pwmL1 > 0) && (gPWML1 <= 0)){
                ctrl = CTRLPORT | INB_L1;
                ctrl &= ~INA_L1;
                CTRLPORT = ctrl;
            }
        }
        TA0CCR3 = pwmL1;
        gPWML1 = pwmL1;
    }
    if ((pwmL2 > -1000) && (pwmL2 < 1000)){
           if ((pwmL2 < 0) && (gPWML2 >= 0)){
               ctrl |= INA_L2;
               ctrl &= ~INB_L2;
           }else{
               if ((pwmL1 > 0) && (gPWML1 <= 0)){
                   ctrl |= INB_L2;
                   ctrl &= ~INA_L2;
               }
           }
           TA0CCR4 = pwmL1;
           gPWML2 = pwmL2;
    }
    CTRLPORT = ctrl;
}



