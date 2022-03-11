/*
 * updateTimerB.c
 *
 *  Created on: Nov. 8, 2021
 *      Author: Rinz
 */
#include <msp430.h>
#include <ucsiUart.h>
#include <updateTimerB.h>
#include <quadEncDec.h>
#include <ucsiUart.h>
#include <mdd_driver.h>
#include <movement.h>

#include <string.h>
#include <stdio.h>
#include <math.h>

/**************************************
 * Function: void timerB0Init()
 *
 *purpose: Initialize timerA0 to the correct settings
 *purpose: also configure the port settings
 *
 *returns nothing
 **************************************/
void updateTimerBInit(){

    TB0CCR0 = 50000; //0xFFFE, 0.05sec*2(ID_1 divides by 2) = 0.1sec  = 10Hz update rate
    TB0CCR1 = 0;  // CCR1
    TB0CCTL1 |= (OUTMOD_7) // reset set mode
             | CM_0 // no capture
             | CCIE    // ie
             & ~CCIFG  // clr flags
             & ~COV;

    TB0EX0 &= 0x000;     // bits 000  divide by 5

    TB0CTL = (TBSSEL_2 | ID_2 | MC_1 | TBCLR); // Timer_B0 control register, SMCLK, ID_1 SMCLK/ , up mode

}
/**************************************
 * Function: void updateTimer()
 *
 *purpose: provide a 10Hz or 100000 clk cycle update function
 *         that will be used for updating pwm dutyCycles and updating the PID loop
 *
 *Created Nov 8 2021
 *Created by: Matt W
 *returns nothing
 **************************************/
void updateTimer(){

    _BIS_SR(GIE);

    volatile signed int error;
    volatile signed int angJ1Current;
    volatile signed int voutM1;
    volatile signed int sendPWM;
    volatile int dir1 = 1;


    volatile signed int error2;
    volatile signed int angJ2Current;
    volatile signed int voutM2;
    volatile signed int sendPWM2;
    volatile int dir2 =1;

  //  volatile char posPrint[25]; // Uart
  //  volatile int ret;


    if (startM1 == 1){

        //------------------- M1 ------------------------
        angJ1Current = (posCount) * DEG_PER_PUL1; // find the current angle
        error = (angJ1Desired) - angJ1Current;// find the error
        if (error < 2  && error > -2) // uncertainty.
        {
            error = 0;
            doneM1=1;
            startM1 =0;
        }
        sendPWM = error * SLOPE; // calculate the speed signal, get rid of floating point math
        sendPWM = sendPWM/100;

        if (sendPWM < 0){ // convert sendPWM to a posotive signal with a direction (dir1)
            sendPWM = sendPWM*-1;
            dir1 = 0; // ccw
        }

        if (sendPWM > MAX_PWM) // constrain max limits
           sendPWM = MAX_PWM;

        if (sendPWM > 0 && sendPWM <= MAX_PWM){ // min voltage condition cw
               if (sendPWM < MIN_VELOCITY && sendPWM > 0 && error != 0) // min speed cw
                   sendPWM  = MIN_VELOCITY;
               else if (sendPWM >= MAX_VELOCITY) // max speed cw
                   sendPWM = MAX_VELOCITY;
        }

        if (dir1 == 1) // send motor the speed signal based on direction
            mddCW(sendPWM);
        else
            mddCCW(sendPWM);
    }


        //------------------------ M2 -----------------------
    if (startM2 == 1){

        angJ2Current = (posCount2) * DEG_PER_PUL;// find the current angle
        error2 = (angJ2Desired) - angJ2Current;// find the error
        if (error2 < 2  && error2 > -2){ // uncertainty
            error2 = 0;
            doneM2=1;
            startM2 =0;
        }
        sendPWM = error2 * SLOPE;// calculate the speed signal, get rid of floating point math
        sendPWM = sendPWM/100;

        if (sendPWM < 0){// convert sendPWM to a posotive signal with a direction (dir1)
            sendPWM = sendPWM*-1;
            dir2 = 0; // ccw
        }

        if (sendPWM > MAX_PWM) // constrain max limits
           sendPWM = MAX_PWM;

        if (sendPWM > 0 && sendPWM <= MAX_PWM){ // min voltage condition cw
               if (sendPWM < MIN_VELOCITY && sendPWM > 0 && error2 != 0) // min speed cw
                   sendPWM  = MIN_VELOCITY;
               else if (sendPWM >= MAX_VELOCITY) // max speed cw
                   sendPWM = MAX_VELOCITY;
        }

        if (dir2 == 1) // send the motor speed signal based on direction
            mddCW2(sendPWM);
        else
            mddCCW2(sendPWM);
    }

}

#pragma vector = TIMER0_B1_VECTOR
interrupt void timer0_B1_ISR(void){
// CCIFG is still set here and TB0IV = 0x02
    switch(__even_in_range(TB0IV,2)){ // reading TB0IV clears CCIFG, TB0R is counting up from zero now.
    //case 0: break; // nothing
    case 2:// TB0CCTL1 CCIFG
        updateTimer();
    break; // TA0CCR1

    default: break;
    }

//    TB0CCTL1 &= ~CCIFG; // CCR0 auto reset

}

