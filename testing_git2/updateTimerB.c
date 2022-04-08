/*
 * updateTimerB.c
 *
 *  Created on: Nov. 8, 2021
 *      Author: Rinz
 */
#include <msp430.h>
#include <ucsiUart.h>
#include <updateTimerB.h>
#include <uartPwmTimerA0.h>
#include <quadEncDec.h>
#include <ucsiUart.h>
#include "mdd_driver.h"
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

    TB0CCR0 = 50000; // 0.05sec*2(ID_1 divides by 2)
    TB0CCR1 = 0;  // CCR1
    TB0CCTL1 |= (OUTMOD_7) // reset set mode
             | CM_0 // no capture
             | CCIE    // ie
             & ~CCIFG  // clr flags
             & ~COV;

    TB0EX0 &= 0x000;     // bits 000  divide by 5

    TB0CTL = (TBSSEL_2 | ID_2 | MC_1 | TBCLR); // Timer_B0 control register, SMCLK, ID_2 SMCLK/ , up mode

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

    //_BIS_SR(GIE);

    volatile signed int sendPWM;
    volatile int dir1 = 1;
    volatile signed int posError1;
    volatile signed int posError1Change;
    volatile signed int posError1Integral;

    volatile signed int sendPWM2;
    volatile int dir2 =1;
    volatile signed int prevPosCountHold2;
    volatile signed int posError2;
    volatile signed int posError2Change;
    volatile signed int posError2Integral;

    volatile int exit;

   //volatile char posPrint[25]; // Uart
   //volatile int ret;

    if (startMoveJ == 1){
        updateIndex++;

        if (updateIndex >= arrayLength-1){ // uncertainty.
            startMoveJ =0;
            TA0CCR4 = 0;
            TA1CCR1 = 0;
            exit = mddInputCtrl2(CTRLBRAKE2);
            exit = mddInputCtrl(CTRLBRAKE);
            P2IFG &= ~0xF0;
        }
        else{
        if (noMove1 == 0){

            posError1 = posArray1[updateIndex] - posCount;
            posError1Change = posError1 - prevError1;
            posError1Integral = posError1Sum;

            sendPWM = ((kP*posError1) + (kI*posError1Integral) + (kD*posError1Change));

            prevError1 = posError1;
            posError1Sum = posError1Sum + posError1;


            if (sendPWM < 0){ // convert sendPWM to a posotive signal with a direction (dir1)
                sendPWM = sendPWM*-1;
                dir1 = 0; // ccw
            }
            if (sendPWM > MAX_PWM) // constrain max limits
               sendPWM = MAX_PWM;
            if (sendPWM> 0 && sendPWM <= MAX_PWM){ // min voltage condition cw
                   if (sendPWM <5 && sendPWM >0) // min speed cw
                       sendPWM  = 5;
                    if (sendPWM >= MAX_VELOCITY) // max speed cw
                       sendPWM = MAX_VELOCITY;
            }

        //    sprintf(posPrint, "pwm %d,vE %d pE %d \n\r", sendPWM, velError1, posError1); // insert the number of characters into the display string
        //    ret = ucsiA1UartTxString(&posPrint); // print the string


            if (dir1 == 1){ // send motor the speed signal based on direction
                exit = mddInputCtrl(CTRLCW);
                exit = timerA0DutyCycleSet(sendPWM); // send the previously pressed dutyCycle
            }
            else{
                exit = mddInputCtrl(CTRLCCW);
                exit = timerA0DutyCycleSet(sendPWM); // send the previously pressed dutyCycle

            }

        }
        //-------------------------------------
        if(noMove2 ==0){

            posError2 = posArray2[updateIndex] - posCount2;
            posError2Change = posError2 - prevError2;
            posError2Integral = posError2Sum;

            sendPWM2 = ((kP*posError2) + (kI*posError2Integral) + (kD*posError2Change));

            prevError2 = posError2;
            posError2Sum = posError2Sum + posError2;



            if (sendPWM2 < 0){ // convert sendPWM to a posotive signal with a direction (dir1)
                sendPWM2 = sendPWM2*-1;
                dir2 = 0; // ccw
            }

            if (sendPWM2 > MAX_PWM) // constrain max limits
               sendPWM2 = MAX_PWM;
            if (sendPWM2 > 0 && sendPWM2 <= MAX_PWM){ // min voltage condition cw
                if (sendPWM2 <5 && sendPWM2 >0) // min speed cw
                    sendPWM2  = 5;
                 if (sendPWM2 >= MAX_VELOCITY) // max speed cw
                       sendPWM2 = MAX_VELOCITY;
            }

          //     sprintf(posPrint, "pwm %d \n\r", sendPWM2); // insert the number of characters into the display string
            //   ret = ucsiA1UartTxString(&posPrint); // print the string


            if (dir2 == 1){ // send motor the speed signal based on direction
                exit = mddInputCtrl2(CTRLCW2);
                exit = timerA0DutyCycleSet2(sendPWM2); // send the previously pressed dutyCycle
            }
            else{
                exit = mddInputCtrl2(CTRLCCW2);
                exit = timerA0DutyCycleSet2(sendPWM2); // send the previously pressed dutyCycle
            }



            }
        }

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

