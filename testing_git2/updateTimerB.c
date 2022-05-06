/*
 * updateTimerB.c
 *
 *  Created on: March 15, 2022
 *      Author: Matthew Wonneberg, Jamie Boyd
 */
#include <msp430.h>
#include <updateTimerB.h>
#include <quadEncDec.h>
#include "mdd_driver.h"
#include <movement.h>

#include <stdio.h>
#include <math.h>
#include <PwmTimerA0.h>

/**************************************
 * Function: void timerB0Init()
 *
 *purpose: Initialize timerB0 to the correct settings
 *purpose: also configure the port settings
 *
 *returns nothing
 *Author: Matthew Wonneberg, Jamie Boyd
 **************************************/
void updateTimerBInit(){

    TB0CCR0 = 50000; // 0.05sec*2(ID_1 divides by 2)
    TB0CCR1 = 0;  // CCR1 initialized to zero
    TB0CCTL1 |= (OUTMOD_7) // reset set mode
             | CM_0 // no capture
             | CCIE    // ie
             & ~CCIFG  // clr flags
             & ~COV;

    TB0EX0 &= 0x000;     // bits 000  divide by 0

    TB0CTL = (TBSSEL_2 | ID_2 | MC_1 | TBCLR); // Timer_B0 control register, SMCLK, ID_2 SMCLK/ , up mode

}
/**************************************
 * Function: void updateTimer()
 *
 *purpose: provides a function that updates every 0.01 seconds in order to operate the motors smoothly
 *         that will be used for updating pwm dutyCycles and updating the PID loop
 *
 *Created March 15 2021
 *Created by: Matthew Wonneberg, Jamie Boyd
 *returns nothing
 **************************************/
void updateTimer(){

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

    // if startMoveJ is set, the move is commanded to start
    if (startMoveJ == 1){
        updateIndex++;

        if (updateIndex >= arrayLength-1){ // exit condition, resets start bit, and applies the braking signal to both motors
            startMoveJ =0; // reset start bit
            TA0CCR4 = 0; // send 0% PWM signals
            TA0CCR3 = 0;
            exit = mddInputCtrl2(CTRLBRAKE2); // apply braking signals to motors
            exit = mddInputCtrl(CTRLBRAKE);
            P2IFG &= ~0xF0; // clear flags due to quadrature encoder interrupts
        }
        else{
            //-------------- Motor 1 ------------------------
            if (noMove1 == 0){// proceed if there is a move of the joint for joint 1

                posError1 = posArray1[updateIndex] - gPosCountL1; // motor 1 position error = setpoint - current position
                posError1Change = posError1 - prevError1; // delta position over time (position derivative)
                posError1Integral = posError1Sum; // summation of all errors throughout the move

                // apply gains to the output signal
                sendPWM = ((kP*posError1) + (kI*posError1Integral) + (kD*posError1Change));

                prevError1 = posError1; // store the current error for the derivative of position
                posError1Sum = posError1Sum + posError1; // update the sum of the errors for the integral of position


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
            //------------- Motor 2 ------------------------
            if(noMove2 ==0){// proceed if there is a move of the joint

                posError2 = posArray2[updateIndex] - gPosCountL2;// motor 2 position error = setpoint - current position
                posError2Change = posError2 - prevError2;// delta position over time (position derivative)
                posError2Integral = posError2Sum;// summation of all errors throughout the move

                // apply gains to the output signal
                sendPWM2 = ((kP*posError2) + (kI*posError2Integral) + (kD*posError2Change));

                prevError2 = posError2;// store the current error for the derivative of position
                posError2Sum = posError2Sum + posError2;// update the sum of the errors for the integral of position



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

    switch(__even_in_range(TB0IV,2)){ // reading TB0IV clears CCIFG, TB0R is counting up from zero now.
    //case 0: break; // nothing
    case 2:// TB0CCTL1 CCIFG
        updateTimer();
    break;

    default: break;
    }

}

