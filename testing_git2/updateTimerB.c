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
    volatile signed int prevPosCountHold;
    volatile signed int velError1;
    volatile signed int posError1;

    volatile signed int error2;
    volatile signed int angJ2Current;
    volatile signed int voutM2;
    volatile signed int sendPWM2;
    volatile int dir2 =1;
    volatile signed int prevPosCountHold2;
    volatile signed int velError2;
    volatile signed int posError2;


    volatile signed int voutM1test;
    volatile signed int sendPWMtest;
    volatile signed int velErrortest;
    volatile signed int posErrortest;


    volatile char posPrint[25]; // Uart
   volatile int ret;

    if (startMoveJ == 1){
        updateIndex++;

        if (updateIndex >= arrayLength-1) // uncertainty.
        {
            startMoveJ =0;
            __disable_interrupt();
            sendPWM =0;
            sendPWM2 =0;
            mddCW(sendPWM);
            mddCW2(sendPWM2);
            prevPosCount = 0;
          //  posCount = posArray1[arrayLength-1];
            prevPosCount2 = 0;
          //  posCount2 = posArray2[arrayLength-1];
            updateIndex=0;

        }
        else{
          if (noMove1 == 0){

        prevPosCountHold = posCount;
        velCount = posCount - prevPosCount;
        posError1 = posCount - posArray1[updateIndex];
        velError1 = velCount - velArray1[updateIndex];

        sendPWM = round((100/35)*(velArray1[updateIndex] - 1*posError1 - 1*velError1));
       // sendPWM = (100/40)*velArray1[updateIndex];
        // WHEN CHANGING MOTORS, ALSO NEED TO CHANGE PUL_PER_DEG in movement.c

        if (sendPWM < 0){ // convert sendPWM to a posotive signal with a direction (dir1)
            sendPWM = sendPWM*-1;
            dir1 = 0; // ccw
        }
        if (sendPWM > MAX_PWM) // constrain max limits
           sendPWM = MAX_PWM;
        if (sendPWM> 0 && sendPWM <= MAX_PWM){ // min voltage condition cw
               if (sendPWM <12 && sendPWM >0) // min speed cw
                   sendPWM  = 12;
               else if (sendPWM >= MAX_VELOCITY) // max speed cw
                   sendPWM = MAX_VELOCITY;
        }

 //       sprintf(posPrint, "pwm %d,vE %d pE %d \n\r", sendPWM, velError1, posError1); // insert the number of characters into the display string
 //       ret = ucsiA1UartTxString(&posPrint); // print the string


        if (dir1 == 1) // send motor the speed signal based on direction
            mddCW(sendPWM);
        else{
            mddCCW(sendPWM);
        }

        prevPosCount = prevPosCountHold;
        }
        //-------------------------------------
        if(noMove2 ==0){

        prevPosCountHold2 = posCount2;
        velCount2 = posCount2 - prevPosCount2;

        posError2 = posCount2 - posArray2[updateIndex];
        velError2 = velCount2 - velArray2[updateIndex];

        sendPWM2 = ((100/47)*(velArray2[updateIndex] - kIntegral*posError2 - kProportional*velError2));
      //  sendPWM = round(velocityConst*velArray2[updateIndex]);


        if (sendPWM2 < 0){ // convert sendPWM to a posotive signal with a direction (dir1)
            sendPWM2 = sendPWM2*-1;
            dir2 = 0; // ccw
        }

        if (sendPWM2 > MAX_PWM) // constrain max limits
           sendPWM2 = MAX_PWM;
        if (sendPWM2 > 0 && sendPWM2 <= MAX_PWM){ // min voltage condition cw
               if (sendPWM2 >= MAX_VELOCITY) // max speed cw
                   sendPWM2 = MAX_VELOCITY;
        }

//        sprintf(posPrint, "pwm %d,vE %d pE %d \n\r", sendPWM2, velError2, posError2); // insert the number of characters into the display string
//        ret = ucsiA1UartTxString(&posPrint); // print the string
        pwmArray[updateIndex] = sendPWM2;

        if (dir2 == 1) // send motor the speed signal based on direction
            mddCW2(sendPWM2);
        else
            mddCCW2(sendPWM2);



        prevPosCount2 = prevPosCountHold2;
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

