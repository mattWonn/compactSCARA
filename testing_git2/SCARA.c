/**
 * main.c
 */
#include <msp430.h>
#include "SCARA.h"
#include "BinaryCmdInterp.h"
#include "eStopLimitSwitch.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <MotorsPWM.h>

#include "libUART1A.h"




/*
#include "mdd_driver.h"
#include "quadEncDec.h"
#include "updateTimerB.h"
#include "UcsControl.h"
#include "movement.h"
*/
/*
 * main.c
 *
 *
 *  using P1.5 as PWM1
 *  using P2.0 as PWM2
 *
 *  using P3.0 as INA1
 *  using P3.1 as INB1
 *
 *  using P3.3 as INA2
 *  using P3.4 as INB2
 *
 *
 *
 */


int main(void) {
    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer
    _disable_interrupts();              // Set SCLK frequency to 20 Mhz
    unsigned char oscFail = 1;
    ucsSelSource(1,1,1,1);
    oscFail = ucsDcoFreqSet (20, 2, 1);
    if (oscFail)
      SCARA_failure ();
    _enable_interrupts();
    motorsPWMinit();                    // initialize TimerA0 and ports for PWM and motor driver direction
    quadEncInit();                      // set up encoders
    /*********************** Set up UART control  *********************************************/
    usciA1UartInit();
    binInterp_init();

    binInterp_addCmd (1, &SCARA_getState);
    binInterp_addCmd (1, &SCARA__getPos);
    binInterp_addCmd (3, &SCARA__setMtrs);
    //binInterp_addCmd (1, binInterp_setESTOP);
    binInterp_run ();

    return 0;
}

/*
    quadEncInit();
           ucsiA1UartInit();
           updateTimerBInit();

           prevState = CURRSTATE1; // read currentState
           preA = (P2IN & 0x20)>>5;
           preB = (P2IN & 0x10)>>4;

           prevState2 = CURRSTATE2; // read currentState
           P2IES = (prevState2 + prevState);
           preB2 = (P2IN & 0x40)>>6;
           preA2 = (P2IN & 0x80)>>7;

           P2IFG &= 0x00; // flags are cleared
       //    TA0CCTL1 &= ~CCIFG;
       //    P2IE &= ~0xFF;       // disable posCount
      //     __enable_interrupt();



   //--------------- Update Loop ----------------

           volatile unsigned int waiting=2;

           posCount = 0;
           posCount2 = 0;
           startMoveJ =0;
           prevPosCount =0;
           prevPosCount2 =0;
           noMove1 =0;
           noMove2 =0;
           armSolChange = 0;

           prevError1 = 0;
           prevError2 = 0;
           posError1Sum = 0;
           posError2Sum = 0;

           kP = 2.3;// map pul/UpdateTime to PWM(0:100); 1uT/0.01s * 1s/6716pul * 100%
           kI = 0; //1.8
           kD = 0.2;


   //----------- Structured Commands -------------------


           volatile signed int angleJ1;
           volatile signed int angleJ2;







    return 0;
}
*/

// [0] numBytes [1]errCode, [2,3]encoder 1 counts, [4,5]encoder 2 counts, [6,7] z axis counts,
// [8,9]motor1 PWM, [10,11]motor2 PWM, [12] tool data
unsigned char SCARA_getState (unsigned char * inputData, unsigned char * outputResults){
    outputResults [0] = 13;   // the number of bytes to send when sending results, including this one which is not sent
    outputResults [1] = 0;   // no errors here, code for no errors
    signed int * encPtr = ((signed int *)&outputResults[2]);
    *encPtr++ =  gPosCount1;
    *encPtr++ = gPosCount2;
    *encPtr++ = gPosCount2;
    *encPtr = gZposCount;
    outputResults [8] = gToolData;
    outputResults [9] = gPwm1;
    outputResults [10] = gPwm2;
    return 1;
}

// better than nothing, blinking LED is some indication to user that things went south.
void SCARA_failure (void){
    P6DIR |= ESTOP_LED;
    while (1){
        P6OUT ^= ESTOP_LED;
        __delay_cycles(50000);  // 2 Hz, the angriest frequency
    }
}
