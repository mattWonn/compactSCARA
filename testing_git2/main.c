/**
 * main.c
 */
#include <msp430.h>
#include "ucsiUart.h"
#include "mdd_driver.h"
#include "UartPwmTimerA0.h"
#include "quadEncDec.h"
#include "updateTimerB.h"
#include "UcsControl.h"
#include "movement.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/*
 * main_lab.c
 *
 *
 *  using P1.2 as PWM1
 *  using P1.3 as PWM2
 *
 *  using P3.0 as INA1
 *  using P3.1 as INB1
 *
 *  using P3.3 as INA2
 *  using P3.4 as INB2
 *
 */

//#define dcoFreq 1      // leaving at 1 works with Uart settings: BR1 =0x00, BR0 = 0x03, RS_1, RF_6.
#define dcoFreq 20     //  BR = 65: BR1 = 0x00 BR0 = 0x41, RS_0 , RF_6

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    P4OUT = 0x00;
    P6OUT = 0x00;

      _disable_interrupts();
      unsigned char oscFail = 1; // clock set
      /*********Set clock frequency*********************************************/
      unsigned char testPass = 1;
      ucsSelSource(1,1,1,1);
      oscFail = ucsDcoFreqSet (dcoFreq, 2, 1);            //set sclk to dcoFreq
      if (oscFail)
          return 1;
      /***End***Set clock frequency*********************************************/



    //-----------UART CONTROL ------------------------
    ucsiA1UartInit();

    volatile int numChars;
    volatile unsigned char stringRet;
    volatile char getsInvalidString[] = "\nInvalid String entry\n\r";
    char returned =1;
    int parseRet;
    CMD mddCmds[MAX_CMDS]; // array of mddCmds of type CMD

    UCA1IE |= UCRXIE;         // Receive interrupt en
  // __enable_interrupt();



   //------------motor control--------------
       volatile unsigned char counter = 0;
       volatile unsigned char dutyReturned =0;
       volatile unsigned char cwRet =0;
       volatile unsigned char ccwRet =0;

       volatile unsigned char counter2 = 0;
       volatile unsigned char dutyReturned2 =0;
       volatile unsigned char cwRet2 =0;
       volatile unsigned char ccwRet2 =0;

       prevClkCountNot = 1;
       countClkWise =0;
       clkWise =1;
       dutyPrev = 0;

       prevClkCountNot2 = 1;
       countClkWise2 =0;
       clkWise2 =1;
       dutyPrev2 = 0;

       timerA0Init(PWMFREQ);        // initialize TimerA0 and ports
       motorCmdInit(mddCmds); // add new commands to this function and define them in the motor driver.h file

       // INA, INB, SEL outputs
       P3OUT |= 0x00;
       P3DIR |= (BIT0 + BIT1 +BIT2 +BIT3 + BIT4 +BIT5); // pins set as output direction
       P3OUT &= (~BIT0 & ~BIT1 & ~BIT2 & ~BIT3 & ~BIT4 & ~BIT5); // P3out set to 0 (led's off)

       cwRet = mddCW(dutyPrev);
       cwRet2 = mddCW2(dutyPrev2);



   //------------- Encoder-----------------
           volatile signed int holdCount;

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


           posCount = 0;
           posCount2 = 0;
           startMoveJ =0;
           prevPosCount =0;
           prevPosCount2 =0;
           noMove1 =0;
           noMove2 =0;




           kProportional =1.6;//1.6
           kIntegral = 1; //1.8
           velocityConst =(100/40);


   //----------- Structured Commands -------------------
           volatile unsigned int waiting=2;

           scaraStateEnd.scaraArm.theta1 =0;
           scaraStateEnd.scaraArm.theta2 =0;
           scaraStateEnd.scaraArm.xPos =15;
           scaraStateEnd.scaraArm.yPos =0;
           scaraStateEnd.scaraArm.armSol =1; //(LHS)


           /*----------------- paste this to send a value to the console--------------------------------
            *
                  volatile char posPrint[25]; // Uart
                  volatile int ret;

                 sprintf(posPrint, "Error1 = %d \n\r", error); // insert the number of characters into the display string
                 ret = ucsiA1UartTxString(&posPrint); // print the string

           */

           /*--------------- paste this in the main loop to access the command interpereter------------------
                 returned = usciA1UartGets(&rxGetString);  // get a string from the console

                 if (returned != 0){
                    parseRet = parseCmd(mddCmds, rxGetString); // send string to motor control
                    if (parseRet == -1)
                        numChars = ucsiA1UartTxString(&getsInvalidString); // print error message
                 }
                 else
                    numChars = ucsiA1UartTxString(&getsInvalidString); // print error message
           */

//           waiting = moveJ(0,90,0,25);
 //          waiting = moveJ(90,-90,25,0);
   //        waiting = moveJ(-90,90,0,-45);

           __disable_interrupt();
           waiting = moveJ(0,90,0,25);
           __enable_interrupt();
           startMoveJ = 1;
           while (startMoveJ == 1){}
           startMoveJ =0;

           __disable_interrupt();
           waiting = moveJ(90,-90,25,0);
           __enable_interrupt();
           startMoveJ = 1;
           while (startMoveJ == 1){}
           startMoveJ =0;

           __disable_interrupt();
           waiting = moveJ(-90,90,0,-45);
           __enable_interrupt();
           startMoveJ = 1;
           while (startMoveJ == 1){}
           startMoveJ =0;
    /*       __disable_interrupt();
           waiting = moveJ(0,90,0,90);
           __enable_interrupt();
           startMoveJ = 1;
           while (startMoveJ == 1){}
           startMoveJ =0;
           __delay_cycles(10000000);

           __disable_interrupt();
           waiting = moveJ(90,0,90,0);
           __enable_interrupt();
           startMoveJ = 1;
           while (startMoveJ == 1){}
           startMoveJ =0;
           __delay_cycles(10000000);*/

          // return 0;

       /*    __disable_interrupt();
           waiting = moveJ(0,90,0,90);
           __enable_interrupt();
       //    __delay_cycles(100000);
           startMoveJ = 1;
           while (startMoveJ == 1){}
           startMoveJ =0;
           __delay_cycles(10000000);*/


    while (1){//--------------- main loop-------------------

  /*    char rxGetString[BUFFLEN] = {0};   // reset getString buffer

      returned = usciA1UartGets(&rxGetString);  // get a string from the console

      if (returned != 0){
         parseRet = parseCmd(mddCmds, rxGetString); // send string to motor control
         if (parseRet == -1)
             numChars = ucsiA1UartTxString(&getsInvalidString); // print error message
      }
      else
         numChars = ucsiA1UartTxString(&getsInvalidString); // print error message*/


   //  waiting = scaraFk((scaraStateEnd.scaraArm.theta1), (scaraStateEnd.scaraArm.theta2), &(scaraStateEnd.scaraArm.xPos), &(scaraStateEnd.scaraArm.yPos));
   //  waiting = scaraIk(&(scaraStateEnd.scaraArm.theta1), &(scaraStateEnd.scaraArm.theta2), (scaraStateEnd.scaraArm.xPos), (scaraStateEnd.scaraArm.yPos), &(scaraStateEnd.scaraArm.armSol));

  /*      waiting = moveJ(0,90,0,90);
    //    __delay_cycles(100000);
        startMoveJ = 1;
        while (startMoveJ == 1){}
        startMoveJ =0;
        __delay_cycles(10000000);


    /*  mddCW(10);
      mddCW2(10);
      __delay_cycles(55000000);
      mddBrake();
      mddBrake2();
      __delay_cycles(35000000);
      mddCCW(10);
      mddCCW2(10);
      __delay_cycles(55000000);
      mddBrake();
      mddBrake2();
      __delay_cycles(35000000);
*/

    }

    return 0;
}
