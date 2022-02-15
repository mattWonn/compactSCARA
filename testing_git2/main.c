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
#include <stdio.h>
#include <string.h>
#include <math.h>
/*
 * main_lab.c
 *
 *
 *  using P1.2 as PWM1
 *  using P1.3 as PWM2
 *  using P3.0 as INA
 *  using P3.1 as INB
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
   __enable_interrupt();

   //------------motor control--------------
       volatile unsigned char counter = 0;
       volatile unsigned char dutyReturned =0;
       volatile unsigned char cwRet =0;
       volatile unsigned char ccwRet =0;

       prevClkCountNot = 1;
       countClkWise =0;
       clkWise =1;
       dutyPrev = 0;

       timerA0Init(PWMFREQ);        // initialize TimerA0 and ports
       motorCmdInit(mddCmds); // add new commands to this function and define them in the motor driver.h file

       // INA, INB, SEL outputs
       P3OUT |= 0x00;
       P3DIR |= (BIT0 + BIT1 +BIT2); // pins set as output direction
       P3OUT &= (~BIT0 & ~BIT1 & ~BIT2); // P3out set to 0 (led's off)
   //--------------------------------------
       cwRet = mddCW(dutyPrev);
   //------------- Encoder-----------------


            //volatile char posPrint[MAX_STRING_LEN] = {0}; // Uart
            //volatile int ret;
            volatile signed int holdCount;

           quadEncInit();
           ucsiA1UartInit();
           updateTimerBInit();

           prevState = (P2IN & 0x30); // read currentState
           P2IES = prevState;
           preA = (P2IN & 0x20)>>5;
           preB = (P2IN & 0x10)>>4;

           P2IFG &= 0x00; // flags are cleared
       //    TA0CCTL1 &= ~CCIFG;
       //    P2IE &= ~0xFF;       // disable posCount
           __enable_interrupt();

           posCount = 0;
           enterLoop = 0;

    while (1){

      char rxGetString[BUFFLEN] = {0};   // reset getString buffer

     /* mddCW(10);
      __delay_cycles(500000); // 1/2 sec
     // mddCW(70);
      __delay_cycles(500000);
     // mddCCW(70);
      __delay_cycles(500000);
      mddCCW(10);
      __delay_cycles(500000);
      mddBrake();
      __delay_cycles(3000000);*/


     // command interpreter input: This should be in its own function
      returned = usciA1UartGets(&rxGetString);  // get a string from the console

      if (returned != 0){
         parseRet = parseCmd(mddCmds, rxGetString); // send string to motor control
         if (parseRet == -1)
             numChars = ucsiA1UartTxString(&getsInvalidString); // print error message
      }
      else
         numChars = ucsiA1UartTxString(&getsInvalidString); // print error message

    }


    return 0;
}
