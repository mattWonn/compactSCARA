/*
 * quadEncDec.c
 *
 *  Created on: Mar. 22, 2021
 *      Author: Rinz
 */

#include <msp430.h>
#include "ucsiUart.h"
#include "quadEncDec.h"
#include <string.h>
#include <stdio.h>

void quadEncInit(){

    //PORT2DIR;
    P2DIR = ~BIT4 &~BIT5 &~BIT6;
   // PORT2IN;
    //RISISTOREN;

    CLEARFLAGS;
    INTERUPTEN;
    PUSHBUTTON;

}

#pragma vector = PORT2_VECTOR // PORT2_VECTOR is defined in msp430.h
 __interrupt void Port2_ISR1 (void) // Port 2 interrupt service routine
{
     currentState = (P2IN & 0x30);
     currA = (currentState & 0x20)>>5; // /32
     currB = (currentState & 0x10)>>4; // /16
     P2IES = (currentState & 0x30);

         if (currB ^ preA){ // CCW
             posCount--;
             dirStatus =1;
             // need to add overflow condition for revs
         }
         else{
             posCount++;  // CW
             dirStatus =0;
             // need to add overflow condition for revs
         }
         preA = currA; // only need to update previousA because we are not using preB

     P2IFG &= ( ~BIT1 & ~BIT4 & ~BIT5 & ~BIT6); // flags are cleared when exiting routine
}
