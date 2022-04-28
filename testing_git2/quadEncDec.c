/*
 * quadEncDec.c
 *
 *  Created on: Mar. 22, 2021
 *      Author: Matthew Wonneberg, Jamie Boyd
 */

#include <msp430.h>
#include "ucsiUart.h"
#include "quadEncDec.h"
#include <string.h>
#include <stdio.h>

void quadEncInit(){

    PORT2DIR;
    CLEARFLAGS;
    INTERUPTEN;
}


#pragma vector = PORT2_VECTOR // PORT2_VECTOR is defined in msp430.h
 __interrupt void Port2_ISR1 (void) // Port 2 interrupt service routine
{

     if (currentState2 != CURRSTATE2){ // check if motor 1 or motor 2 interrupt
         currentState2 = (P2IN & 0xC0);
         currB2 = (currentState2 & 0x40)>>6; // shift low bit into currB2 variable
         currA2 = (currentState2 & 0x80)>>7; // shift high bit into currA2 variable
         P2IES = ((currentState+currentState2) & 0xF0);// edge select for both motors
             if (currB2 ^ preA2){ // CCW
                 posCount2--;
                 dirStatus2 =1;
             }
             else{
                 posCount2++;  // CW
                 dirStatus2 =0;
             }
             preA2 = currA2; // only need to update previousA because we are not using preB
         P2IFG &= ( ~BIT6 & ~BIT7); // flags are cleared when exiting routine
     }
     else if (currentState != CURRSTATE1){
     currentState = (P2IN &0x30);
     currA = (currentState & 0x20)>>5; // /32
     currB = (currentState & 0x10)>>4; // /16
     P2IES = ((currentState+currentState2) & 0xF0);//fixxx
         if (currB ^ preA){ // CCW
             posCount--;
             dirStatus =1;
         }
         else{
             posCount++;  // CW
             dirStatus =0;
         }
         preA = currA; // only need to update previousA because we are not using preB
     P2IFG &= ( ~BIT4 & ~BIT5 ); // flags are cleared when exiting routine*/
     }
     else
         P2IFG &= ~0xF0; // clear all flags if stuck


}
