/*
 * quadEncDec.c
 *
 *  Created on: Mar. 22, 2021
 *      Author: Matthew Wonneberg, Jamie Boyd
 */

#include <msp430.h>
#include "quadEncDec.h"

volatile signed int gPosCountL1 = 1789;
volatile signed int gPosCountL2 = -1984;


void quadEncInit(){

    PORT2DIR;
    CLEARFLAGS;
    INTERUPTEN;
    prevState = CURRSTATE1; // read currentState
    preA = (P2IN & 0x20)>>5;
    preB = (P2IN & 0x10)>>4;

    prevState2 = CURRSTATE2; // read currentState
    P2IES = (prevState2 + prevState);
    preB2 = (P2IN & 0x40)>>6;
    preA2 = (P2IN & 0x80)>>7;

    P2IFG &= 0x00; // flags are cleared
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
                 gPosCountL2--;
                 dirStatus2 =1;
             }
             else{
                 gPosCountL2++;  // CW
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
             gPosCountL1--;
             dirStatus =1;
         }
         else{
             gPosCountL1++;  // CW
             dirStatus =0;
         }
         preA = currA; // only need to update previousA because we are not using preB
     P2IFG &= ( ~BIT4 & ~BIT5 ); // flags are cleared when exiting routine*/
     }
     else
         P2IFG &= ~0xF0; // clear all flags if stuck


}
