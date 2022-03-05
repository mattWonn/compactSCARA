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

    P2DIR = ~BIT4 &~BIT5 &~BIT6 &~BIT7;
    //PORT2DIR;
    CLEARFLAGS;
    INTERUPTEN;
}


#pragma vector = PORT2_VECTOR // PORT2_VECTOR is defined in msp430.h
 __interrupt void Port2_ISR1 (void) // Port 2 interrupt service routine
{

     counting1++;
     if ((P2IFG & 0x80) || (P2IFG & 0x40)){
         counting2++;

      /*   currentState2 = CURRSTATE2;
         currA2 = (currentState2 & 0x40)>>6; // /32
         currB2 = (currentState2 & 0x80)>>7; // /16
         P2IES = (0x80);

         if ((preA2 == 1) && (currA2 == 1)){ // CCW
             if (currB2 == 0){
                 posCount2++;
                 dirStatus2 =1;
             }
             else{
                 posCount2--;
                 dirStatus2 =0;
             }
         }
         preA2 = currA2; // only need to update previousA because we are not using preB*/

         currentState2 = CURRSTATE2;
         currA2 = (currentState2 & 0x40)>>6; // /32
         currB2 = (currentState2 & 0x80)>>7; // /16
         P2IES = (((currentState + currentState2) & 0xF0));

             if (currB2 ^ preA2){ // CCW
                 posCount2--;
                 dirStatus2 =1;
                 // need to add overflow condition for revs
             }
             else{
                 posCount2++;  // CW
                 dirStatus2 =0;
                 // need to add overflow condition for revs
             }
             preA2 = currA2; // only need to update previousA because we are not using preB*/
         P2IFG &= ( ~BIT6 & ~BIT7); // flags are cleared when exiting routine
     }



     else{
     currentState = CURRSTATE1;
     currA = (currentState & 0x20)>>5; // /32
     currB = (currentState & 0x10)>>4; // /16
     P2IES = (((currentState + currentState2) & 0xF0));

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

     P2IFG &= ( ~BIT4 & ~BIT5 ); // flags are cleared when exiting routine
     }
}
