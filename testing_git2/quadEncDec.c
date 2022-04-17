/*
 * quadEncDec.c
 *
 *  Created on: Mar. 22, 2021
 *      Author:Matthew Wonneberg / Jamie Boyd
 */

#include <msp430.h>
#include "quadEncDec.h"

volatile signed int gPosCountL1 = 1776;
volatile signed int gPosCountL2 = -1984;



/* Encoder State = 0 to 3
 * A____|--------|________|----
 *
 * B_________|--------|________
 *    0   1    2   3    0    1
 *

 * 0) A is low and B is low. clear PxIES for A, clear PxIES for B
 *      if A goes high, count up    goto state 2
 *      if B goes high, count down  goto state 1
 *
 * 1) A is high, B is low. set PxIES for A, clear PxIES for B
 *      if B goes high, count up    goto state 3
 *      if A goes low, count down  goto state 0
 *
 * 2) A is High, B is High. clear PxIES for A, clear PxIES for B
 *      if A goes low, count up    goto state 1
 *      if B goes low, count down    goto state 2
 *
 * 3) A is low, B is high. clear PxIES for A, set PxIES for B
 *       if B goes low, count up    goto state 0
 *       if A goes high, count down  goto state 3
 */


void quadEncInit(){
    gPosCountL1 =0;
    gPosCountL2 =0;
    P2DIR &= ~(CHA_L1 | CHB_L1 | CHA_L2 | CHB_L2);
    P2IES &= ~(CHA_L1 | CHB_L1 | CHA_L2 | CHB_L2);
    P2IE |=   (CHA_L1 | CHB_L1 | CHA_L2 | CHB_L2);
    P2IFG = 0;
}



#pragma vector = PORT2_VECTOR
__interrupt void QuadDecoder(void) {
    static unsigned char encState_L1 = 0;
    static unsigned char encState_L2 = 0;

    switch(__even_in_range(P2IV,16)) {
    case  0: break;                 // Vector  0: no interrupt
    case  2: break;                 // Vector  2: P1.0
    case  4: break;                 // Vector  4: P1.1
    case  6: break;                 // Vector  6: P1.2
    case  8: break;                 // Vector  8: P1.3
    case 10:                        // Vector 10: P1.4    CHB_L1
        switch (encState_L1){       // L1 CHB toggled
        case 0:                     // CHA was low, CHB was low, P2IES was cleared for A and B, CHB went high
            gPosCountL1 -=1;
            encState_L1 = 3;
            P2IES |= CHB_L1;
            break;
        case 1:                     // CHA was high, CHB was low, P2IES was set for A and cleared for B, CHB went high
            gPosCountL1 +=1;
            encState_L1 += 1;
            P2IES |= CHB_L1;
            break;
        case 2:                     //CHA was high, CHB was high. P2IES cleared for A and for B, CHB went low
            gPosCountL1 -=1;
            encState_L1 -= 1;
            P2IES &= ~CHB_L1;
            break;
        case 3:                     //A was low, B was high. P2IES cleared for A, set for B, CHB went low
            gPosCountL1 +=1;
            encState_L1 = 0;
            P2IES &= ~CHA_L1;
            break;
        }
        P2IFG &= ~CHB_L1;
        break;
    case 12:                        // Vector 12: P1.5    CHA_L1
        switch (encState_L1){       // L1 CHA toggled
        case 0:                     // CHA was low, CHB was low, P2IES was cleared for A and B, CHA went high
            gPosCountL1 +=1;
            encState_L1 += 1;
            P2IES |= CHA_L1;
            break;
        case 1:                     // CHA was high, CHB was low, P2IES was set for A and cleared for B, CHA went low
            gPosCountL1 -=1;
            encState_L1 -= 1;
            P2IES &= ~CHA_L1;
            break;
        case 2:                     //CHA was high, CHB was high. P2IES cleared for A and for B, CHA went low
            gPosCountL1 +=1;
            encState_L1 += 1;
            P2IES &= ~CHA_L1;
            break;
        case 3:                     //A was low, B was high. P2IES cleared for A, set for B, CHA went high
            gPosCountL1 -=1;
            encState_L1 -= 1;
            P2IES |= CHA_L1;
            break;
        }
        P2IFG &= ~CHA_L1;
        break;
    case 14:                        // Vector 14: P1.6    CHB_L2
        switch (encState_L2){       // L2 CHB toggled
           case 0:                  // CHA was low, CHB was low, P2IES was cleared for A and B, CHB went high
               gPosCountL2 -=1;
               encState_L2 = 3;
               P2IES |= CHB_L2;
               break;
           case 1:                 // CHA was high, CHB was low, P2IES was set for A and cleared for B, CHB went high
               gPosCountL2 +=1;
               encState_L2 += 1;
               P2IES |= CHB_L2;
               break;
           case 2:                 //CHA was high, CHB was high. P2IES cleared for A and for B, CHB went low
               gPosCountL2 -=1;
               encState_L2 -= 1;
               P2IES &= ~CHB_L2;
               break;
           case 3:                 //A was low, B was high. P2IES cleared for A, set for B, CHB went low
               gPosCountL2 +=1;
               encState_L2 = 0;
               P2IES &= ~CHA_L2;
               break;
           }
           P2IFG &= ~CHB_L2;
           break;
    case 16:                        // Vector 16: P1.7    CHA_L2
        switch (encState_L2){       // L2 CHA toggled
           case 0:                  // CHA was low, CHB was low, P2IES was cleared for A and B, CHA went high
               gPosCountL2 +=1;
               encState_L2 += 1;
               P2IES |= CHA_L2;
               break;
           case 1:                 // CHA was high, CHB was low, P2IES was set for A and cleared for B, CHA went low
               gPosCountL2 -=1;
               encState_L2 -= 1;
               P2IES &= ~CHA_L2;
               break;
           case 2:                 //CHA was high, CHB was high. P2IES cleared for A and for B, CHA went low
               gPosCountL2 +=1;
               encState_L2 += 1;
               P2IES &= ~CHA_L2;
               break;
           case 3:                 //A was low, B was high. P2IES cleared for A, set for B, CHA went high
               gPosCountL2 -=1;
               encState_L2 -= 1;
               P2IES &= ~CHA_L2;
               break;
           }
           P2IFG &= ~CHA_L2;
           break;
    default: break;
    }
}
