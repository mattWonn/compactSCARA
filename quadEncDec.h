/*
 * quadEncDec.h
 *
 *  Created on: Mar. 22, 2021
 *      Author: Rinz
 */

#ifndef QUADENCDEC_H_
#define QUADENCDEC_H_

#define CHAMASK 0xDF
#define CHBMASK 0xEF

#define IND BIT6
#define CHA BIT5
#define CHB BIT4
#define RST BIT1
#define CURRSTATE (P2IN & 0x30)

#define PORT2DIR P2DIR |= (CHB | CHA | IND & ~RST) // input, this is wrong!!!!!!!!!!!!
#define PORT2IN P2IN |= (CHB | CHA | IND & ~RST) // read only
#define RISISTOREN P2REN |= ( CHB | CHA | IND | RST) // resistors enabled
#define CLEARFLAGS P2IFG = 0x00
#define INTERUPTEN P2IE |= (CHB | CHA | IND | RST) //
#define PUSHBUTTON P2OUT |= RST;  P2IES |= RST // falling



void quadEncInit();

volatile unsigned int prevState;
volatile unsigned int currentState;
volatile unsigned int dirStatus;
volatile signed int posCount;

volatile unsigned char preA;
volatile unsigned char preB;
volatile unsigned char currA;
volatile unsigned char currB;

#endif /* QUADENCDEC_H_ */
