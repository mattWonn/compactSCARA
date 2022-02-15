/*************************************************************************************************
 * ucsiUart.h
 * - - C interface file for MSP430 UCSI UART A1, A0
 *
 *  Author: Greg Scutt
 *  Created on: March 1, 2017
 *  Modified: Feb 19, 2018
 **************************************************************************************************/

#ifndef UCSIUART_H_
#define UCSIUART_H_

#include <mdd_driver.h>

#define _UART_A1PSEL P4SEL |= PM_UCA1TXD | PM_UCA1RXD // use macros for short expressions.

//----------Uart config--------------
#define TXD_A1 BIT4         //Transmit Data on P4.4
#define RXD_A1 BIT5         //Recieve Data on P4.5

#define UCBRF 0x60   //6
#define UCBRS 0x2    //1
#define LOWBYTE 0x03
#define HIGHBYTE 0x00

//---------------------------------
void ucsiA1UartInit();

void ucsiA1UartTxChar(unsigned char txChar);
int ucsiA1UartTxString(unsigned char* txChar);
//int usciA1UartTXBuffer(char *buffer, int bufLen);

char usciA1UartGets(char *rxString);

#endif /* UCSIUART_H_ */
