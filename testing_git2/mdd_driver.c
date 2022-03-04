/*
 * Uartmddhpi.c
 *
 *  Created on: Feb. 20, 2021
 *      Author: Rinz
 */

#include <mdd_driver.h>
#include <msp430.h>
#include <ucsiUart.h>
#include <UartpwmTimerA0.h>
#include <quadEncDec.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
/* ********************************
 * funciton: void displayPos
 *
 * Purpose:  displays the current position of the motor to the terminal
 *
 * return 0 if in range and -1 if not
 *********************************/
void displayPos(){
    volatile char posPrint[BUFFLEN] = {0}; // Uart
    volatile int ret;

    if (clkWise == 1){ // CW
       sprintf(posPrint, "\nCount = %d dir = CW  \n\r", posCount); // insert the number of characters into the display string
       ret = ucsiA1UartTxString(&posPrint); // print the string

    }
    else if (countClkWise == 1){ // CCW
       sprintf(posPrint, "\nCount = %d dir = CCW \n\r", posCount); // insert the number of characters into the display string
       ret = ucsiA1UartTxString(&posPrint); // print the string
    }
}


void displayPos2(){
    volatile char posPrint[BUFFLEN] = {0}; // Uart
    volatile int ret;

    if (clkWise2 == 1){ // CW
       sprintf(posPrint, "\nCount = %d dir = CW  \n\r", posCount2); // insert the number of characters into the display string
       ret = ucsiA1UartTxString(&posPrint); // print the string

    }
    else if (countClkWise2 == 1){ // CCW
       sprintf(posPrint, "\nCount = %d dir = CCW \n\r", posCount2); // insert the number of characters into the display string
       ret = ucsiA1UartTxString(&posPrint); // print the string
    }
}

/* ********************************
 * funciton: char mddInputCtrl(unsigned char ctrl)
 *
 * Purpose:  Sends cntrl to output if in range 0x0 -> 0x7
 *
 * return 0 if in range and -1 if not
 *********************************/
char mddInputCtrl(unsigned char ctrl)
{
    volatile signed char result =0;

    if ((ctrl & ~CTRLMASK) == ZEROVAL) // check that cntrl is within 0x0 - 0x7
    {
       CTRLPORT = ((CTRLMASK2 & CTRLPORT)+ctrl) ;   //send ctrl to output
    }
    else
    {
        result =-1; // if ctrl is out of range
    }

    return result;
}

char mddInputCtrl2(unsigned char ctrl)
{
   volatile signed char result =0;

    if ((ctrl & ~CTRLMASK2) == ZEROVAL) // check that cntrl is within 0x0 - 0x7
    {
        CTRLPORT = ((CTRLMASK & CTRLPORT)+ctrl) ;   //send ctrl to output
    }
    else
    {
        result =-1; // if ctrl is out of range
    }

    return result;
}
/* *******************************
 * char mddCW(unsigned char dutyCycle)
 *
 * clockwise ctrl
 * Executes CW control signals with requested duty cycle to vnh
 *
 * sets SEL low
 *
 * returns 0 if successful and -1 if not
 **********************************/
char mddCW(unsigned char dutyCycle)
{
    volatile signed char result =0;
    volatile signed char inputRet =0;
    volatile unsigned char dutyRet =0;

    clkWise =1;              // what the user is requesting
    countClkWise=0;          // change the old direction

    if (prevClkCountNot == 0){ // brake process if a change of direction is requested
        dutyPrev = dutyCycle;
        mddBrake();

    }
    else      // same direction requested
    {
        dutyRet = timerA0DutyCycleSet(dutyCycle); // send the previously pressed dutyCycle

        if (dutyRet ==0){    // if the duty was changed successfully
            inputRet = mddInputCtrl(CTRLCW); // send cntrl values to the port
            if (inputRet == -1)
                result = -1;
        }
        else{
            result = -1;
        }
    }

    return result;
}

char mddCW2(unsigned char dutyCycle)
{
    volatile signed char result =0;
    volatile signed char inputRet =0;
    volatile unsigned char dutyRet =0;

    clkWise2 =1;              // what the user is requesting
    countClkWise2=0;          // change the old direction

    if (prevClkCountNot2 == 0){ // brake process if a change of direction is requested
        dutyPrev2 = dutyCycle;
        mddBrake2();

    }
    else      // same direction requested
    {
        dutyRet = timerA0DutyCycleSet2(dutyCycle); // send the previously pressed dutyCycle

        if (dutyRet ==0){    // if the duty was changed successfully
            inputRet = mddInputCtrl2(CTRLCW2); // send cntrl values to the port
            if (inputRet == -1)
                result = -1;
        }
        else{
            result = -1;
        }
    }

    return result;
}
/* *******************************
 * char mddCCW(unsigned char dutyCycle)
 * Counter clockwise ctrl
 * Executes CW control signals with requested duty cycle to vnh
 *
 * sets SEL low
 *
 * returns0 if possible and -1 if not possible
 **********************************/
char mddCCW(unsigned char dutyCycle)
{
        volatile signed char result =0;
        volatile signed char inputRet =0;
        volatile unsigned char dutyRet =0;

        countClkWise = 1;   // what the user is requesting
        clkWise =0;         // change the old direction

        if (prevClkCountNot == 1){   // brake process if a change of direction is requested
                dutyPrev = dutyCycle;
                mddBrake();

        }
        else   // same direction requested
        {
            dutyRet = timerA0DutyCycleSet(dutyCycle);  // send the previously pressed dutyCycle

            if (dutyRet ==0){   // if the duty was changed successfully
                inputRet = mddInputCtrl(CTRLCCW);   // send cntrl values to the port
               if (inputRet == -1)
                  result = -1;
            }

            else{
                result = -1;
            }
        }

   return result;
}

char mddCCW2(unsigned char dutyCycle)
{
        volatile signed char result =0;
        volatile signed char inputRet =0;
        volatile unsigned char dutyRet =0;

        countClkWise2 = 1;   // what the user is requesting
        clkWise2 =0;         // change the old direction

        if (prevClkCountNot2 == 1){   // brake process if a change of direction is requested
                dutyPrev2 = dutyCycle;
                mddBrake2();

        }
        else   // same direction requested
        {
            dutyRet = timerA0DutyCycleSet2(dutyCycle);  // send the previously pressed dutyCycle

            if (dutyRet ==0){   // if the duty was changed successfully
                inputRet = mddInputCtrl2(CTRLCCW2);   // send cntrl values to the port
               if (inputRet == -1)
                  result = -1;
            }

            else{
                result = -1;
            }
        }

   return result;
}
/**********************************
 * function: char mddBrake()
 *
 * Executes vnh cntrl brake signal
 *
 * calls vnhInputCtrl
 * ramps up to the previous dutyCycle going the other direction
 *
 * pwm set HIGH
 * SEL set low
 *
 * Returns 0 if successful and -1 if not
 **********************************/
char mddBrake()
{
    volatile signed char result =0;
    volatile signed char inputRet =0;
    volatile unsigned char dutyRet =0;
    volatile unsigned char dutyEnd;

    dutyEnd = dutyPrev; // save the current dutyCycle

    inputRet = mddInputCtrl(CTRLBRAKE); // send braking ctrl values to output, INA low, INB low, sel Low

    if (inputRet != -1){
        dutyRet = timerA0DutyCycleSet(0); // set pwm low
        __delay_cycles(100000);// brake signal delay

        // decide if normal braking or if braking was used to change directions
        if (prevClkCountNot == 1 && clkWise == 1){ // same direction braking to stop
            inputRet = mddInputCtrl(CTRLCW); // send ctrl values to output to keep the same direction to CW
                if (inputRet == -1)
                     result = -1;
        }
        else if (prevClkCountNot == 0 && countClkWise == 1){ // same direction braking to stop
            inputRet = mddInputCtrl(CTRLCCW); // send ctrl values to output to keep the same direction to CCW
                if (inputRet == -1)
                      result = -1;
        }
        else if (prevClkCountNot == 0 && clkWise == 1){  // ramp back up to the previous dutyCycle
                  inputRet = mddInputCtrl(CTRLCW); // send ctrl values to output
                      if (inputRet != -1){
                          dutyRet = timerA0DutyCycleSet(dutyEnd); // set dutyCycle as it was previously
                               if (dutyRet == 0)
                                   prevClkCountNot = 1; // set new direction
                               else
                                   result = -1;
                      }
                      else
                         result = -1;
        }
        else if (prevClkCountNot == 1 && countClkWise == 1){ // ramp back up to the previous dutyCycle
                  inputRet = mddInputCtrl(CTRLCCW); // send ctrl values to output
                      if (inputRet != -1){
                          dutyRet = timerA0DutyCycleSet(dutyEnd); // set dutyCycle as the previous
                             if (dutyRet == 0)
                                 prevClkCountNot =0; // set new direction
                             else
                                 result = -1;
                      }
                      else
                         result = -1;

        }
        else
            result =-1;
    }
    else
    {
        result = -1;
    }

    return result;
}

char mddBrake2()
{
    volatile signed char result =0;
    volatile signed char inputRet =0;
    volatile unsigned char dutyRet =0;
    volatile unsigned char dutyEnd;

    dutyEnd = dutyPrev2; // save the current dutyCycle

    inputRet = mddInputCtrl2(CTRLBRAKE2); // send braking ctrl values to output, INA low, INB low, sel Low

    if (inputRet != -1){
        dutyRet = timerA0DutyCycleSet2(0); // set pwm low
        __delay_cycles(2000000);// brake signal delay

        // decide if normal braking or if braking was used to change directions
        if (prevClkCountNot2 == 1 && clkWise2 == 1){ // same direction braking to stop
            inputRet = mddInputCtrl2(CTRLCW2); // send ctrl values to output to keep the same direction to CW
                if (inputRet == -1)
                     result = -1;
        }
        else if (prevClkCountNot2 == 0 && countClkWise2 == 1){ // same direction braking to stop
            inputRet = mddInputCtrl2(CTRLCCW2); // send ctrl values to output to keep the same direction to CCW
                if (inputRet == -1)
                      result = -1;
        }
        else if (prevClkCountNot2 == 0 && clkWise2 == 1){  // ramp back up to the previous dutyCycle
                  inputRet = mddInputCtrl2(CTRLCW2); // send ctrl values to output
                      if (inputRet != -1){
                          dutyRet = timerA0DutyCycleSet2(dutyEnd); // set dutyCycle as it was previously
                               if (dutyRet == 0)
                                   prevClkCountNot2 = 1; // set new direction
                               else
                                   result = -1;
                      }
                      else
                         result = -1;
        }
        else if (prevClkCountNot2 == 1 && countClkWise2 == 1){ // ramp back up to the previous dutyCycle
                  inputRet = mddInputCtrl2(CTRLCCW2); // send ctrl values to output
                      if (inputRet != -1){
                          dutyRet = timerA0DutyCycleSet2(dutyEnd); // set dutyCycle as the previous
                             if (dutyRet == 0)
                                 prevClkCountNot2 =0; // set new direction
                             else
                                 result = -1;
                      }
                      else
                         result = -1;

        }
        else
            result =-1;
    }
    else
    {
        result = -1;
    }

    return result;
}

