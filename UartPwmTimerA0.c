/*
 * UartPwmTimerA0.c
 *
 *  Created on: Feb. 20, 2021
 *      Author: Rinz
 */
#include <msp430.h>
#include <ucsiUart.h>
#include <mdd_driver.h>
#include <UartpwmTimerA0.h>

/**************************************
 * Function: void timerA0Init()
 *
 *purpose: Initialize timerA0 to the correct settings
 *purpose: also configure the port settings
 *
 *returns nothing
 **************************************/
void timerA0Init(unsigned int pwmFreq){                                         // definit
    volatile unsigned char pwmFreqSetRet =0;

    TA0CCR1 = 0;  // CCR1 PWM duty cycle %
    TA0CCTL1 |= (OUTMOD_7); // reset set mode
    TA0EX0 = 0x004;     // bits 000  divide by 1

    pwmFreqSetRet = timerA0PwmFreqSet(pwmFreq);

    TA0CTL = (TASSEL_2 | ID_2 | MC_1); // Timer_A0 control register, SMCLK, SMCLK/1 , Up mode

    P1DIR |= BIT2;    // output pin 2    TA0.1 pin
    P1SEL |= BIT2;    // pin 2
    P1OUT &= ~BIT0;   // Reset pin






}
/* ***************************************
 * timerA0PwmFreqSet(unsigned int pwmFreq)
 *
 * Computes and sets TACCR0 value using pwmFreq.  Does not perform clock division on SMCLK
 *
 * pwmFreqmin and max needs to be computed by designer.  If the requested pwmFreq iss outside the
 * range then TACCR0 is not change and -1 is returned
 *
 * otherwise TACCR0 is computed to implement a pwm freqency equal to pwmFreq and 0 is returned
 *
 ****************************************/
char timerA0PwmFreqSet(unsigned int pwmFreq)
{
    volatile unsigned char result=0;

    if ((pwmFreq <= PWMFREQMAX) && (pwmFreq >= PWMFREQMIN))
    {
        TA0CCR0 = (1048000/pwmFreq)-1; // assign CCR0 value if possible
    }
    else
    {
        result =-1; // not possible
    }
    return result;
}

/* ***************************************
 * char timerA0DutyCycleSet(unsigned char dutyCycle)
 *
 * Computes and sets TACCRx register to the appropriate vallue using dutyCycle who's
 * range is integers 0:10
 *
 * returns 0.  if dutyCycle is not within the range 0:10 then return -1.
 *
 ****************************************/
char timerA0DutyCycleSet(unsigned char dutyCycle)
{
    volatile signed char value =0;
    volatile unsigned char num = 0;
    volatile unsigned char start = 0;
    volatile double percentDuty =0; // duty cycle as a percent
    volatile signed int sign =0;   // determine sign
    volatile char decreaseDuty =0; // 1 if ramping down
    volatile signed int ramp; // checking if a ramp is needed

    sign = dutyCycle - dutyPrev; // absolute value of difference
    if (sign <0)
        ramp = (-1) * sign;
    else
        ramp = sign;
    if (dutyCycle < dutyPrev) // ramp direction
        decreaseDuty =1;


     if (dutyCycle <= DUTYCYCLEMAX && dutyCycle >= 0){

      if ((clkWise == 1 && prevClkCountNot == 0) || (countClkWise == 1 && prevClkCountNot ==1) || (dutyCycle == DUTY_INC)){  // changing direction, holding PWM high
               //TA0CCR1 = TA0CCR0+1;  // pwm goes high for braking on vnh7070 board
               TA0CCR1 =0;             // pwm low for braking on cytron driver
               dutyPrev =0;
               __delay_cycles(5000);  // 1/Mclk * t =  delay for braking
      }

      if (ramp >= DUTY_RAMP_MIN && dutyCycle != 0){ // checking if ramping is needed
              if (decreaseDuty ==1)
                  num = (dutyPrev - dutyCycle);
              else
                  num = (dutyCycle - dutyPrev);
          for (start; start<num; start++){
              if (decreaseDuty == 1)
                  dutyPrev--;                            // increase the previous dutyCycle
              else
                  dutyPrev++;
              percentDuty = (double)dutyPrev/(double)DUTY_INC;
              TA0CCR1 = (percentDuty*(TA0CCR0+1));
              __delay_cycles(500);                 // 1/Mclk * t = .5second delay for ramping motor
          }
      }
      else{
          percentDuty = (double)dutyCycle/(double)DUTY_INC;
          TA0CCR1 = (percentDuty*(TA0CCR0+1));
          dutyPrev = dutyCycle;
          }
    }
    else
    {
        value =-1;
    }

    return value;
}








