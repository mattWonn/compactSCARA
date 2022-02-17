/*
 * Uartvnh7070hpi.h
 *
 *  Created on: Feb. 20, 2021
 *      Author: Rinz
 */

#ifndef MDD_DRIVER_H_
#define MDD_DRIVER_H_

#define CTRLPORT P3OUT

#define CTRLMASK (BIT0 + BIT1 +BIT2 )
#define ZEROVAL 0x00
#define INA BIT0 // P3.0
#define INB BIT1 // P3.1
#define SEL BIT2 // not used
#define CTRLOUT (P3DIR |= 0x7); (P3SEL |= 0x7)

#define CTRLMASK2 (BIT3 + BIT4 +BIT5)
#define INA2 BIT3 // P3.3
#define INB2 BIT4 // P3.4
#define SEL2 BIT5 // not used
#define CTRLOUT2 (P3DIR |= BIT3+BIT4+BIT5); (P3SEL |= BIT3+BIT4+BIT5)

#define CTRLCW (CTRLMASK & (INA & ~INB & ~SEL))
#define CTRLCCW (CTRLMASK & (INB & ~INA & ~SEL))
#define CTRLBRAKE (CTRLMASK & (~INA & ~INB & ~SEL))

#define CTRLCW2 (CTRLMASK2 & (INA2 & ~INB2 & ~SEL2))
#define CTRLCCW2 (CTRLMASK2 & (INB2 & ~INA2 & ~SEL2))
#define CTRLBRAKE2 (CTRLMASK2 & (~INA2 & ~INB2 & ~SEL2))

//-----------------------------------

#define NULLNUM 0x00
#define MAX_ARGS 2
#define MAX_CMDS 11

#define BUFFLEN 60


//-------- cmdInterpreter -----------
#define CMD0 "pwmFreqSet"
#define CMD0_NARGS 1
#define CMD1 "moveM1"
#define CMD1_NARGS 2
#define CMD2 "brakeM1"
#define CMD2_NARGS 0
#define CMD3 "displayPosM1"
#define CMD3_NARGS 0
#define CMD4 "moveJM1"
#define CMD4_NARGS 1
#define CMD5 "resetCountM1"
#define CMD5_NARGS 0

#define CMD6 "moveM2"
#define CMD6_NARGS 2
#define CMD7 "brakeM2"
#define CMD7_NARGS 0
#define CMD8 "displayPosM2"
#define CMD8_NARGS 0
#define CMD9 "moveJM2"
#define CMD9_NARGS 1
#define CMD10 "resetCountM2"
#define CMD10_NARGS 0

volatile unsigned char data;
volatile unsigned char x;
volatile unsigned char done;
volatile unsigned char rxBuffer[99];

typedef struct CMD {
    const char *name;   // command name
    int nArgs;          // number of input arguments for a command
    int args[MAX_ARGS]; // arguments
}CMD;

void motorCmdInit(CMD *vnhCmdList);
int parseCmd(CMD * cmdList, char * cmdLine);
int validateCmd(CMD *cmdList ,char * cmdName);
int executeCmd(CMD *cmdList, int cmdIndex);

//------- loop and position variables---------
//#define ANG_RES 0  // N =
//#define L1 5;
#define MAX_V 8.1 // 90% * 9V
#define MIN_V -8.1 // max in negative direction
#define MIN_V_MOVE 0.09 // 1% * 9V
#define SLOPE 0.0225
#define DEG_PER_PUL 0.128571

#define TRANS_FUNC_V_TO_PWM 11.1111 //90/8.1


volatile double angJ1Desired;
volatile double angJ2Desired;
volatile int enterLoop;
volatile int enterLoop2;


//------- motor section-----------------------------
#define PWMFREQMAX 20000     //18.935?  20kHz?
#define PWMFREQMIN 100       // 18.9 Hz
#define PWMFREQ 1000 // current pwm frequency

#define DUTYCYCLEMIN 0
#define DUTYCYCLEMAX 90
#define DUTY_INC 100
#define DUTY_RAMP_MIN 60

//---- CW and CCW / brake global vars--------------

unsigned char prevClkCountNot; // 1 = clockwise, 0 = counter clockwise
unsigned char countClkWise; // current direction
unsigned char clkWise; // current direction
unsigned char brake; // !!!!!!!!!! not used ?
unsigned int dutyPrev; // previous dutyCycle

unsigned char prevClkCountNot2; // 1 = clockwise, 0 = counter clockwise
unsigned char countClkWise2; // current direction
unsigned char clkWise2; // current direction
unsigned char brake2; // !!!!!!!!!! not used ?
unsigned int dutyPrev2; // previous dutyCycle

void displayPos();
char mddInputCtrl(unsigned char ctrl);
char mddCW(unsigned char dutyCycle);
char mddCCW(unsigned char dutyCycle);
char mddBrake();

void displayPos2();
char mddInputCtrl2(unsigned char ctrl);
char mddCW2(unsigned char dutyCycle);
char mddCCW2(unsigned char dutyCycle);
char mddBrake2();

#endif /* MDD_DRIVER_H_ */
