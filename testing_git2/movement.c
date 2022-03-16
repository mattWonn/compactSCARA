/*
 * movement.c
 *
 *  Created on: Mar. 6, 2022
 *      Author: Rinz
 */

#include <msp430.h>
#include "ucsiUart.h"
#include "mdd_driver.h"
#include "UartPwmTimerA0.h"
#include "quadEncDec.h"
#include "updateTimerB.h"
#include "UcsControl.h"
#include "movement.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

unsigned int moveJ(signed int startAng1, signed int endAng1, signed int startAng2, signed int endAng2){
    volatile unsigned int exit =0;

    volatile unsigned int x=0;
    volatile unsigned int displacement1;
    volatile unsigned int displacement2;
    volatile signed int direction1 = 0; // 0 is negative, 1 is positive direction
    volatile signed int direction2 = 0; // 0 is negative, 1 is positive direction
    volatile signed int deltaD;
    volatile signed int deltaD2;
    volatile double timeForMove;
    volatile double vMaxMove;
    volatile double aMaxMove;
    volatile double vMaxMove2;
    volatile double aMaxMove2;
    volatile unsigned int masterJoint;

    //----------------------------
    volatile unsigned int tInc = 0;
    volatile double velocityDegPerSec;
    volatile signed int velocityCountPerUpdate;
    volatile double positionDeg;
    volatile signed int positionCounts;

    volatile double velocityDegPerSec2;
    volatile signed int velocityCountPerUpdate2;
    volatile double positionDeg2;
    volatile signed int positionCounts2;
    volatile double w = 0;


 //     volatile char posPrint[45]; // Uart
//      volatile int ret;


    if (endAng1 <= MAX_ABS_THETA1 && endAng1 >= -MAX_ABS_THETA1){ // check range
        if (endAng2 <= MAX_ABS_THETA2 && endAng2 >= -MAX_ABS_THETA2){ // check range
            displacement1 = abs(endAng1 - startAng1);
            displacement2 = abs(endAng2 - startAng2);

            if (endAng1 == startAng1)
                noMove1 =1;
            else
                noMove1 =0;
            if (endAng2 == startAng2)
                noMove2 = 1;
            else
                noMove2 =0;

            if (displacement1 >= displacement2){ // determine which joint has to move the farthest and base calculations on that
                masterJoint =1;
                deltaD = (endAng1 - startAng1);
                deltaD2 = (endAng2 - startAng2);
                if (endAng1 >= startAng1)
                    direction1=1;
                if (endAng2 >= startAng2)
                    direction2=1;
            }
            else{
                masterJoint =2;
                deltaD = (endAng2 - startAng2);
                deltaD2 = (endAng1 - startAng1);
                if (endAng1 >= startAng1)
                    direction2=1;
                if (endAng2 >= startAng2)
                    direction1=1;
            }




            timeForMove = sqrt((abs(deltaD)*2*PI)/A_MAX); // calc T for parabolic profile
            w = (2*PI)/timeForMove;


            vMaxMove = (2*A_MAX)/w; // calc the Vmax for the move

            if (vMaxMove > W_MAX){// check within the limit of the motor, otherwise you have to recalculate everything
                vMaxMove = W_MAX;
                timeForMove = (2*abs(deltaD))/(W_MAX);
                aMaxMove = (abs(deltaD)*w)/(timeForMove);
                aMaxMove2 = (abs(deltaD2)*w)/(timeForMove);
            }
            else{ // if the velocity is within the limit
                aMaxMove = abs(A_MAX);
                aMaxMove2 = ((abs(deltaD2)*w)/timeForMove);
            }

            // assign variables
            if (masterJoint == 1){ // arm one moves further
                scaraStateSet.scaraVel.controlJoint =1;
                scaraStateSet.scaraVel.timeMove =timeForMove;

                w = (2*PI)/timeForMove;

                arrayLength = (timeForMove/T_UPDATE)+1;


                if (direction1 == 0) // relative to which joint moves the furthest
                    aMaxMove = -1*aMaxMove;
                if (direction2 == 0)
                    aMaxMove2 = -1*aMaxMove2;


                for(tInc; tInc<arrayLength; tInc++){

                    velocityDegPerSec = RadToDeg(DegToRad(aMaxMove)/w)  -  RadToDeg(DegToRad(aMaxMove)*(cos(w*(tInc*T_UPDATE)))/w);
                    velocityDegPerSec2 = RadToDeg(DegToRad(aMaxMove2)/w)  -  RadToDeg(DegToRad(aMaxMove2)*(cos(w*(tInc*T_UPDATE)))/w);
                    positionDeg = RadToDeg((DegToRad(aMaxMove)*(tInc*T_UPDATE))/w)  -  RadToDeg(DegToRad(aMaxMove)*(sin(w*(tInc*T_UPDATE)))/pow(w,2))+startAng1;
                    positionDeg2 = RadToDeg((DegToRad(aMaxMove2)*(tInc*T_UPDATE))/w)  -  RadToDeg(DegToRad(aMaxMove2)*(sin(w*(tInc*T_UPDATE)))/pow(w,2))+startAng2;

                    velocityCountPerUpdate = round(velocityDegPerSec*(7.7778037)*(T_UPDATE/1)); // in units pulses per update
                    velocityCountPerUpdate2 = round(velocityDegPerSec2*(PUL_PER_DEG_N70)*(T_UPDATE/1)); // in units pulses per update
                    positionCounts = round(positionDeg*7.7778037);
                    positionCounts2 = round(positionDeg2*PUL_PER_DEG_N70);


                    velArray1[tInc] = velocityCountPerUpdate;//round(velocityDegPerSec*(PUL_PER_DEG_N70)*(T_UPDATE/1)); // in units pulses per update
                    posArray1[tInc] = positionCounts;//round(positionDeg*PUL_PER_DEG_N70);
                    velArray2[tInc] = velocityCountPerUpdate2;//round(velocityDegPerSec2*(PUL_PER_DEG_N70)*(T_UPDATE/1));
                    posArray2[tInc] = positionCounts2;//round(positionDeg2*PUL_PER_DEG_N70);

                 /*   __disable_interrupt();
                     sprintf(posPrint, "Velocity1 = %d,   Position1 = %d \n\r", velArray1[tInc], posArray1[tInc]); // insert the number of characters into the display string
                     ret = ucsiA1UartTxString(&posPrint); // print the string
                     sprintf(posPrint, "Velocity2 = %d,   Position2 = %d \n\n\r", velArray2[tInc], posArray2[tInc]); // insert the number of characters into the display string
                     ret = ucsiA1UartTxString(&posPrint); // print the string
                     __enable_interrupt();*/
                }
            }
            else if (masterJoint == 2){ // arm two moves further
                scaraStateSet.scaraVel.controlJoint =2;
                scaraStateSet.scaraVel.timeMove =timeForMove;

                w = (2*PI)/timeForMove;

                if (direction1 == 0)
                    aMaxMove = -1*aMaxMove;
                if (direction2 == 0)
                    aMaxMove2 = -1*aMaxMove2;

                arrayLength = (timeForMove/T_UPDATE)+1;
                for(tInc; tInc<arrayLength; tInc++){

                    velocityDegPerSec = RadToDeg(DegToRad(aMaxMove)/w)  -  RadToDeg(DegToRad(aMaxMove)*(cos(w*(tInc*T_UPDATE)))/w);
                    velocityDegPerSec2 = RadToDeg(DegToRad(aMaxMove2)/w)  -  RadToDeg(DegToRad(aMaxMove2)*(cos(w*(tInc*T_UPDATE)))/w);
                    positionDeg = RadToDeg((DegToRad(aMaxMove)*(tInc*T_UPDATE))/w)  -  RadToDeg(DegToRad(aMaxMove)*(sin(w*(tInc*T_UPDATE)))/pow(w,2))+startAng2;
                    positionDeg2 = RadToDeg((DegToRad(aMaxMove2)*(tInc*T_UPDATE))/w)  -  RadToDeg(DegToRad(aMaxMove2)*(sin(w*(tInc*T_UPDATE)))/pow(w,2))+startAng1;

                    velocityCountPerUpdate = round(velocityDegPerSec*(PUL_PER_DEG_N70)*(T_UPDATE/1)); // in units pulses per update
                    velocityCountPerUpdate2 = round(velocityDegPerSec2*(7.778)*(T_UPDATE/1)); // in units pulses per update
                    positionCounts = round(positionDeg*PUL_PER_DEG_N70);
                    positionCounts2 = round(positionDeg2*7.778);


                    velArray2[tInc] = velocityCountPerUpdate;//round(velocityDegPerSec*(PUL_PER_DEG_N70)*(T_UPDATE/1)); // in units pulses per update
                    posArray2[tInc] = positionCounts;//round(positionDeg*PUL_PER_DEG_N70);
                    velArray1[tInc] = velocityCountPerUpdate2;//round(velocityDegPerSec2*(PUL_PER_DEG_N70)*(T_UPDATE/1));
                    posArray1[tInc] = positionCounts2;//round(positionDeg2*PUL_PER_DEG_N70);

              /*      __disable_interrupt();
                      sprintf(posPrint, "Velocity1 = %d,   Position1 = %d \n\r", velArray1[tInc], posArray1[tInc]); // insert the number of characters into the display string
                      ret = ucsiA1UartTxString(&posPrint); // print the string
                      sprintf(posPrint, "Velocity2 = %d,   Position2 = %d \n\n\r", velArray2[tInc], posArray2[tInc]); // insert the number of characters into the display string
                      ret = ucsiA1UartTxString(&posPrint); // print the string
                      __enable_interrupt();*/
                }
            }

       /*     x = arrayLength-1;
            for(x;x<201;x++){
            velArray2[x] = 0;//round(velocityDegPerSec*(PUL_PER_DEG_N70)*(T_UPDATE/1)); // in units pulses per update
            posArray2[x] = posArray2[arrayLength-1];//round(positionDeg*PUL_PER_DEG_N70);
            velArray1[x] = 0;//round(velocityDegPerSec2*(PUL_PER_DEG_N70)*(T_UPDATE/1));
            posArray1[x] = posArray1[arrayLength-1];//round(positionDeg2*PUL_PER_DEG_N70);
            }*/
        }
    }
    return (exit);
}

/***********************************************************
* Name: unsigned int scaraFk
* function: provides the calculations to control the robot with forward kinematics
* arguments
*            ang1:  (signed int - theta1
*            ang2:  (signed int - theta2
*            toolX: (pointer type double - points to x position of arm
*            toolY: (pointer type double - points to y position of arm
*
* created by: Matthew Wonneberg, Jamie Boyd
* Date: March 24 2020
* Modified: March 5 2022
************************************************************/
unsigned int scaraFk(signed int ang1, signed int ang2, double* toolX, double* toolY){

    volatile unsigned int exit = 0;
    volatile double toolX1, toolY1;
    volatile double thetaB, a;

    if (ang1 > MAX_ABS_THETA1) // if theta1 is over the limit return 1
        exit = 1;
    else if (ang2 > MAX_ABS_THETA2) // if theta2 is over the limit return 1
        exit =1;
    else
    {
        toolY1 = (L1 * sin(DegToRad(ang1))) + (L2 * sin(DegToRad(ang2))); // eqn for y coordinate
        toolX1 = (L1 * cos(DegToRad(ang1))) + (L2 * cos(DegToRad(ang2))); // eqn for x coordinate

        if (toolX1 <= MAX_ABS_X && toolX1 >= MIN_ABS_X) // check within range
        {
            *toolX = toolX1;   //assign X position
        }
        if (toolY1 <= MAX_ABS_Y && toolY1 >= MIN_ABS_Y) // check within range
        {
            *toolY = toolY1;   //assign Y position
        }

    }

    return (exit);
}

/***********************************************************
* Name: int scaraIk
* function: provides the calculations to control the robot with inverse kinematics
* arguments
*            ang1:  (pointer type signed int - points to theta1
*            ang2:  (pointer type signed int - points to theta2
*            toolX: (double - x position of arm
*            toolY: (double - y position of arm
*
*
* created by: Matthew Wonneberg, Jamie Boyd
* Date: april 4 2022
************************************************************/
unsigned int scaraIk(signed int *ang1, signed int *ang2, double toolX, double toolY, int *armSolution){

    volatile unsigned int exit = 0;

    volatile signed int angJ1;
    volatile signed int angJ2;

    volatile double B;     // length from origin to x,y
    volatile double beta;  // cosine law angle
    volatile double alpha; // angle of x,y


    B = sqrt(pow(toolX, 2) + pow(toolY, 2)); // straight line distance from origin to (x,y) point
    alpha = RadToDeg(atan2(toolY, toolX)); // angle of B from origin to (x,y) point
    beta = RadToDeg(acos((pow(L2, 2) - pow(B, 2) - pow(L1, 2)) / (-2 * B * L1))); // cosine law to find beta

    if (*armSolution == 1) { // left hand solution
        angJ1 = alpha + beta;
        if (angJ1 >= MAX_ABS_THETA1 || angJ1 <= -MAX_ABS_THETA1) { // switch solutions if the selected solution was impossible
            angJ1 = alpha - beta;
            *armSolution =0; // changed to Right hand solution
            if (angJ1 > MAX_ABS_THETA1 || angJ1 <= -MAX_ABS_THETA1)
                exit = 1;       // error if both solutions do not work
           // else
               // printf("\nYour solution was changed to Right Hand\n");
        }
    }
    if (*armSolution == 0) { // right hand solution
        angJ1 = alpha - beta;
        if (angJ1 <= -MAX_ABS_THETA1 || angJ1 >= MAX_ABS_THETA1) { // switch solutions if the selected solution was not possible
            angJ1 = alpha + beta;
            *armSolution =1; // changed to left hand solution
            if (angJ1 <= -MAX_ABS_THETA1 || angJ1 >= MAX_ABS_THETA1)
                exit = 1;        // error if both solutions do not work
           // else
                //printf("\nYour solution was changed to Left Hand\n");
        }
    }

    angJ2 = RadToDeg(atan2(toolY - (L1 * sin(DegToRad(angJ1))), toolX - (L1 * cos(DegToRad(angJ1)))));  // calculate joint2 angle
    if (angJ2 < -MAX_ABS_THETA2 || angJ2 > MAX_ABS_THETA2)
        exit = 1; // error if joint 2 angle is impossible to reach

    if (exit == 0) { // if the solution is possible then update structure values
        *ang1 = round(angJ1);
        *ang2 = round(angJ2);
    }
  //  else if (exit == 1)
       // printf("\nERROR detected!\n"); // print error message if something went wrong

    return (exit);
}
//---------------------------------------------------------------------------------------
// Returns angle in radians from input angle in degrees
double DegToRad(double angDeg)
{
   return (PI/180.0)*angDeg;
}

//---------------------------------------------------------------------------------------
// Returns angle in radians from input angle in degrees
double RadToDeg(double angRad)
{
   return (180.0/PI)*angRad;
}
