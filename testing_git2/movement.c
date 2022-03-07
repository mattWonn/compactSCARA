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

    volatile unsigned int displacement1;
    volatile unsigned int displacement2;
    volatile unsigned int deltaD;
    volatile unsigned int deltaD2;
    volatile double timeForMove;
    volatile double vMaxMove;
    volatile double aMaxMove;
    volatile double vMaxMove2;
    volatile double aMaxMove2;
    volatile unsigned int masterJoint;


    if (endAng1 <= MAX_ABS_THETA1 && endAng1 >= -MAX_ABS_THETA1){ // check range
        if (endAng2 <= MAX_ABS_THETA2 && endAng2 >= -MAX_ABS_THETA2){ // check range
            displacement1 = abs(endAng1 - startAng1);
            displacement2 = abs(endAng2 - startAng2);

            if (displacement1 >= displacement2){ // determine which joint has to move the farthest and base calculations on that
                masterJoint =1;
                deltaD = displacement1;
                deltaD2 = displacement2;
            }
            else{
                masterJoint =2;
                deltaD = displacement2;
                deltaD2 = displacement1;
            }

            timeForMove = sqrt((6*deltaD)/A_MAX); // calc T for parabolic profile
            vMaxMove = (A_MAX*timeForMove)/4; // calc the Vmax for the move

            if (vMaxMove > W_MAX){// check within the limit of the motor, otherwise you have to recalculate everything
                vMaxMove = W_MAX;
                timeForMove = (3*deltaD)/(2*vMaxMove);
                aMaxMove = (6*deltaD)/(pow(timeForMove,2));

                aMaxMove2 = (6*deltaD2)/(pow(timeForMove,2));
                // no need to calc vMaxMove2 because it will be under limit guarenteed
            }
            else{ // if the velocity is within the limit
                aMaxMove = (6*deltaD)/(pow(timeForMove,2));
                aMaxMove2 = (6*deltaD2)/(pow(timeForMove,2));
            }

            // assign variables
            if (masterJoint == 1){
                scaraStateSet.scaraVel.controlJoint =1;
                scaraStateSet.scaraVel.timeMove =timeForMove;
                scaraStateSet.scaraVel.aMaxCTRLJoint =aMaxMove;
                scaraStateSet.scaraVel.vMaxCTRLJoint =vMaxMove;
                scaraStateSet.scaraVel.aMaxSlowJoint =aMaxMove2;
            }
            else if (masterJoint == 2){
                scaraStateSet.scaraVel.controlJoint =2;
                scaraStateSet.scaraVel.timeMove =timeForMove;
                scaraStateSet.scaraVel.aMaxCTRLJoint =aMaxMove;
                scaraStateSet.scaraVel.vMaxCTRLJoint =vMaxMove;
                scaraStateSet.scaraVel.aMaxSlowJoint =aMaxMove2;
            }


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
