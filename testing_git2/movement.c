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


unsigned int scaraIk(signed int *ang1, signed int* ang2, double toolX, double toolY, int armSolution){

    volatile unsigned int exit = 0;

    volatile signed int angJ1;
    volatile signed int angJ2;

    volatile double B;     // length from origin to x,y
    volatile double beta;  // cosine law angle
    volatile double alpha; // angle of x,y


    B = sqrt(pow(toolX, 2) + pow(toolY, 2)); // straight line distance from origin to (x,y) point
    alpha = RadToDeg(atan2(toolY, toolX)); // angle of B from origin to (x,y) point
    beta = RadToDeg(acos((pow(L2, 2) - pow(B, 2) - pow(L1, 2)) / (-2 * B * L1))); // cosine law to find beta

    if (armSolution == 1) { // left hand solution
        angJ1 = alpha + beta;
        if (angJ1 >= MAX_ABS_THETA1 || angJ1 <= -MAX_ABS_THETA1) { // switch solutions if the selected solution was impossible
            angJ1 = alpha - beta;
            armSolution =0; // changed to Right hand solution
            if (angJ1 > MAX_ABS_THETA1 || angJ1 <= -MAX_ABS_THETA1)
                exit = 1;       // error if both solutions do not work
           // else
               // printf("\nYour solution was changed to Right Hand\n");
        }
    }
    if (armSolution == 0) { // right hand solution
        angJ1 = alpha - beta;
        if (angJ1 <= -MAX_ABS_THETA1 || angJ1 >= MAX_ABS_THETA1) { // switch solutions if the selected solution was not possible
            angJ1 = alpha + beta;
            armSolution =1; // changed to left hand solution
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
        scaraState.scaraArm.theta1 = round(angJ1);
        scaraState.scaraArm.theta2 = round(angJ2);
        scaraState.scaraArm.armSol = armSolution;
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
