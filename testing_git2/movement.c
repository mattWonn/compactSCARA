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


            timeForMove = sqrt((abs(deltaD)*2*PI)/A_MAX); // calc T for sinusoidal profile
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

            //-------------- assign variables-------------------
            if (masterJoint == 1){ // arm one moves further
            //    scaraStateSet.scaraVel.controlJoint =1;
           //     scaraStateSet.scaraVel.timeMove =timeForMove;
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

                }
            }
            else if (masterJoint == 2){ // arm two moves further
          //      scaraStateSet.scaraVel.controlJoint =2;
          //      scaraStateSet.scaraVel.timeMove =timeForMove;

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

                }
           }
        }
        else
            exit = 1;
    }
    else
        exit = 1;

    return (exit);
}

/***********************************************************
* Name: int moveScaraL
* function: used to calculate a linear move with the tool by
*           using small moveJ increments.
* arguments
*            SCARA_ROBOT *scaraState1:  pointer variable of type SCARA_ROBOT
*            LINE_DATA newLine: newLine contains both start and end x and y variables as well as number of points in the line
*
* returns value
* created by: Matthew Wonneberg, Jamie Boyd
* Date: March 17 2022
************************************************************/
int moveScaraL(SCARA_ROBOT* scaraState, LINE_DATA newLine){

    volatile int value=0;
    volatile unsigned int returned =0;

    volatile double deltaX;
    volatile double deltaY;
    volatile unsigned int deltaD;
    volatile unsigned int arraySize;
    volatile unsigned int pathPlanError;
    volatile unsigned int armSolution;

    volatile double timeForMove;
    volatile double w = 0;
    volatile double d = 0;
    volatile double xHold =0;
    volatile double yHold =0;
    volatile unsigned int attemptedArmSolution;
    volatile signed int holdPosition1 = 0;
    volatile signed int holdPosition2 = 0;
    volatile double aMaxMove;

    volatile unsigned int tInc = 0;

    volatile unsigned int startingQuadrent = 0;
    volatile unsigned int endingQuadrent = 0;

    if ((newLine.pA.x < 0 && newLine.pA.y < 0) || (newLine.pB.x < 0 && newLine.pB.y < 0))
        scaraState->scaraPos.armSol = LEFT_ARM_SOLUTION;//quad 3
    else if ((newLine.pA.x < 0 && newLine.pA.y > 0) || (newLine.pB.x < 0 && newLine.pB.y > 0))
        scaraState->scaraPos.armSol = RIGHT_ARM_SOLUTION;// quad 2
    else if ((newLine.pA.x < 0 && newLine.pA.y > 0) && (newLine.pB.x < 0 && newLine.pB.y < 0))
        value = 1;
    else if ((newLine.pA.x < 0 && newLine.pA.y < 0) && (newLine.pB.x < 0 && newLine.pB.y > 0))
        value = 1;

    // else arm solution stays the same as it was assigned

    deltaX = newLine.pB.x - newLine.pA.x;
    deltaY = newLine.pB.y - newLine.pA.y;
    deltaD = (sqrt(pow(deltaX, 2) + pow(deltaY, 2))); // pythagreom theorum for line distance in x,y

    timeForMove = sqrt((abs(deltaD)*2*PI)/A_MAX_LINEAR); // calc T for sinusoidal profile
    w = (2*PI)/timeForMove;

    arrayLength = (timeForMove/T_UPDATE)+1;
    aMaxMove = A_MAX_LINEAR;

    for(tInc; tInc<arrayLength; tInc++){ // calculate linear array in terms of d(t) and then fill X and Y positions
        d = (((aMaxMove)*(tInc*T_UPDATE))/w)  -  ((aMaxMove)*(sin(w*(tInc*T_UPDATE)))/pow(w,2));
        xHold = newLine.pA.x + (d*(deltaX)/deltaD);
        yHold = newLine.pA.y + (d*(deltaY)/deltaD);
        returned = scaraIk(&(posArray1[tInc]), &(posArray2[tInc]), xHold, yHold, scaraState);

        if (returned == 0){
            posArray1[tInc] = round(posArray1[tInc]*7.778); // calculate position in terms of pulses
            posArray2[tInc] = round(posArray2[tInc]*PUL_PER_DEG_N70);

            velArray1[tInc] = posArray1[tInc]-holdPosition1; // calculate velocity in terms of pulses per update time
            velArray2[tInc] = posArray2[tInc]-holdPosition2;

            holdPosition1 = posArray1[tInc]; // store previous position
            holdPosition2 = posArray2[tInc];
        }
        else{
            value = 1; // exit calculations if the move is not possible
            tInc = arrayLength;
        }
    }

    return (value);
}

/***********************************************************
* Name: void pathPlanning
* function: finds which armsolutions are possible for each point that it is sent
* arguments
*            SCARA_ROBOT *line:  pointer variable of type SCARA__ROBOT that has x and y coordinates
*                                and has the armSol changed through it
*
* returns:
* created by: Matthew Wonneberg, Jamie Boyd
* Date: March 17 2022
************************************************************/
/*void pathPlanning(SCARA_ROBOT* line)
{
    volatile unsigned int value =1;

    double theta1 = line->scaraArm.theta1;
    double theta2 = line->scaraArm.theta2;
    volatile signed int leftSolution=0;
    volatile signed int rightSolution=0;

    if (line->scaraArm.x >= 0){ // if quadrent 1 or 4 then both solutions work
        leftSolution = 1;
        rightSolution = 1;
    }
    if (line->scaraArm.y >= 0 && line->scaraArm.x <=0){ // if quadrent 2 then the right arm is the only solution
        leftSolution = -1;
        rightSolution = 1;
    }
    if (line->scaraArm.y <= 0 && line->scaraArm.x <=0){ // if quadrent 3 then the left arm is the only solution
        leftSolution = 1;
        rightSolution = -1;
    }

    if (leftSolution == 1 && rightSolution == -1){ // set solution to left arm
        line->scaraArm.armSol = LEFT_ARM_SOLUTION;
    }
    else if (leftSolution == -1 && rightSolution == 1){ // set solution to right arm
        line->scaraArm.armSol = RIGHT_ARM_SOLUTION;
    }
    else if (leftSolution == -1 && rightSolution == -1){ // no solution
        line->scaraArm.armSol = 2;
    }
    // if both solutions work then do nothing , armSol is already defined

}*/
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

        if (toolX1 <= MAX_ABS_X && toolX1 >= MIN_ABS_X){ // check within range
            *toolX = toolX1;   //assign X position
        }
        if (toolY1 <= MAX_ABS_Y && toolY1 >= MIN_ABS_Y){ // check within range
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
* Date: March 5 2022
************************************************************/
unsigned int scaraIk(signed int *ang1, signed int *ang2, double toolX, double toolY, SCARA_ROBOT *scaraState1){

    volatile unsigned int exit = 0;
    volatile signed int angJ1;
    volatile signed int angJ2;
    volatile double B;     // length from origin to x,y
    volatile double beta;  // cosine law angle
    volatile double alpha; // angle of x,y

    B = sqrt(pow(toolX, 2) + pow(toolY, 2)); // straight line distance from origin to (x,y) point
    alpha = RadToDeg(atan2(toolY, toolX)); // angle of B from origin to (x,y) point
    beta = RadToDeg(acos((pow(L2, 2) - pow(B, 2) - pow(L1, 2)) / (-2 * B * L1))); // cosine law to find beta

    if (scaraState1->scaraPos.armSol == LEFT_ARM_SOLUTION) { // left hand solution
        angJ1 = alpha + beta;
        if (angJ1 > MAX_ABS_THETA1 || angJ1 < -MAX_ABS_THETA1){  // switch solutions if the selected solution was impossible
            angJ1 = alpha - beta;
            scaraState1->scaraPos.armSol = 0; // changed to Right hand solution
            if (angJ1 > MAX_ABS_THETA1 || angJ1 < -MAX_ABS_THETA1)
                exit =1;
        }
    }
    else if (scaraState1->scaraPos.armSol == RIGHT_ARM_SOLUTION) { // right hand solution
        angJ1 = alpha - beta;
        if (angJ1 < -MAX_ABS_THETA1 || angJ1 > MAX_ABS_THETA1){  // switch solutions if the selected solution was not possible
            angJ1 = alpha + beta;
            scaraState1->scaraPos.armSol = 1; // changed to left hand solution
            if (angJ1 < -MAX_ABS_THETA1 || angJ1 > MAX_ABS_THETA1)
                exit =1;
        }
    }

    angJ2 = RadToDeg(atan2(toolY - (L1 * sin(DegToRad(angJ1))), toolX - (L1 * cos(DegToRad(angJ1)))));  // calculate joint2 angle
    if (angJ2 < -MAX_ABS_THETA2 || angJ2 > MAX_ABS_THETA2)
        exit = 1; // error if joint 2 angle is impossible to reach

    if (exit == 0) { // if the solution is possible then update structure values
        *ang1 = round(angJ1);
        *ang2 = round(angJ2);
    }

    return (exit);
}

/***********************************************************
* Name: LINE_DATA initLine
* function: used to set up the points of the line between the start and end points
* arguments
*            double xA:  x value for the robots start position
*            double yA:  y value for the robots start position
*            double xB:  x value for where the user wants the robot to end
*            double yB:  y value for where the user wants the robot to end
*            int numPts:  the number of points in the line that is used
*
* returns: LINE_DATA lineInit
* created by: Matthew Wonneberg, Jamie Boyd
* Date: March 17 2022
************************************************************/
LINE_DATA initLine(double xB, double yB, double xA, double yA, int numPts)
{
    LINE_DATA lineInit;

      lineInit.pA.x = xA; // pointsA are start points of the line
      lineInit.pA.y = yA;

      lineInit.pB.x = xB; // pointsB are end points of the line
      lineInit.pB.y = yB;
      lineInit.numPts = numPts; // num pts is the number of points that the robot moves by

    return (lineInit);
}


/***********************************************************
* Name: SCARA_ROBOT scaraInitState
* function: used to set up scaraState in with user entered values
* arguments
*            double x:  x value for where the user wants the robot to go
*            double y:  y value for where the user wants the robot to go
*            int armSol: either leftarm solution or rightarm solution 1 or 0
*            char mtrSpeed: either H, M, L
*
* returns SCARA_ROBOT Init
* created by: Matthew Wonneberg, Jamie Boyd
* Date: March 17
************************************************************/
SCARA_ROBOT scaraInitState(double x, double y, int armSol,char penState)
{
    SCARA_ROBOT Init;


    Init.scaraPos.x = x; // whatever the user enters for x and y position
    Init.scaraPos.y = y;
    Init.scaraPos.theta1 = 0;  // theta 1  and 2 are set to 0 because they are yet to be calculated
    Init.scaraPos.theta2 = 0;
    Init.scaraPos.armSol = armSol;     // armSol and penState and mtrSpeed are defined
    Init.scaraTool.penPos = penState;

    return Init;
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
