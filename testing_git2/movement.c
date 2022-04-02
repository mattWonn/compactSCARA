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

unsigned int sendMoveJ(SCARA_ROBOT scaraStateStart, SCARA_ROBOT scaraStateEnd){
    unsigned int exit =0;

    __disable_interrupt();
    exit = moveJ(scaraStateStart.scaraPos.theta1,scaraStateEnd.scaraPos.theta1,scaraStateStart.scaraPos.theta2,scaraStateEnd.scaraPos.theta2); // start end M1, start end M2;
    if (exit == 0){
        __enable_interrupt();
        startMoveJ = 1;
        while (startMoveJ == 1){}
        __delay_cycles(10000);
        startMoveJ =0;
    }

    return exit;
}


unsigned int moveJ(signed int startAng1, signed int endAng1, signed int startAng2, signed int endAng2){

    unsigned int exit =0;
    unsigned int x=0;
    unsigned int displacement1;
    unsigned int displacement2;
    signed int direction1 = 0; // 0 is negative, 1 is positive direction
    signed int direction2 = 0; // 0 is negative, 1 is positive direction
    signed int deltaD;
    signed int deltaD2;
    float timeForMove;
    float vMaxMove;
    float aMaxMove;
    float vMaxMove2;
    float aMaxMove2;
    unsigned int masterJoint;

    //----------------------------
    unsigned int tInc = 0;
    float velocityDegPerSec;
    signed int velocityCountPerUpdate;
    float positionDeg;
    signed int positionCounts;

    float velocityDegPerSec2;
    signed int velocityCountPerUpdate2;
    float positionDeg2;
    signed int positionCounts2;
    float w = 0;


 //     volatile char posPrint[45]; // Uart
//      volatile int ret;


    if (endAng1 <= MAX_ABS_THETA1_PUL && endAng1 >= -MAX_ABS_THETA1_PUL){ // check range
        if (endAng2 <= MAX_ABS_THETA2_PUL && endAng2 >= -MAX_ABS_THETA2_PUL){ // check range
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


            timeForMove = sqrt((abs(deltaD)*2*PI)/A_MAX_PUL); // calc T for sinusoidal profile
            w = (2*PI)/timeForMove;
            vMaxMove = (2*A_MAX_PUL)/w; // calc the Vmax for the move

            if (vMaxMove > W_MAX_PUL){// check within the limit of the motor, otherwise you have to recalculate everything
                vMaxMove = W_MAX_PUL;
                timeForMove = (2*abs(deltaD))/(W_MAX_PUL);
                aMaxMove = (abs(deltaD)*w)/(timeForMove);
                aMaxMove2 = (abs(deltaD2)*w)/(timeForMove);
            }
            else{ // if the velocity is within the limit
                aMaxMove = abs(A_MAX_PUL);
                aMaxMove2 = ((abs(deltaD2)*w)/timeForMove);
            }

            //-------------- assign variables-------------------
            if (masterJoint == 1){ // arm one moves further

                w = (2*PI)/timeForMove;
                arrayLength = (timeForMove/T_UPDATE)+1;

                if (direction1 == 0) // relative to which joint moves the furthest
                    aMaxMove = -1*aMaxMove;
                if (direction2 == 0)
                    aMaxMove2 = -1*aMaxMove2;

                for(tInc; tInc<arrayLength; tInc++){

                    velArray1[tInc] = (RadToPul(PulToRad(aMaxMove)/w)  -  RadToPul(PulToRad(aMaxMove)*(cos(w*(tInc*T_UPDATE)))/w))*T_UPDATE;
                    velArray2[tInc] = (RadToPul(PulToRad(aMaxMove2)/w)  -  RadToPul(PulToRad(aMaxMove2)*(cos(w*(tInc*T_UPDATE)))/w))*T_UPDATE;
                    posArray1[tInc] = RadToPul((PulToRad(aMaxMove)*(tInc*T_UPDATE))/w)  -  RadToPul(PulToRad(aMaxMove)*(sin(w*(tInc*T_UPDATE)))/pow(w,2))+startAng1;
                    posArray2[tInc] = RadToPul((PulToRad(aMaxMove2)*(tInc*T_UPDATE))/w)  -  RadToPul(PulToRad(aMaxMove2)*(sin(w*(tInc*T_UPDATE)))/pow(w,2))+startAng2;

                    if ((posArray2[tInc] > (posArray1[tInc]+ RELATIVE_THETA2_PUL)) || (posArray2[tInc] < (-RELATIVE_THETA2_PUL + posArray1[tInc]))){ // on the fly max theta2 value verification
                        exit = 1;
                        tInc = arrayLength;
                    }
                }
            }
            else if (masterJoint == 2){ // arm two moves further

                w = (2*PI)/timeForMove;
                arrayLength = (timeForMove/T_UPDATE)+1;

                if (direction1 == 0)
                    aMaxMove = -1*aMaxMove;
                if (direction2 == 0)
                    aMaxMove2 = -1*aMaxMove2;

                for(tInc; tInc<arrayLength; tInc++){

                    velArray2[tInc] = (RadToPul(PulToRad(aMaxMove)/w)  -  RadToPul(PulToRad(aMaxMove)*(cos(w*(tInc*T_UPDATE)))/w))*T_UPDATE;
                    velArray1[tInc] = (RadToPul(PulToRad(aMaxMove2)/w)  -  RadToPul(PulToRad(aMaxMove2)*(cos(w*(tInc*T_UPDATE)))/w))*T_UPDATE;
                    posArray2[tInc] = RadToPul((PulToRad(aMaxMove)*(tInc*T_UPDATE))/w)  -  RadToPul(PulToRad(aMaxMove)*(sin(w*(tInc*T_UPDATE)))/pow(w,2))+startAng1;
                    posArray1[tInc] = RadToPul((PulToRad(aMaxMove2)*(tInc*T_UPDATE))/w)  -  RadToPul(PulToRad(aMaxMove2)*(sin(w*(tInc*T_UPDATE)))/pow(w,2))+startAng2;

                    if ((posArray2[tInc] > (posArray1[tInc]+ RELATIVE_THETA2_PUL)) || (posArray2[tInc] < (-RELATIVE_THETA2_PUL + posArray1[tInc]))){ // on the fly max theta2 value verification
                        exit = 1;
                        tInc = arrayLength;
                    }
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

unsigned int sendMoveL(SCARA_ROBOT *scaraStateSolution, LINE_DATA drawLine){

    volatile unsigned int result =0;
    volatile signed int angleJ1;
    volatile signed int angleJ2;
    volatile unsigned int originalArmSolution;

    originalArmSolution = scaraStateSolution->scaraPos.armSol;

    __disable_interrupt();
    result = moveScaraL(scaraStateSolution, drawLine);
    if(result == 3){
        result = moveScaraL(scaraStateSolution, holdLine); // after first arm solution was not successfull, calculate part of the first line with the original arm solution
        if(result == 0){
            __enable_interrupt();
            __delay_cycles(10000);
            startMoveJ=1;
            while (startMoveJ == 1){}
            startMoveJ =0;
            __disable_interrupt();
            /**********TOOLUP************/
            armSwitchSol =0;

            result = scaraIk(&angleJ1, &angleJ2, holdLine.pB.x, holdLine.pB.y, scaraStateSolution); // only changing the solution
            if (result == 0){
                scaraStateSet.scaraPos.theta1 = angleJ1; // start spot, old solution
                scaraStateSet.scaraPos.theta2 = angleJ2;
                if (originalArmSolution == LEFT_ARM_SOLUTION)
                    scaraStateSolution->scaraPos.armSol = RIGHT_ARM_SOLUTION;//switch arm solutions
                else
                    scaraStateSolution->scaraPos.armSol = LEFT_ARM_SOLUTION;//return arm solution

                result = scaraIk(&angleJ1, &angleJ2, endLine.pA.x, endLine.pA.y, scaraStateSolution); // only changing the solution
                if (result == 0){
                    scaraStateEnd.scaraPos.theta1 = angleJ1; // same spot, new solution
                    scaraStateEnd.scaraPos.theta2 = angleJ2;

                    result = sendMoveJ(scaraStateSet, scaraStateEnd); // start end M1, start end M2;
                    if (result == 0){
                        endLine = initLine(armChangeEnd.x, armChangeEnd.y, armChangeStart.x, armChangeStart.y, 0);//xb yb xa ya npts
                        result = moveScaraL(scaraStateSolution, endLine);
                        if(result == 0){
                            __enable_interrupt();
                            __delay_cycles(10000);
                            startMoveJ=1;
                            while (startMoveJ == 1){}
                            startMoveJ =0;
                        }
                    }
                }
            }
        }
    }
    else if(result == 0){
        __enable_interrupt();
        __delay_cycles(10000);
        startMoveJ=1;
        while (startMoveJ == 1){}
        startMoveJ =0;
    }


    return result;
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

    volatile unsigned int value=0;
    volatile unsigned int returned =0;

    signed int deltaX;
    signed int deltaY;
    unsigned int deltaD;
    unsigned int arraySize;
    volatile unsigned int armSolution;

    float timeForMove;
    float w = 0;
    volatile float d = 0;
    volatile float xHoldPrev =0;
    volatile float yHoldPrev =0;
    volatile unsigned int attemptedArmSolution;
    volatile signed int holdPosition1 = 0;
    volatile signed int holdPosition2 = 0;
    volatile unsigned int aMaxMove;

    volatile unsigned int tInc = 0;

    signed int currentAngJ1;
    signed int currentAngJ2;
    volatile signed int angleJ1;
    volatile signed int angleJ2;

    SCARA_ROBOT scaraStateSet;
    SCARA_ROBOT scaraStateEnd;


    // multiplied so that floats are not used in computation
    newLine.pA.x = newLine.pA.x*XY_RES_FACTOR;
    newLine.pA.y = newLine.pA.y*XY_RES_FACTOR;
    newLine.pB.x = newLine.pB.x*XY_RES_FACTOR;
    newLine.pB.y = newLine.pB.y*XY_RES_FACTOR;
    // call Ik using newLine and armPos to get the starting angles
    // compare the starting angles with posCount
    // if they are different, do a moveJ with the tool up

    returned = scaraIkPulses(&angleJ1, &angleJ2, newLine.pA.x, newLine.pA.y, scaraState); // checking the starting angles for the given solution
    currentAngJ1 = posCount;
    currentAngJ2 = posCount2;


    if (currentAngJ1 != angleJ1 || currentAngJ2 != angleJ2){
        /*******TOOL_UP**************/
        scaraStateSet.scaraPos.theta1 = currentAngJ1;
        scaraStateSet.scaraPos.theta2 = currentAngJ2;

        scaraStateEnd.scaraPos.theta1 = angleJ1;
        scaraStateEnd.scaraPos.theta2 = angleJ2;

        value = sendMoveJ(scaraStateSet, scaraStateEnd); // start end M1, start end M2;
        /*******RETURN TOOL**********/
    }

    if (value == 0){
        attemptedArmSolution = scaraState->scaraPos.armSol;

        deltaX = newLine.pB.x - newLine.pA.x;
        deltaY = newLine.pB.y - newLine.pA.y;
        deltaD = (sqrt(pow(deltaX, 2) + pow(deltaY, 2))); // pythagreom theorum for line distance in x,y

        timeForMove = (sqrt((abs(deltaD/XY_RES_FACTOR)*2*PI)/A_MAX_LINEAR));
        w = (2*(PI))/timeForMove;

        arrayLength = (timeForMove/T_UPDATE)+1;

        aMaxMove = A_MAX_LINEAR;

        holdPosition1 = currentAngJ1;
        holdPosition2 = currentAngJ2;

        for(tInc; tInc<arrayLength; tInc++){ // calculate linear array in terms of d(t) and then fill X and Y positions
            d = ((aMaxMove*(tInc*T_UPDATE))/w -  (aMaxMove)*(sin(w*(tInc*T_UPDATE)))/pow(w,2))*XY_RES_FACTOR;
            xHold = newLine.pA.x + (d*(deltaX)/deltaD);
            yHold = newLine.pA.y + (d*(deltaY)/deltaD);

            if ((abs(xHold) < INNER_CIRCLE_BOUNDS*XY_RES_FACTOR) && (abs(yHold) < INNER_CIRCLE_BOUNDS*XY_RES_FACTOR)){ // inner circle violated
                value = 1;
                tInc = arrayLength;
            }
            else{
            returned = scaraIkPulses(&(posArray1[tInc]), &(posArray2[tInc]), xHold, yHold, scaraState);

            if (returned == 0){
                if (armSolChange == 1){
                    newLine.pA.x = newLine.pA.x/XY_RES_FACTOR;
                    newLine.pA.y = newLine.pA.y/XY_RES_FACTOR;
                    newLine.pB.x = newLine.pB.x/XY_RES_FACTOR;
                    newLine.pB.y = newLine.pB.y/XY_RES_FACTOR;

                    armChangeStart.x = xHoldPrev/XY_RES_FACTOR;
                    armChangeStart.y = yHoldPrev/XY_RES_FACTOR;
                    armChangeEnd.x = newLine.pB.x/XY_RES_FACTOR;
                    armChangeEnd.y = newLine.pB.y/XY_RES_FACTOR;
                    holdLine = initLine(xHoldPrev/XY_RES_FACTOR, yHoldPrev/XY_RES_FACTOR, newLine.pA.x/XY_RES_FACTOR, newLine.pA.y/XY_RES_FACTOR, 0);//xb yb xa ya npts
                    endLine = initLine(newLine.pB.x/XY_RES_FACTOR, newLine.pB.y/XY_RES_FACTOR, armChangeStart.x/XY_RES_FACTOR, armChangeStart.y/XY_RES_FACTOR, 0);

                    scaraState->scaraPos.armSol = attemptedArmSolution; // return to original arm solution
                    armSolChange =0;
                    armSwitchSol =0;
                    solutionMoveJ2 =1; // set second solution change
                    return(3);
                }
                xHoldPrev = xHold;
                yHoldPrev = yHold;

                if ((posArray2[tInc] > (posArray1[tInc]+1375)) || (posArray2[tInc] < (-1375 + posArray1[tInc]))){ //  theta2 value verification
                    value = 1;
                    tInc = arrayLength;
                }
                else{

                velArray1[tInc] = posArray1[tInc]-holdPosition1; // calculate velocity in terms of pulses per update time
                velArray2[tInc] = posArray2[tInc]-holdPosition2;

                holdPosition1 = posArray1[tInc]; // store previous position
                holdPosition2 = posArray2[tInc];
                }

            }
            else{
                value = 1; // exit calculations if the move is not possible
            }
            }
        }
    }

    return (value);
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

    unsigned int exit = 0;
    signed int angJ1;
    signed int angJ2;
    float B;     // length from origin to x,y
    float beta;  // cosine law angle
    float alpha; // angle of x,y


    B = sqrt(pow(toolX, 2) + pow(toolY, 2)); // straight line distance from origin to (x,y) point
    alpha = RadToDeg(atan2(toolY, toolX)); // angle of B from origin to (x,y) point
    beta = RadToDeg(acos((pow(L2, 2) - pow(B, 2) - pow(L1, 2)) / (-2 * B * L1))); // cosine law to find beta

    if (scaraState1->scaraPos.armSol == LEFT_ARM_SOLUTION) { // left hand solution
        angJ1 = alpha + beta;
        if (angJ1 > MAX_ABS_THETA1 || angJ1 < -MAX_ABS_THETA1){  // switch solutions if the selected solution was impossible
            angJ1 = alpha - beta;
            scaraState1->scaraPos.armSol = RIGHT_ARM_SOLUTION; // changed to Right hand solution
            armSolChange = 1;
            if (angJ1 > MAX_ABS_THETA1 || angJ1 < -MAX_ABS_THETA1)
                exit =1;
        }
    }
    else if (scaraState1->scaraPos.armSol == RIGHT_ARM_SOLUTION) { // right hand solution
        angJ1 = alpha - beta;
        if (angJ1 < -MAX_ABS_THETA1 || angJ1 > MAX_ABS_THETA1){  // switch solutions if the selected solution was not possible
            angJ1 = alpha + beta;
            scaraState1->scaraPos.armSol = LEFT_ARM_SOLUTION; // changed to left hand solution
            armSolChange = 1;
            if (angJ1 < -MAX_ABS_THETA1 || angJ1 > MAX_ABS_THETA1)
                exit =1;
        }
    }

    angJ2 = RadToDeg(atan2(toolY - (L1 * sin(DegToRad(angJ1))), toolX - (L1 * cos(DegToRad(angJ1)))));  // calculate joint2 angle
    if ((angJ2 < -MAX_ABS_THETA2 || angJ2 > MAX_ABS_THETA2))
        exit = 1; // error if joint 2 angle is impossible to reach

    if (exit == 0) { // if the solution is possible then update structure values
        *ang1 = round(angJ1);
        *ang2 = round(angJ2);
    }

    return (exit);
}


unsigned int scaraIkPulses(signed int *ang1, signed int *ang2, float toolX, float toolY, SCARA_ROBOT *scaraState1){

    unsigned int exit = 0;
    signed int angJ1;
    signed int angJ2;
    float B;     // length from origin to x,y
    float beta;  // cosine law angle
    float alpha; // angle of x,y
    unsigned int l1 = L1 * XY_RES_FACTOR;
    unsigned int l2 = L2 * XY_RES_FACTOR;

    B = sqrt(pow(toolX, 2) + pow(toolY, 2)); // straight line distance from origin to (x,y) point
    alpha = RadToPul(atan2(toolY, toolX)); // angle of B from origin to (x,y) point
    beta = RadToPul(acos((pow(l2, 2) - pow(B, 2) - pow(l1, 2)) / (-2 * B * l1))); // cosine law to find beta

    if (scaraState1->scaraPos.armSol == LEFT_ARM_SOLUTION) { // left hand solution
        angJ1 = alpha + beta;
        if (angJ1 > MAX_ABS_THETA1_PUL || angJ1 < -MAX_ABS_THETA1_PUL){  // switch solutions if the selected solution was impossible
            angJ1 = alpha - beta;
            scaraState1->scaraPos.armSol = RIGHT_ARM_SOLUTION; // changed to Right hand solution
            armSolChange = 1;
            if (angJ1 > MAX_ABS_THETA1_PUL || angJ1 < -MAX_ABS_THETA1_PUL)
                exit =1;
        }
    }
    else if (scaraState1->scaraPos.armSol == RIGHT_ARM_SOLUTION) { // right hand solution
        angJ1 = alpha - beta;
        if (angJ1 < -MAX_ABS_THETA1_PUL || angJ1 > MAX_ABS_THETA1_PUL){  // switch solutions if the selected solution was not possible
            angJ1 = alpha + beta;
            scaraState1->scaraPos.armSol = LEFT_ARM_SOLUTION; // changed to left hand solution
            armSolChange = 1;
            if (angJ1 < -MAX_ABS_THETA1_PUL || angJ1 > MAX_ABS_THETA1_PUL)
                exit =1;
        }
    }

    angJ2 = RadToPul(atan2(toolY - (l1 * sin(PulToRad(angJ1))), toolX - (l1 * cos(PulToRad(angJ1)))));  // calculate joint2 angle
    if ((angJ2 < -MAX_ABS_THETA2_PUL || angJ2 > MAX_ABS_THETA2_PUL))
        exit = 1; // error if joint 2 angle is impossible to reach

    if (exit == 0) { // if the solution is possible then update structure values
        *ang1 = angJ1;
        *ang2 = angJ2;
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

double PulToRad(double pulses){
    return (PI/1708)*pulses; // 1707.96 deg/ rad
}

double RadToPul(double radians){
    return (1708/PI)*radians;
}
