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

void sendMoveJ(SCARA_ROBOT scaraStateEnd){
    unsigned int exit;

    __disable_interrupt();
    exit = moveJ(scaraStateEnd.scaraPos.theta1,scaraStateEnd.scaraPos.theta2); // start end M1, start end M2;
    if (exit == 0){
        kP = kPAng;// map pul/UpdateTime to PWM(0:100); 1uT/0.01s * 1s/6716pul * 100%
        kI = kIAng; //1.8
        kD = kDAng;
        __enable_interrupt();
        startMoveJ = 1;
        while (startMoveJ == 1){}
        //__delay_cycles(10000);
        startMoveJ = 0;
        TA0CCR4 = 0;
        TA1CCR1 = 0;
        exit = mddInputCtrl(CTRLBRAKE);
        exit = mddInputCtrl2(CTRLBRAKE2);
        prevPosCount = 0;
     //   posCount = posArray1[arrayLength-1];
        prevPosCount2 = 0;
     //   posCount2 = posArray2[arrayLength-1];
        updateIndex = 0;
        noMove1 = 0;
        noMove2 = 0;
    }

}


unsigned int moveJ(signed int endAng1, signed int endAng2){

    unsigned int exit =0;
    unsigned int x=0;
    signed int startAng1;
    signed int startAng2;
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


    startAng1 = posCount;
    startAng2 = posCount2;


    if (endAng1 <= MAX_ABS_THETA1_PUL && endAng1 >= -MAX_ABS_THETA1_PUL){ // check range
        if (endAng2 <= MAX_ABS_THETA2_PUL && endAng2 >= -MAX_ABS_THETA2_PUL){ // check range
            displacement1 = abs(endAng1 - startAng1);
            displacement2 = abs(endAng2 - startAng2);

            // determine if a no-move condition is requested
            if (endAng1 == startAng1)
                noMove1 =1;
            else
                noMove1 =0;
            if (endAng2 == startAng2)
                noMove2 = 1;
            else
                noMove2 =0;

            // determine which joint has to move the farthest and bases calculations on that joint
            if (displacement1 >= displacement2){
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


            timeForMove = sqrt((abs(deltaD)*2*PI)/A_MAX_PUL); // calculate period (T) for sinusoidal profile
            w = (2*PI)/timeForMove;
            vMaxMove = (2*A_MAX_PUL)/w; // calc the Vmax for the move

            // check if you are within the velocity limit, otherwise you have to recalculate using the set max velocity
            if (vMaxMove > W_MAX_PUL){
                vMaxMove = W_MAX_PUL;
                timeForMove = (2*abs(deltaD))/(W_MAX_PUL);
                aMaxMove = (abs(deltaD)*w)/(timeForMove);
                aMaxMove2 = (abs(deltaD2)*w)/(timeForMove);
            }
            else{ // if the velocity is within the limit, then calculate acceleration values for each motor
                aMaxMove = abs(A_MAX_PUL);
                aMaxMove2 = ((abs(deltaD2)*w)/timeForMove);
            }

            //-------------- assign variables-------------------
            if (masterJoint == 1){ // if arm one moves further, then calculate the array length based on it
                w = (2*PI)/timeForMove;
                arrayLength = (timeForMove/T_UPDATE)+1;

                if (direction1 == 0) // relative to which joint moves the furthest
                    aMaxMove = -1*aMaxMove;
                if (direction2 == 0)
                    aMaxMove2 = -1*aMaxMove2;

                for(tInc; tInc<arrayLength; tInc++){
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
                    posArray2[tInc] = RadToPul((PulToRad(aMaxMove)*(tInc*T_UPDATE))/w)  -  RadToPul(PulToRad(aMaxMove)*(sin(w*(tInc*T_UPDATE)))/pow(w,2))+startAng2;
                    posArray1[tInc] = RadToPul((PulToRad(aMaxMove2)*(tInc*T_UPDATE))/w)  -  RadToPul(PulToRad(aMaxMove2)*(sin(w*(tInc*T_UPDATE)))/pow(w,2))+startAng1;
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

void sendMoveL(SCARA_ROBOT *scaraStateSolution, LINE_DATA drawLine){

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
                // move for the first line
            kP = kPLin;
            kI = kILin;
            kD = kDLin;
                __enable_interrupt();
                __delay_cycles(10000);
                startMoveJ=1;
                while (startMoveJ == 1){}
                startMoveJ =0;
                updateIndex = 0;
                prevPosCount = 0;
            //    posCount = posArray1[arrayLength-1];
                prevPosCount2 = 0;
             //   posCount2 = posArray2[arrayLength-1];
                noMove1 = 0;
                noMove2 = 0;
                __disable_interrupt();
                /**********TOOLUP************/
                //armSwitchSol =0;

            //result = scaraIk(&angleJ1, &angleJ2, holdLine.pB.x, holdLine.pB.y, scaraStateSolution); // only changing the solution

            if (result == 0){
                if (originalArmSolution == LEFT_ARM_SOLUTION)
                    scaraStateSolution->scaraPos.armSol = RIGHT_ARM_SOLUTION;//switch arm solutions
                else
                    scaraStateSolution->scaraPos.armSol = LEFT_ARM_SOLUTION;//return arm solution

                result = scaraIkFloat(&angleJ1, &angleJ2, endLine.pA.x, endLine.pA.y, scaraStateSolution); // only changing the solution, calculate the arm angles

                if (result == 0){
                    // same spot at midpoint of full line, new solution arm angles
                    scaraStateEnd.scaraPos.theta1 = angleJ1*PUL_PER_DEG_N70;
                    scaraStateEnd.scaraPos.theta2 = angleJ2*PUL_PER_DEG_N70;

                    // switch arm solutions
                    sendMoveJ(scaraStateEnd); // start end M1, start end M2;

                    // perform the second part of the full line with the new arm solution
                    if (result == 0){
                        endLine = initLine(armChangeEnd.x, armChangeEnd.y, armChangeStart.x, armChangeStart.y, 0);//xb yb xa ya npts
                        result = moveScaraL(scaraStateSolution, endLine);
                        if(result == 0){
                            kP = kPLin;
                            kI = kILin;
                            kD = kDLin;
                            __enable_interrupt();
                            __delay_cycles(10000);
                            startMoveJ=1;
                            while (startMoveJ == 1){}
                            startMoveJ =0;
                            updateIndex = 0;
                            prevPosCount = 0;
                        //    posCount = posArray1[arrayLength-1];
                            prevPosCount2 = 0;
                        //    posCount2 = posArray2[arrayLength-1];
                            noMove1 = 0;
                            noMove2 = 0;
                        }
                    }
                }
            }
        }
    }
    else if(result == 0){
        kP = kPLin;
        kI = kILin;
        kD = kDLin;
        __enable_interrupt();
        __delay_cycles(10000);
        startMoveJ=1;
        while (startMoveJ == 1){}
        startMoveJ =0;
        updateIndex = 0;
        prevPosCount = 0;
    //    posCount = posArray1[arrayLength-1];
        prevPosCount2 = 0;
   //     posCount2 = posArray2[arrayLength-1];
        noMove1 = 0;
        noMove2 = 0;
        P2IFG &= ~0xF0;
    }

}


/***********************************************************
* Name: int moveScaraL
* function: used to perform the steps necessary to calculate a linear move with the tool
* arguments
*            SCARA_ROBOT *scaraState:  pointer variable of type SCARA_ROBOT
*            LINE_DATA newLine: newLine contains both start and end x and y variables as well as number of points in the line
*
* returns value
* created by: Matthew Wonneberg, Jamie Boyd
* Date: March 17 2022
************************************************************/
int moveScaraL(SCARA_ROBOT* scaraState, LINE_DATA newLine){

    volatile unsigned int value=0;
    volatile unsigned int returned =0;

    volatile float deltaX;
    volatile float deltaY;
    volatile float deltaD;
    volatile unsigned int armSolution;

    float timeForMove;
    float w = 0;
    volatile float d = 0;
    volatile float xHoldPrev =0;
    volatile float yHoldPrev =0;
    volatile float j1HoldAng =0;
    volatile float j2HoldAng =0;
    volatile unsigned int attemptedArmSolution;
    volatile signed int holdPosition1 = 0;
    volatile signed int holdPosition2 = 0;
    volatile unsigned int aMaxMove;

    volatile unsigned int tInc = 0;

    signed int currentAngJ1;
    signed int currentAngJ2;
    volatile signed int angleJ1;
    volatile signed int angleJ2;

    volatile unsigned int diff;
    volatile unsigned int diff2;

    SCARA_ROBOT scaraStateSet;
    SCARA_ROBOT scaraStateEnd;

    //!!!!!!!!!!!!!!!!!!!!!!!!! add arm solution change condition

    scaraStateSet.scaraPos.theta1 = posCount*DEG_PER_PUL_N70;
    scaraStateSet.scaraPos.theta2 = posCount2*DEG_PER_PUL_N70;

    value = scaraFk(scaraStateSet.scaraPos.theta1, scaraStateSet.scaraPos.theta2, &newLine.pA.x, &newLine.pA.y);

    attemptedArmSolution = scaraState->scaraPos.armSol; // store the attempted arm solution


    // arm solution change condition for first point
        // get current joint angles
        currentAngJ1 = posCount;
        currentAngJ2 = posCount2;
        // calc joint angles for arm solution beginning x,y
        value = scaraIkFloat(&j1HoldAng, &j2HoldAng, newLine.pA.x, newLine.pA.y, scaraState);// lowercase k
        // compare within one degree
        if (currentAngJ1 > ((j1HoldAng*PUL_PER_DEG_N70)+9) || currentAngJ1 < ((j1HoldAng*PUL_PER_DEG_N70)-9) || currentAngJ2 > ((j2HoldAng*PUL_PER_DEG_N70)+9) || currentAngJ2 < ((j2HoldAng*PUL_PER_DEG_N70)-9)){
            // if different, movJ to the same point with the correct arm solution
            /********TOOL_UP******************/
            scaraStateEnd.scaraPos.theta1 = j1HoldAng*PUL_PER_DEG_N70;
            scaraStateEnd.scaraPos.theta2 = j2HoldAng*PUL_PER_DEG_N70;

            sendMoveJ(scaraStateEnd);
            /*********TOOL_RETURN*************/
        }


    // calculate line displacements
    deltaX = newLine.pB.x - newLine.pA.x;
    deltaY = newLine.pB.y - newLine.pA.y;
    deltaD = (sqrt(pow(deltaX, 2) + pow(deltaY, 2))); // pythagreom theorum for line distance in x,y

    timeForMove = (sqrt((abs(deltaD)*2*PI)/A_MAX_LINEAR));
    w = (2*(PI))/timeForMove;

    arrayLength = (timeForMove/T_UPDATE)+1;

    aMaxMove = A_MAX_LINEAR;


    // fill the position and velocity array targets
    for(tInc; tInc < arrayLength; tInc++){

        // calculate linear array in terms of d(t) and then fill X and Y positions
        d = ((aMaxMove * (tInc*T_UPDATE))/w -  (aMaxMove)*(sin(w*(tInc*T_UPDATE)))/pow(w,2));
        xHold = newLine.pA.x + (d*(deltaX)/deltaD);
        yHold = newLine.pA.y + (d*(deltaY)/deltaD);

        if ((abs(xHold*10) < INNER_CIRCLE_BOUNDS) && (abs(yHold*10) < INNER_CIRCLE_BOUNDS)){ // inner circle violation exit condition
            value = 1;
            tInc = arrayLength;
        }
        else{
        //returned = scaraIk(&(posArray1[tInc]), &(posArray2[tInc]), xHold, yHold, scaraState); // calculate arm angles of x,y point
            returned = scaraIkFloat(&j1HoldAng, &j2HoldAng, xHold, yHold, scaraState); // calculate arm angles of x,y point
        if (returned == 0){
            if (armSolChange == 1){ // determine if a armSolution change was needed

                // hold values to be used for arm solution change
                armChangeStart.x = xHoldPrev;
                armChangeStart.y = yHoldPrev;
                armChangeEnd.x = newLine.pB.x;
                armChangeEnd.y = newLine.pB.y;
                holdLine = initLine(xHoldPrev, yHoldPrev, newLine.pA.x, newLine.pA.y, 0);//xb yb xa ya npts
                endLine = initLine(newLine.pB.x, newLine.pB.y, armChangeStart.x, armChangeStart.y, 0);

                scaraState->scaraPos.armSol = attemptedArmSolution; // return to original arm solution
                armSolChange =0;
                //armSwitchSol =0;
                //solutionMoveJ2 =1; // set second solution change
                return(3);
            }
            //posArray1[tInc] = posArray1[tInc]*PUL_PER_DEG_N70; // store position values in pulses
            //posArray2[tInc] = posArray2[tInc]*PUL_PER_DEG_N70;
            posArray1[tInc] = j1HoldAng*PUL_PER_DEG_N70;
            posArray2[tInc] = j2HoldAng*PUL_PER_DEG_N70;

            xHoldPrev = xHold; // store the previous x,y values incase an arm solution change is needed
            yHoldPrev = yHold;

            if ((posArray2[tInc] > (posArray1[tInc]+RELATIVE_THETA2_PUL)) || (posArray2[tInc] < (-RELATIVE_THETA2_PUL + posArray1[tInc]))){ //  theta2 value verification
                value = 1;
                tInc = arrayLength;
            }


        }
        else{
            value = 1; // exit calculations if the move is not possible
            tInc = arrayLength;
        }
      }
    }


    return (value);
}
/***********************************************************
* Name: void sendMoveC
* function: used to perform the steps necessary send a circular move with the tool
* arguments
*            SCARA_ROBOT *scaraStateSolution:  pointer variable of type SCARA_ROBOT
*
*
* returns value
* created by: Matthew Wonneberg, Jamie Boyd
* Date: April 13 2022
************************************************************/
void sendMoveC(SCARA_ROBOT *scaraStateSolution){

    volatile unsigned int result =0;
    volatile signed int angleJ1;
    volatile signed int angleJ2;
    volatile unsigned int originalArmSolution;
    SCARA_ROBOT midState;

    // store the user defined arm solution
    originalArmSolution = scaraStateSolution->scaraPos.armSol;

    // calculate the position array's of each joint to create the arc
    __disable_interrupt();
    result = moveScaraC(scaraStateSolution);

    if(result == 3){ // if an armSolution change is necessary
        result = moveScaraC(scaraStateSolution); // after first arm solution was not successfull, calculate part of the first arc with the original arm solution

        if(result == 0){ // if the calculation was successful, perform the first arc
            kP = kPLin;
            kI = kILin;
            kD = kDLin;
            __enable_interrupt();
            __delay_cycles(10000);
            startMoveJ=1;
            while (startMoveJ == 1){}
            startMoveJ =0;
            updateIndex = 0;
            prevPosCount = 0;
            posCount = posArray1[arrayLength-1];
            prevPosCount2 = 0;
            posCount2 = posArray2[arrayLength-1];
            noMove1 = 0;
            noMove2 = 0;
            __disable_interrupt();
            /**********TOOLUP************/

            // switch arm solutions
            if (originalArmSolution == LEFT_ARM_SOLUTION)
                scaraStateSolution->scaraPos.armSol = RIGHT_ARM_SOLUTION;
            else
                scaraStateSolution->scaraPos.armSol = LEFT_ARM_SOLUTION;

            // calculate joint angles for the same coordinate, but with a new arm solution
            result = scaraIk(&angleJ1, &angleJ2, armChangeStart.x, armChangeStart.y, scaraStateSolution);

            if (result == 0){
                midState.scaraPos.theta1 = angleJ1*PUL_PER_DEG_N70; //assign same spot, new arm solution angles
                midState.scaraPos.theta2 = angleJ2*PUL_PER_DEG_N70;

                // perform the moveJ to the same coordiante, just with switched arm solutions
                sendMoveJ(midState); // start end M1, start end M2;

                /**********TOOLRETURN************/
                // perform final arc
                if (result == 0){
                    scaraStateSet.scaraPos.theta1 = scaraStateEnd.scaraPos.theta1; // set to midpoint of arc
                    scaraStateEnd.scaraPos.theta1 = scaraStateSet.scaraPos.theta2; // restore original end position

                    // calculate the position array's of each joint to create the arc
                    result = moveScaraC(scaraStateSolution);
                    if(result == 0){ // if the calculation is successful, perform the move
                        kP = kPLin;
                        kI = kILin;
                        kD = kDLin;
                        __enable_interrupt();
                        __delay_cycles(10000);
                        startMoveJ=1;
                        while (startMoveJ == 1){}
                        startMoveJ =0;
                        updateIndex = 0;
                        prevPosCount = 0;
                        posCount = posArray1[arrayLength-1];
                        prevPosCount2 = 0;
                        posCount2 = posArray2[arrayLength-1];
                        noMove1 = 0;
                        noMove2 = 0;
                    }
                }
            }

        }
    }
    else if(result == 0){ // perform the move if the calculation was successful and no arm solution change was necessary
        kP = kPLin;
        kI = kILin;
        kD = kDLin;
        __enable_interrupt();
        __delay_cycles(10000);
        startMoveJ=1;
        while (startMoveJ == 1){}
        startMoveJ =0;
        updateIndex = 0;
        prevPosCount = 0;
        posCount = posArray1[arrayLength-1];
        prevPosCount2 = 0;
        posCount2 = posArray2[arrayLength-1];
        noMove1 = 0;
        noMove2 = 0;
    }

}


/***********************************************************
* Name: int moveScaraC
* function: used to calculate a circular move with the tool
* arguments
*            SCARA_ROBOT *scaraState:  pointer variable of type SCARA_ROBOT
*
*
* returns value
* created by: Matthew Wonneberg, Jamie Boyd
* Date: April 13 2022
************************************************************/


int moveScaraC(SCARA_ROBOT* scaraState){

    volatile unsigned int value=0;
    volatile unsigned int returned =0;

    volatile float Xb;
    volatile float Yb;
    volatile float Xc;
    volatile float Yc;

    volatile float deltaXa;
    volatile float deltaYa;
    volatile float deltaXb;
    volatile float deltaYb;

    volatile float deltaD;
    volatile float resultant;

    volatile unsigned int armSolution;
    volatile signed int direction;

    float timeForMove;
    volatile float arcAngle;
    volatile float holdArcAngle;
    float w = 0;
    volatile float d = 0;
    volatile float xHoldPrev =0;
    volatile float yHoldPrev =0;
    volatile float j1HoldAng =0;
    volatile float j2HoldAng =0;

    volatile unsigned int attemptedArmSolution;
    volatile unsigned int aMaxMove;
    volatile unsigned int tInc = 0;
    volatile signed int currentAngJ1;
    volatile signed int currentAngJ2;
    volatile signed int thetaStart;
    volatile signed int thetaEnd;
    volatile unsigned int radius;

    thetaStart = scaraStateSet.scaraPos.theta1;
    thetaEnd = scaraStateEnd.scaraPos.theta1;
    radius = scaraStateSet.scaraPos.radius;


    // determine the direction to draw the circle
    if (thetaStart > thetaEnd){
        direction = -1; // negative direction / cw
    }
    else
        direction = 1; // positive direciton / ccw

    attemptedArmSolution = scaraState->scaraPos.armSol; // store the attempted arm solution

    //currentAngJ1 = posCount*DEG_PER_PUL_N70;
    //currentAngJ2 = posCount2*DEG_PER_PUL_N70;

    // Forward kinematics to find current (x, y) coordinate
    //value = scaraFk(currentAngJ1, currentAngJ2, &scaraStateSet.scaraPos.x, &scaraStateSet.scaraPos.y);// lowercase k
    value = scaraFkPulse(posCount, posCount2, &scaraStateSet.scaraPos.x, &scaraStateSet.scaraPos.y);// lowercase k)

    // arm solution change condition for first point
    // get current joint angles
    currentAngJ1 = posCount;
    currentAngJ2 = posCount2;
    // calc joint angles for arm solution beginning x,y
    value = scaraIkFloat(&j1HoldAng, &j2HoldAng, scaraStateSet.scaraPos.x, scaraStateSet.scaraPos.y, scaraState);// lowercase k
    // compare
    if (currentAngJ1 > (int)((j1HoldAng*PUL_PER_DEG_N70)+9) || currentAngJ1 < (int)((j1HoldAng*PUL_PER_DEG_N70)-9) || currentAngJ2 > (int)((j2HoldAng*PUL_PER_DEG_N70)+9) || currentAngJ2 < (int)((j2HoldAng*PUL_PER_DEG_N70)-9)){
        // if different, movJ to the same point with the correct arm solution
        /********TOOL_UP******************/
        scaraStateEnd.scaraPos.theta1 = j1HoldAng*PUL_PER_DEG_N70;
        scaraStateEnd.scaraPos.theta2 = j2HoldAng*PUL_PER_DEG_N70;

        sendMoveJ(scaraStateEnd);
        /*********TOOL_RETURN*************/
    }


    // find delta xy between the current coordinate of the robot and the arc center position
    deltaXa = radius*cos(PulToRad(thetaStart));
    deltaYa = radius*sin(PulToRad(thetaStart));

    // find the center (x, y) coordiante of the arc
    Xc = scaraStateSet.scaraPos.x - deltaXa;
    Yc = scaraStateSet.scaraPos.y - deltaYa;

    // find the delta xy between the center coordiante of the arc and the end coordinate of the arc
    deltaXb = radius*cos(PulToRad(thetaEnd));
    deltaYb = radius*sin(PulToRad(thetaEnd));

    // find the end (x, y) coordiante of the arc
    Xb = Xc-deltaXb;
    Yb = Yc-deltaYb;

    deltaD = radius*PulToRad(thetaEnd - thetaStart); // find the linear distance of the arc that is requested

    // calculate the time for the move
    timeForMove = (sqrt((abs(deltaD)*2*PI)/A_MAX_LINEAR));
    w = (2*(PI))/timeForMove;

    // calculate the arrayLength and set the acceleration to the maximum TCP acceleration
    arrayLength = (timeForMove/T_UPDATE)+1;
    aMaxMove = A_MAX_LINEAR;

    // fill the position and velocity array targets
    for(tInc; tInc < arrayLength; tInc++){

        // calculate linear array in terms of d(t) and then fill X and Y positions
        d = ((aMaxMove * (tInc*T_UPDATE))/w -  (aMaxMove)*(sin(w*(tInc*T_UPDATE)))/pow(w,2));

        // calculate the arc angle with respect to time, while accounting for direction
        if (direction == 1)
            arcAngle = ((d/deltaD)*(thetaEnd - thetaStart) )+ thetaStart;
        else
            arcAngle = ((d/deltaD)*(thetaStart - thetaEnd) )+ thetaStart;

        // calculate the (x, y) coordinate for the corisponding circle angle wrt time
        xHold = radius*cos(PulToRad(arcAngle)) + Xc;
        yHold = radius*sin(PulToRad(arcAngle)) + Yc;

        if ((abs(xHold*10) < INNER_CIRCLE_BOUNDS) && (abs(yHold*10) < INNER_CIRCLE_BOUNDS)){ // inner circle violation exit condition
            value = 1;
            tInc = arrayLength;
        }
        else{
            //returned = scaraIk(&(posArray1[tInc]), &(posArray2[tInc]), xHold, yHold, scaraState); // calculate arm angles of x,y point
            returned = scaraIkFloat(&j1HoldAng, &j2HoldAng, xHold, yHold, scaraState); // calculate arm angles of x,y point

            if (returned == 0){
                if (armSolChange == 1){ // determine if a armSolution change was needed

                    // hold values to be used for arm solution change
                    armChangeStart.x = xHoldPrev;
                    armChangeStart.y = yHoldPrev;

                    scaraStateSet.scaraPos.theta2 = thetaEnd;
                    scaraStateEnd.scaraPos.theta1 = holdArcAngle;

                    scaraState->scaraPos.armSol = attemptedArmSolution; // return to original arm solution
                    armSolChange =0;
                    return(3);
                }
                posArray1[tInc] = j1HoldAng*PUL_PER_DEG_N70; // store position values in pulses
                posArray2[tInc] = j2HoldAng*PUL_PER_DEG_N70;

                xHoldPrev = xHold; // store the previous x,y values incase an arm solution change is needed
                yHoldPrev = yHold;
                holdArcAngle = arcAngle;

                if ((posArray2[tInc] > (posArray1[tInc]+RELATIVE_THETA2_PUL)) || (posArray2[tInc] < (-RELATIVE_THETA2_PUL + posArray1[tInc]))){ //  theta2 value verification
                    value = 1;
                    tInc = arrayLength;
                }
            }
            else{// exit calculations if the move is not possible
                value = 1;
                tInc = arrayLength;
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
unsigned int scaraFk(signed int ang1, signed int ang2, float* toolX, float* toolY){

    volatile unsigned int exit = 0;
    volatile double toolX1, toolY1;
    volatile double thetaB, a;

    if (abs(ang1) > MAX_ABS_THETA1) // if theta1 is over the limit return 1
        exit = 1;
    else if (abs(ang2) > MAX_ABS_THETA2) // if theta2 is over the limit return 1
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
unsigned int scaraFkPulse(signed int pul1, signed int pul2, float* toolX, float* toolY){

    volatile unsigned int exit = 0;
    volatile double toolX1, toolY1;
    volatile double thetaB, a;

    if (abs(pul1) > MAX_ABS_THETA1_PUL) // if theta1 is over the limit return 1
        exit = 1;
    else if (abs(pul2) > MAX_ABS_THETA2_PUL) // if theta2 is over the limit return 1
        exit =1;
    else
    {
        toolY1 = (L1 * sin(PulToRad(pul1))) + (L2 * sin(PulToRad(pul2))); // eqn for y coordinate
        toolX1 = (L1 * cos(PulToRad(pul1))) + (L2 * cos(PulToRad(pul2))); // eqn for x coordinate

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
    volatile float angJ1;
    volatile float angJ2;
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
        *ang1 = (angJ1);
        *ang2 = (angJ2);
    }

    return (exit);
}

unsigned int scaraIkFloat(float *ang1, float *ang2, double toolX, double toolY, SCARA_ROBOT *scaraState1){

    unsigned int exit = 0;
    volatile float angJ1;
    volatile float angJ2;
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
        *ang1 = (angJ1);
        *ang2 = (angJ2);
    }

    return (exit);
}


/*unsigned int scaraIkPulses(signed int *ang1, signed int *ang2, float toolX, float toolY, SCARA_ROBOT *scaraState1){

    volatile unsigned int exit = 0;
    volatile signed int angJ1;
    volatile signed int angJ2;
    float B;     // length from origin to x,y
    float beta;  // cosine law angle
    float alpha; // angle of x,y
    unsigned int armSol = scaraState1->scaraPos.armSol;

    B = sqrt(pow(toolX, 2) + pow(toolY, 2)); // straight line distance from origin to (x,y) point
    alpha = RadToPul(atan2(toolY, toolX)); // angle of B from origin to (x,y) point
    beta = RadToPul(acos((pow(L2, 2) - pow(B, 2) - pow(L1, 2)) / (-2 * B * L1))); // cosine law to find beta

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

    angJ2 = RadToPul(atan2(toolY - (L1 * sin(PulToRad(angJ1))), toolX - (L1 * cos(PulToRad(angJ1)))));  // calculate joint2 angle
    if ((angJ2 < -MAX_ABS_THETA2_PUL || angJ2 > MAX_ABS_THETA2_PUL))
        exit = 1; // error if joint 2 angle is impossible to reach

    if (exit == 0) { // if the solution is possible then update structure values
        *ang1 = angJ1;
        *ang2 = angJ2;
    }

    return (exit);
}*/
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
