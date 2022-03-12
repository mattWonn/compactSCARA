/*
 * movement.h
 *
 *  Created on: Mar. 6, 2022
 *      Author: Rinz
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#define PI 3.1415
#define NUM_LINES 8           // number of lines for the main loop
#define MAX_POINTS 50          // maximum number of points in a line
#define LEFT_ARM_SOLUTION 1           // index that can be used to indicate left arm
#define RIGHT_ARM_SOLUTION 0           // index that can be used to indicate right arm
#define L1 15       // inner arm length
#define L2 15       // outer arm length
#define MAX_ABS_THETA1 90.0       // max angle of  arm 1
#define MAX_ABS_THETA2 270.0       // max angle of outer arm relative to x axis
#define MAX_ABS_X 30      // max x value
#define MAX_ABS_Y 30       // max y value
#define MIN_ABS_X 0      // min x value
#define MIN_ABS_Y -30       // min y value

#define PUL_PER_DEG_N70 9.48866 // 3415.92pul/360deg
#define T_UPDATE 0.01
#define W_MAX 360 // deg/s
#define A_MAX 1130 //deg/s^2

#define MAX_ARRAY 101


typedef struct PARABOLIC_PROFILE{
     volatile unsigned int controlJoint;
     volatile double timeMove;

}PARABOLIC_PROFILE;

// definition of a point
typedef struct POINT_2D {
    volatile double x;
    volatile double y;
}
POINT_2D;

// info needed to draw a line
typedef struct LINE_DATA{
    POINT_2D pA;   // start point A
    POINT_2D pB;  // end point B
    volatile int numPts; // number of points on line (includes endpoints)

}
LINE_DATA;


// define the SCARA TCP and joint angles
typedef struct SCARA_POS{
    volatile double xPos, yPos;
    volatile signed int theta1, theta2; // TCP coordinate and joint variables
    volatile int armSol;  // right(0) or left(1) arm solution
}SCARA_POS;

// define the SCARA tool (pen) position state
typedef struct SCARA_TOOL {
    volatile char penPos;         // pend state: up or down
}SCARA_TOOL;

// using above structures, define he SCARA ROBOT
typedef struct SCARA_ROBOT{
     PARABOLIC_PROFILE scaraVel;
     SCARA_POS scaraArm;
     SCARA_TOOL scaraTool;
     volatile int motorSpeed1;
     volatile int motorSpeed2;
 }SCARA_ROBOT;


 SCARA_ROBOT scaraStateEnd;
 SCARA_ROBOT scaraStateSet;


 double DegToRad(double);  // returns angle in radians from input angle in degrees
 double RadToDeg(double);  // returns angle in degrees from input angle in radians

 volatile signed int velArray1 [MAX_ARRAY];
 volatile signed int posArray1 [MAX_ARRAY];
 volatile signed int velArray2 [MAX_ARRAY];
 volatile signed int posArray2 [MAX_ARRAY];
 volatile unsigned int arrayLength ;


unsigned int moveJ(signed int startAng1, signed int endAng1, signed int startAng2, signed int endAng2);
unsigned int scaraFk(signed int ang1, signed int ang2, double* toolX, double* toolY);
unsigned int scaraIk(signed int *ang1, signed int * ang2, double toolX, double toolY, int *armSolution);

#endif /* MOVEMENT_H_ */
