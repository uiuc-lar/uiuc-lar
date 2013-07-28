/*************************************************************
***                                                        ***
***  THIS FUNCTION: iCub Arm Motor Initialization          *** 
***  USED WITH: Balance a Ball on a Beam w/ PID Control    ***
***  USED WITH FILENAME: balance.cpp			   ***
***  AUTHOR: Aaron F. Silver                               ***
***  VERSION: 2.0                                          ***
***  DATE: 3/13/2012                                       ***
***  DESCRIPTION:                                          ***
***     This code is used to initialize the iCub's arm     ***
***  motors such that it grabs the beam in the correct     ***
***  configuration.  It takes as input the current         ***
***  position of all left arm motors and the initial       ***
***  desired wrist angle of the left arm.  It correctly    ***
***  configures and moves the left arm to grasp the        ***
***  platform (beam).                                      ***
***                                                        ***
**************************************************************/

#include "balance.h"

  
void initialize(IPositionControl *pos, int initWristAngle, Vector command)
{
   
   //Ask to calibrate or not during the first run
   int calibrate = 0;
   printf("\nEnter 1 if you wish to Calibrate & set start configuration, 0 if you wish to skip: ");
   scanf("%d", &calibrate);
        
   if(calibrate==1)
   {
      //first zero all joints
      command=0;
      command[0]=0;
      command[1]=10;
      command[2]=0;
      command[3]=0;
      command[4]=0;
      command[5]=0;
      command[6]=0;
      command[7]=0;
      command[8]=0;
      command[9]=0;
      command[10]=0;
      command[11]=0;
      command[12]=0;
      command[13]=0;
      command[14]=0;
      command[15]=0;
      pos->positionMove(command.data());
    
      //wait 10 seconds
      Time::delay(10);
   }

   //now set to proper values to hold the testing aparatus
   command[0]=-60;
   command[1]=6;   
   command[2]=0;
   command[3]=20;
   command[4]=initWristAngle;
   command[5]=0;
   command[6]=1;
   command[7]=0;
   command[8]=45;
   command[9]=30;
   command[10]=50;
   command[11]=90;
   command[12]=90;
   command[13]=90;
   command[14]=90;
   command[15]=190;
    
    //and move to that position
    pos->positionMove(command.data());
}
