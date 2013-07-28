/*************************************************************
***                                                        ***
***  THIS FUNCTION: iCub Wrist Calibration                 *** 
***  USED WITH: Balance a Ball on a Beam w/ PID Control    ***
***  USED WITH FILENAME: balance.cpp			   ***
***  AUTHOR: Aaron F. Silver                               ***
***  VERSION: 2.0                                          ***
***  DATE: 3/13/2012                                       ***
***  DESCRIPTION:                                          ***
***     This code is used to calibrate the iCub's wrist    ***
***  motor such that it levels the beam perpendicular      ***
***  to the gravity vector. It takes as input the y        ***
***  positions of the pink and orange balls (on the end    ***
***  of the beam), and outputs the correct wrist offset    ***
***  angle to level the platform.                          ***
***                                                        ***
**************************************************************/

#include "balance.h"

double calibration(int * calibrate, int * counter, double pink_yMean, double orange_yMean, int writer, double offset)
{
   int dummy = 0;
   if(1 == *calibrate)
   {
      if(pink_yMean-orange_yMean>0.2)
      {
         offset+=0.1; 
      }
      else if(pink_yMean-orange_yMean<-0.2)
      {
         offset-=0.1; 
      }
      else
      {
         //Flag for calibration complete
         dummy=1;
      }

      //Safety angles to saturate at
      if(offset>25)
      {
         offset=25;
      }
      if(offset<-25)
      {
         offset=-25;
      }

      //If caliabration is complete
      if(dummy==1)
      {
         cout << "Calibration complete! Enter any number to continue: ";
         cin >> dummy;
         dummy=1;
         *calibrate=0;

    	 //Write data to file, if writer flag is triggered
	 dataWrite(writer, 1, offset, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
         *counter=*counter+1;
      }
   }
   return offset;
}

