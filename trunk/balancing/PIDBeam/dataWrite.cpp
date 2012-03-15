/*************************************************************
***                                                        ***
***  THIS FUNCTION: Experimental Data Writing to Text File *** 
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

ofstream myfile;

void dataWrite(int writer, int flag, double data1, double data2, double data3, double data4, double data5, double data6, double data7, double data8)
{
   if(writer==1)
   {
      //Initialization Writing	
      if(flag==0)
      {
         myfile.open("ballbeamdata.txt");
	 myfile << "DATA FROM BALL BEAM SIMULATION\n------------------------------\n";         
	 myfile << "The data in this file is in the following order:\n";
	 myfile << "Time of simulation, delta T for each step, Pink ball X, Pink ball Y, Orange ball X, Orange ball Y, Green ball X, Green ball Y, error, derivative error, integral error, control (u), motor encoder value.\n\n\n";
	 myfile << "motor speed (deg/sec) = ";         
	 myfile << data1;     
         myfile << "\nK = ";         
	 myfile << data2;     
	 myfile << "\nKdiff = ";         
	 myfile << data3;  
	 myfile << "\nKint = ";         
	 myfile << data4;  
      } 

      //Calibration (offset angle) Writing
      if(flag==1)
      {
    	 myfile << "\nInitial angle offset = ";   
    	 myfile << data1; 
    	 myfile << "\n\n\n"; 
      } 

      //Experimental Data Writing pt 1 (ball position info, time info)
      if(flag==2)
      {
         myfile << data1;
    	 myfile << ", ";
         myfile << data2;
    	 myfile << ", ";
         myfile << data3;
    	 myfile << ", ";
         myfile << data4;
    	 myfile << ", ";
         myfile << data5;
	 myfile << ", ";
         myfile << data6;
	 myfile << ", ";
         myfile << data7;
	 myfile << ", ";
         myfile << data8;
	 myfile << ", ";
      }


      //Experimental Data Writing pt 2 (controller info)
      if(flag==3)
      {
         myfile << data1;
    	 myfile << ", ";
         myfile << data2;
    	 myfile << ", ";
         myfile << data3;
    	 myfile << ", ";
         myfile << data4;
    	 myfile << ", ";
      }

      //Experimental Data Writing pt 3 (motor encoder info)
      if(flag==4)
      {
         myfile << data1;
         myfile << "\n";
      }


      //Close Output File When Done
      if(flag==5)
      {
         myfile.close();
         cout << "\nfile written!\n";
      }

   }

}
