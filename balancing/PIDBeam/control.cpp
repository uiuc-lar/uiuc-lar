/*************************************************************
***                                                        ***
***  THIS FUNCTION: iCub PID Balance Control               *** 
***  USED WITH: Balance a Ball on a Beam w/ PID Control    ***
***  USED WITH FILENAME: balance.cpp			   ***
***  AUTHOR: Aaron F. Silver                               ***
***  VERSION: 2.0                                          ***
***  DATE: 3/14/2012                                       ***
***  DESCRIPTION:                                          ***
***     This code is used to perform the PID Control       ***
***  algorithm that balances the green ball in the center  ***
***  of the beam (between the orange and pink balls). It   ***
***  takes as input all of the position and time info, as  ***
***  well as any intialization offsets used.  It outputs   ***
***  the angle that the robot's wrist should turn to.      ***
***                                                        ***
**************************************************************/

#include "balance.h"

//variables retaining memory in between function calls
double u=0;
double uold=0;
double uold2=0;
double error=0;
double derror=0;
double actderror=0;
double interror=0;
double old_error=0;
double old_derror=0;
double oldold_derror=0;
double oldoldold_derror=0;
  
double control(double dist_to_pink, double dist_to_orange, double K, double Kint, double Kdiff, double alpha, double w, double deltaT, int counter, double initWristAngle, double offset, int debug, int writer)
{

   //Set old  error equal to current  error 
   old_error=error;    

   //distance off from the center of the beam
   error = -alpha/2*(dist_to_pink-dist_to_orange); 
    
   //Set old derivative error equal to current derivative error (propigated)
   oldoldold_derror=oldold_derror;
   oldold_derror=old_derror;
   old_derror=derror;

   //derivative of error
   //derror=(error-old_error)/(deltaT);

   //derror=0.5*actderror+0.5*old_derror;

   //Filtered derivative of error
   derror=1/(w+2/deltaT)*((2*w/deltaT)*(error-old_error)-(w-2/deltaT)*old_derror);
   //derror=1/3*derror+1/3*old_derror+1/3*oldold_derror;//+0.25*oldoldold_derror;

   //Derivative Filter  - To be used Later
   //derror=1/(nTimeElapsed*w+2)*(2*w*(error-old_error)-(nTimeElapsed*w-2)*old_derror);

   //integral of error (simple parellelogram area approximation for integral)
   interror+=(error+old_error)/2*(deltaT);

   if(counter<=1)
   {
      interror=0;
      derror=0;
   }
       
   //printf("counter=%d\n",counter);
   //printf("error=%g\nderror(filtered)=%g\nderror(unfiltered)=%g\ninterror=%g\n",error,derror,actderror,interror);

   if(error>-0.5 && error<0.5 && derror>-0.02 && derror<0.02)
   {
      printf("Success!");
   }
   else
   {
      //PID Control Law
      uold2=uold;
      uold=u;
      u=((K*error)+(Kint*interror)+(Kdiff*derror)+initWristAngle+offset);

      //Averaging Filter (LPF)
      //u=(0.5*u+0.5*uold);

      //Safety Limits    
      if(u<-6+initWristAngle+offset)
      {
         u=-6+initWristAngle+offset;
      }

      if(u>6+initWristAngle+offset)
      {
         u=6+initWristAngle+offset;
      }
   }

   if(debug==1)
   {
      printf("the control = %g\nerror=%g\nderror=%g\ninterror=%g\n",u,error,derror,interror);
   }
   
   //Write data to file, if writer flag is triggered
   dataWrite(writer, 3, error, derror, interror, u, 0.0, 0.0, 0.0, 0.0);



   return u;
}
