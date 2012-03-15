/*************************************************************
***                                                        ***
***  THIS FUNCTION: iCub Vision Data Acquisition           *** 
***  USED WITH: Balance a Ball on a Beam w/ PID Control    ***
***  USED WITH FILENAME: balance.cpp			   ***
***  AUTHOR: Aaron F. Silver                               ***
***  VERSION: 2.0                                          ***
***  DATE: 3/13/2012                                       ***
***  DESCRIPTION:                                          ***
***     This code gathers visual input from the iCub's     ***
***  left eye (camera) of 3 balls.  It discerns the        ***
***  two-dimensional coordinates of each colored ball      ***
***  (orange, pink, and green).  It takes in raw visual    ***
***  data and outputs the (x,y) positions (in pixels) of   ***
***  all 3 balls.                                          ***
***                                                        ***
**************************************************************/

#include "balance.h"

void vision(int debug, double * pinkX, double * pinkY, double * orangeX, double * orangeY, double * greenX, double * greenY, ImageOf<PixelRgb> * image)
{
	//Local Vision Variables
   double green_xMean = 0;
   double green_yMean = 0;
   double orange_xMean = 0;
   double orange_yMean = 0;
   double pink_xMean = 0;
   double pink_yMean = 0;
   int green_ct = 0;
   int orange_ct = 0;
   int pink_ct = 0;

   int xlimit=image->width();
   int ylimit=image->height()-40;
   int green_marker[xlimit][ylimit];
   int green_object[xlimit][ylimit];
   //printf("xlimit = %d\n",xlimit);
      
   //check to see we actually gofind_location some image
   if(image!=NULL)
   {
      //debugging: display the size of image recieved
      //printf("We got an image of size %dx%d\n", image->width(), image->height());

      // FOR THE LEFT HALF OF THE IMAGE
      for(int x=0; x<xlimit/2; x++)
      {
         for(int y=40; y<ylimit; y++)
	 {
	    PixelRgb& pixel = image->pixel(x,y);
            green_object[x][y]=0;
            green_marker[x][y]=0;
 
            //threshold for green ping pong ball
	    if(pixel.r>112 && pixel.r<142 && pixel.g>142 && pixel.g<172 && pixel.b>116 && pixel.b<146)
	    {
               green_marker[x][y]=1;
	    }
      

	    //threshold for pink ball
	    if(pixel.r>105 && pixel.r<145 && pixel.g>32 && pixel.g<72 && pixel.b>36 && pixel.b<76)
	    {
	       pink_xMean += x;
	       pink_yMean += y;
	       pink_ct++;
	    }
	 }   
      }

      // FOR THE RIGHT HALF OF THE IMAGE
      for(int x=xlimit/2; x<xlimit; x++)
      {
         for(int y=40; y<ylimit; y++)
	 {
	    PixelRgb& pixel = image->pixel(x,y);
            green_object[x][y]=0;
            green_marker[x][y]=0;

        
            //threshold for green ping pong ball
	    if(pixel.r>112 && pixel.r<142 && pixel.g>142 && pixel.g<172 && pixel.b>116 && pixel.b<146)
	    {
               green_marker[x][y]=1;
	    }
	    //threshold for orange ball
	    if(pixel.r>142 && pixel.r<182 && pixel.g>56 && pixel.g<96 && pixel.b>43 && pixel.b<83)
	    {
	       orange_xMean += x;
	       orange_yMean += y;
	       orange_ct++;
	    }
	 }   
      }

      //Perform Hough Transform and detect green ball
      for(int y=7; y<ylimit-7; y++)
      {
         for(int x=7; x<xlimit-7; x++)
	 { 
            if(green_marker[x][y]==1)
            {
               int green_counted=0;
               for(int cntx=0;cntx<=14;cntx++)
               {
                  for(int cnty=0;cnty<=14;cnty++)
                  {
                     if(green_marker[x+cntx-7][y+cnty-7]==1 )
                     {
                        green_object[x][y]++;
                     }
                     if(green_object[x][y]>30 && green_counted==0)
                     {
                        green_xMean += x;
	                green_yMean += y;
	                green_ct++; 
                        green_counted=1;
                     }
                  }
               }
            }
	 }   
      }

      //Find average centers of x & y position
      if(green_ct>0)
      {
         green_xMean /= green_ct;
	 green_yMean /= green_ct;
      }

      if(pink_ct>0)
      {
	 pink_xMean /= pink_ct;
	 pink_yMean /= pink_ct;
      }

      if(orange_ct>0)
      {
	 orange_xMean /= orange_ct;
	 orange_yMean /= orange_ct;
      }
   }

   //printf("green ball coordinate: %g , %g  green ct = %d\n", green_xMean, green_yMean, green_ct);
   //FOR DEBUGGING: Print to the terminal the centroids of the balls
   if(debug==1)
   {
      if(green_ct>(image->width()/60)*(image->height()/60))
      {
         printf("green ball coordinate: %g , %g  green ct = %d\n", green_xMean, green_yMean, green_ct);
      }

      if(pink_ct>(image->width()/60)*(image->height()/60))
      {
	 printf("pink ball coordinate: %g , %g\n", pink_xMean, pink_yMean);
      }

      if(orange_ct>(image->width()/60)*(image->height()/60))
      {
	 printf("orange ball coordinate: %g , %g\n", orange_xMean, orange_yMean);
      }
   }

  //set output values
  *pinkX=pink_xMean;
  *pinkY=pink_yMean;
  *orangeX=orange_xMean;
  *orangeY=orange_yMean;
  *greenX=green_xMean;
  *greenY=green_yMean;

}
