/*************************************************************
***                                                        ***
***  CODE: Balance a Ball on a Beam w/ PID Control         ***
***  AUTHOR: Aaron F. Silver                               ***
***  VERSION: 2.0                                          ***
***  DATE: 3/13/2012                                       ***
***  DESCRIPTION:                                          ***
***     This code gathers visual input from the iCub's     ***
***  left eye (camera) of 3 balls.  The goal is to         ***
***  stabalize the green ball on abeam exactly in the      ***
***  the middle of the orange and pink balls.  The left    ***
***  wrist of the robot is used to control the angle of    ***
***  the beam, and the control used is a standard linear   ***
***  PID feedback controller.                              ***
***                                                        ***
**************************************************************/

#include "balance.h"

//FLAGS
int writer = 0;
int killMotors = 0;
int debug = 0;
int initWristAngle = -25;  
int run=1;
int counter = 0;

// USED FOR MILLISECOND TIMER
int GetMilliCount()
{
   timeb tb;
   ftime ( &tb );
   int nCount = tb.millitm + (tb.time & 0xfffff) * 1000;
   return nCount;
}

// USED FOR MILLISECOND TIMER
int GetMilliSpan( int nTimeStart )
{
   int nSpan = GetMilliCount() - nTimeStart;
   if(nSpan<0)
   {
      nSpan += 0x100000 * 1000;
   }
   return nSpan;
}


// Begin main function
int main(int argc, char *argv[]) 
{
   //---------------------------------------------------------------------------------------------
   //BEGIN INITIALIZATION ROUTINE


   //Set up Yarp
   Network yarp;

   Property params;
   params.fromCommand(argc, argv);

   //make a port for reading images
   BufferedPort<ImageOf<PixelRgb> > imagePort; 
 
   //for output as a YARP Vector
   BufferedPort<Vector> targetPort;
   targetPort.open("/test/target/out");
   
   //give the port a name
   imagePort.open("/test/image/in"); 

   std::string robotName=params.find("robot").asString().c_str();
   std::string remotePorts="/icub/left_arm";

   std::string localPorts="/test/client";

   Property options;
   options.put("device", "remote_controlboard");
   options.put("local", localPorts.c_str());   //local port names
   options.put("remote", remotePorts.c_str());         //where we connect to

   // create a device
   PolyDriver robotDevice(options);
   if (!robotDevice.isValid()) 
   {
      printf("Device not available.  Here are the known devices:\n");
      printf("%s", Drivers::factory().toString().c_str());
      return 0;
   }

   IPositionControl *pos;
   IEncoders *encs;

   //Make sure we can acquire interfaces
   bool ok;
   ok = robotDevice.view(pos);
   ok = ok && robotDevice.view(encs);
   if (!ok) 
   {
      printf("Problems acquiring interfaces\n");
      return 0;
   }

   int nj=0;

   //Assigns an integer value to nj (number of joints) based on remotePorts we entered earlier
   pos->getAxes(&nj);

   Vector encoders;
   Vector command;
   Vector tmp;

   encoders.resize(nj);
   tmp.resize(nj);
   command.resize(nj);

   //setting the reference acceleration limits for intialization   
   int i;
   for (i = 0; i < nj; i++) 
   {
      tmp[i] = 10.0;
   }
   pos->setRefAccelerations(tmp.data());
    
   //setting the reference speed limits for intialization 
   for (i = 0; i < nj; i++) 
   {
      tmp[i] = 10;
   }
   pos->setRefSpeeds(tmp.data());




   //Initialize robot to hold platform (if necessary)    
   initialize(pos, initWristAngle, command);
 
   //Flag for zeroing out platform at begining  
   int calibrate = 1;


   //DONE WITH INITIALIZATION ROUTINE
   //---------------------------------------------------------------------------------------------





   //---------------------------------------------------------------------------------------------
   //SET UP VARIABLES FOR CODE

   //variables to calculate center of balls
   double green_xMean = 0;
   double green_yMean = 0;
   double orange_xMean = 0;
   double orange_yMean = 0;
   double pink_xMean = 0;
   double pink_yMean = 0;

   //variables for calculating error & time
   double dist_to_pink;
   double dist_to_orange; 
   double totalTime=0;
   int nTimeStart;
   int nTimeElapsed;
   double deltaT=0;
   double u=0;

   //PID GAINS FOR CONTROL
   ///////////////////////////////////////////////////////////////////////////////////////////////
   double K = 0.15587;
   double Kdiff = 0.18338;
   double Kint = 0.057296;
   ///////////////////////////////////////////////////////////////////////////////////////////////

   //FILTER VALUES (if filter is used)
   double alpha = 0.115;
   double w = 50;

   //Initial wrist offset
   double offset=0.00;
   Network::connect("/icub/camcalib/left/out","/test/image/in");

   // read an image
   ImageOf<PixelRgb> *image = imagePort.read();


   //DONE WITH SETTING UP VARIABLES
   //---------------------------------------------------------------------------------------------





   //---------------------------------------------------------------------------------------------
   //SET REFERENCE ACCELERATIONS & SPEEDS


   //for setting the reference acceleration limits
   for (int i = 0; i < nj; i++)
   {
      tmp[i] = 10.0;
   }
   pos->setRefAccelerations(tmp.data());

   //setting the reference speed limits
   for (int i = 0; i < nj; i++) 
   {
      tmp[i] = 10;
      pos->setRefSpeed(i, tmp[i]);
      //printf("i=%i nj=%i\n",i,nj);
   }

   //Write data to file, if writer flag is triggered
   dataWrite(writer, 0, tmp[4], K, Kdiff, Kint, 0.0, 0.0, 0.0, 0.0);


   //DONE WITH SETTING REFERENCE ACCELERATIONS & SPEEDS
   //---------------------------------------------------------------------------------------------





   //---------------------------------------------------------------------------------------------
   //BEGIN LOOP TO CONTINUE TO RUN

   while(run)
   {
      top:
       
      // Begin the timer
      if(counter>0)
      {
         nTimeStart = GetMilliCount();
      }
    

      //read an image
      ImageOf<PixelRgb> *image = imagePort.read();
      //Run Vision Algorithm, Detect X,Y centers of balls
      vision(debug, &pink_xMean, &pink_yMean, &orange_xMean, &orange_yMean , &green_xMean, &green_yMean, image);
      

      //If first time running, calibrate beam so it is approximately level
      offset = calibration(&calibrate, &counter, pink_yMean, orange_yMean, writer, offset); 
      if(calibrate==1)
      {
         //move wrist
          command[0]=-60;
          command[1]=6;   
          command[2]=0;
          command[3]=20;
          command[4]=initWristAngle+offset;
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
         
         pos->positionMove(command.data()); 

         Time::delay(0.1);
	 goto top;
      }         



      //calculating parts for new error
      dist_to_pink = sqrt((green_xMean-pink_xMean)*(green_xMean-pink_xMean)+(green_yMean-pink_yMean)*(green_yMean-pink_yMean));
      dist_to_orange = sqrt((orange_xMean-green_xMean)*(orange_xMean-green_xMean)+(orange_yMean-green_yMean)*(orange_yMean-green_yMean));
    

      //Get time elapsed for the calculation since begining of while loop
      nTimeElapsed = GetMilliSpan( nTimeStart );
      deltaT = nTimeElapsed/1000.0;
      if(debug==1)
      {
         printf("deltaT = %g\n",deltaT);
      }
      totalTime+=deltaT;

      //Write data to file, if writer flag is triggered
      dataWrite(writer, 2, totalTime, deltaT, pink_xMean, pink_yMean, orange_xMean, orange_yMean, green_xMean, green_yMean);

      //Do control algorithm
      u=control(dist_to_pink, dist_to_orange, K, Kint, Kdiff, alpha, w, deltaT, counter, initWristAngle, offset, debug, writer);


      //MOVE!
      if(killMotors==0)
      {
         command[0]=-60;
         command[1]=6;
         command[2]=0;
         command[3]=20;
         command[4]=u;
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
	
	 pos->positionMove(command.data());
         
      }

      
      //Write data to file, if writer flag is triggered
      dataWrite(writer, 4, encoders[4], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);


      //if writer flag is triggered, check running time
      if(writer==1)
      {
         printf("totalTime= %g\n",totalTime);
         if(totalTime>60)
         {
            run=0;
         }
      }
      counter++;
   }

   //DONE WITH RUNNING LOOP
   //---------------------------------------------------------------------------------------------

   //Finalize writing (if necessary) and end program
   dataWrite(writer, 5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   robotDevice.close();

   return 0;
}
