 // -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
 
 #include <string>
 #include <cstdio>
 
 #include <yarp/os/Network.h>
 #include <yarp/dev/ControlBoardInterfaces.h>
 #include <yarp/dev/PolyDriver.h>
 #include <yarp/os/Time.h>
 #include <yarp/sig/Vector.h>

#include <yarp/os/all.h>   
#include <yarp/sig/all.h>
 
 using namespace std;
 using namespace yarp::dev;
 using namespace yarp::sig;
 using namespace yarp::os;
 
 int main(int argc, char *argv[]) 
 {
     Network yarp;

    // code for port on receiver side
    // ////////
    RpcServer singer;
    string singer_name ="/piano:i";
    singer.open(singer_name.c_str());
    yarp::os::Bottle piano_trigger, piano_ack;
 
     Property params;
     params.fromCommand(argc, argv);
 
     if (!params.check("robot"))
     {
         fprintf(stderr, "Please specify the name of the robot\n");
         fprintf(stderr, "--robot name (e.g. icub)\n");
         return 1;
     }
     //start driver for right arm
     std::string robotName=params.find("robot").asString().c_str();
     std::string remotePorts="/";
     remotePorts+=robotName;
     remotePorts+="/right_arm";
 
     std::string localPorts="/test/client_ra";

     Property options;
     options.put("device", "remote_controlboard");
     options.put("local", localPorts.c_str());   //local port names
     options.put("remote", remotePorts.c_str());         //where we connect to
 
     // create a device
     PolyDriver rightArmDevice(options);
     if (!rightArmDevice.isValid()) {
         printf("Device not available.  Here are the known devices:\n");
         printf("%s", Drivers::factory().toString().c_str());
         return 0;
     }

     //start driver for left arm
     std::string remotePorts0="/";
     remotePorts0+=robotName;
     remotePorts0+="/left_arm";
 
     std::string localPorts0="/test/client_la";

     Property options0;
     options0.put("device", "remote_controlboard");
     options0.put("local", localPorts0.c_str());   //local port names
     options0.put("remote", remotePorts0.c_str());         //where we connect to
 
     // create a device
     PolyDriver leftArmDevice(options0);
     if (!leftArmDevice.isValid()) {
         printf("Device not available.  Here are the known devices:\n");
         printf("%s", Drivers::factory().toString().c_str());
         return 0;
     }



     std::string remotePorts1="/";
     remotePorts1+=robotName;
     remotePorts1+="/head";
 
     std::string localPorts1="/test/client_h";

     Property options1;
     options1.put("device", "remote_controlboard");
     options1.put("local", localPorts1.c_str());   //local port names
     options1.put("remote", remotePorts1.c_str());         //where we connect to
 
     // create a device
     printf("Starting head driver \n");
     PolyDriver headDevice(options1);
     if (!headDevice.isValid()) {
         printf("Device not available.  Here are the known devices:\n");
         printf("%s", Drivers::factory().toString().c_str());
         return 0;
     }
     printf("Done with head driver \n");
 
     IPositionControl *pos_ra;
     IPositionControl *pos_la;
     IPositionControl *pos_h;
     IEncoders *encs;
 
     bool ok;
     ok = headDevice.view(pos_ra);
     ok = headDevice.view(pos_la);
     ok = headDevice.view(pos_h);
     ok = ok && headDevice.view(encs);
 
     if (!ok) {
         printf("Problems acquiring interfaces\n");
         return 0;
     }
 

     printf("Checking if OK \n");
     int nj=0;
     int i;
     pos_ra->getAxes(&nj);
     Vector encoders_right;
     Vector command_right;
     Vector tmp_right;
     encoders_right.resize(nj);
     tmp_right.resize(nj);
     command_right.resize(nj);
     for (i = 0; i < nj; i++) {
          tmp_right[i] = 50.0;
     }
     pos_ra->setRefAccelerations(tmp_right.data());
     for (i = 0; i < nj; i++) {
         tmp_right[i] = 10.0;
         pos_ra->setRefSpeed(i, tmp_right[i]);
     }

     pos_la->getAxes(&nj);
     Vector encoders_left;
     Vector command_left;
     Vector tmp_left;
     encoders_left.resize(nj);
     tmp_left.resize(nj);
     command_left.resize(nj);
     for (i = 0; i < nj; i++) {
          tmp_left[i] = 50.0;
     }
     pos_la->setRefAccelerations(tmp_left.data());
     for (i = 0; i < nj; i++) {
         tmp_left[i] = 10.0;
         pos_la->setRefSpeed(i, tmp_left[i]);
     }

     pos_h->getAxes(&nj);
     Vector encoders_head;
     Vector command_head;
     Vector tmp_head;
     encoders_head.resize(nj);
     tmp_head.resize(nj);
     command_head.resize(nj);
     for (i = 0; i < nj; i++) {
          tmp_head[i] = 50.0;
     }
     pos_h->setRefAccelerations(tmp_head.data());
     for (i = 0; i < nj; i++) {
         tmp_head[i] = 10.0;
         pos_h->setRefSpeed(i, tmp_head[i]);
     }
 
 
     //pos->setRefSpeeds(tmp.data()))
     
     //fisrst read all encoders
     //
     printf("waiting for encoders");
     while(!encs->getEncoders(encoders_right.data()))
     {
         Time::delay(0.1);
         printf(".");
     }
     while(!encs->getEncoders(encoders_left.data()))
     {
         Time::delay(0.1);
         printf(".");
     }
     while(!encs->getEncoders(encoders_head.data()))
     {
         Time::delay(0.1);
         printf(".");
     }
     printf("\n;");
 
     command_right=encoders_right;
     command_left=encoders_left;
     command_head=encoders_head;
     //now set the shoulder to some value
     //
     command_head[0] = 0;
     pos_h->positionMove(command_head.data());

     /*command_right[0]=-25;
     command_right[1]=40;
     command_right[2]=-10;
     command_right[3]=15;
     pos_ra->positionMove(command_right.data());

     command_left[0]=-25;
     command_left[1]=40;
     command_left[2]=-10;
     command_left[3]=15;
     pos_la->positionMove(command_left.data());
     */
     
     bool done=false;
     while(!done)
     {
         pos_ra->checkMotionDone(&done);
         Time::delay(0.1);
     }

     done=false;
     while(!done)
     {
         pos_la->checkMotionDone(&done);
         Time::delay(0.1);
     }
     
     done=false;
     while(!done)
     {
         pos_h->checkMotionDone(&done);
         Time::delay(0.1);
     }
 
     int times=0;
     int toggle=0;
     while(true)
     {
         printf("Waiting for trigger...\n");
         singer.read(piano_trigger, true);

         times++;
         if (times%2)
         {
             if (toggle){
                 command_head[1]=-10;
                 toggle = 0;
             } else {
                 command_head[1] = 10;
                 toggle = 1;
             }

         }
         else 
         {
              command_head[1]=0;
         }
 
         pos_h->positionMove(command_head.data());

         int count=5;
         while(count--)
         {
                 Time::delay(0.1);
         }
         //// do stuff here
         singer.reply(piano_ack);
 
         /*int count=50;
         while(count--)
             {
                 Time::delay(0.1);
                 bool ret=encs->getEncoders(encoders_head.data());
                 
                 if (!ret)
                 {
                     fprintf(stderr, "Error receiving encoders, check connectivity with the robot\n");
                 }
                 else
                 { 
                     printf("%.1lf %.1lf %.1lf %.1lf\n", encoders_head[0], encoders_head[1], encoders_head[2], encoders_head[3]);
                 }
             }
             */
     }
 
     rightArmDevice.close();
     leftArmDevice.close();
     headDevice.close();
     
     return 0;
 }
