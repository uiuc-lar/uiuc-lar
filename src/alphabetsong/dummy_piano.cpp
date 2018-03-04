// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-   
//
// This script must be run with the simulator and simFaceExpressions in order to work
//

#include <stdio.h>   
#include <math.h>   
#include <string.h>

#include <ios>
#include <iostream>
#include <fstream>
#include <sstream>
   
#include <yarp/os/all.h>   
   
#include <yarp/sig/all.h>

using namespace std;
using namespace yarp::os;   
//using namespace yarp::sig;   
//using namespace yarp::dev;   
 
int main(int argc, char *argv[]) {   

    Property params;
    params.fromCommand(argc, argv);

    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return -1;
    }
    std::string robotName=params.find("robot").asString().c_str();

    // Initialize the network before we can do anything else
    Network::init();   


    // code for port on receiver side
    // ////////
    RpcServer singer;
    string singer_name ="/piano:i";
    singer.open(singer_name.c_str());
    yarp::os::Bottle piano_trigger, piano_ack;

    piano_ack.addInt(1); //contents shouldn't matter, just that it sends something

    
    
    // get ready to setup facial expressions
    //////////////////////////////////////////
    
    while(1) {
        
        printf("Waiting for trigger...\n");
        singer.read(piano_trigger, true);
        //// do stuff here
        int x = -10;
        x = piano_trigger.pop().asInt();
        printf("%i", x);
        singer.reply(piano_ack);
    
    }


    Network::fini();   

    return 0;   
}   



