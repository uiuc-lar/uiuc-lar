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
   
#include <yarp/dev/PolyDriver.h>   
#include <yarp/dev/AudioGrabberInterfaces.h>   
#include <yarp/sig/all.h>

yarp::sig::Sound synthSound(std::string file_name);
void read_file (const char * fname);
  
using namespace std;
using namespace yarp::os;   
using namespace yarp::sig;   
using namespace yarp::dev;   
 


// 
yarp::sig::Sound synthSound(std::string file_name){
    Sound synth;
    double t=0;
    double value;
    int sample_index, channel_index;
    ifstream datafile;
    
    datafile.open("wavfiles/jingle_bells.txt");
    

    
    //printf("just read: %f", value);

    
    synth.setFrequency(8000);
    printf("\n\nFrequency: %i\n", synth.getFrequency());
    printf("Samples: %i\n", synth.getSamples());
    
    synth.resize(1000000, 2);
    
    printf("Samples: %i\n", synth.getSamples());
    
    
    
    for(sample_index=0; sample_index<synth.getSamples(); sample_index++){
    
        datafile>>value;// grab value from the next line in the file
        
        if (sample_index < 10){
            printf("Value #%i: %f\n", sample_index, value);
        }
        
        // thow it into the sound object
        for(channel_index=0; channel_index<synth.getChannels(); channel_index++){
            synth.set(value*3, sample_index, channel_index);
        }
        
    }
    datafile.close();
    //synth.resize(110000, 2);
    return(synth);
}


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
    std::string remotePorts="/";
    remotePorts+=robotName;
    remotePorts+="/face/raw/in";

    // Initialize the network before we can do anything else
    Network::init();   
    
    // port for emotion interface
    Port toEmotionInterface;
    std::string portname = "/facewriter"; // does the port name matter? I don't really think it does...
    toEmotionInterface.open(portname.c_str());
    
    //Network::connect(toEmotionInterface.getName(),"/icubSim/face/raw/in");
    Network::connect(toEmotionInterface.getName(), remotePorts);

    
    // Setup bottles with mouth position commands
    Bottle open_mouth;
	open_mouth.addString("M16");
	
	Bottle big_mouth;
	big_mouth.addString("M1E");

    Bottle close_mouth;
	close_mouth.addString("M1A");
	
	Bottle no_mouth;
	no_mouth.addString("M08"); // actually still a mouth
	
	Bottle mouth; // this will get set as each face is chosen
	
	Bottle rbrow;
	rbrow.addString("R04");
	
	Bottle lbrow;
	lbrow.addString("L04");
	
	Bottle rlowbrow;
	rlowbrow.addString("R01");

	Bottle llowbrow;
	llowbrow.addString("L01");	

	
	//Bottle brows;
	//brows.addString("L01");
	//brows.addString("R01");	
	
	toEmotionInterface.write(rbrow);
	toEmotionInterface.write(lbrow);
	//toEmotionInterface.write(brows);
		
    toEmotionInterface.write(no_mouth);
    
    // Get an audio device, default to portaudio
    Property p;   
    //if (argc>1) {   
    //    p.fromCommand(argc,argv);   
    //} else {   
    p.fromString("(device portaudio)");   
    //}
    
    PolyDriver poly(p);   
    if (!poly.isValid()) {   
        printf("cannot open driver\n");   
        return 1;   
    }   
   
   
    // Make sure we can both read and write sound     
    IAudioRender *put;   
    
    // make sure the interface opened correctly
    poly.view(put);   
    if (put==NULL) {   
        printf("cannot open interface\n");   
        return 1;   
    }   

    // get sound object from custom synthesis function
    Sound soundy=synthSound("test_file.txt");
    Sound synth_seg; // will grab segments of soundy
    synth_seg.setFrequency(soundy.getFrequency());
    
    // get ready to setup facial expressions
    //////////////////////////////////////////
    ifstream label_file;
    
    label_file.open("wavfiles/jingle_bells_mouth.txt");
    int i;
    std::string line;
    std::string note;
    int previous_type;
    double start=0;
    double sample_index=0;
    double seg_start;
    double seg_stop;
    
    bool started = false;
    
    //start = 0;//Time::now();
    //put->renderSound(soundy); // render the sound to the speakers
    

    while(std::getline(label_file, line)){
    
        std::istringstream iss(line);

        iss >> seg_start;
        iss >> seg_stop;
        iss >> note;
        
        printf("Start: %f\nStop:%f\n", seg_start, seg_stop);
        printf("Note: %s\n", note.c_str());
        
        //std::string note_type(1, note.at(0));
        
        // load sound
        synth_seg.resize(int((seg_stop-seg_start)*soundy.getFrequency()));
        for(i=0; i<=((seg_stop-seg_start)*soundy.getFrequency())-1;i++){
            synth_seg.set(soundy.get(sample_index+i), i);
        }
        sample_index += i;
        
        // choose mouth
        if (seg_stop-seg_start < 0.1) {
            
            // Take no action the segment is very small. Wait and grab the next segment. 
            //Time::delay(seg_stop-seg_start);
            
        } else {

            switch(note[0]) {
                case 'S':
                    //toEmotionInterface.write(no_mouth);
                    //Time::delay(stop-start);
                    mouth = no_mouth;
                    previous_type = 0;
                    break;
                case 'F': 
                    //toEmotionInterface.write(close_mouth);
                    //Time::delay(stop-start);
                    mouth = close_mouth;
                    previous_type = 0;
                    break;
                    
                case 'P': // effectively does the same as case 'V'
                case 'V':
                    if (seg_stop-seg_start > 0.15) {
                        if (previous_type == 1) {
                            //toEmotionInterface.write(big_mouth);
                            mouth = big_mouth;
                            previous_type = 0;
                        } else {
                            //toEmotionInterface.write(open_mouth);
                            mouth = open_mouth;
                            previous_type = 1;
                        }
                    }
                    //Time::delay(stop-start);
                    break;
            }
        }
        
        //printf("HELLO!!!!!!!!!!!@#$$$$$$$$$$$$$$$$$$$$$$$$$$");
        if(!started) {
            start = Time::now();
        }
        
        while(seg_start > double(Time::now() - start)-0.1){
            // do nothing?
            //Time::delay(0.05);
        }
        
        toEmotionInterface.write(mouth);
        put->renderSound(synth_seg);
        //put->renderSound(synth_seg);  
            
    }
    toEmotionInterface.write(no_mouth);

    Network::fini();   

    return 0;   
}   



