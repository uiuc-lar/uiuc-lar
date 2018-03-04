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
    
    //datafile.open("wavfiles/alphabetsong.txt");
    datafile.open(file_name.c_str());
    

    
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
    //synth.resize(110000, 2); //test if resizing affects the playback
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

    // Open up ports for playing piano
    RpcClient piano;
    string piano_port_name ="/piano:o";
    piano.open(piano_port_name.c_str());
    
    yarp::os::Bottle piano_trigger, piano_ack;
    
    
    // Setup bottles with mouth position commands
    Bottle open_mouth;
	open_mouth.addString("M16");
	
	Bottle big_mouth;
	big_mouth.addString("M1E");

    Bottle close_mouth;
	close_mouth.addString("M1A");
	
	Bottle no_mouth;
	no_mouth.addString("M08"); // actually still a mouth

    Bottle smile;
    smile.addString("M09");
	
	Bottle rbrow;
	rbrow.addString("R04");
	
	Bottle lbrow;
	lbrow.addString("L04");
	
	Bottle rhighbrow;
	rhighbrow.addString("R08");

	Bottle lhighbrow;
	lhighbrow.addString("L08");	

    // port for emotion interface
    Port toEmotionInterface;

    std::string portname = "/facewriter"; // does the port name matter? I don't really think it does...
    toEmotionInterface.open(portname.c_str());
    
    //Network::connect(toEmotionInterface.getName(),"/icubSim/face/raw/in");
    //Network::connect(toEmotionInterface.getName(), remotePorts.c_str());

    do
    { 
        cout << '\n' << "Press any key to continue...";
    } while(cin.get() != '\n');

    cout << "Done!";

	
    /*
	toEmotionInterface.write(rbrow);
	toEmotionInterface.write(lbrow);
	//toEmotionInterface.write(brows);
		
    toEmotionInterface.write(no_mouth);
    
    
    // Get an audio device, default to portaudio
    Property p;   
    p.fromString("(device portaudio)");   
    
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
    std::string letter_file;
    letter_file = "a.txt";
    Sound soundy=synthSound("a.txt")
    */

    // get ready to setup facial expressions
    //////////////////////////////////////////
    
    // set mouth and eyebrows into start position
    
    // initialize array for sending a note to the piano port
    int note[] = {0, 0, 4, 4, -2, -2, 4, 
                3, 3, 2, 2, 1, 1, 1, 1, 0, 
                4, 4, 3, 2, 2, 1,
                4, 4, 4, 3, 2, 2, 1,
                0, 0, 4, 4, -2, -2, 4, 
                3, 3, 2, 2, 1, 1, 0};

    std::string letter_file = "a_mouth.txt";

    // loop over all notes of the song
    for(int k=0; k<26; k++) {
        cout << "YEs";
        //change first character to correspond to the next letter of the alphabet
        letter_file[0]++;

        // open mouth file
        ifstream label_file;
        label_file.open("wavfiles/alphamouth.txt");
        //label_file.open(letter_file);

        int i;
        std::string line;
        std::string mouth;
        int previous_type;
        double start;
        double stop;
        
        
        piano_trigger.clear();
        piano_trigger.addInt(note[k]);

        // send trigger and wait for reply. 
        // will not wait if no port is connected at all
        piano.write(piano_trigger, piano_ack);
        // print reply if want it
        //cout << piano_ack.toString() << endl;


        //put->renderSound(soundy); // render the sound to the speakers
        

        while(std::getline(label_file, line)){
            //// in main loop perform the next two lines
        
        
            std::istringstream iss(line);

            iss >> start;
            iss >> stop;
            iss >> mouth;

            
            printf("Start: %f\nStop:%f\n", start, stop);
            printf("Note: %s\n", mouth.c_str());
            
            //std::string note_type(1, note.at(0));
            
            
            if (stop-start < 0.1) {
            
                //toEmotionInterface.write(no_mouth);
                
                // Take no action the segment is very small. Wait and grab the next segment. 
                Time::delay(stop-start);
                
            } else {

                switch(mouth[0]) {
                    case 'S':
                        toEmotionInterface.write(no_mouth);
                        Time::delay(stop-start);
                        previous_type = 0;
                        break;
                    case 'F': 
                        toEmotionInterface.write(close_mouth);
                        Time::delay(stop-start);
                        previous_type = 0;
                        break;
                    case 'P':
                    case 'V':
                        if (stop-start > 0.15) {
                            if (previous_type == 1) {
                                toEmotionInterface.write(big_mouth);
                                previous_type = 0;
                            } else {
                                toEmotionInterface.write(open_mouth);
                                previous_type = 1;
                            }

                        }
                        Time::delay(stop-start);
                        break;
                }
                Time::delay(stop-start);
            }        
                
        }

    }
    toEmotionInterface.write(rhighbrow);
    toEmotionInterface.write(lhighbrow);
    toEmotionInterface.write(smile);

    Network::fini();   

    return 0;   
}   



