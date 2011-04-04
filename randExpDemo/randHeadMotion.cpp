/*
 * randHeadMotion.cpp
 * Lydia Majure
 * Moves head randomly
 * params:
 * --robot <robotname>
 */

//yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

//system
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

//gsl
#include <gsl/gsl_rng.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

int main(int argc, char *argv[]){
	Network yarp;

	//initialize rand num gen
	const gsl_rng_type *T;
	gsl_rng *r;
	gsl_rng_env_setup();
	T = gsl_rng_default;
	r = gsl_rng_alloc(T);

	Property params;
	params.fromCommand(argc,argv);

	if (!params.check("robot")){
		fprintf(stderr, "Please specify robot name");
		fprintf(stderr, "e.g. --robot icub");
		return -1;
	}
	std::string robotName = params.find("robot").asString().c_str();
	std::string remotePorts = "/";
	remotePorts += robotName;
	remotePorts += "/head";
	std::string localPorts = "/randHead/cmd";

	Property options;
	options.put("device", "remote_controlboard");
	options.put("local", localPorts.c_str());
	options.put("remote", remotePorts.c_str());

	PolyDriver robotDevice(options);
	if (!robotDevice.isValid()){
		printf("Device not available. Here are known devices: \n");
		printf("%s", Drivers::factory().toString().c_str());
		Network::fini();
		return 1;
	}

	IPositionControl *pos;
	IEncoders *enc;

	bool ok;
	ok = robotDevice.view(pos);
	ok = ok && robotDevice.view(enc);

	if (!ok){
		printf("Problems acquiring interfaces\n");
		return 0;
	}

	int nj = 0;
	pos->getAxes(&nj);
	Vector encoders;
	Vector command;
	Vector commandCart;
	Vector tmp;
	encoders.resize(nj);
	tmp.resize(nj);
	command.resize(nj);
	commandCart.resize(nj);

    for (int i = 0; i < nj; i++) {
         tmp[i] = 50.0;
    }
    pos->setRefAccelerations(tmp.data());

    for (int i = 0; i < nj; i++) {
        tmp[i] = 10.0;
        pos->setRefSpeed(i, tmp[i]);
    }

    command = 0;

    pos->positionMove(command.data());
    bool done = false;
    while (!done){
    	pos->checkMotionDone(&done);
    	Time::delay(0.1);
    }

    //move the head
    // if moving object in view, center on it
    // if not, move randomly
    // be sure to grab the motion AFTER the head moves
    // robot needs to be able to look at the object
    // have robot learn to look at anything it can already see

    //bool attn = 0;
    //attn = Network::exists("/salience/map");
    //if (attn) {
    //	Port sal;
    //	sal.open("/randHead/salience");
    //	Network::connect("/salience/map","/randHead/salience");
    //}

    //just move randomly for now
    while (true){
    	tmp = command;
    	command[0] += 20*(2*gsl_rng_uniform(r)-1);
    	command[1] += 20*(2*gsl_rng_uniform(r)-1);
    	command[2] += 20*(2*gsl_rng_uniform(r)-1);
    	if (command[0] < -30 || command[0] > 30){
    		command[0] = tmp[0];
    	}
    	if (command[1] < -30 || command[1] > 30){
    		command[1] = tmp[1];
    	}
    	if (command[2] <-30 || command[2] > 30){
    		command[2] = tmp[2];
    	}
    	pos->positionMove(command.data());
    	done = false;
    	while(!done){
    	    pos->checkMotionDone(&done);
    	    Time::delay(0.1);
    	}
    	Time::delay(10);
    }


    robotDevice.close();
    gsl_rng_free(r);

    return 0;
}
