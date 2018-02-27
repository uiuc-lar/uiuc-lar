/*
 * randArmExplore.cpp
 * Lydia Majure
 * randomly moves arm, checking for self-collisions
 * params:
 * --robot <robotname>
 * --side <left/right>
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
	//Port<Bottle> armPlan;
	//Port<Bottle> armPred;
	Port armPlan;
	Port armPred;
	armPlan.open("/randArm/plan");
	armPred.open("/randArm/pred");
	bool fwCvOn = 0;
	fwCvOn = Network::connect("/randArm/plan","/fwdConv:i");
	fwCvOn *= Network::connect("/fwdConv:o","/randArm/pred");
	if (!fwCvOn){
		printf("Please run command:\n ./fwdConv --input /fwdConv:i --output /fwdConv:o");
		return 1;
	}

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
	remotePorts += "/";
	if (params.check("side")){
		remotePorts += params.find("side").asString().c_str();
	}
	else{
		remotePorts += "left";
	}
	remotePorts += "_arm";
	std::string localPorts = "/randArm/cmd";

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
         tmp[i] = 25.0;
    }
    pos->setRefAccelerations(tmp.data());

    for (int i = 0; i < nj; i++) {
        tmp[i] = 5.0;
        pos->setRefSpeed(i, tmp[i]);
    }

    command = 0;

    //set the arm joints to "middle" values
    command[0] = -45;
    command[1] = 45;
    command[2] = 0;
    command[3] = 45;
    command[9] = 50;
    command[10] = 50;
    command[12] = 50;
    command[13] = 50;
    command[14] = 50;
    pos->positionMove(command.data());

    bool done = false;
    while (!done){
    	pos->checkMotionDone(&done);
    	Time::delay(0.1);
    }

    while (true){
    	tmp = command;
    	command[0] += 15*(2*gsl_rng_uniform(r)-1);
    	command[1] += 15*(2*gsl_rng_uniform(r)-1);
    	command[2] += 15*(2*gsl_rng_uniform(r)-1);
    	command[3] += 15*(2*gsl_rng_uniform(r)-1);
    	printf("%.1lf %.1lf %.1lf %.1lf\n", command[0], command[1], command[2], command[3]);
    	//above 0 doesn't seem to be safe for joint 0
    	if (command[0] > 0 || command[0] < -90){
    		command[0] = tmp[0];
    	}
    	if (command[1] > 160 || command[1] < -0){
    		command[1] = tmp[1];
    	}
    	if (command[2] > 100 || command[2] < -35){
    		command[2] = tmp[2];
    	}
    	if (command[3] > 100 || command[3] < 10){
    		command[3] = tmp[3];
    	}
    command[9] = 50;
    command[10] = 50;
    command[12] = 50;
    command[13] = 50;
    command[14] = 50;
    	//use fwd kin to find end effector position
    	Bottle plan, pred;
    	for (int i = 0; i < nj; i++){
    		plan.add(command[i]);
    	}
    	armPlan.write(plan);
    	armPred.read(pred);
    	for (int i = 0; i < 3; i++){
    		commandCart[i] = pred.get(i).asDouble();
    	}
    	double rad = sqrt(commandCart[0]*commandCart[0]+commandCart[1]*commandCart[1]);
    	// safety radius back to 30 cm
    	if (rad > 0.3){
    		pos->positionMove(command.data());
    		done = false;
    		while(!done){
    			pos->checkMotionDone(&done);
    			Time::delay(0.1);
    		}
    	}
    	else{
    		printf("Self collision detected!\n");
    	}
    }

    robotDevice.close();
    gsl_rng_free(r);

    return 0;
}
