/*
 * visMotorMapModule.cpp
 * Lydia Majure
 */

//system
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

//yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <math.h>
#include <gsl/gsl_rng.h>

#include "SOM.h"

using namespace std;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

class visMotorMapThread : public RateThread {
protected:
	ResourceFinder &rf;

	BufferedPort<Vector> *headLoc;
	BufferedPort<Vector> *armLoc;

	string robotName;
	string name;

	//resolution, how many degrees apart are neurons along a single dimension
	double res;
	double dTMax;

	//SOM params
	int mmapSize;
	int usedJoints;

	//rng
	//const gsl_rng_type *T;
	//gsl_rng *r;

	//limits: don't let these be changed by file for now
	int yawMin; int yawMax;
	int pitchMin; int pitchMax;
	int verMin; int verMax;

	//number of units along a dimension
	int Y;
	int P;
	int V;

	SOM ****visField;

	//PolyDriver robotDevice;

	//IPositionControl *pos;
	//IEncoders *enc;

	//Vector encoders;
	//Vector command;

public:

	visMotorMapThread(int period, ResourceFinder &_rf) : RateThread(period), rf(_rf){}

	virtual bool handleParams(){
		if(rf.check("robot")){
			robotName = rf.find("robot").asString().c_str();
		}
		else{
			printf("Must specify robot name\n");
			return false;
		}

		name = rf.check("name",Value("visMotor")).asString().c_str();
		//units per visual field degree
		res = rf.check("res",Value(0.25)).asDouble();
		dTMax = rf.check("dTMax",Value(0.1)).asDouble();
		mmapSize = rf.check("mmapSize",Value(5)).asInt();
		usedJoints = rf.check("usedJoints",Value(4)).asInt();
		return true;
	}

	virtual bool threadInit(){

		if(!handleParams()){
			return false;
		}

		yawMin = -55; yawMax = 55;
		pitchMin = -30; pitchMax = 30;
		verMin = 0; verMax = 80;
		//number of units along a dimension
		Y = (yawMax-yawMin)*res;
		P = (pitchMax-pitchMin)*res;
		V = (verMax-verMin)*res;

		//initialize model
		visField = new SOM***[Y];
		for (int y = 0; y < Y; y ++){
			visField[y] = new SOM**[P];
			for (int p = 0; p < P; p++){
				visField[y][p] = new SOM*[V];
				for (int v = 0; v < V; v++){
					visField[y][p][v] = new SOM(usedJoints,mmapSize);
				}
			}
		}

		headLoc = new BufferedPort<Vector>;
		headLoc->open("/visMotor/head:i");
		armLoc = new BufferedPort<Vector>;
		armLoc->open("/visMotor/arm:i");

		/*

		//add motor stuff here
		string remotePorts = "/";
		remotePorts += robotName;
		remotePorts += "/head";
		string localPorts = "/visMotor/cmd";

		Property options;
		options.put("device", "remote_controlboard");
		options.put("local", localPorts.c_str());
		options.put("remote", remotePorts.c_str());

		robotDevice.open(options);
		if (!robotDevice.isValid()) {
			printf("Device not available. Here are known devices: \n");
			printf("%s", Drivers::factory().toString().c_str());
			Network::fini();
			return false;
		}

		bool ok;
		ok = robotDevice.view(pos);
		ok = ok && robotDevice.view(enc);

		if (!ok) {
			printf("Problems acquiring interfaces\n");
			return false;
		}

		int nj = 0;
		pos->getAxes(&nj);

		Vector tmp;
		encoders.resize(nj);
		tmp.resize(nj);

		for (int i = 0; i < nj; i++) {
			tmp[i] = 50.0;
		}
		pos->setRefAccelerations(tmp.data());

		for (int i = 0; i < nj; i++) {
			tmp[i] = 10.0;
			pos->setRefSpeed(i, tmp[i]);
		}

		tmp = 0;


		pos->positionMove(tmp.data());
		bool done = false;
		while (!done) {
			printf("Initializing head\n");
			pos->checkMotionDone(&done);
			Time::delay(0.1);
		}


		//initialize rng
		gsl_rng_env_setup();
		T = gsl_rng_default;
		r = gsl_rng_alloc(T);
		*/

		printf("Done initializing\n");

		return true;
	}

	void mapWrite(string fName){
		ofstream mapFile;
		mapFile.open(fName.c_str());
		//do something
		mapFile << Y << " " << P << " " << V << endl;
		mapFile << mmapSize << " " << usedJoints << endl;
		for(int y = 0; y < Y; y++){
			for(int p = 0; p < P; p++){
				for(int v = 0; v < V; v++){
					mapFile << y << " " << p << " " << v << endl;
					for(int k = 0; k < mmapSize; k++){
						mapFile << k << endl;
						for(int n = 0; n < usedJoints; n++){
							mapFile << (*visField[y][p][v]).weights[k][n];
							mapFile << " ";
						}
						mapFile << endl;
					}
				}
			}
		}
		mapFile.close();
		return;
	}



	virtual void run(){
		if(headLoc->getInputCount() && armLoc->getInputCount()){
			Vector *head = headLoc->read();
			Vector *arm = armLoc->read();
			Bottle tStamp;
			headLoc->getEnvelope(tStamp);
			if(Time::now() - tStamp.get(0).asDouble() < dTMax){
				printf("Found a sufficiently recent head position, training map\n");
				int wY = floor(((*head)[0]-yawMin)/Y);
				int wP = floor(((*head)[2]-pitchMin)/P);
				int wV = floor(((*head)[5]-verMin)/V);
				(*visField[wY][wP][wV]).update(arm,0.1);
			}
		}
		else{
			printf("Check connections to /visMotor/head:i and /visMotor/arm:i\n");
		}
	}


	virtual void threadRelease(){
		//robotDevice.close();
		//gsl_rng_free(r);
		//delete r;
		headLoc->interrupt();
		armLoc->interrupt();
		headLoc->close();
		armLoc->close();
		delete visField;
		delete headLoc;
		delete armLoc;
		//delete pos;
		//delete enc;
	}
};

class visMotorMapModule : public RFModule{
protected:
	visMotorMapThread *thr;
	Port *rpcPort;

public:
	visMotorMapModule() {}

	bool respond(const Bottle& command, Bottle& reply){
		string msg(command.get(0).asString().c_str());
		if(msg == "write"){
			string fName(command.get(1).asString().c_str());
			thr->mapWrite(fName);
		}
		reply = command;
		return true;
	}

	virtual bool configure(ResourceFinder &rf){
		rpcPort = new Port;
		rpcPort->open("/visMotor");
		attach(*rpcPort);
		thr = new visMotorMapThread(500,rf);
		bool ok = thr->start();
		if(!ok){
			delete thr;
			return false;
		}
		thr->mapWrite("initMap.dat");
		return true;
	}

	virtual bool interruptModule(){
		rpcPort->interrupt();
		thr->stop();
		return true;
	}

	virtual bool close(){
		rpcPort->close();
		//thr->stop();
		delete rpcPort;
		delete thr;
		return true;
	}

	virtual double getPeriod() { return 1.0; }
	virtual bool updateModule() {return true;}

};

int main(int argc, char *argv[]){
	Network yarp;
	if(!yarp.checkNetwork()){
		return -1;
	}
	ResourceFinder rf;
	rf.configure("ICUB_ROOT",argc,argv);
	visMotorMapModule mod;
	mod.runModule(rf);
	return 0;
}
