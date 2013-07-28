/*
 * learnedReachLook.cpp
 * Lydia Majure
 * Takes a learned visuomotor map and uses it to reach to specified location in headspace
 *
 * params:
 * robot
 * arm
 * map (takes file name)
 *
 * ports:
 * /learnedReach/plan : safety checking, Bottle of joint values
 * /learnedReach/pred : safety check, Bottle of cartesian values
 * /learnedReach/loc:i : az/el/ver, Vector of point to reach in head space terms
 * /learnedReach/arm:o : vector of arm joints
 */

//yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

//system
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "SOM.h"

using namespace std;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

#define BETA 1

double basin(double beta, double x){
	return exp(-beta*x*x);
}

//calculates weighting for motor control from current arm pos. and weights
void gWeights(double *N, double *x, double **weights, int mmapSize, int usedJoints){
	//double* N;
	//N = new double(mmapSize);
	double* dist;
	dist = new double[mmapSize];
	for(int k = 0; k < mmapSize; k++){
		N[k] = 0;
		dist[k] = 0;
		for(int i = 0; i < usedJoints; i++){
			dist[k] += (x[i] - weights[k][i])*(x[i] - weights[k][i]);
		}
		dist[k] = basin(0.0000001,dist[k]);
	}
	for(int k = 0; k < mmapSize; k++){
		double div = 0;
		for(int m = 0; m < mmapSize; m++){
			if (m != k){
				div += dist[m];
			}
		}
		N[k] = dist[k]/div;
	}
	delete dist;
	return;

}

YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char *argv[]){

	YARP_REGISTER_DEVICES(icubmod)

	Network yarp;
	Port armPlan;
	Port armPred;
	BufferedPort<Vector> objAngles;
	BufferedPort<Vector> armOut;
	armPlan.open("/learnedReach/plan");
	armPred.open("/learnedReach/pred");
	objAngles.open("/learnedReach/loc:i");
	armOut.open("/learnedReach/arm:o");
	bool fwCvOn = 0;
	fwCvOn = Network::connect("/learnedReach/plan","/fwdConv:i");
	fwCvOn *= Network::connect("/fwdConv:o","/learnedReach/pred");
	if (!fwCvOn){
		printf("Please run command:\n ./fwdConv --input /fwdConv:i --output /fwdConv:o\n");
		return -1;
	}

	Property params;
	params.fromCommand(argc,argv);

	if (!params.check("robot")){
		fprintf(stderr, "Please specify robot name\n");
		fprintf(stderr, "e.g. --robot icub\n");
		return -1;
	}
	std::string robotName = params.find("robot").asString().c_str();
	std::string remotePorts = "/";
	remotePorts += robotName;
	remotePorts += "/";
	if (params.check("arm")){
		remotePorts += params.find("arm").asString().c_str();
	}
	else{
		remotePorts += "left";
	}
	remotePorts += "_arm";
	std::string localPorts = "/learnedReach/cmd";
	if(!params.check("map")){
		fprintf(stderr, "Please specify learned visuomotor map file\n");
		fprintf(stderr, "e.g. --map map.dat\n");
		return -1;
	}
	string fName = params.find("map").asString().c_str();

	bool fMotor;
	if(params.check("fMotor")){
		fMotor = true;
	}
	else{
		fMotor = false;
	}

	Property option;
	option.put("device","gazecontrollerclient");
	option.put("remote","/iKinGazeCtrl");
	option.put("local","/client/gaze");

	PolyDriver clientGazeCtrl(option);

	IGazeControl *igaze=NULL;

	if (clientGazeCtrl.isValid()) {
		clientGazeCtrl.view(igaze);
	}

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
	commandCart.resize(3);

	for (int i = 0; i < nj; i++) {
		tmp[i] = 25.0;
	}
	pos->setRefAccelerations(tmp.data());

	for (int i = 0; i < nj; i++) {
		tmp[i] = 20.0;
	    pos->setRefSpeed(i, tmp[i]);
	}

	command = 0;

	//set the arm joints to "middle" values
	command[0] = -45;
	command[1] = 45;
	command[2] = 0;
	command[3] = 45;
	pos->positionMove(command.data());

	bool done = false;
	while (!done){
		pos->checkMotionDone(&done);
		Time::delay(0.1);
	}

	//not really yaw and pitch
	int azMin = -80; int azMax = 0;
	int elMin = -60; int elMax = 0;
	int verMin = 0; int verMax = 20;

	int Y; int P; int V;
	int mmapSize; int usedJoints;

	//read in first lines to get map dimensions
	string line;
	string buf;
	ifstream mapFile(fName.c_str());
	if(mapFile.is_open()){
		getline(mapFile,line);
		stringstream ss(line);
		ss >> buf;
		Y = atoi(buf.c_str());
		ss >> buf;
		P = atoi(buf.c_str());
		ss >> buf;
		V = atoi(buf.c_str());
		ss.clear();
		getline(mapFile,line);
		ss.str(line);
		ss >> buf;
		mmapSize = atoi(buf.c_str());
		ss >> buf;
		usedJoints = atoi(buf.c_str());
	}
	else{
		printf("Unable to open map file\n");
		return -1;
	}

	printf("Y: %i, P: %i, V: %i\n", Y, P, V);
	printf("mmapSize: %i, usedJoints: %i\n", mmapSize, usedJoints);

	//create SOM structure
	SOM ****visField;
	visField = new SOM***[Y];
	for (int y = 0; y < Y; y ++){
		visField[y] = new SOM**[P];
		for (int p = 0; p < P; p++){
			visField[y][p] = new SOM*[V];
			for (int v = 0; v < V; v++){
				visField[y][p][v] = new SOM(mmapSize,usedJoints);
			}
		}
	}

	double res = V/90.0;

	//populate it with values
	printf("reading map values\n");
	stringstream ss;
	while(getline(mapFile,line)){
		ss.clear();
		//getline(mapFile,line);
		ss.str(line);
		ss >> buf;
		int y = atoi(buf.c_str());
		ss >> buf;
		int p = atoi(buf.c_str());
		ss >> buf;
		int v = atoi(buf.c_str());
		for(int k = 0; k < mmapSize; k++){
			ss.clear();
			getline(mapFile,line);
			getline(mapFile,line);
			ss.str(line);
			for(int n = 0; n < usedJoints; n++){
				ss >> buf;
				visField[y][p][v]->weights[k][n] = atof(buf.c_str());
			}
		}
	}
	mapFile.close();

	printf("done reading map values\n");

	double *cmdCart;
	cmdCart = new double[3];
	double *mWeights;
	mWeights = new double[mmapSize];
	double *cmd;
	cmd = new double[nj];
	double *cmdMap;
	cmdMap = new double[usedJoints];

	//read target from port
	//use control law to pick motor neuron to be used
	//don't worry about real control law for now, just drive to a joint config
	//be sure to check values for safety
	while(true){
		//if(objAngles.getInputCount() && armOut.getOutputCount()){
		if(objAngles.getInputCount()){
			Vector *objLoc = objAngles.read();
			if(objLoc != NULL){
				printf("Target angles: %.1lf %.1lf %.1lf\n", (*objLoc)(0), (*objLoc)(1), (*objLoc)(2));
				int wY = floor(((*objLoc)(0)-azMin)*res);
				int wP = floor(((*objLoc)(1)-elMin)*res);
				int wV = floor(((*objLoc)(2)-verMin)*res);
				double foo = ((*objLoc)(0)-azMin)*res;
				printf("Selected visual location: %i %i %i\n", wY, wP, wV);
				Vector lookHere(3); lookHere(0) = (*objLoc)(0); lookHere(1) = (*objLoc)(1); lookHere(2) = (*objLoc)(2);
				igaze->lookAtAbsAngles(lookHere);
				enc->getEncoders(encoders.data());
				//mWeights = gWeights(encoders.data(),visField[wY][wP][wV]->weights,mmapSize,usedJoints);
				gWeights(mWeights,encoders.data(),visField[wY][wP][wV]->weights,mmapSize,usedJoints);
				int maxInd = 0;
				for(int i = 0; i < mmapSize; i++){
					if(mWeights[i] > mWeights[maxInd]){
						maxInd = i;
					}
				}
				cmdMap = visField[wY][wP][wV]->weights[maxInd];
				if (cmdMap[0] > 0 || cmdMap[0] < -60){
					printf("Faulty map\n");
				    return -1;
				}
				else{
					cmd[0] = cmdMap[0];
				}
				if (cmdMap[1] > 100 || cmdMap[1] < -0){
					printf("Faulty map\n");
					return -1;
				}
				else{
					cmd[1] = cmdMap[1];
				}
				if (cmdMap[2] > 60 || cmdMap[2] < -35){
					printf("Faulty map\n");
					return -1;
				}
				else{
					cmd[2] = cmdMap[2];
				}
				if (cmdMap[3] > 100 || cmdMap[3] < 10){
					printf("Faulty map\n");
					return -1;
				}
				else{
					cmd[3] = cmdMap[3];
				}
				for(int i = 4; i < nj; i++){
					cmd[i] = 0.0;
				}
				//use fwd kin to find end effector position
				Bottle plan;
				//plan->clear();
				Bottle pred;
				//pred->clear();
				for (int i = 0; i < nj; i++){
					plan.addDouble(cmd[i]);
				}
				armPlan.write(plan);
				armPred.read(pred);
				for (int i = 0; i < 3; i++){
					cmdCart[i] = pred.get(i).asDouble();
				}
				double rad = sqrt(cmdCart[0]*cmdCart[0]+cmdCart[1]*cmdCart[1]);
				if(rad > 0.3){
					printf("Moving to: %.1lf %.1lf %.1lf %.1lf\n", cmd[0], cmd[1], cmd[2], cmd[3]);
					Vector &armAct = armOut.prepare();
					armAct.resize(3);
					for(int i = 0; i < 3; i++){
						armAct[i] = cmdCart[i];
					}
					armOut.write();
					pos->positionMove(cmd);
					done = false;
					while(!done){
						pos->checkMotionDone(&done);
						Time::delay(0.1);
					}
					armOut.unprepare();
					/*
					Bottle fTrack, reply;
					fTrack.clear();
					fTrack.add(true);
					readySig.write(fTrack, reply);
					*/
				}
				else{
					printf("Unsafe value\n");
					Vector &armAct = armOut.prepare();
					armOut.write();
					armOut.unprepare();
				}
			}
			else{
				//write a null bottle to preserve timing
				Vector &armAct = armOut.prepare();
				armOut.write();
				armOut.unprepare();
			}
		}
	}
	robotDevice.close();
	delete pos; delete enc;
	delete cmd; delete cmdCart; delete cmdMap; delete mWeights;
	delete visField;
	//armPlan.close(); armPred.close(); objAngles.close(); armOut.close();
	return 0;
}
