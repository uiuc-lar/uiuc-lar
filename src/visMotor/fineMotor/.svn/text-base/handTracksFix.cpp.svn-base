/*
 * handTracksFix.cpp
 *
 *  Created on: Feb 27, 2013
 *      Author: lydia
 *
 *  this is an incomplete implementation of control
 *  a complete implementation would allow for multiple active motor units
 *  this is just a proof of concept
 *
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

//gsl
#include <gsl/gsl_rng.h>

//opencv
#include <cv.h>

#include <math.h>
#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>

#include <boost/lexical_cast.hpp>

#include <time.h>

#include "SOM.h"

const int PERIOD = 50;

using namespace std;
using namespace cv;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

YARP_DECLARE_DEVICES(icubmod)

class handTracksFixThread : public RateThread{
protected:
	ResourceFinder &rf;

	Port *armPlan;
	Port *armPred;

	string name;
	string robotName;
	string arm;

	string mFile;
	string cFile;

	PolyDriver *clientGazeCtrl;
	PolyDriver *robotDevice;

	IGazeControl *igaze;
	IVelocityControl *vel;
	IEncoders *enc;

	int nj;
	yarp::sig::Vector *command;
	yarp::sig::Vector *tmp;

	//some SOM parameters

	double xMin; double xMax;
	double yMin; double yMax;
	double zMin; double zMax;

	//res = neurons/pixel
	double res;

	int mmapSize;
	int usedJoints;

	int X; int Y; int Z;

	SOM ****egoMotMap;

	//training counts for units
	double ***numTimes;

	int count;

	double gain;
	double noise;

public:
	handTracksFixThread(ResourceFinder &_rf, int period) : RateThread(period), rf(_rf){}

	virtual bool handleParams(){
		if(rf.check("robot")){
				robotName = rf.find("robot").asString().c_str();
			}
		else{
			printf("Must specify robot name\n");
			return false;
		}

		name = rf.check("name",Value("handTracksFix")).asString().c_str();
		arm = rf.check("arm", Value("left")).asString().c_str();

		mFile = rf.check("mFile",Value("none")).asString().c_str();
		cFile = rf.check("cFile",Value("none")).asString().c_str();

		res = rf.check("res",Value(20)).asDouble();

		mmapSize = rf.check("mmapSize",Value(4)).asInt();
		usedJoints = rf.check("usedJoints",Value(4)).asInt();

		gain = rf.check("gain",Value(1)).asDouble();
		noise = rf.check("noise",Value(0)).asDouble();


		return true;
	}

	virtual bool threadInit(){
		if(!handleParams()){
			return false;
		}

		armPlan = new Port;
		armPred = new Port;

		armPlan->open("/handTracksFix/plan:o");
		armPred->open("/handTracksFix/pred:i");

		Property options;
		string localPorts = "/handTracksFix/cmd";
		string remotePorts = "/" + robotName + "/" + arm + "_arm";

		options.put("device", "remote_controlboard");
		options.put("local", localPorts.c_str());
		options.put("remote", remotePorts.c_str());

		robotDevice = new PolyDriver;
		robotDevice->open(options);

		options.clear();
		options.put("device","gazecontrollerclient");
		options.put("remote","/iKinGazeCtrl");
		options.put("local","/handTracksFix/gaze");
		clientGazeCtrl = new PolyDriver;
		clientGazeCtrl->open(options);



		if(clientGazeCtrl->isValid()){
			clientGazeCtrl->view(igaze);
		}
		else{
			return false;
		}

		if (!robotDevice->isValid()){
			printf("Device not available. Here are known devices: \n");
			printf("%s", Drivers::factory().toString().c_str());
			Network::fini();
			return false;
		}

		bool ok;
		ok = robotDevice->view(vel);
		ok = ok && robotDevice->view(enc);

		if (!ok){
			printf("Problems acquiring interfaces\n");
			return false;
		}

		vel->getAxes(&nj);

		command = new yarp::sig::Vector;
		tmp = new yarp::sig::Vector;
		command->resize(nj);
		tmp->resize(nj);

		for (int i = 0; i < nj; i++) {
		         (*tmp)[i] = 25.0;
		}
		vel->setRefAccelerations(tmp->data());

		bool fwCvOn = 0;
		fwCvOn = Network::connect("/handTracksFix/plan:o","/fwdConv:i");
		fwCvOn *= Network::connect("/fwdConv:o","/handTracksFix/pred:i");
		if (!fwCvOn){
			printf("Please run command:\n ./fwdConv --input /fwdConv:i --output /fwdConv:o\n");
			return false;
		}

		xMin = -0.4; xMax = 0.0;
		yMin = -0.5; yMax = 0.0;
		zMin = -0.2; zMax = 0.5;



		//number of units along a dimension
		X = (xMax-xMin)*res;
		Y = (yMax-yMin)*res;
		Z = (zMax-zMin)*res;

		//initialize model

		egoMotMap = new SOM***[X];
		for (int x = 0; x < X; x ++){
			egoMotMap[x] = new SOM**[Y];
			for (int y = 0; y < Y; y++){
				egoMotMap[x][y] = new SOM*[Z];
				for (int z = 0; z < Z; z++){
					egoMotMap[x][y][z] = new SOM(mmapSize,usedJoints);
				}
			}
		}

		numTimes = new double**[X];
		for (int x = 0; x < X; x++){
			numTimes[x] = new double*[Y];
			for (int y = 0; y < Y; y++){
				numTimes[x][y] = new double[Z];
				for (int z = 0; z < Z; z++){
					numTimes[x][y][z] = 0;
				}
			}
		}

		count = 0;
		if(mFile != "none"){
			ifstream mapFile;
			mapFile.open(mFile.c_str());
			string s;
			//get training count
			getline(mapFile,s);
			istringstream cnt(s);
			cnt >> count;
			//discard lines
			getline(mapFile,s);
			getline(mapFile,s);
			for(int x = 0; x < X; x++){
				for(int y = 0; y < Y; y++){
					for(int z = 0; z < Z; z++){
						//discard
						getline(mapFile,s);
						for(int k = 0; k < mmapSize; k++){
							//discard
							getline(mapFile,s);
							//split the string
							getline(mapFile,s);
							istringstream iss(s);
							double val;
							for(int n = 0; n < usedJoints; n++){
								iss >> val;
								egoMotMap[x][y][z]->weights[k][n] = val;
							}
						}
					}
				}
			}
			mapFile.close();
		}

		if(cFile != "none"){
			ifstream countFile;
			countFile.open(cFile.c_str());
			string s;
			//get training count
			getline(countFile,s);
			istringstream cnt(s);
			cnt >> count;
			//discard lines
			getline(countFile,s);
			getline(countFile,s);
			for(int x = 0; x < X; x++){
				for(int y = 0; y < Y; y++){
					for(int z = 0; z < Z; z++){
						//discard
						getline(countFile,s);
						getline(countFile,s);
						istringstream iss(s);
						double val;
						iss >> val;
						numTimes[x][y][z] = val;
					}
				}
			}
			countFile.close();
		}
		return true;
	}

	virtual void run(){

		//while(isStopping() != true){
		while(1){

			yarp::sig::Vector headFix(3);
			igaze->getFixationPoint(headFix);

			//reach for nearest reachable spot?

			int wX = floor((headFix[0]-xMin)*res);
			int wY = floor((headFix[1]-yMin)*res);
			int wZ = floor((headFix[2]-zMin)*res);

			if(wX < 0){
				wX = 0;
			}
			if(wX >= X){
				wX = X-1;
			}
			if(wY < 0){
				wY = 0;
			}
			if(wY >= Y){
				wY = Y - 1;
			}
			if(wZ < 0){
				wZ = 0;
			}
			if(wZ >= Z){
				wZ = Z - 1;
			}

			int axis = 0;
			while(numTimes[wX][wY][wZ] == 0){
				//printf("Adjusting to nearest reachable\n");
				switch(axis)
				{
				case 0:
					if(wX < X/2){
						wX++;
					}
					else{
						wX--;
					}
					axis++;
					break;
				case 1:
					if(wY < Y/2){
						wY++;
					}
					else{
						wY--;
					}
					axis++;
					break;
				case 2:
					if(wZ < Z/2){
						wZ++;
					}
					else{
						wZ--;
					}
					axis = 0;
					break;
				}
			}

			if(numTimes[wX][wY][wZ] > 0){
				//get current arm pose
				yarp::sig::Vector armPose;
				armPose.resize(nj);
				enc->getEncoders(armPose.data());

				//find activations
				double act[mmapSize];
				for(int i = 0; i < mmapSize; i++){
					act[i] = 0;
					for(int j = 0; j < usedJoints; j++){
						act[i] += egoMotMap[wX][wY][wZ]->weights[i][j]*armPose[j];
					}
				}

				//find winning motor unit
				int winMotU = 0;
				double maxAct = 0;
				for(int i = 0; i < mmapSize; i++){
					if(act[i] > maxAct){
						maxAct = act[i];
						winMotU = i;
					}
				}

				//safety check: joint ranges and target hand position
				Bottle plan, pred;
				plan.clear();
				pred.clear();
				*command = 0;
				for(int i = 0; i < usedJoints; i++){
					(*command)[i] = egoMotMap[wX][wY][wZ]->weights[winMotU][i];
				}
				for (int i = 0; i < nj; i++){
					plan.add((*command)[i]);
				}
				armPlan->write(plan);
				armPred->read(pred);
				yarp::sig::Vector commandCart(3);
				for (int i = 0; i < 3; i++){
					commandCart[i] = pred.get(i).asDouble();
				}
				double rad = sqrt(commandCart[0]*commandCart[0]+commandCart[1]*commandCart[1]);
				if(rad > 0.3 && (*command)[0] > -60 && (*command)[0] < -25 && (*command)[1] > 10 && (*command)[1] < 100 && (*command)[2] > 0 && (*command)[2] < 60 && (*command)[3] > 10 && (*command)[3] < 100){
					//calculate velocity
					yarp::sig::Vector cmdVel;
					cmdVel.resize(nj);
					cmdVel = 0;
					for(int i = 0; i < usedJoints; i++){
						cmdVel[i] = (*command)[i] - armPose[i];
					}
					//scale velocity appropriately
					//not for now
					//calculate safety for velocity
					yarp::sig::Vector nextPos;
					nextPos.resize(nj);
					nextPos = 0;
					for(int i = 0; i < usedJoints; i++){
						nextPos[i] = armPose[i] + cmdVel[i]*PERIOD/1000;
					}
					plan.clear();
					pred.clear();
					for (int i = 0; i < nj; i++){
						plan.add(nextPos[i]);
					}
					armPlan->write(plan);
					armPred->read(pred);
					for (int i = 0; i < 3; i++){
						commandCart[i] = pred.get(i).asDouble();
					}
					rad = sqrt(commandCart[0]*commandCart[0]+commandCart[1]*commandCart[1]);

					if(rad > 0.3 && nextPos[0] > -60 && nextPos[0] < -25 && nextPos[1] > 10 && nextPos[1] < 100 && nextPos[2] > 0 && nextPos[2] < 60 && nextPos[3] > 10 && nextPos[3] < 100){
						vel->velocityMove(cmdVel.data());
					}
					else{
						printf("Something wrong: next arm pose radius < 30cm or out of joint limits\n");
					}
				}
				else{
					//set velocity to zero
					printf("Something wrong: location not trained\n");
				}
			}
		}
	}

	virtual void threadRelease(){
		armPlan->close();
		armPred->close();
		clientGazeCtrl->close();
		robotDevice->close();
	}

};

class handTracksFixModule : public RFModule{
protected:
	handTracksFixThread *thr;
	Port *rpcPort;

public:
	handTracksFixModule(){}

	bool respond(const Bottle& command, Bottle& reply){
		string msg(command.get(0).asString().c_str());
		string mp(command.get(1).asString().c_str());
		return true;
	}

	virtual bool configure(ResourceFinder &rf){
		rpcPort = new Port;
		rpcPort->open("/handTracksFix");
		attach(*rpcPort);
		thr = new handTracksFixThread(rf,PERIOD);
		bool ok = thr->start();
		if(!ok){
			delete thr;
			return false;
		}
		return true;
	}

	virtual bool interruptModule(){
		rpcPort->interrupt();
		return true;
	}

	virtual bool close(){
		rpcPort->close();
		thr->stop();
		delete rpcPort;
		delete thr;
		return true;
	}

	virtual double getPeriod() { return PERIOD; }
	virtual bool updateModule() {return true;}
};

int main(int argc, char *argv[]){
	YARP_REGISTER_DEVICES(icubmod)
	Network yarp;
	if(!yarp.checkNetwork()){
		return -1;
	}
	ResourceFinder rf;
	rf.configure("ICUB_ROOT",argc,argv);
	handTracksFixModule mod;
	mod.runModule(rf);
	return 0;
}

