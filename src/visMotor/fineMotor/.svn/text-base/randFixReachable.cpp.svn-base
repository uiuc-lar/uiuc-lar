/*
 * randFixReachable.cpp
 *
 *  Created on: Mar 12, 2013
 *      Author: lydia
 *
 *      randomly generate fixation points in reachable space
 *      hit space bar or something for next fixation
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

class randFixReachableThread : public RateThread{
protected:
	ResourceFinder &rf;

	string name;
	string robotName;

	string mFile;
	string cFile;

	PolyDriver *clientGazeCtrl;

	IGazeControl *igaze;

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

	const gsl_rng_type *Type;
	gsl_rng *r;

public:
	randFixReachableThread(ResourceFinder &_rf, int period) : RateThread(period), rf(_rf){}

	virtual bool handleParams(){
		if(rf.check("robot")){
				robotName = rf.find("robot").asString().c_str();
			}
		else{
			printf("Must specify robot name\n");
			return false;
		}

		name = rf.check("name",Value("handTracksFix")).asString().c_str();

		mFile = rf.check("mFile",Value("none")).asString().c_str();
		cFile = rf.check("cFile",Value("none")).asString().c_str();

		res = rf.check("res",Value(20)).asDouble();

		mmapSize = rf.check("mmapSize",Value(4)).asInt();
		usedJoints = rf.check("usedJoints",Value(4)).asInt();

		return true;
	}

	virtual bool threadInit(){
		if(!handleParams()){
			return false;
		}

		gsl_rng_env_setup();
		Type = gsl_rng_default;
		r = gsl_rng_alloc(Type);
		gsl_rng_set(r,time(NULL));


		Property options;

		options.clear();
		options.put("device","gazecontrollerclient");
		options.put("remote","/iKinGazeCtrl");
		options.put("local","/client/gaze");
		clientGazeCtrl = new PolyDriver;
		clientGazeCtrl->open(options);



		if(clientGazeCtrl->isValid()){
			clientGazeCtrl->view(igaze);
		}
		else{
			return false;
		}

		yarp::sig::Vector headFix(3);
		headFix[0] = xMin + 1.0/res;
		headFix[1] = yMin + 5.0/res;
		headFix[2] = zMin + 5.0/res;
		igaze->lookAtFixationPoint(headFix);

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

		int wX = 0; int wY = 0; int wZ = 0;
		//random gaze walk with E[diff] equal to 1 unit across each dim
		//yarp::sig::Vector dFix(3);
		//yarp::sig::Vector headFix(3);
		//igaze->getFixationPoint(headFix);
		//do{
		//	for(int i = 0; i < 3; i++){
		//		dFix[i] = headFix[i] + -0.04 + 0.08*gsl_rng_uniform(r);
		//		printf("%.2lf\n", dFix[i]);
		//	}
		//	wX = floor((dFix[0]-xMin)*res);
		//	wY = floor((dFix[1]-yMin)*res);
		//	wZ = floor((dFix[2]-zMin)*res);
		//	//check for out of bounds indices

		//} while((wX >= 0 && wX < X && wY >= 0 && wY < Y && wZ >=0 && wZ < Z && numTimes[wX][wY][wZ] == 0) || (wX < 0 || wX >= X || wY < 0 || wY >= Y || wZ < 0 || wZ >=Z));

		yarp::sig::Vector headFix(3);
		while(numTimes[wX][wY][wZ] == 0){
			wX = floor(X*gsl_rng_uniform(r));
			wY = floor(Y*gsl_rng_uniform(r));
			wZ = floor(Z*gsl_rng_uniform(r));
		}

		headFix[0] = xMin + (double)wX/res;
		headFix[1] = yMin + (double)wY/res;
		headFix[2] = zMin + (double)wZ/res;
		igaze->lookAtFixationPoint(headFix);
		printf("Looking at %.2lf, %.2lf, %.2lf\n",headFix[0],headFix[1],headFix[2]);
		//look at it until spacebar pressed or something
		printf("Press any key for a new random fixation\n");
		std::cin.ignore();


	}

	virtual void threadRelease(){
		clientGazeCtrl->close();
	}

};

class randFixReachableModule : public RFModule{
protected:
	randFixReachableThread *thr;
	Port *rpcPort;

public:
	randFixReachableModule(){}

	bool respond(const Bottle& command, Bottle& reply){
		string msg(command.get(0).asString().c_str());
		string mp(command.get(1).asString().c_str());
		return true;
	}

	virtual bool configure(ResourceFinder &rf){
		rpcPort = new Port;
		rpcPort->open("/randFixReachable");
		attach(*rpcPort);
		thr = new randFixReachableThread(rf,PERIOD);
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
	srand(time(NULL));
	Network yarp;
	if(!yarp.checkNetwork()){
		return -1;
	}
	ResourceFinder rf;
	rf.configure("ICUB_ROOT",argc,argv);
	randFixReachableModule mod;
	mod.runModule(rf);
	return 0;
}





