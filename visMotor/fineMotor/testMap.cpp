/*
 * testMap.cpp
 *
 *  calculate error for each receptive field
 *  option to calculate average error
 *
 *  requires fwdConv be running
 *
 *  output file as:
 *  # training samples
 *  dimensions
 *  map size and num motors
 *  map loc
 *  subunit
 *  error
 *  subunit
 *  ...
 *  avg error for maploc
 *  next maploc
 *
 *  print to terminal average error?
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

#include "SOM.h"


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

class testThread : public Thread{
protected:
	ResourceFinder &rf;

	Port *armJ; //joint config
	Port *armC; //cartesian location

	int nj;

	string name;
	string arm;
	string robotName;

	string rmFile;
	string emFile;

	string rOutFile;
	string eOutFile;

	double neckTT, eyeTT;

	SOM ****retMotMap;
	SOM ****egoMotMap;

	int count;

	PolyDriver *clientGazeCtrl;

	IGazeControl *igaze;

	//res = neurons/pixel
	double eRes;
	double rRes;

	int mmapSize;
	int usedJoints;

	int U; int V; int D;

	int Y; int P; int G;

	int uMin; int uMax;
	int vMin; int vMax;
	int dMin; int dMax;

	int azMin; int azMax;
	int elMin; int elMax;
	int verMin; int verMax;

	bool realRobot;

	int maxDiv;

public:
	testThread(ResourceFinder &_rf) : Thread(), rf(_rf){}

	virtual bool handleParams(){
		name = rf.check("name",Value("testMap")).asString().c_str();
		arm = rf.check("arm", Value("left")).asString().c_str();
		robotName = rf.check("robotName", Value("icubSim")).asString().c_str();

		rmFile = rf.check("rMFile",Value("none")).asString().c_str();
		emFile = rf.check("eMFile",Value("none")).asString().c_str();

		rOutFile = rf.check("rOutFile",Value("rResults.log")).asString().c_str();
		eOutFile = rf.check("eOutFile",Value("eResults.log")).asString().c_str();

		eRes = rf.check("eRes",Value(0.2)).asDouble();

		rRes = rf.check("rRes",Value(0.2)).asDouble();

		mmapSize = rf.check("mmapSize",Value(4)).asInt();
		usedJoints = rf.check("usedJoints",Value(3)).asInt();

		neckTT = rf.check("nt",Value(3.0)).asDouble();
		eyeTT = rf.check("et",Value(0.5)).asDouble();

		maxDiv = rf.check("maxDiv",Value(10)).asInt();

		return true;
	}

	virtual bool threadInit(){
		if(!handleParams()){
			printf("Parameter handling problem\n");
			return false;
		}

		armJ = new Port;
		armC = new Port;

		armJ->open("/testMap:o");
		armC->open("/testMap:i");

		bool fwCvOn = 0;
		fwCvOn = Network::connect("/testMap:o","/fwdConv:i");
		fwCvOn *= Network::connect("/fwdConv:o","/testMap:i");
		if (!fwCvOn){
			printf("Please run command:\n ./fwdConv --input /fwdConv:i --output /fwdConv:o\n");
			return false;
		}

		nj = 16;

		igaze = NULL;

		Property options;
		options.put("device","gazecontrollerclient");
		options.put("remote","/iKinGazeCtrl");
		options.put("local","/client/gaze");
		clientGazeCtrl = new PolyDriver;
		clientGazeCtrl->open(options);

		if(clientGazeCtrl->isValid()){
			clientGazeCtrl->view(igaze);
		}
		else{
			printf("No valid gaze controller\n");
			return false;
		}

        igaze->setNeckTrajTime(neckTT);
        igaze->setEyesTrajTime(eyeTT);

		if(robotName == "icub"){
			realRobot = true;
		}
		else if(robotName == "icubSim"){
			realRobot = false;
		}
		else{
			printf("Not a valid robot\n");
			return false;
		}

		uMin = 0; uMax = 320;
		vMin = 0; vMax = 240;
		dMin = 0; dMax = maxDiv;


		//not really yaw and pitch
		azMin = -80; azMax = 0;
		elMin = -60; elMax = 0;
		verMin = 0; verMax = 20;

		//number of units along a dimension
		U = (uMax-uMin)*rRes;
		V = (vMax-vMin)*rRes;
		D = (dMax-dMin)*rRes;

		Y = (azMax-azMin)*eRes;
		P = (elMax-elMin)*eRes;
		G = (verMax-verMin)*eRes;


		//initialize model
		retMotMap = new SOM***[U];
		for (int u = 0; u < U; u ++){
			retMotMap[u] = new SOM**[V];
			for (int v = 0; v < V; v++){
				retMotMap[u][v] = new SOM*[D];
				for (int d = 0; d < D; d++){
					retMotMap[u][v][d] = new SOM(mmapSize,usedJoints);
				}
			}
		}

		egoMotMap = new SOM***[Y];
		for (int y = 0; y < Y; y ++){
			egoMotMap[y] = new SOM**[P];
			for (int p = 0; p < P; p++){
				egoMotMap[y][p] = new SOM*[G];
				for (int g = 0; g < G; g++){
					egoMotMap[y][p][g] = new SOM(mmapSize,usedJoints);
				}
			}
		}

		count = 0;
		if(rmFile != "none"){
			ifstream mapFile;
			mapFile.open(rmFile.c_str());
			string s;
			//get training count
			getline(mapFile,s);
			istringstream cnt(s);
			cnt >> count;
			//discard lines
			getline(mapFile,s);
			getline(mapFile,s);
			for(int u = 0; u < U; u++){
				for(int v= 0; v < V; v++){
					for(int d = 0; d < D; d++){
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
								retMotMap[u][v][d]->weights[k][n] = val;
							}
						}
					}
				}
			}
			mapFile.close();
		}

		if(emFile != "none"){
			ifstream mapFile;
			mapFile.open(emFile.c_str());
			string s;
			//get training count
			getline(mapFile,s);
			istringstream cnt(s);
			cnt >> count;
			//discard lines
			getline(mapFile,s);
			getline(mapFile,s);
			for(int y = 0; y < Y; y++){
				for(int p= 0; p < P; p++){
					for(int g = 0; g < G; g++){
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
								egoMotMap[y][p][g]->weights[k][n] = val;
							}
						}
					}
				}
			}
			mapFile.close();
		}

		return true;
	}

	virtual void run(){
		//start printing outfile
		ofstream resFile;
		resFile.open(eOutFile.c_str());
		resFile << count << endl;
		resFile << Y << " " << P << " " << G << endl;
		resFile << mmapSize << " " << usedJoints << endl;
		for(int y = 0; y < Y; y++){
			for(int p = 0; p < P; p++){
				for(int g = 0; g < G; g++){
					double cumlErr = 0; //divide me by mmapSize for avg
					for(int k = 0; k < mmapSize; k++){
						//put joint values in a bottle and get back cart. val.
						Bottle plan, pred;
						plan.clear();
						pred.clear();
						for (int n = 0; n < nj; n++){
							if(n < usedJoints){
								plan.add(egoMotMap[y][p][g]->weights[k][n]);
							}
							else{
								plan.add(0.0);
							}
						}
						armJ->write(plan);
						armC->read(pred);
						yarp::sig::Vector commandCart(3);
						for (int i = 0; i < 3; i++){
							commandCart[i] = pred.get(i).asDouble();
						}
						yarp::sig::Vector headAng(3);
						headAng[0] = (double)y/eRes + azMin;
						headAng[1] = (double)p/eRes + elMin;
						headAng[2] = (double)g/eRes + verMin;
						igaze->lookAtAbsAngles(headAng);
						bool done = false;
						while(!done){
							igaze->checkMotionDone(&done);
							sleep(1);
						}
						yarp::sig::Vector realCart(3);
						igaze->getFixationPoint(realCart);
						yarp::sig::Vector testAng(3);
						igaze->getAngles(testAng);
						//test this is working until here
						printf("Head angles: %.2f, %.2f, %.2f\n",headAng[0],headAng[1],headAng[2]);
						printf("Head angles: %.2f, %.2f, %.2f\n",testAng[0],testAng[1],testAng[2]);
						printf("Learned cartesian position: %.2f, %.2f, %.2f\n",commandCart[0],commandCart[1],commandCart[2]);
						printf("Desired cartesian position: %.2f, %.2f, %.2f\n",realCart[0],realCart[1],realCart[2]);
					}
				}
			}
		}

	}

};


class testMod : public RFModule{
protected:
	testThread *thr;
	Port *rpcPort;

public:
	testMod(){}

	bool respond(const Bottle& command, Bottle& reply){
		return true;
	}

	virtual bool configure(ResourceFinder &rf){
		rpcPort = new Port;
		rpcPort->open("/testMap");
		attach(*rpcPort);
		thr = new testThread(rf);
		bool ok = thr->start();
		if(!ok){
			delete thr;
			return false;
		}
		return true;
	}

	virtual bool interruptModule(){
		rpcPort->interrupt();
		thr->stop();
		return true;
	}

	virtual bool close(){
		rpcPort->close();
		thr->stop();
		delete rpcPort;
		delete thr;
		return true;
	}

	virtual double getPeriod() { return 1.0; }
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
	testMod mod;
	mod.runModule(rf);
	return 0;
}



