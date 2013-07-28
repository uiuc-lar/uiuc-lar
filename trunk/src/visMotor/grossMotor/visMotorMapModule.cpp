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

	//res = neurons/degree
	double res;
	double dTMax;

	//SOM params
	int mmapSize;
	int usedJoints;

	//rng
	//const gsl_rng_type *T;
	//gsl_rng *r;

	//limits: don't let these be changed by file for now
	int azMin; int azMax;
	int elMin; int elMax;
	int verMin; int verMax;

	//number of units along a dimension
	int Y;
	int P;
	int V;

	SOM ****visField;

	int count;

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
		res = rf.check("res",Value(0.2)).asDouble();
		dTMax = rf.check("dTMax",Value(1)).asDouble();
		//mmapSize = rf.check("mmapSize",Value(3)).asInt();
		mmapSize = rf.check("mmapSize",Value(2)).asInt(); //try this
		usedJoints = rf.check("usedJoints",Value(4)).asInt();
		return true;
	}

	virtual bool threadInit(){

		if(!handleParams()){
			return false;
		}

		azMin = -80; azMax = 0;
		elMin = -80; elMax = 0;
		verMin = 0; verMax = 20;
		//number of units along a dimension
		Y = (azMax-azMin)*res;
		P = (elMax-elMin)*res;
		V = (verMax-verMin)*res;

		//initialize model
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

		headLoc = new BufferedPort<Vector>;
		headLoc->open("/visMotor/head:i");
		armLoc = new BufferedPort<Vector>;
		armLoc->open("/visMotor/arm:i");

		count = 0;

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
			Vector *head = headLoc->read(false);
			Vector *arm = armLoc->read(false);
			if(arm && head){
				Bottle tStamp;
				headLoc->getEnvelope(tStamp);
				if(Time::now() - tStamp.get(0).asDouble() < dTMax){
					printf("Found a sufficiently recent head position, training map\n");
					count++;
					int wY = floor(((*head)(0)-azMin)*res);
					int wP = floor(((*head)(1)-elMin)*res);
					int wV = floor(((*head)(2)-verMin)*res);
					printf("Head position: %.1lf %.1lf %.1lf\n", (*head)(0), (*head)(1), (*head)(2));
					if(!(wY < 0 || wY >= Y || wP < 0 || wP >= P || wV < 0 || wV >= V)){
						double step = 0.5*exp(-count*1.0/(5*Y*P*V*mmapSize));
						printf("Current step size %.3lf\n", step);
						visField[wY][wP][wV]->update(arm,step);
						//if(count > (5*Y*P*V*mmapSize)){
						//	printf("Training visual neighbors\n");
						//	for(int i = -1; i < 2; i++){
						//		for(int j = -1; j < 2; j++){
						//			for(int k = -1; k < 2; k++){
						//				if(!(i==0 && j==0 && k==0)){
						//					if(!((wY+i) < 0 || (wY+i) >= Y || (wP+j) < 0 || (wP+j) >= P || (wV+k) < 0 || (wV+k) >= V)){
						//						visField[wY+i][wP+j][wV+k]->update(arm,step/4);
						//					}
						//				}
						//			}
						//		}
						//	}
						//}
					}
				}
			}
		}
		else{
			printf("Check connections to /visMotor/head:i and /visMotor/arm:i\n");
		}
	}


	virtual void threadRelease(){
		headLoc->interrupt();
		armLoc->interrupt();
		headLoc->close();
		armLoc->close();
		delete visField;
		delete headLoc;
		delete armLoc;
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
			reply = command;
		}
		else{
			reply = "Unrecognized command\n";
		}
		return true;
	}

	virtual bool configure(ResourceFinder &rf){
		rpcPort = new Port;
		rpcPort->open("/visMotor");
		attach(*rpcPort);
		thr = new visMotorMapThread(50,rf);
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
		//thr->stop();
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
