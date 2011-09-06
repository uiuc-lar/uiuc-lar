/*
 * dummyTrack.cpp
 * Lydia Majure
 * Utility for testing visuomotor map learning
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

#include <gsl/gsl_rng.h>


using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

YARP_DECLARE_DEVICES(icubmod)


int main(int argc, char *argv[]){
	YARP_REGISTER_DEVICES(icubmod)
	Network yarp;
	//cartesian arm location
	BufferedPort<Vector> armLoc;
	armLoc.open("/dummyTrack/arm:i");
	//head location from iKinGazeCtrl
	BufferedPort<Vector> headLoc;
	headLoc.open("/dummyTrack/head:o");

	Property option;
	option.put("device","gazecontrollerclient");
	option.put("remote","/iKinGazeCtrl");
	option.put("local","/client/gaze");

	PolyDriver clientGazeCtrl(option);

	IGazeControl *igaze=NULL;

	if (clientGazeCtrl.isValid()) {
		clientGazeCtrl.view(igaze);
	}

	//initialize rand num gen
	const gsl_rng_type *T;
	gsl_rng *r;
	gsl_rng_env_setup();
	T = gsl_rng_default;
	r = gsl_rng_alloc(T);

	while(true){
		Vector *armC = armLoc.read(false);
		if (armC){
			printf("Arm found\n");
			//add noise to it
			Vector noisyArm;
			int nj = (*armC).size();
			noisyArm.resize(nj);
			for(int i = 0; i <nj; i++){
				noisyArm[i] = (*armC)[i] + 0.05*(2*gsl_rng_uniform(r)-1);
			}
			igaze->lookAtFixationPoint(noisyArm);
			igaze->waitMotionDone();
			Vector &headAng = headLoc.prepare();
			igaze->getAngles(headAng);
			Bottle tStamp;
			tStamp.clear();
			tStamp.add(Time::now());
			headLoc.setEnvelope(tStamp);
			headLoc.write();
		}
	}
	clientGazeCtrl.close();
	headLoc.close(); armLoc.close();
	return 0;
}
