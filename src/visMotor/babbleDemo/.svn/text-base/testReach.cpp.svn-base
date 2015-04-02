/*
 * testReach.cpp
 *
 *Lydia Majure
 *
 * generate a random visual fixation point in head centered coordinates
 * send to reaching program, track with dummyTrack
 * report on error
 *
 * ports:
 * /testReach/fixAng:o
 * /testReach/headLoc:i
 */

//yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//system
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

//gsl
#include <gsl/gsl_rng.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

#define PI 3.141593


int main(int argc, char *argv[]){
	Network yarp;

	const gsl_rng_type *T;
	gsl_rng *r;
	gsl_rng_env_setup();
	T = gsl_rng_default;
	r = gsl_rng_alloc(T);

	BufferedPort<Vector> headLoc;
	headLoc.open("/testReach/headLoc:i");
	BufferedPort<Vector> fixAng;
	fixAng.open("/testReach/fixAng:o");
	int nTests = 0;
	double avgErr = 0;
	while(true){
		if(headLoc.getInputCount() && fixAng.getOutputCount()){
		//if(fixAng.getOutputCount()){
			Vector &angOut = fixAng.prepare();
			angOut.resize(3);
			angOut[0] = -10 - 50*gsl_rng_uniform(r);
			angOut[1] = -10 - 30*gsl_rng_uniform(r);
			angOut[2] = 15*gsl_rng_uniform(r);
			fixAng.write();
			printf("Desired fixation: %.1lf %.1lf %.1lf\n", angOut[0], angOut[1], angOut[2]);
			Vector *actLoc = headLoc.read();
			/*if(actLoc != NULL){
				printf("Desired fixation: %.1lf %.1lf %.1lf\n", angOut[0], angOut[1], angOut[2]);
				printf("Actual arm location: %.1lf %.1lf %.1lf\n", (*actLoc)[0], (*actLoc)[1], (*actLoc)[2]);
				double dAz = angOut[0] - (*actLoc)[0];
				double dEl = angOut[1] - (*actLoc)[1];
				double dVer = angOut[2] - (*actLoc)[2];
				double dist = sqrt(dAz*dAz + dEl*dEl + dVer*dVer);
				printf("Error: %.1lf\n", dist);
				nTests++;
				avgErr = ((avgErr*(nTests-1)) + dist)/nTests;
				printf("%i tests run, average error %.1lf\n", nTests, avgErr);
			}*/
		}
	}
	gsl_rng_free(r);
	return 0;
}
