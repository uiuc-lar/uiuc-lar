#define LTHRESH -3.5
#define EPOCHS 70
#define MAX16 32767
#define SAMPLERATE 24000

//external libraries
#include <torch/general.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_rng.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <sndfile.h>
#include <vector>
#include <string>

//internal libraries
#include "../RMLE/HMM.hh"
#include "../RMLE/StochasticClassifier.hh"
#include "../RMLE/Gaussian.hh"
#include "../speech/synthftr.h"
#include "../speech/simplevad.h"
#include "../speech/spechgen.h"
#include "SequenceLearner.h"

//yarp libraries
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

int main(int argc, char **argv) {

	//model parameters
	int r = 5; //hidden model dimensionality
	int b = 8; //size of HMM bank
	real prior = 0.005; //minimum value of Aij
	int lpcPoles = 6;
	int d = lpcPoles-1; //observation dimensionality
	int framesize = 256;

	//classifiers
	SequenceLearner * S = new SequenceLearner(r,d,b,30,-10.0,0.0,10000);
	FILE * dloger;

	//signal processing and synth things
	double * frame;
	real ** samples;
	synthfeature            * ssf;
	synthfeature_generator 	* ssfg;

    // Get a portaudio read device.
    // Open the network
    Network yarp;
    Port inPort;
    inPort.open("/classifier");


	//initialization of audio processing
	ssf = create_synthfeature(lpcPoles);
	ssfg = create_sfgenerator(SAMPLERATE, lpcPoles, 0, framesize, 1,
			(int)(SAMPLERATE/500+0.5), (int)(SAMPLERATE/70+0.5),
			4, 1000.0/(double)(SAMPLERATE), 0.25);

	int initCount = 0;
	//dloger = fopen("larouts.dat","w");
	//while there are still files left to go,
	while (1) {

		//grab a buffer of data from the port and load it as our frame
		Vector data;
		inPort.read(data);
		printf("DATA RECEIVED: %d samples\n",data.length());

		frame = new double[data.length()];
		for (int i = 0; i < data.length(); i++) {
			frame[i] = data[i];
		}

		//create the feature vectors
		int z = data.length()/framesize;
		samples = new real*[z];
		for (int i = 0; i < z; i++) {

			samples[i] = new real[d];
			extract_synthfeature(ssfg, ssf, frame+i*framesize);
			ssf->energy = log2(calc_energy(frame+i*framesize, framesize, NULL));
			//samples[i][d] = ssf->energy;
			for (int j = 0; j < d; j++) {
				samples[i][j] = 5*ssf->lar[j+1];
				//fprintf(dloger,"%f,",samples[i][j]);
			}
			//fprintf(dloger,"\n");

		}

		//fprintf(dloger,"%f,%f,%f\n",-12345,-12345,-12345);

		//S->train(samples, z);

		if (initCount < b) {
			S->initialize(samples,z,initCount);
			initCount++;
			S->nInitialized++;

		}
		else {
			//break;
			S->classify(samples,z);
		}


		delete frame;
		for (int i = 0; i < z; i++) {
			delete samples[i];
		}
		delete samples;

	}
	//fclose(dloger);
	S->printAll();

	return 0;
}
