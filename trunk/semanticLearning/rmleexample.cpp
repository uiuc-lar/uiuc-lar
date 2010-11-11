#define LTHRESH -5
#define EPOCHS 10
#define MAX16 32767

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
#include "SequenceLearner.h"


using namespace std;

int main(int argc, char **argv) {

	//model parameters
	int r = 3; //hidden model dimensionality
	int b = 1; //size of HMM bank
	real prior = 0.001; //minimum value of Aij
	int lpcPoles = 2;
	int d = lpcPoles; //observation dimensionality
	int framesize = 512;
	int initCount = 0;

	//classifiers
	SequenceLearner * S = new SequenceLearner(r,d,b,5,-10.0,0.03,3.0);

	//data
	IMat initData(1,d);

	//signal processing
	real ** samples;


	//libsndfile stuff
	FILE *sounds;
	FILE *mfccs;
	int bytesRead = 1;
	vector<string> soundFiles;

	//get the list of all files to open
	sounds = fopen("calinon.ini","r");
	while (bytesRead == 1)  {

		char fname[100];
		bytesRead = fscanf(sounds, "%s", fname);
		if (bytesRead == 1) {
			string temp(fname);
			soundFiles.push_back(temp);
		}
	}
	fclose(sounds);

	int lMaxIdx = b+1;
	real lMax = -1.7e+300;

	//while there are still files left to go,
	for (int ftg = 0; ftg < soundFiles.size(); ftg++) {

		printf("%s\n",soundFiles.at(ftg).c_str());
		mfccs = fopen(soundFiles.at(ftg).c_str(), "r");
		int z = 0;
		char line[512];
		char * coeff;

		//find the size of the file
		while (!feof(mfccs)) {
			fscanf(mfccs,"%s",line);
			z++;
		}
		rewind(mfccs);
		z--;

		//create the feature vectors
		samples = new real*[z];
		for (int i = 0; i < z; i++) {

			samples[i] = new real[d];
			fscanf(mfccs,"%s",line);
			coeff = strtok(line,",");
			samples[i][0] = atof(coeff);
			for (int j = 1; j < lpcPoles; j++) {
				coeff = strtok(NULL,",");
				samples[i][j] = atof(coeff);
			}

		}
		fclose(mfccs);

		//present the samples to the HMMs
		/*if (initCount < b) {
			S->initialize(samples,z,initCount);
			initCount++;
			S->nInitialized++;
		} else {
			S->classify(samples,z);
		}
		//S->printAll();
		 * */

		S->train(samples,z);


		for (int i = 0; i < z; i++) {
			//printf("%f %f\n",samples[i][0],samples[i][1]);
			delete samples[i];
		}
		delete samples;

	}

	S->printAll();

	return 0;
}
