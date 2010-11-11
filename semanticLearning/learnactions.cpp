#define LTHRESH -5
#define EPOCHS 3
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


using namespace std;

int main(int argc, char **argv) {

	//model parameters
	int r = 10; //hidden model dimensionality
	int b = 10; //size of HMM bank
	real prior = 0.001; //minimum value of Aij
	int lpcPoles = 3;
	int d = lpcPoles; //observation dimensionality
	int framesize = 512;

	//classifiers
	Allocator * allocator = new Allocator;
	HMM ** p = new HMM * [b];
	Gaussian ** obs_dist = new Gaussian * [b];
	real * likelihood = new real[b];
	int nInitialized = 0;

	//data
	IMat initData(1,d);

	//signal processing
	real ** samples;


	//libsndfile stuff
	FILE *sounds;
	FILE *mfccs;
	int bytesRead = 1;
	vector<string> soundFiles;


	//initialize the banks randomly
	for (int i = 0; i < b; i++) {
		obs_dist[i] = new(allocator) Gaussian(r,d,true);
		p[i] = new(allocator) HMM;
		p[i]->init(obs_dist[i], NULL, false, prior, 1, false, false);
	}

	//get the list of all files to open
	sounds = fopen("actionlist.ini","r");
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


		//reset the initial internal probabilities
		for (int k = 0; k < nInitialized; k++) {
			likelihood[k] = 0;
			for (int l = 0; l < r; l++) {
				p[k]->prob->ptr[l] = 1.0/(double)r;
			}
		}
		lMaxIdx = b+1;
		lMax = -1.7e+300;

		//make an ML classification
		for (int j = 0; j < z; j++) {
			//over all HMMs in the bank
			for (int k = 0; k < nInitialized; k++) {
				p[k]->Classify(samples[j]);
				likelihood[k] +=log10(1.0/p[k]->scale)*(double)(1.0/(z+1));
			}
		}

		for (int k = 0; k < nInitialized; k++) {
			printf("%f ",likelihood[k]);
			if (likelihood[k] > lMax && likelihood[k] > LTHRESH) {
				lMax = likelihood[k];
				lMaxIdx = k;
			}
		}
		printf("%d \n", lMaxIdx);

		//if no winners (thresholded) present, re-initialize a new HMM to that sequence
		if (lMaxIdx > b && nInitialized < b) {

			double thisLikelihood = -1.7e+308;
			while (isnan(thisLikelihood) || thisLikelihood <= LTHRESH) {

				//reinitialize until it doesnt return NAN again (hackish?)
				initData.resize(z,d);
				for (int j = 0; j < z; j++) {
					for (int l = 0; l < d; l++) {
						initData(j,l) = samples[j][l];
					}
				}
				obs_dist[nInitialized]->KMeansInit(&initData,true,1000);

				//reset the initial internal probabilities
				for (int l = 0; l < r; l++) {
					p[nInitialized]->prob->ptr[l] = 1.0/(double)r;
				}

				//make an ML classification
				thisLikelihood = 0;
				for (int j = 0; j < z; j++) {
					p[nInitialized]->Classify(samples[j]);
					thisLikelihood +=log10(1.0/p[nInitialized]->scale)*(double)(1.0/(z+1));
				}
				int dummy = 0;
				//p[nInitialized]->print();

			}
			lMaxIdx = nInitialized;
			nInitialized++;
			for (int l = 0; l < r; l++) {
				p[lMaxIdx]->prob->ptr[l] = 1.0/(double)r;
			}
			for (int i = 0; i < EPOCHS; i++) {
				for (int j = 0; j < z; j++) {
					p[lMaxIdx]->Classify(samples[j]);
					p[lMaxIdx]->RMLEUpdate();
					//p[lMaxIdx]->print();
				}
			}

		}
		else {
			//if a winner is present, it trains on the sequence
			for (int i = 0; i < EPOCHS; i++) {
				for (int j = 0; j < z; j++) {
					p[lMaxIdx]->Classify(samples[j]);
					p[lMaxIdx]->RMLEUpdate();
				}
			}
		}

		for (int i = 0; i < z; i++) {
			delete samples[i];
		}
		delete samples;
	}

	for (int i = 0; i < nInitialized; i++) {
		p[i]->print();
	}
	printf("n inited: %d\n",nInitialized);

	return 0;
}
