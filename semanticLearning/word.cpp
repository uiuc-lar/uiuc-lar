#define LTHRESH -5
#define EPOCHS 70
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
#include "../speech/synthftr.h"
#include "../speech/simplevad.h"
#include "../speech/spechgen.h"

using namespace std;

int main(int argc, char **argv) {

	//model parameters
	int r = 5; //hidden model dimensionality
	int b = 15; //size of HMM bank
	real prior = 0.005; //minimum value of Aij
	int lpcPoles = 4;
	int d = lpcPoles; //observation dimensionality
	int framesize = 256;

	//classifiers
	Allocator * allocator = new Allocator;
	HMM ** p = new HMM * [b];
	Gaussian ** obs_dist = new Gaussian * [b];
	real * likelihood = new real[b];
	int nInitialized = 0;

	//data
	IMat initData(1,d);

	//signal processing and synth things
	double * frame;
	real ** samples;
	synthfeature            * ssf;
	synthfeature_generator 	* ssfg;


	//libsndfile stuff
	FILE *sounds;
	SNDFILE *sf;
	SF_INFO * sninfo = new SF_INFO;
	int bytesRead = 1;
	vector<string> soundFiles;

	//initialization of audio processing
	ssf = create_synthfeature(lpcPoles);
	ssfg = create_sfgenerator(sninfo->samplerate, lpcPoles, 0, framesize, 1,
			(int)(sninfo->samplerate/500+0.5), (int)(sninfo->samplerate/70+0.5),
			4, 1000.0/(double)(sninfo->samplerate), 0.25);

	//initialize the banks randomly
	for (int i = 0; i < b; i++) {
		obs_dist[i] = new(allocator) Gaussian(r,d,true);
		p[i] = new(allocator) HMM;
		p[i]->init(obs_dist[i], NULL, false, prior, 1, false, false);
	}

	//get the list of all files to open
	sounds = fopen("soundlist3.ini","r");
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
		//open the given sample
		sninfo->format = 0;
		sf = sf_open(soundFiles.at(ftg).c_str(), SFM_READ, sninfo);

		//read the whole sample in
		frame = new double[sninfo->frames];
		sf_readf_double(sf, frame, sninfo->frames);
		sf_close(sf);

		//scale
		for (int k = 0; k < sninfo->frames; k++) {
			frame[k] *= MAX16;
		}
		sounds = fopen("larouts.dat","w");

		//create the feature vectors
		int z = (int)(sninfo->frames/framesize);
		double baseEng = 0;
		samples = new real*[z];
		for (int i = 0; i < z; i++) {

			samples[i] = new real[d];
			extract_synthfeature(ssfg, ssf, frame+i*framesize);
			if (i == 0) {
				baseEng = log2(calc_energy(frame+i*framesize, framesize, NULL));
			}
			ssf->energy = log2(calc_energy(frame+i*framesize, framesize, NULL));
			//fprintf(sounds,"%f\n",ssf->energy);
			samples[i][0] = 5*(ssf->energy-baseEng);
			for (int j = 1; j < d; j++) {
				samples[i][j] = 5*ssf->lar[j];
			}
		}
		fclose(sounds);

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
				obs_dist[nInitialized]->reset();
				obs_dist[nInitialized]->KMeansInit(&initData,true,1000);
				p[nInitialized]->reset();
				p[nInitialized]->init(obs_dist[nInitialized], NULL, false, prior, 1, false, false);

				//reset the initial internal probabilities and train
				for (int l = 0; l < r; l++) {
					p[nInitialized]->prob->ptr[l] = 1.0/(double)r;
				}
				for (int i = 0; i < EPOCHS; i++) {
					for (int j = 0; j < z; j++) {
						p[nInitialized]->Classify(samples[j]);
						p[nInitialized]->RMLEUpdate();
					}
				}
				//reset again
				for (int l = 0; l < r; l++) {
					p[nInitialized]->prob->ptr[l] = 1.0/(double)r;
				}

				//make an ML classification
				thisLikelihood = 0;
				for (int j = 0; j < z; j++) {
					p[nInitialized]->Classify(samples[j]);
					thisLikelihood +=log10(1.0/p[nInitialized]->scale)*(double)(1.0/(z+1));
				}
				printf("%f \n",thisLikelihood);

				int dummy = 0;

			}
			lMaxIdx = nInitialized;
			nInitialized++;
			for (int i = 0; i < EPOCHS; i++) {
				for (int j = 0; j < z; j++) {
					p[nInitialized]->Classify(samples[j]);
					p[nInitialized]->RMLEUpdate();
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

		delete frame;
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
