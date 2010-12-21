#define LTHRESH -3.5
#define EPOCHS 1
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
#include "SequenceLearnerDisc.h"
#include "../speech/mfcc.h"
#include "SequenceLearner.h"

//yarp libraries
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

int * vitClassify(HMM *, real **, int);
void dumbCSVReader(string, IMat &, int, int);

int main(int argc, char **argv) {

	//model parameters
	int r = 14; //phonetic classifier dimensionality
	int s = 7; //sequence recognizer dim.
	int b = 12; //size of HMM bank
	real prior = 0.001; //minimum value of Aij
	int d = 15; //observation dimensionality
	int w = 30; //number of mel windows
	int framesize = 256;
	int overlap = 128;
	double samplerate = 22050;

	int ninit = 0;
	IMat initData;

	//classifiers
	SequenceLearnerDisc * S = new SequenceLearnerDisc(s, &r, 1, b, 60, -50, true); //was -50
	//SequenceLearner * S = new SequenceLearner(s,r,b,10,-50.0,0.0,10000);

	Allocator *allocator = new Allocator;
	HMM * p = new(allocator) HMM;
	Gaussian * obs_dist;
	FILE * dloger;

	//signal processing and synth things
	double * frame;
	real ** samples;
	MFCCProcessor * M;

	//libsndfile stuff
	FILE *sounds;
	SNDFILE *sf;
	SF_INFO * sninfo = new SF_INFO;
	int bytesRead = 1;
	vector<string> soundFiles;



	M = new MFCCProcessor(d, w, framesize+2*overlap, overlap, samplerate, 0, samplerate/2, false);

	/*      CLIPPPPPPPPPPP HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
	//first read in the 'init' file and put it into the phonetic classifier
	//open the given sample
	sninfo->format = 0;
	//sf = sf_open("/home/logan/workspace/scripts/loganactionwords-full.wav", SFM_READ, sninfo);
	//sf = sf_open("/home/logan/workspace/scripts/phonetictest/trainingsample.wav", SFM_READ, sninfo);
	sf = sf_open("/home/logan/workspace/scripts/logan-trainingsample3.wav", SFM_READ, sninfo);

	//read the whole sample in
	frame = new double[sninfo->frames];
	sf_readf_double(sf, frame, sninfo->frames);
	sf_close(sf);

	//scale
	for (int k = 0; k < sninfo->frames; k++) {
		frame[k] *= 1e4;
	}

	//set up the feature extractor
	M->pushData(frame, sninfo->frames);

	printf("Loading data...\n");

	//load the init data
	int z = (int)(sninfo->frames/framesize);
	initData.resize(z,d);
	double * mtrash;
	samples = new real*[z];
	for (int i = 0; i < z; i++) {
		samples[i] = new real[d];
		if (!M->getMFCCs(mtrash)) {
			//break;
		}
		else {
			for (int j = 0; j < d; j++) {
				//load this two different ways because of dumbness
				samples[i][j] = (real) mtrash[j]/5;
				initData(i,j) = (real) mtrash[j]/5;
			}
			delete mtrash;		//mfcc updated to not allocate if returns false
		}

		printf("frame %d has been loaded\n",i);
	}

	printf("Beginning initialization...\n");

	//do a kmeans init on the output dists
	obs_dist = new(allocator) Gaussian(r, d, NULL, &initData, 100);
	printf("K-means complete...\n");

	obs_dist->print(500,10);

	//initialize HMM
	p->init(obs_dist, NULL, false, prior, 1, false, false);

	printf("EPSILON: %f\n",p->eps0);

	printf("Initialization complete...\n");

	//intermediate cleanup
	delete frame;
	//initData;
	M->flushPipeline();

	printf("Beginning training...\n");
	//train HMM on init data
	for (int k = 0; k < EPOCHS; k++) {
		printf("Training epoch %d\n",k);
		for (int i = 0; i < z; i++) {
			p->Classify(samples[i]);
			p->RMLEUpdate();
		}
	}
	printf("Training complete...\n");

	p->print(2500,10);

	FINISH CLIIIIIIIIIIIIIIIIIIIIIIIPPPPPPPPPPPPPINGGGGGGGGG HEREEEEEEEEEEEEEEEE*/

	///* CLIP **************************************************************************
	//load in phonetic classifier parameters from a file (if desired)
	string aFile("/home/logan/workspace/scripts/phonetictest/goodparams3.A");
	string muFile("/home/logan/workspace/scripts/phonetictest/goodparams3.MU");
	string uFile("/home/logan/workspace/scripts/phonetictest/goodparams3.U");

	IMat fA;
	IMat fMU;
	IMat fU;

	dumbCSVReader(aFile, fA, r, r);
	dumbCSVReader(muFile, fMU, r, d);
	dumbCSVReader(uFile, fU, r, d*d);

	//init gaussian obs
	obs_dist = new(allocator) Gaussian(r, d, &fMU, &fU, prior, true, false);
	//initialize HMM
	p->init(obs_dist, &fA, false, prior, 1, true, false);

	obs_dist->print(500,10);
	//CLIP *************************************************************** */

	//load in the novel data samples files
	sounds = fopen("soundlist4.ini","r");
	while (bytesRead == 1)  {

		char fname[100];
		bytesRead = fscanf(sounds, "%s", fname);
		if (bytesRead == 1) {
			string temp(fname);
			soundFiles.push_back(temp);
		}
	}
	fclose(sounds);

	//now do a phonetic classification on all the data piece by piece
	for (int ftg = 0; ftg < soundFiles.size(); ftg++) {
	//for (int ftg = 0; ftg < b; ftg++) {

		FILE *mfccs;
		//int * vsequence;
		int ** sequence;

		char tnum[10];
		string curFile("/home/logan/workspace/scripts/phonetictest/mfcc-symb");
		string mfccFile(curFile);

		printf("Classifying %s\n",soundFiles.at(ftg).c_str());
		//open the given sample
		sninfo->format = 0;
		sf = sf_open(soundFiles.at(ftg).c_str(), SFM_READ, sninfo);

		//read the whole sample in
		frame = new double[sninfo->frames];
		sf_readf_double(sf, frame, sninfo->frames);
		sf_close(sf);

		//scale
		for (int k = 0; k < sninfo->frames; k++) {
			frame[k] *= 1e4;
		}

		sprintf(tnum,"%d",ftg+1);
		curFile += tnum;
		curFile += ".dat";
		mfccFile += tnum;
		mfccFile += ".mfcc";
		mfccs = fopen(mfccFile.c_str(),"w");

		//set up the feature extractor
		M->pushData(frame, sninfo->frames);

		//create the feature vectors
		int z = (int)(sninfo->frames/framesize);
		double * mtrash;
		samples = new real*[z];
		for (int i = 0; i < z; i++) {
			samples[i] = new real[d];
			if (!M->getMFCCs(mtrash)) {
				//break;
			}
			else {
				for (int j = 0; j < d; j++) {
					samples[i][j] = (real) mtrash[j]/5;
					fprintf(mfccs, "%f,", samples[i][j]);
				}
				fprintf(mfccs,"\n");
				delete mtrash;
			}
		}

		//flush out anything not used
		M->flushPipeline();
		fclose(mfccs);

		sounds = fopen(curFile.c_str(),"w");

		//classify
		//vsequence = vitClassify(p, samples, z);
		sequence = new int * [z];
		p->prob->fill(1.0/(float)r);
		for (int i = 0; i < z; i++) {

			//discrete version
			sequence[i] = new int[1];
			sequence[i][0] = p->Classify(samples[i]);
			//printf("%d,",sequence[i][0]);
			fprintf(sounds,"%d\n",sequence[i][0]);
		}
		//printf("\n");
		//load sequence into learner
		if (ninit < b) {
			//S->train(sequence,z);
			S->initialize(sequence,z,ninit);
			ninit++;
			S->nInitialized++;
		} else {
			int gromb = S->classify(sequence,z);
			//S->train(sequence,z);
			printf("%d\n",gromb);
		}

		//delete vsequence;

		//cleanup
		fclose(sounds);
		delete sequence;
		delete frame;
		for (int i = 0; i < z; i++) {
			delete samples[i];
		}
		delete samples;

	}

	//p->print(500,10);
	//S->printAll();

	string modelNames("/home/logan/workspace/scripts/phonetictest/word-model");
	S->printToFile(modelNames);

	return 0;
}


int * vitClassify(HMM * p, real ** samples, int length) {

	int aMax;
	real vMax;

	int * states = new int[length];
	int ** bpoints = new int *[length];
	real ** phi = new real *[length];

	for (int i = 0; i < length; i++) {
		bpoints[i] = new int[p->r];
		phi[i] = new real[p->r];
	}

	//pi = uniform
	p->b->Classify(samples[0]);
	for (int i = 0; i < p->r; i++) {
		//p->prob->ptr[i] = (real)(1/r);
		phi[0][i] = log10((1.0/(real)p->r));
		phi[0][i] += log10(p->b->prob->ptr[i]);
		bpoints[0][i] = -1;
	}

	//begin
	for (int i = 1; i < length; i++) {
		//calculate post probs
		for (int j = 0; j < p->r; j++) {

			//search over all transitions in
			vMax = -1.7E+308;
			for (int k = 0; k < p->r; k++) {
				real tVal = phi[i-1][k] + log10(p->A->ptr[k][j]);
				if (tVal > vMax) {
					vMax = tVal;
					aMax = k;
				}
			}

			//tack on this penalty
			p->b->Classify(samples[i]);
			phi[i][j] = vMax + log10(p->b->prob->ptr[j]);
			bpoints[i][j] = aMax;
		}

	}

	//pick the most likely end state
	vMax = -1.7E+308;
	aMax = -1;
	for (int i = 0; i < p->r; i++) {
		if (phi[length-1][i] > vMax) {
			vMax = phi[length-1][i];
			aMax = i;
		}
	}

	//now scan backwards through the pointer chain
	states[length-1] = aMax;
	for (int i = length-1; i >= 1; i--) {
		states[i-1] = bpoints[i][aMax];
		aMax = states[i-1];
	}

	return states;

}

void dumbCSVReader(string fileName, IMat &tM, int length, int width) {

	//load the csv file into the target matrix
	FILE *fp;
	int z = length;
	char line[4096];
	char * coeff;

	fp = fopen(fileName.c_str(), "r");

	//assume IMAT already has some stuff allocated
	tM.resize(z,width);
	for (int i = 0; i < z; i++) {
		fscanf(fp,"%s",line);
		coeff = strtok(line,",");
		tM.ptr[i][0] = atof(coeff);
		for (int j = 1; j < width; j++) {
			coeff = strtok(NULL,",");
			tM.ptr[i][j] = atof(coeff);
		}
	}

	fclose(fp);

}
