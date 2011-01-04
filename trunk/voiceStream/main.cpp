/*
 * main.cpp
 *
 *  Created on: Sep 3, 2010
 *      Author: logan
 */

#define PI 3.14159
#define DMIN 10e-8
#define ALPHA 0.1 //exp filter coeff

#include <stdio.h>
#include <sndfile.h>
#include <math.h>
#include <stdlib.h>

#include "../speech/mfcc.h"

int main() {

	//audio parameters
	int frameSize = 512;
	int overlap = 128;
	double alpha = 0.1;
	double threshold = -60;

	//declarations
	double * frame;
	double * vadSig;
	int offset;
	double energy;

	//mfcc things
	MFCCProcessor * M;

	//libsndfile stuff
	SNDFILE *sf;
	SF_INFO * sninfo = new SF_INFO;
	FILE * fp;

	//open the file
	sninfo->format = 0;
	sf = sf_open("/home/logan/workspace/scripts/logan-trainingsample3.wav", SFM_READ, sninfo);

	//read the whole sample in
	frame = new double[sninfo->frames];
	sf_readf_double(sf, frame, sninfo->frames);
	sf_close(sf);

	//load it onto the mfcc thing
	M = new MFCCProcessor(15, 30, frameSize, overlap, sninfo->samplerate, 0, sninfo->samplerate/2, true);
	M->pushData(frame, sninfo->frames);

	//do vad things
	int z = sninfo->frames/(frameSize-2*overlap);
	vadSig = new double[z];
	double * mTemp;
	fp = fopen("/home/logan/workspace/scripts/vadtest.dat","w");
	for (int i = 0; i < z; i++) {
		printf("loading sample %d\n",i+1);
		if (!M->getMFCCs(mTemp)) {
			//break;
		}
		else {
			//export the vad signal (exp filtered)
			if (i < 2) {
				vadSig[i] = mTemp[0];
			} else {
				vadSig[i] = alpha*mTemp[0] + (1-alpha)*vadSig[i-1];
			}
		}
		if (vadSig[i] > threshold) {
			fprintf(fp,"%f\n",1.0);
		} else {
			fprintf(fp,"%f\n",0.0);
		}
		delete mTemp;
	}

	fclose(fp);
	delete frame;
	delete vadSig;
	//delete M;
	return 0;

}
