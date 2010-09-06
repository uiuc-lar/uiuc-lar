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
#include <fftw3.h>
#include <stdlib.h>

int main() {

	//parameters
	int frameSize = 512;
	int mwLb = 2;
	int mwUb = 256;

	//declarations
	double * frame;
	double * thisFrame = new double[frameSize];
	double * window = new double[frameSize];
	double * melWindow = new double[frameSize/2];
	double * windowedSig = new double [frameSize/2];
	double * energySig;
	int offset;
	double energy;

	//fftw stuff
    fftw_complex *out;
    fftw_plan p;
    out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (frameSize/2));

	//libsndfile stuff
	FILE *fp;
	SNDFILE *sf;
	SF_INFO * sninfo = new SF_INFO;

	//load windows into memory
	for (int i = 0; i < frameSize; i++) {
		window[i] = 0.54 + 0.46*cos((double)(2*PI*(-(frameSize/2)+i)/frameSize));
	}
	fp = fopen("melwindow.dat","r");
	for (int i = 0; i < frameSize/2; i++)  {
		char garbage[100];
		//fscanf(fp, "%lf", melWindow+i);
		fscanf(fp, "%s", garbage);
		melWindow[i] = atof(garbage);
	}
	fclose(fp);

	//open the file
	sninfo->format = 0;
	sf = sf_open("momactionwords-full.wav", SFM_READ, sninfo);

	//read the whole sample in
	frame = new double[sninfo->frames];
	sf_readf_double(sf, frame, sninfo->frames);
	sf_close(sf);

	int n = (int)(2*sninfo->frames/frameSize); //number of frames

	energySig = new double[n];
	energySig[0] = 0;
	fp = fopen("energyout.dat","w");
	for (int i = 1; i < n; i++) {

		//fft things
		p = fftw_plan_dft_r2c_1d(frameSize, thisFrame, out, NULL);

		//read a buffer of data in and apply window
		offset = i*(frameSize/2)-(frameSize/4);
		for (int j = 0; j < frameSize; j++) {
			thisFrame[j] = frame[j+offset];
			thisFrame[j] *= window[j];
		}

		//take the fft
		fftw_execute(p);

		energy = 0;
		//apply mel window and sum
		for (int j = 0; j < frameSize/2; j++) {
			double tmp = sqrt(out[j][0]*out[j][0]+out[j][1]*out[j][1])*melWindow[j];
			if (tmp < DMIN) {
				tmp = DMIN;
			}
			energy += tmp;
		}
		energy = log(energy);


		//filter (exp) and check threshold
		energySig[i] = ALPHA*energy + (1-ALPHA)*energySig[i-1];
		fprintf(fp,"%f\n",energySig[i]);

		//cleanup
		fftw_destroy_plan(p);
	}

	fclose(fp);
	return 0;

}
