/*
 * main.cpp
 *
 *  Created on: Sep 3, 2010
 *      Author: logan
 */

#define PI 3.14159
#define DMIN 10e-8
#define ALPHA 0.1 //exp filter coeff
#define INT_MAX 32767
#define MFRAMES 400
#define SENDTHRESH 10000

#include <stdio.h>
#include <sndfile.h>
#include <math.h>
#include <fftw3.h>
#include <stdlib.h>

//yarp includes
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Sound.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;

int main() {

	//parameters
	int frameSize = 512;
	int mwLb = 2;
	int mwUb = 256;
	int bsize;

	//primary data buffer
	double * oldbuffer = new double[frameSize/2];
	double * currentbuffer = new double[frameSize/2];
	double * newbuffer =  new double[frameSize/2];

	//declarations
	double * thisFrame = new double[frameSize];
	double * window = new double[frameSize];
	double * melWindow = new double[frameSize/2];
	double * windowedSig = new double [frameSize/2];
	int offset;
	double energy;
	double oldenergy;
	double initenergy;
	bool first = true;

	//vad parameters (use the first 5s to get a noise basline)
	double thresh = 1.7e+300;
	double maxBaseline = 0.0;
	int frameCount = 0;

	//fftw stuff
    fftw_complex *out;
    fftw_plan p;
    out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (frameSize/2));
	FILE *fp;

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

    // Get a portaudio read device.
    // Open the network
    Network yarp;
    BufferedPort<Sound> myPort;
    //BufferedPort<Vector> outPort;
    Port outPort;
    outPort.open("/samples");
    myPort.open("/receiver");
    Network::connect("/grabber", "/receiver");
    Network::connect("/samples", "/classifier");

    //read the first few in to get us started (decimate by 2)
    Sound *s;
    s = myPort.read(true);
	for (int i = 0; i < s->getSamples()/2; i++) {
		oldbuffer[i] = ((double)s->getSafe(i*2,1)/(double)INT_MAX);
	}
    s = myPort.read(true);
	for (int i = 0; i < s->getSamples()/2; i++) {
		currentbuffer[i] = ((double)s->getSafe(i*2,1)/(double)INT_MAX);
	}
    s = myPort.read(true);
	for (int i = 0; i < s->getSamples()/2; i++) {
		newbuffer[i] = ((double)s->getSafe(i*2,1)/(double)INT_MAX);
	}

	//fft things
	p = fftw_plan_dft_r2c_1d(frameSize, thisFrame, out, NULL);

	fp = fopen("energyout.dat","w");
	oldenergy = 0.0;

	//prime the pump
	//Vector & data = outPort.prepare();
	Vector data;

	while (1) {

		//slide down and read a new buffer of data in
		delete oldbuffer;
		oldbuffer = currentbuffer;
		currentbuffer = newbuffer;
		newbuffer = new double[frameSize/2];
		s = myPort.read(true);

		//DECIMATION BY 2 OCCURS HERE WATCH OUT
		for (int i = 0; i < s->getSamples()/2; i++) {
			newbuffer[i] = ((double)s->getSafe(i*2,0)/(double)INT_MAX);
			//fprintf(fp,"%f\n",newbuffer[i]);
		}
		/*//fprintf(fp,"%d\n",s->getSamples());

		while (newbuffer[0] = 0.00) {
			s = myPort.read(true);
			for (int i = 0; i < s->getSamples()/2; i++) {
				newbuffer[i] = ((double)s->getSafe(i*2,0)/(double)INT_MAX);
			}
		}

		for (int i = 0; i < frameSize/2; i++) {
			fprintf(fp,"%f\n",newbuffer[i]);
		}
		fprintf(fp,"\n");*/

		//apply window
		/*for (int j = 0; j < frameSize; j++) {
			if (j < frameSize/4) {
				thisFrame[j] = oldbuffer[3*frameSize/4+j];
			}
			else if (j > frameSize/4 && j < 3*frameSize/4) {
				thisFrame[j] = currentbuffer[j-frameSize/4];
			}
			else {
				thisFrame[j] = newbuffer[j-3*frameSize/4];
			}
			fprintf(fp,"%f,",thisFrame[j]);
			thisFrame[j] *= window[j];
		}
		fprintf(fp,"\n");*/


		for (int i = 0; i < frameSize/4; i++) {
			thisFrame[i] = oldbuffer[i+frameSize/4];
		}
		for (int i = 0; i < frameSize/2; i++) {
			thisFrame[i+frameSize/4] = currentbuffer[i];
		}
		for (int i = 0; i < frameSize/4; i++) {
			thisFrame[i+3*frameSize/4] = newbuffer[i];
		}
		for (int i = 0; i < frameSize; i++) {
			fprintf(fp,"%f,",thisFrame[i]);
			thisFrame[i] *= window[i];
		}
		fprintf(fp,"\n");

		//take the fft
		fftw_execute(p);

		energy = 0;
		//apply mel window and sum
		for (int j = 0; j < frameSize/2; j++) {
			double tmp = sqrt(out[j][0]*out[j][0]+out[j][1]*out[j][1])*melWindow[j];
			if (tmp < DMIN) {
				tmp = DMIN;
			}
			//fprintf(fp,"%f,",tmp);
			energy += tmp;
		}
		//fprintf(fp,"\n");
		energy = log(energy);


		//filter (exp) and check threshold
		if(first) {
			initenergy = energy;
			first = false;
		}
		else {
			//get a baseline threshold
			if (frameCount < MFRAMES) {
				frameCount++;
				if(maxBaseline < (energy-initenergy)) {
					maxBaseline = energy-initenergy;
				}
			}
			else {

				//thresh = maxBaseline+0.05;
				thresh = 0.1;
				oldenergy = ALPHA*(energy-initenergy) + (1-ALPHA)*oldenergy;

				if (oldenergy > thresh) {
					for (int i = 0; i < frameSize/2; i++) {

						//some audio has been detected
						//add the buffer to the larger buffer of speech data
						data.push_back(currentbuffer[i]);
					}

				}
				else {
					for (int i = 0; i < frameSize/2; i++) {

						//no audio has been detected
						//check to see if the buffer has data ready to be sent
						if (data.length() != 0) {

							printf("BUFFER TRANSMITTING. LENGTH = %d\n",data.length());
							//flush buffer

							if (data.length() > SENDTHRESH) {
								outPort.write(data);
							}
							data.clear();

							//request a new output buffer (dont fill yet)
							//data = outPort.prepare();

						}
					}
				}
			}

		}

	}
	//cleanup
	fftw_destroy_plan(p);

	fclose(fp);
	return 0;

}
