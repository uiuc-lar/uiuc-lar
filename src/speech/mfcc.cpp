//ultra simple MFCC routine based in FFTW3
//based on method of Davis and Mermelstein (1980)

#include "mfcc.h"


MFCCProcessor::MFCCProcessor(int c_, int m_, int n_, int overlap_, double fs_, double fl_, double fh_, bool getZeroth)
	: m(m_), n(n_), c(c_), fs(fs_), fl(fl_), fh(fh_), overlap(overlap_), z(getZeroth) {

	//set up processing
	init();

}

MFCCProcessor::~MFCCProcessor() {

	//destroy fftw things
	fftw_destroy_plan(p);
	fftw_destroy_plan(q);

	fftw_free(out);
	delete tmfcc;

	//clear out data
	data.clear();

	delete frame;
	delete logEnergy;

	//clear out window
	delete window;

	//clear out banks
	for (int i = 0; i < m; i++) {
		delete banks[i];
	}
	delete banks;

}

void MFCCProcessor::init() {

	//get a queue and then flush it
	data.clear();
	flushPipeline();

	//fill in the banks
	banks = new double * [m];
	for (int i = 0; i < m; i++) {
		banks[i] = new double[n/2];
	}

	fillBanks();

	//set up windowing function (hamming)
	window = new double[n];
	for (int i = 0; i < n; i++) {
		window[i] = 0.54 + 0.46*cos((double)(2*PI*(-(n/2)+i)/n));
	}

	//grab space for processing
	//frame = new double[n];
	//logEnergy = new double[m];
	frame = (double*) fftw_malloc(sizeof(double) * n);
	logEnergy = (double*) fftw_malloc(sizeof(double) * m);

	//set up fft/dct stuff
    out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (n/2)+1);
    //tmfcc = new double[m];
    tmfcc = (double*) fftw_malloc(sizeof(double) * m);
    p = fftw_plan_dft_r2c_1d(n, frame, out, NULL);
    q = fftw_plan_r2r_1d(m, logEnergy, tmfcc, FFTW_REDFT10, NULL);

}

void MFCCProcessor::fillBanks() {

	double * cf;

	//calculate the center frequencies
	cf = new double[m+2];
	for (int i = 0; i < m+2; i++) {
		cf[i] = (n/fs)*700*(exp((i*2595.0*log10(1+fs/1400.0)/(m+1))/1127.0)-1);
		cf[i] = (int)cf[i];
	}

	//fill in each bank
	for (int i = 1; i < m+1; i++) {
		for (int j = 0; j < n/2; j++) {
			if (j < (int)cf[i-1]) {
				banks[i-1][j] = 0;
			}
			else if (j >= (int)cf[i-1] && j < (int)cf[i]) {
				banks[i-1][j] = (double)(j-cf[i-1])/(cf[i]-cf[i-1]);
			}
			else if (j >= (int)cf[i] && j < (int)cf[i+1]) {
				banks[i-1][j] = (double)(cf[i+1]-j)/(cf[i+1]-cf[i]);
			}
			else {
				banks[i-1][j] = 0;
			}
		}
	}

	delete cf;

}

bool MFCCProcessor::pushData(double * incData, int l) {


	for (int i = 0; i < l; i++) {
		data.push_back(incData[i]);
	}

	return true;

}

void MFCCProcessor::flushPipeline() {

	//clear out the queue
	data.clear();

	/*
	//prime it with some zeros
	for (int i = 0; i < overlap; i++) {
		data.push_back(0.0);
	}
	*/

}


int MFCCProcessor::queuedSamples() {

	return data.size();

}

void MFCCProcessor::zeroPad(int n, int side) {

	//side <= 0 -> pad front of queue
	//side > 0	-> pad back of queue

	if (side > 0) {
		for (int i = 0; i < overlap; i++) {
			data.push_back(0.0);
		}
	} else {
		for (int i = 0; i < overlap; i++) {
			data.push_front(0.0);
		}
	}

}

/* getMFCCs - calculate the MFCCs
 *
 * input parameters:
 *
 * 		mfccs - list of cepstral coeffs (to be filled)
 *
 */
bool MFCCProcessor::getMFCCs(double *& mfccs) {

	//check for a frame
	//if (data.empty()) {
	if (data.size() < n) {

		return false;

	}
	else {

		mfccs = new double[c];

		copy(data.begin(),data.begin()+n,frame);
		for (int i = 0; i < n; i++) {
			frame[i] *= window[i];
		}

		//take dft
		fftw_execute(p);

		//project abs onto banks
		for (int i = 0; i < m; i++) {
			logEnergy[i] = 0.0;
			for (int j = 0; j < n/2; j++) {
				logEnergy[i] += sqrt(out[j][0]*out[j][0]+out[j][1]*out[j][1])*banks[i][j];
			}
			//take log
			logEnergy[i] = log10(logEnergy[i]);
		}

		//take DCT
		fftw_execute(q);

		//loadout
		for (int i = 0; i < c; i++) {
			if (z) {
				mfccs[i] = tmfcc[i];
			} else {
				mfccs[i] = tmfcc[i+1];
			}
		}

		//de-queue a 'frame' (frameSize = n-2*overlap)
		for (int i = 0; i < (n-overlap*2); i++) {
			data.pop_front();
		}

	}

	return true;

}
