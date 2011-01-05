#ifndef MFCC_H_
#define MFCC_H_

//libraries
#include <stdio.h>
#include <math.h>
#include <fftw3.h>
#include <stdlib.h>
#include <deque>
#include <algorithm>
//#include <vector>


//misc defines
#define PI 3.14159

using namespace std;

class MFCCProcessor  {

public:

	//constructor/destructor
	MFCCProcessor(int, int, int, int, double, double, double, bool);
	virtual ~MFCCProcessor();

	//logistics
	bool pushData(double *, int);
	void flushPipeline();
	int queuedSamples();
	void zeroPad(int, int);

	//processing
	bool getMFCCs(double *&);


private:

	//auxiliary functions
	void init();
	void fillBanks();

	//parameters
	int m;			//number of filterbanks to use
	int n;			//frame size
	int c;			//number of coefficients returned
	bool z;			//flag for returning the 0th mfcc
	double fs;		//sampling freq
	double fl;		//filter bank low freq
	double fh;		//filter bank high freq
	int overlap;	//num of samples to take from prev and next buff

	//banks
	double ** banks;
	double * window;

	//data queue
	deque<double> data;
	//vector<double> data;
	double * frame;
	double * logEnergy;

	//fftw/dct data holders
    fftw_complex *out;
	double * tmfcc;
    fftw_plan p;
    fftw_plan q;


};

#endif
