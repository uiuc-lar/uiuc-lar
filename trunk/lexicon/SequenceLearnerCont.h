/*
 * SequenceLearnerCont.h
 *
 *  Created on: Sep 8, 2010
 *      Author: logan
 */

#ifndef SequenceLearnerCont_H_
#define SequenceLearnerCont_H_

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
#include "SequenceLearner.h"
#include "../RMLE/HMM.hh"
#include "../RMLE/StochasticClassifier.hh"
#include "../RMLE/Gaussian.hh"
#include "../imatlib/IMat.hh"
#include "../imatlib/IVec.hh"
#include "../imatlib/IVecInt.hh"
#include "../imatlib/IMatVecOps.hh"

using namespace std;

class SequenceLearnerCont : public SequenceLearner
{

public:

	//constructor/destructor
	SequenceLearnerCont(int, int, int, int, double, double, double,double,double); //with covreg
	SequenceLearnerCont(int, int, int, int, double,double,double);				//w/o covreg
	virtual ~SequenceLearnerCont();
	void init();

	//training (with classification)
	int train(real **, int);
	int initialize(real **, int, int);

	//clasification
	int classify(real **, int);
	double evaluate(real **, int, int);

	//auxiliary
	bool getType() { return true; }	//identifies as continuous
	void printAll();
	void printToFile(string);
	//void packA(Bottle &, int);
	void packObs(Bottle &, int);

	//int nInitialized;


private:

	//model parameters
	int d;		//observation size


	//classifier data structures
	Gaussian ** obs_dist;
	IMat initData;


	//options
	double alpha; 		//controls covariance matrix regularization
	double xi;			//upper bound of covariance
	bool stent;

};

#endif
