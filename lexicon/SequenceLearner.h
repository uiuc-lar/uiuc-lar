/*
 * SequenceLearner.h
 *
 *  Created on: Sep 8, 2010
 *      Author: logan
 */

#ifndef SEQUENCELEARNER_H_
#define SEQUENCELEARNER_H_

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
#include "../imatlib/IMat.hh"
#include "../imatlib/IVec.hh"
#include "../imatlib/IVecInt.hh"
#include "../imatlib/IMatVecOps.hh"

using namespace std;

class SequenceLearner
{

public:

	//constructor/destructor
	SequenceLearner(int r_, int b_, int epochs_, double thresh_, double prior_, double eps_)
	: r(r_), b(b_), epochs(epochs_), lThresh(thresh_), prior(prior_), eps(eps_) {

		eps_decay = 1.0;

	}

	virtual ~SequenceLearner();

	virtual void init();

	//training (with classification)
	virtual int train(int **, int) { return -1; }
	virtual int train(real **, int) { return -1; }

	//clasification
	virtual int initialize(real **, int, int) { return -1; }
	virtual int classify(real **, int) { return -1; }
	virtual double evaluate(real **, int, int) { return -1.0; }
	virtual int initialize(int **, int, int) { return -1; }
	virtual int classify(int **, int) { return -1; }
	virtual double evaluate(int **, int, int) { return -1.0; }

	//auxiliary
	virtual bool getType() { return true; }
	virtual void printAll();
	virtual void printToFile();

	//public members (so they can be set from outside)
	int nInitialized;
	double eps_decay;


protected:

	//model parameters
	int r;		//model size
	int b;		//bank size

	//classifier data structures
	Allocator * allocator;
	HMM ** p;
	IVec ** pi; //initial probs

	//options
	double lThresh;		//learning parameter
	int epochs;			//controls movement distance
	double prior;		//thing
	double eps;			//learning rate

};

#endif
