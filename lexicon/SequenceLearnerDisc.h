/*
 * SequenceLearnerDisc.h
 *
 *  Created on: Nov 29, 2010 (from SequenceLearner.h)
 *      Author: logan
 */

#ifndef SEQUENCELEARNERDISC_H_
#define SEQUENCELEARNERDISC_H_

//external libraries
#include <torch/general.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_rng.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <sndfile.h>
#include <vector>
#include <string>
#include <algorithm>

//internal libraries
#include "SequenceLearner.h"
#include "../RMLE/HMM.hh"
#include "../RMLE/StochasticClassifier.hh"
#include "../RMLE/IndepPMF.hh"
#include "../imatlib/IMat.hh"
#include "../imatlib/IVec.hh"
#include "../imatlib/IMatVecOps.hh"

using namespace std;

class SequenceLearnerDisc : public SequenceLearner
{

public:

	//constructor/destructor
	SequenceLearnerDisc(int, int*, int, int, int, double, bool,double,double);
	virtual ~SequenceLearnerDisc();
	void init();

	//training (with classification)
	int train(int **, int);
	int initialize(int **, int, int);

	//clasification
	int classify(int **, int);
	double evaluate(int **, int, int);

	//auxiliary
	bool getType() { return false; }	//identifies as discrete
	void printAll();
	void printToFile(string);
	//void packA(Bottle &, int);
	void packObs(Bottle &, int);

	//int nInitialized;


private:

	//model parameters
	int * d;	//dictionary sizes (can handle multiple B matrices)
	int nOuts;  //number of output distributions


	//classifier data structures
	IndepPMF ** obs_dist;
	IMat initData;

	//options
	bool makeLR;		//make this a left to right model


	//internal auxiliary functions
	void makeALR(int);
	int ProbProject(IMat *, int);

};

#endif
