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
#include "../RMLE/HMM.hh"
#include "../RMLE/StochasticClassifier.hh"
#include "../RMLE/IndepPMF.hh"
#include "../imatlib/IMat.hh"
#include "../imatlib/IVec.hh"
#include "../imatlib/IMatVecOps.hh"

using namespace std;

class SequenceLearnerDisc
{

public:

	//constructor/destructor
	SequenceLearnerDisc(int, int*, int, int, int, double, bool);
	virtual ~SequenceLearnerDisc();
	void init();

	//training (with classification)
	int train(int **, int);
	int initialize(int **, int, int);

	//clasification
	int classify(int **, int);
	double evaluate(int **, int, int);

	//auxiliary
	void printAll();
	void printToFile(string);
	int nInitialized;


private:

	//model parameters
	int r;		//model size
	int b;		//bank size
	int * d;	//dictionary sizes (can handle multiple B matrices)
	int nOuts;  //number of output distributions


	//classifier data structures
	Allocator * allocator;
	HMM ** p;
	IndepPMF ** obs_dist;
	IMat initData;
	IVec ** pi; //initial probs


	//options
	double lThresh;		//learning parameter
	int epochs;			//controls movement distance
	double prior;		//gobbledygook
	double myeps;		//change the epsilon for learning
	bool makeLR;		//make this a left to right model


	//internal auxiliary functions
	void makeALR(int);
	int ProbProject(IMat *, int);

};

#endif
