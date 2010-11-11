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


class SequenceLearner
{

public:

	//constructor/destructor
	SequenceLearner(int, int, int, int, double, double, double);
	virtual ~SequenceLearner();
	void init();

	//training (with classification)
	int train(real **, int);
	int initialize(real **, int, int);

	//clasification
	int classify(real **, int);
	double evaluate(real **, int, int);

	//auxiliary
	void printAll();

	int nInitialized;


private:

	//model parameters
	int r;		//model size
	int b;		//bank size
	int d;		//observation size


	//classifier data structures
	Allocator * allocator;
	HMM ** p;
	Gaussian ** obs_dist;
	real * likelihood;
	IMat initData;
	IVec ** pi; //initial probs


	//options
	double lThresh;		//learning parameter
	int epochs;			//controls movement distance
	double prior;		//gobbledygook
	double alpha; 		//controls covariance matrix regularization
	double xi;			//upper bound of covariance



};

#endif
