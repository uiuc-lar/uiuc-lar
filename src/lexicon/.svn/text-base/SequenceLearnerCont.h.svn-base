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
#include <vector>
#include <string>

//internal libraries
#include "SequenceLearner.h"
#include "../RMLE/HMM.hh"
#include "../RMLE/StochasticClassifier.hh"
#include "../RMLE/Gaussian.hh"
#include "../RMLE/N.hh"
#include "../imatlib/IMat.hh"
#include "../imatlib/IVec.hh"
#include "../imatlib/IVecInt.hh"
#include "../imatlib/IMatVecOps.hh"

using namespace std;

class SequenceLearnerCont : public SequenceLearner
{

public:

	//constructor/destructor
	SequenceLearnerCont(int r_, int d_, int b_, int epochs_, double thresh_, double prior_, double eps_, double alpha_, double xi_, bool makeLR_ = false); //with covreg
	SequenceLearnerCont(int r_, int d_, int b_, int epochs_, double thresh_, double prior_, double eps_, bool makeLR_ = false);				//w/o covreg
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
	void printToFile(string, int);
	//void packA(Bottle &, int);
	void packObs(Bottle &, int);
	bool generateSequence(IMat &data, int n, double dscale = 1.0);
	void setScaling(int, double, IVec *);
	void scale(real **, int);

	//int nInitialized;


private:

	//model parameters
	int d;		//observation size


	//classifier data structures
	Gaussian ** obs_dist;
	IMat initData;

	//exemplar info storage (mainly for generation)
	IVec ** exemplar_initPos;
	int * exemplar_length;

	//options
	bool makeLR;		//implement pseudo-lr init
	double alpha; 		//controls covariance matrix regularization
	double xi;			//upper bound of covariance
	bool stent;
	int scm;			//scale mode
	double scf, ascf, xscf;		//single scaling coeff
	IVec scv, alvec, xivec;			//scaling vector
	double kv, kp;


};

#endif
