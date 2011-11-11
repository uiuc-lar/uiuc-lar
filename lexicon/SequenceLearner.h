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

//yarp libs
#include <yarp/os/Bottle.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;

class SequenceLearner
{

public:

    SequenceLearner(int r_, int b_, int epochs_, double thresh_, double prior_, double eps_)
    :r(r_), b(b_), epochs(epochs_), lThresh(thresh_), prior(prior_), eps(eps_)
    {
        eps_decay = 1.0;
    }

    //initialization
    virtual ~SequenceLearner();
    virtual void init();

    //training
    virtual int train(int**, int) { return -1; }
    virtual int train(real**, int) { return -1; }
    virtual int initialize(real**, int, int) { return -1; }
    virtual int classify(real**, int) { return -1; }
    virtual double evaluate(real**, int, int) { return -1.0; }
    virtual int initialize(int**, int, int) { return -1; }
    virtual int classify(int**, int) { return -1; }
    virtual double evaluate(int**, int, int) { return -1.0; }

    //auxiliary
    virtual bool getType() { return true; }
    virtual void printAll();
    virtual void printToFile(string);
    virtual void printToFile(string, int);
    virtual void packA(Bottle& , int);
    virtual void packObs(Bottle& , int);
    virtual void packPi(Bottle& , int);
    virtual bool generateSequence(IMat &data, int n, double dscale = 1.0);
    virtual bool generateSequence(IVecInt &data, int length, int n);

    //getters/setters
    int getEpochs() const { return epochs; }
    double getEps() const { return eps; }
    double getEps_decay() const { return eps_decay; }
    double getPrior() const  { return prior; }
    double getThresh() const { return lThresh; }
    void setEpochs(int epochs) { this->epochs = epochs; }
    void setEps(double eps) { this->eps = eps; }
    void setEps_decay(double eps_decay) { this->eps_decay = eps_decay; }
    void setPrior(double prior) { this->prior = prior; }
    void setThresh(double lThresh) { this->lThresh = lThresh; }

    int nInitialized;
    double eps_decay;
    int r;
    int b;
	bool upobs;

protected:

    Allocator *allocator;
    HMM **p;
    IVec **pi;
    IVec *** cvex;
    double lThresh;
    int epochs;
    double prior;
    double eps;
};

#endif
