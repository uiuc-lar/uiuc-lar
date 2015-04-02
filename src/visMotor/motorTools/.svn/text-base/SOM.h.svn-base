/*
 * SOM.h
 * Lydia Majure
 */

#ifndef SOM_H_
#define SOM_H_

#include <yarp/sig/all.h>

using namespace yarp::sig;

class SOM {

public:
 SOM(int k,int n);
 ~SOM();
 int N; //number of joint angles learned
 int K; //number of neurons in map
 double *activation; //1D array of activations
 double **weights; //2D array of weights
 int getState();
 void update(Vector* input,double step);
 double mapDist(int i,int winner);
 void setVals(int k, int n, double val);
};

#endif /* SOM_H_ */
