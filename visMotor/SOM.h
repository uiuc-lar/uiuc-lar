/*
 * SOM.h
 * Lydia Majure
 */

#ifndef SOM_H_
#define SOM_H_

#include <yarp/sig/all.h>

using namespace yarp::sig;

class SOM {
private:
 double *activation; //1D array of activations
 //double **weights; //2D array of weights
 int N; //number of joint angles learned
 int K; //number of neurons in map

public:
 SOM(int n,int k);
 ~SOM();
 double **weights; //2D array of weights
 int getState();
 void update(Vector* input,double step);
 double mapDist(int i,int winner);
};

#endif /* SOM_H_ */
