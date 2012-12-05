/*
 * SOM.cpp
 * Lydia Majure
 */

#include "SOM.h"

//yarp
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
//#include <yarp/sig/all.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>

//system
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

//math
#include <gsl/gsl_rng.h>

//debug
#include <assert.h>


using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

SOM::SOM(int k, int n) {
	N = n;
	K = k;
	const gsl_rng_type *T;
	gsl_rng *r;
	gsl_rng_env_setup();
	//srand(time(0));
	gsl_rng_default_seed = rand();
	T = gsl_rng_default;
	r = gsl_rng_alloc(T);
	activation = new double[k];
	//weights = new double[k][n];
	weights = new double*[k];
	for (int i = 0; i < k; i++){
		weights[i] = new double[n];
	}
	//init activations to 0
	for (int i = 0; i < k; i++){
		activation[i] = 0;
	}
	//init weights to random safe joint angles
	for (int i = 0; i < k; i++){
		weights[i][0] = -60 + -35*gsl_rng_uniform(r);
		weights[i][1] = 10 + 90*gsl_rng_uniform(r);
		//weights[i][2] = 60 - 60*gsl_rng_uniform(r);
		weights[i][2] = 10 + 90*gsl_rng_uniform(r);
	}

	gsl_rng_free(r);
	//delete T;
	//delete r;
	return;
}

SOM::~SOM() {
	delete activation;
	delete weights;
	return;
}

void SOM::update (Vector* input, double step){
	for (int i = 0; i < K; i++){
		activation[i] = 0;
		for (int j = 0; j < N; j++){
			activation[i] += weights[i][j]*(*input)(j);
		}
	}
	int winner = 0;
	for (int i = 0; i < K; i++){
		if (activation[i] > activation[winner]){
			winner = i;
		}
	}
	for (int i = 0; i < K; i++){
		for (int j = 0; j < N; j++){
			weights[i][j] += step*((*input)(j)-weights[i][j])*mapDist(i,winner);
		}
	}
}

//calculate the distance between two neurons arranged in a 1D circle
//weights from gaussian pdf w/ stddev 1 neuron distance

double SOM::mapDist (int i, int winner){
	int dista = 0; int distb = 0;
	if (i > winner){
		dista = i - winner;
	}
	else {
		dista = i + (K - winner);
	}
	if (winner > i){
		distb = winner - i;
	}
	else {
		distb = winner + (K - i);
	}
	if (i == winner){
		dista = 0; distb = 0;
	}
	int dist = 0;
	if (dista < distb){
		dist = dista;
	}
	else {
		dist = distb;
	}
	if (dist == 0){
		return 1;
	}
	if (dist == 1){
		return 0.25;
	}
	if (dist == 2){
		return 0;
	}
	if (dist > 2){
		return 0;
	}
}

void SOM::setVals (int k, int n, double val){
	weights[k][n] = val;
}

