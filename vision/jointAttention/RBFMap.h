/*
 *  RBFMap.h
 *
 * 	Logan Niehaus
 * 	5/30/11
 * 	radial basis function (gaussian) neural network class used for learning functional
 * 		maps from some input space to some output space. can take up to a four-
 * 		layered structure with a two-layered hidden region with fully connected weights
 *
 *  network size, ranges, and basis function covariances (assumes diagonal) need to be
 *  	set prior to training the network. see function descriptions for more detail.
 *
 */

#ifndef RBFMAP_H_
#define RBFMAP_H_

#define PI 3.14159

//yarp includes
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/math/Math.h>

//misc includes
#include <string>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

//namespaces
using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;


class RBFMap {

public:

	//constructor
	RBFMap(double _eta = 0.0, double _leak = 0.0);

	//data access
	Matrix getW() { return W; }
	Matrix getM() { return M; }
	Matrix getN() { return N; }
	double getEta() { return eta; }
	double getLeak() { return leak; }
	void setEta(double _eta) { eta = _eta; }
	void setLeak(double _leak) { leak = _leak; }

	//initialization
	bool setNetworkSize(const yarp::sig::Vector &_m, const yarp::sig::Vector &_n);
	bool setNeuronRanges(const yarp::sig::Vector &_mr, const yarp::sig::Vector &_nr);
	bool setActivationRadii(const yarp::sig::Vector &_sm, const yarp::sig::Vector &_sn);
	bool initFromFiles(string weightsfile, string inputsfile, string outputsfile);
	bool initNetwork();
	void resetWeights() { W.zero(); }

	//training/processing
	yarp::sig::Vector forwardActivation(const yarp::sig::Vector &in);
	yarp::sig::Vector backwardActivation(const yarp::sig::Vector &fb);
	yarp::sig::Vector evaluate(const yarp::sig::Vector &in, yarp::sig::Vector &act);
	yarp::sig::Vector trainHebb(const yarp::sig::Vector &in, const yarp::sig::Vector &fb, yarp::sig::Vector &act);

	//destructor
	~RBFMap();

protected:

	//running params
	yarp::sig::Vector m;
	yarp::sig::Vector n;
	yarp::sig::Vector mr;
	yarp::sig::Vector nr;
	double eta, leak;
	Matrix Sm, Sn;

	//data
	Matrix W, M, N;
	Matrix iSm, iSn;
	yarp::sig::Vector nmlzr;
	double sqDetSm, sqDetSn;

	//misc and auxiliary
	bool initialized;
	int tm;
	int tn;

};


#endif
