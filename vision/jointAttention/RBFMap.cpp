/*
 *  RBFMap.cpp
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

#include "RBFMap.h"

//namespaces
using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;

//constructors
RBFMap::RBFMap(double _eta, double _leak) : eta(_eta), leak(_leak), initialized(false) {

	m.resize(0);
	n.resize(0);
	mr.resize(0);
	nr.resize(0);
	Sm.resize(0,0);
	Sn.resize(0,0);

	W.resize(0,0);
	M.resize(0,0);
	N.resize(0,0);

}

RBFMap::~RBFMap() {


}

/*
 * initFromFiles(string weightsfile, string inputsfile, string outputsfile)
 *
 * 	loads the matrices W, M, and N from tab delimited files. must have all three
 * 	to function.
 *
 */
bool RBFMap::initFromFiles(string weightsfile, string inputsfile, string outputsfile) {

	ImageOf<PixelFloat> Wimg;
	ImageOf<PixelFloat> Isimg;
	ImageOf<PixelFloat> Osimg;

	//load neural net data
	if (file::read(Wimg, weightsfile.c_str())) {
		W.resize(Wimg.height(),Wimg.width());
		for (int i = 0; i < Wimg.width(); i++) {
			for (int j = 0; j < Wimg.height(); j++) {
				W(j,i) = Wimg.pixel(i,j);
			}
		}
	} else {

		fprintf(stderr,"could not read weight file, failing...\n");
		return false;

	}

	if (file::read(Isimg, inputsfile.c_str())) {
		M.resize(Isimg.height(),Isimg.width());
		for (int i = 0; i < Isimg.width(); i++) {
			for (int j = 0; j < Isimg.height(); j++) {
				M(j,i) = Isimg.pixel(i,j);
			}
		}
	} else {

		fprintf(stderr,"could not read inputs file, failing...\n");
		return false;

	}

	if (file::read(Osimg, outputsfile.c_str())) {
		N.resize(Osimg.height(),Osimg.width());
		nmlzr.resize(Osimg.height());
		for (int i = 0; i < Osimg.width(); i++) {
			for (int j = 0; j < Osimg.height(); j++) {
				N(j,i) = Osimg.pixel(i,j);
				nmlzr(j) = 1;
			}
		}

	} else {

		fprintf(stderr,"could not read outputs file, failing...\n");
		return false;

	}

	m.resize(0);
	n.resize(0);
	mr.resize(0);
	nr.resize(0);
	Sm.resize(0,0);
	Sn.resize(0,0);

	return true;

}

/*
 * setNetworkSize(const yarp::sig::Vector &_m, const yarp::sig::Vector &_n)
 *
 *	sets the number of input and output neurons. RBFNNs can be thought of as a mapping
 *	from R^i to R^j. therefore m should be a vector of length i, with each element defining
 *	the number of neurons that should span the specified range of that given dimension
 *
 */
bool RBFMap::setNetworkSize(const yarp::sig::Vector &_m, const yarp::sig::Vector &_n) {

	if (_m.size() > 0 && _n.size() > 0) {

		m = _m;
		n = _n;
		tm = tn = 1;
		for (int i = 0; i < m.size(); i++) {
			tm *= m[i];
		}
		for (int j = 0; j < n.size(); j++) {
			tn *= n[j];
		}
		return true;

	} else {

		fprintf(stderr,"error: invalid network size specified\n");
		return false;

	}

}

/*
 * setNeuronRanges(const yarp::sig::Vector &_mr, const yarp::sig::Vector &_nr)
 *
 *	sets the centers for each of the radial basis functions. mr should be a vector of the form
 *	[m1_low m1_high m2_low m2_high ... ], with twice as many elements as the vector m above.
 *	centers are placed uniformly in a grid covering the specified input/output ranges
 *
 */
bool RBFMap::setNeuronRanges(const yarp::sig::Vector &_mr, const yarp::sig::Vector &_nr) {

	if (_mr.size() != 2*m.size() || _nr.size() != 2*n.size()) {

		fprintf(stderr,"error: network ranges provided don't match expected input/output space dimensions\n");
		return false;

	} else {

		mr = _mr;
		nr = _nr;

	}

	return true;

}

/*
 * setActivationRadii(const yarp::sig::Vector &_sm, const yarp::sig::Vector &_sn)
 *
 *	sets the variance of the gaussian radial basis functions along each dimension of the
 *	input/output spaces. assuming diagonal covariance, sm will be a vector of length equal
 *	to that of m, and defines the diagonal entries of the input cov. mat.
 *
 */
bool RBFMap::setActivationRadii(const yarp::sig::Vector &_sm, const yarp::sig::Vector &_sn) {

	if (_sm.size() != m.size() || _sn.size() != n.size()) {

		fprintf(stderr,"error: number of basis function radii provided does not match expected input/output space dimensions\n");
		return false;

	}
	else {

		Sm.resize(_sm.size(),_sm.size());
		Sm.zero();
		Sm.diagonal(_sm);
		Sn.resize(_sn.size(),_sn.size());
		Sn.zero();
		Sn.diagonal(_sn);

		//pre-calculate inverses
		iSm = luinv(Sm);
		iSn = luinv(Sn);
		sqDetSm = sqrt(det(Sm));
		sqDetSn = sqrt(det(Sn));

	}

	return true;

}

/*
 * initNetwork()
 *
 *	call only after the three above functions have been used to specify the networks
 *	parameters, and after the learning and leakage rates have been set. also if saved
 *	data is to be loaded, it should be called before this.
 *
 */
bool RBFMap::initNetwork() {

	bool tinit = true;

	//check to see if all sizes/ranges/variances have been set
	tinit &= (m.size() != 0 && n.size() != 0);
	tinit &= (mr.size() != 0 && nr.size() != 0);
	tinit &= (Sm.rows() != 0 && Sm.cols() != 0 && Sn.rows() != 0 && Sn.cols() != 0);

	if (!tinit) {

		fprintf(stderr,"error: Please set input/output dimensions, ranges, and activation variances before initializing\n");
		return false;

	}

	//if all pre-requisites check out and no previous data was thrown in then initialize
	if (W.rows()*W.cols() == 0 || M.rows()*M.cols() == 0 || N.rows()*N.cols() == 0) {

		double tmp;
		int counter, rem;

		//init weight matrix to zero
		W.resize(tn,tm);
		W.zero();

		//set input neuron center locations
		M.resize(tm,2);
		rem = tm;
		for (int i = 0; i < m.size(); i++) {
			rem /= m[i];
			counter = 0;
			for (int j = 0; j < tm/(rem*m[i]); j++) {
				for (int k = 0; k < m[i]; k++) {
					tmp = k*((mr[2*i+1]-mr[2*i])/(double)(m[i]-1)) + mr[2*i];
					for (int l = 0; l < rem; l++) {
						M(counter, i) = tmp;
						counter++;
					}
				}
			}
		}

		//set output neuron center locations
		N.resize(tn,2);
		nmlzr.resize(tn);
		rem = tn;
		for (int i = 0; i < n.size(); i++) {
			rem /= n[i];
			counter = 0;
			for (int j = 0; j < tn/(rem*n[i]); j++) {
				for (int k = 0; k < n[i]; k++) {
					tmp = k*((nr[2*i+1]-nr[2*i])/(double)(n[i]-1)) + nr[2*i];
					for (int l = 0; l < rem; l++) {
						N(counter, i) = tmp;
						nmlzr(counter) = 1.0;
						counter++;
					}
				}
			}
		}
	}

	//if pre-loaded, check for sanity with rest of loaded params
	else {
		if (N.rows() != tn || M.rows() != tm) {
			fprintf(stderr,"loaded weights do not match other layout parameters, failing...\n");
			return false;
		}
	}

	initialized = true;

	if (eta <= 0.0 || leak <= 0.0) {

		fprintf(stderr,"warning: eta and leak have not been properly set\n");

	}

	return initialized;

}

/*
 * forwardActivation(const yarp::sig::Vector &in)
 *
 *	return the activation of each input neuron (first 'hidden layer') for a given
 *	input. returned vector has size m1*m2*m3*...mi. indices of the vector are given
 *	in ascending order by a linear, binary-encoding style lookup table, where m1
 *	is the most significant digit and mi is the least significant. vector is
 *	not normalized.
 *
 */
yarp::sig::Vector RBFMap::forwardActivation(const yarp::sig::Vector &in) {

	yarp::sig::Vector Af(0);

	//validity checks
	if (in.size() != m.size() || !initialized) {

		fprintf(stderr, "can not evaluate forward activation, please give proper size input, or reinitialize\n");

	}
	else {

		//get activations
		Af.resize(M.rows());
		yarp::sig::Vector mc(m.size());
		for (int i = 0; i < M.rows(); i++) {
			mc = in-M.getRow(i);
			Af[i] = (1/(sqDetSm*2*PI))*exp(-0.5*dot(mc,(iSm*mc)));
		}

	}

	return Af;

}

/*
 * backwardActivation(const yarp::sig::Vector &fb)
 *
 *	return the feedback activation of each output neuron (second 'hidden layer')
 *	for a given output. returned vector has size n1*n2*n3*...nj. neuron center
 *	locations can be referenced as before. not normalized.
 *
 */
yarp::sig::Vector RBFMap::backwardActivation(const yarp::sig::Vector &fb) {

	yarp::sig::Vector Ar(0);

	//validity checks
	if (fb.size() != n.size() || !initialized) {

		fprintf(stderr, "can not evaluate reverse activation, please give proper size input, or reinitialize\n");

	}
	else {

		//get activations
		Ar.resize(N.rows());
		yarp::sig::Vector mc(n.size());
		for (int i = 0; i < M.rows(); i++) {
			mc = fb-N.getRow(i);
			Ar[i] = (1/(sqDetSn*2*PI))*exp(-0.5*dot(mc,(iSn*mc)));
		}

	}

	return Ar;


}

/*
 * evaluate(const yarp::sig::Vector &in, yarp::sig::Vector &act)
 *
 *	evaluate the partial and complete feed-forward activations given an
 *	input from R_i. Returns a vector in R_j. Raw activation scores of each
 *	output neuron are normalized and placed in 'act'. These are calculated
 *  from multiplying the input activations above by the weight matrix. The
 *  output activations are then multiplied by the neuron center and summed
 *  to produce the predicted mapping into R_j.
 *
 */
yarp::sig::Vector RBFMap::evaluate(const yarp::sig::Vector &in, yarp::sig::Vector &act) {

	yarp::sig::Vector Ap;
	yarp::sig::Vector Ax;
	yarp::sig::Vector out(n.size());
	double nc;

	//get forward activation
	Ap = forwardActivation(in);

	//get second hidden layer activation
	Ax = W*Ap;

	//normalize
	nc = 1.0/dot(Ax,nmlzr);
	Ax = nc*Ax;
	act = Ax;

	//find mapped point
	out = N.transposed()*Ax;

	return out;

}

/*
 * trainHebb(const yarp::sig::Vector &in, const yarp::sig::Vector &fb, yarp::sig::Vector &act)
 *
 *	train the neural network using hebb's rule with matching samples from R_i and R_j.
 *	the tensor product of forward and backward neuron activations are weighted by the
 *	learning rate, and added to the current weight matrix. a leakage factor is
 *	also applied to the current weight matrix esimate in order to forget very old training
 *	points.
 *
 */
yarp::sig::Vector RBFMap::trainHebb(const yarp::sig::Vector &in, const yarp::sig::Vector &fb, yarp::sig::Vector &act) {

	yarp::sig::Vector Ap;
	yarp::sig::Vector Ax;
	Matrix Apm, Axm;
	Matrix Wh;
	double wsum = 0.0;

	//get forward activation
	Ap = forwardActivation(in);
	Apm.resize(1,Ap.size());
	Apm.setRow(0, Ap);

	//get feedback activation
	Ax = backwardActivation(fb);
	Axm.resize(Ax.size(),1);
	Axm.setCol(0, Ax);

	//take outer product of forward and backward activations, normalize
	Wh = Axm*Apm;
	for (int i = 0; i < Wh.rows(); i++) {
		for (int j = 0; j < Wh.cols(); j++) {
			wsum += Wh(i,j);
		}
	}
	Wh = (1.0/wsum)*Wh;

	//update weight matrix with hebb rule
	W = eta*Wh + leak*W;

	//return the feedforward evaluation for the updated W
	return evaluate(in, act);

}
