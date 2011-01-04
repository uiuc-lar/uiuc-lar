/*
 *  lexiconLearner.cpp
 *
 * 	Logan Niehaus
 * 	12/21/10
 *
 *  YARP Module implementation of lexicon learning algorithm.
 *  A technical overview of the algorithm can be read in my master's
 *  thesis. This module is written to be as general as possible,
 *  with most details filled in by specific parameters.
 *
 * Module Args:
 * 	argname -- description (Required/Optional, Default DefaultValue)

	[general model parameters]
	d	-- obs dimens.	(R)
	r	--  state space size	(R)
	b	--  max num of lexical elements	(R)
	mode	--  flag for input type (0 - cont (D), 1 - discrete)
	thresh	--  novelty threshold (R)
	epochs	--  number of epochs to train (O,D 20)
	prior	--  min. value for pmfs (O,D 0.0001)
	eps		--  learning rate (O,D 0.001)
	decay	--  learning rate decay (O, D 1.0 (no decay))

	[parameters for continuous dists]
	alpha	--  cov eigenvalue minimum (O)
	xi		--  cov eigenvalue maximum (O)
	constrain	--  flag if neither of these get set

	[parameters for discrete dists]
	lr	--  left-to-right model flag (O)

	[module parameters]
	input	-- input port name (O, D /lexIn)
	output	-- output port name (O, D /lexOut)

 *
 * PORTS:
 *	Inputs: /pc/words   	(Bottle of bottles/ints. Corresponding to continuous or discrete sequences)
 *	Outputs: /lex/class		(Bottle of with one int, corresponding to classification of the sequence)
 */

//yarp network
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Value.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

//internal libraries
#include "../lexicon/SequenceLearner.h"
#include "../lexicon/SequenceLearnerCont.h"
#include "../lexicon/SequenceLearnerDisc.h"


//misc
#include <string>
#include <math.h>
#include <deque>

//namespaces
using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


//wrapper thread for the learning algorithm
class LexiconThread : public Thread {

protected:

	/* actual data: ports, objects, etc... */
	SequenceLearner * S;
	BufferedPort<Bottle> * inPort;
	Port *outPort;

	string recvPort;
	string sendPort;

public:


	LexiconThread(SequenceLearner *& S_, string recvPort_, string sendPort_)
	: S(S_), recvPort(recvPort_), sendPort(sendPort_) {

		//and the ports
		inPort = new BufferedPort<Bottle>;
		outPort = new Port;

	}

	virtual bool threadInit() {

		inPort->open(recvPort.c_str());
		outPort->open(sendPort.c_str());
		return true;

	}

	virtual void threadRelease() {

		inPort->close();
		outPort->close();

		S->printAll();

	}

	virtual void run() {

		while (isStopping() != true) {


			int lex;
			double val;
			//read a bottle off the port
			Bottle * b;

			b = inPort->read(false);	//dont block

			if (b != NULL) {

				printf("sequence received,... ");

				if (S->getType()) {

					//unpack
					real ** samplesC;
					samplesC = new real * [b->size()];
					for (int i = 0; i < b->size(); i++) {
						Bottle * c = b->get(i).asList();
						samplesC[i] = new real[c->size()];
						for (int j = 0; j < c->size(); j++) {
							samplesC[i][j] = 20*c->get(j).asDouble();
						}
					}

					//train or classify
					lex = S->classify(samplesC, b->size());
					if (lex != -1)
						val = S->evaluate(samplesC, b->size(),lex);
					else
						val = -1.0e+300;
					lex = S->train(samplesC, b->size());

				} else {

					//unpack
					int ** samplesD;
					samplesD = new int * [b->size()];
					//can only handle vectors for now
					for (int i = 0; i < b->size(); i++) {
						samplesD[i] = new int;
						samplesD[i][0] = b->get(i).asInt();
					}

					//train or classify
					lex = S->classify(samplesD, b->size());
					if (lex != -1)
						val = S->evaluate(samplesD, b->size(),lex);
					else
						val = -1.0e+300;
					lex = S->train(samplesD, b->size());

				}


				printf("classified as %d\t log likelihood: %f\n",lex,val);

				//bottle up the result, pass along timestamps, and publish
				Bottle result;
				result.add(lex);

				Bottle tStamps;
				inPort->getEnvelope(tStamps);
				outPort->setEnvelope(tStamps);

				outPort->write(result);

			}
		}
	}

};

//module for handling inputs and objects
class LexiconModule : public RFModule {

protected:

	/* list of potential parameters for lexicon learner.
	 * some of these are requried (R), some are optional (O).
	 * defaults marked with (D)
	 */
	//model sizes
	int d;	//obs dimens.	(R)
	int r;	//state space size	(R)
	int b;	//max num of lexical elements	(R)

	//continuous or discrete
	bool mode;		//flag for input type (0 - cont (D), 1 - discrete)

	//gen. learning parameters
	double thresh;	//novelty threshold (R)
	int epochs;		//number of epochs to train (O,D 20)
	double prior;	//min. value for pmfs (O,D 0.0001)
	double eps;		//learning rate (O,D 0.001)
	double decay;	//learning rate decay (O, D 1.0 (no decay))

	//parameters for continuous dists
	double alpha;	//cov eigenvalue minimum (O)
	double xi;		//cov eigenvalue maximum (O)
	bool constrain;	//flag if neither of these get set

	//parameters for discrete dists
	bool lr;		//left-to-right model flag (O)

	/* actual data: ports, objects, etc... */
	SequenceLearner * S;
	string recvPort;
	string sendPort;
	LexiconThread * L;


public:

	bool loadParams(ResourceFinder &rf) {

		//get port names
		recvPort = rf.check("input",Value("/lexIn"),"input port name").asString();
		sendPort = rf.check("output",Value("/lexOut"),"output port name").asString();

		//get required params
		Value dV= rf.find("d");
		Value rV = rf.find("r");
		Value bV = rf.find("b");
		Value threshV = rf.find("threshold");
		if (dV.isNull() || rV.isNull() || bV.isNull() || threshV.isNull()) {
			printf("Input size, HMM size, Lexicon size and threshold MUST be specified\n");
			return false;
		} else {
			d = dV.asInt();
			b = bV.asInt();
			r = rV.asInt();
			thresh = threshV.asDouble();
		}

		//get either disc or cont
		if (rf.check("discrete") && rf.check("continuous")) {
			printf("Both continuous and discrete specified, please choose only one\n");
			return false;
		}
		else if (rf.check("discrete") && !rf.check("continuous")) {
			mode = false;
		}
		else if (!rf.check("discrete") && rf.check("continuous")) {
			mode = true;
		}
		else {
			printf("Please specific either discrete or continuous observations");
			return false;
		}

		//gather some params based on mode
		if (mode) {
			//continuous
			Value alphaV = rf.find("alpha");
			Value xiV = rf.find("xi");
			if (alphaV.isNull() && xiV.isNull()) {
				constrain = false;
			} else {
				constrain = true;
				alpha = rf.check("alpha",Value(0.0),"cov. lower bound").asDouble();
				xi = rf.check("xi",Value(1.0e+300),"cov. upper bound").asDouble();
			}
		} else {
			//discrete
			lr = rf.check("lefttoright");
		}

		epochs = rf.check("epochs",Value(20),"number of training epochs").asInt();
		prior = rf.check("prior",Value(0.0001),"min values for parameters").asDouble();
		eps = rf.check("eps",Value(0.001),"learning rate").asDouble();
		decay = rf.check("decay",Value(1.0),"learning rate decay value (0.0-1.0)").asDouble();

		return true;

	}

	virtual bool configure(ResourceFinder &rf) {

		//load the model parameters in
		if (!loadParams(rf)) {
			return false;
		}

		//set up the model
		if (mode) {
			//continuous
			if (constrain) {
				S = new SequenceLearnerCont(r, d, b, epochs, thresh, alpha, xi, prior, eps);
				S->eps_decay = decay;
			} else {
				S = new SequenceLearnerCont(r, d, b, epochs, thresh, prior, eps);
				S->eps_decay = decay;
			}
		} else {
			//discrete (fixed at one output obs for disc at this time
			S = new SequenceLearnerDisc(r, &d, 1, b, epochs, thresh, lr, prior, eps);
			S->eps_decay = decay;
		}


		//pass everything off to the execution thread
		L = new LexiconThread(S, recvPort, sendPort);

		return L->start();

	}

	virtual bool close() {

		L->stop();
		delete L;
		delete S;

		return true;

	}

	virtual double getPeriod() { return 1.0; }
	virtual bool updateModule() { return true; }

};


int main(int argc, char *argv[])
{
	// we need to initialize the drivers list
	Network yarp;
	if (!yarp.checkNetwork())
		return -1;

	LexiconModule mod;

	ResourceFinder rf;

	rf.configure("ICUB_ROOT", argc, argv);

	return mod.runModule(rf);
}


