/*
 * Copyright (C) 2011 Logan Niehaus
 *
 * 	Author: Logan Niehaus
 * 	Email:  niehaula@gmail.com
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
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
	updateobs	--  0 to turn of observation dist updating after init (D 1)
	dt		--  sampling rate, used for scaling derivs in generation (D 0.01)
	scale	--  scaling coeff to apply to each dimension. can be single or vector valued

	[parameters for discrete dists]
	lefttoright	--  left-to-right model flag (O)

	[module parameters]
	input	-- input port name (O, D /lex:i)
	output	-- output port name (O, D /lex:o)
	greq	-- input port for generation commands (O, D /lex:r)
	gen		-- generated element output port (O, D /lex:g)
	rpc		-- rpc port name (O, D /lex/rpc)

 *
 * PORTS:
 *	Inputs: /pc/words   	(Bottle of bottles/ints. Corresponding to continuous or discrete sequences)
 *	Inputs: /assocMem/symb  (Bottle w/ single int. Tells the module to generate a sequence for a given model)
 *	Outputs: /lex/class		(Bottle of with one int, corresponding to classification of the sequence)
 *	Outputs: /lex/gen		(Bottle of either ints or more bottles. generated sample from HMM encoding)
 *	RPC: /lex/rpc			(RPC communication port to interact with the running module)
 */

//yarp network
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/RpcServer.h>
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

//csv functions
void dumbCSVReader(const char *, IMat &, int, int);
void dumbCSVWriter(const char *, IMat &);

//wrapper thread for the learning algorithm
class LexiconThread : public Thread {

protected:

	/* actual data: ports, objects, etc... */
	SequenceLearner * S;
	SequenceLearnerCont * C;
	SequenceLearnerDisc * D;
	BufferedPort<Bottle> * inPort;
	Port *outPort;
	Port * generated;
	string recvPort;
	string sendPort;
	string genPort;


public:


	LexiconThread(SequenceLearner *& S_, SequenceLearnerCont *& C_, SequenceLearnerDisc *& D_,
			string recvPort_, string sendPort_, string genPort_)
	: S(S_), C(C_), D(D_), recvPort(recvPort_), sendPort(sendPort_), genPort(genPort_) {

		//and the ports
		inPort = new BufferedPort<Bottle>;
		outPort = new Port;
		generated = new Port;

	}

	virtual bool threadInit() {

		//inport (track when a connection arrives)
		inPort->open(recvPort.c_str());

		//outport
		outPort->open(sendPort.c_str());

		//set up generator port
		generated->open(genPort.c_str());
		generated->setTimeout(5.0);

		return true;

	}

	virtual void threadRelease() {

		inPort->close();
		outPort->close();
		S->printAll();

	}

	Bottle generateOutput(int n, double scale) {

		Bottle gendSequence;
		Bottle replySeq;
		bool gresult;

		if (S->getType()) {

			//continuous
			IMat data;
			gresult = S->generateSequence(data, n, scale);
			for (int i = 0; i < data.m; i++) {
				Bottle & samp = gendSequence.addList();
				for (int j = 0; j < data.n; j++) {
					samp.add(data(i,j));
				}
			}
			if (gresult) {
				replySeq.copy(gendSequence);
				generated->write(gendSequence);
			}

		} else {

			//discrete
			printf("discrete generation not yet implemented\n");
			replySeq.copy(gendSequence);
			gendSequence.add(-1);

		}

		return replySeq;

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

				int nprev = S->nInitialized;

				//continuous
				if (S->getType()) {

					//unpack
					real ** samplesC;
					Bottle * c;
					samplesC = new real * [b->size()];
					for (int i = 0; i < b->size(); i++) {
						c = b->get(i).asList();
						samplesC[i] = new real[c->size()];
						for (int j = 0; j < c->size(); j++) {
							samplesC[i][j] = c->get(j).asDouble();
						}
					}

					//train or classify
					C->scale(samplesC, b->size());
					lex = S->train(samplesC, b->size());
					val = S->evaluate(samplesC, b->size(),lex);

				}

				//discrete
				else {

					//unpack
					int ** samplesD;
					samplesD = new int * [b->size()];
					//can only handle vectors for now
					for (int i = 0; i < b->size(); i++) {
						samplesD[i] = new int;
						samplesD[i][0] = b->get(i).asInt();
					}

					//train or classify
					lex = S->train(samplesD, b->size());
					val = S->evaluate(samplesD, b->size(),lex);

				}

				//report result to console (but don't gum it up)
				if (nprev < S->nInitialized) {
					printf("new element %d initialized, trained log likelihood: ",lex);
					if (val < -1e+100) {
						printf("-inf\n");
					} else {
						printf("%f\n",val);
					}
				} else {
					printf("classified as %d\t log likelihood: ",lex);
					if (val < -1e+100) {
						printf("-inf\n");
					} else {
						printf("%f\n",val);
					}
				}


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

class GenReqPort : public BufferedPort<Bottle> {

protected:

	LexiconThread * L;
	double scale;


public:

	GenReqPort(LexiconThread *& L_, double scale_) : L(L_), scale(scale_) { }

	virtual void onRead(Bottle& b) {

		//if the input is poorly formed, yell about it
		if (!(b.get(0).isInt() || b.get(0).isDouble())) {
			printf("Received poorly formed request, ignoring\n");
		}
		else {
			L->generateOutput(b.get(0).asInt(), scale);
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
	double dt;		//sampling rate, used for scaling derivs
	bool upobs;		//flag for turning obs dist updating on/off
	int scm;		//scaling mode (-1 single coeff, 0 off, 1 vector)
	double scs;		//single scaling coeff
	IVec scv;		//scaling vector

	//parameters to use a LR model (different behavior for disc. and cont.)
	bool lr;		//left-to-right model flag (O)

	//logging
	bool save;
	string logname;

	/* actual data: ports, objects, etc... */
	SequenceLearner * S;
	SequenceLearnerCont * C;
	SequenceLearnerDisc * D;
	string recvPort;
	string sendPort;
	string rpcName;
	string genPort;
	string greqName;
	LexiconThread * L;
	GenReqPort * greqPort;
	Port rpcPort;


public:

	bool loadParams(ResourceFinder &rf) {

		//get port names
		recvPort = rf.check("input",Value("/lex:i"),"input port name").asString();
		sendPort = rf.check("output",Value("/lex:o"),"output port name").asString();
		greqName = rf.check("greq",Value("/lex:r"),"generation request port").asString();
		genPort = rf.check("gen",Value("/lex:g"),"generated example port name").asString();
		rpcName = rf.check("rpc",Value("/lex/rpc"),"rpc port name").asString();

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
			//continuous specific params
			dt = rf.check("dt",Value(0.01),"dt for calc of derivs").asDouble();
			Value alphaV = rf.find("alpha");
			Value xiV = rf.find("xi");
			if (alphaV.isNull() && xiV.isNull()) {
				constrain = false;
			} else {
				constrain = true;
				alpha = rf.check("alpha",Value(0.0),"cov. lower bound").asDouble();
				xi = rf.check("xi",Value(1.0e+300),"cov. upper bound").asDouble();
			}
			if ((bool)rf.check("updateobs",Value(1)).asInt()) {
				upobs = true;
			} else {
				upobs = false;
			}
			if (!rf.check("scale")) {
				scm = 0;
			} else {
				Bottle scalc;
				scalc = rf.findGroup("scale");
				if (scalc.size() == 2) {
					scm = -1;
					scs = scalc.get(1).asDouble();
				}
				else if (scalc.size() == d+1) {
					scm = 1;
					scv.resize(d);
					for (int i = 0; i < d; i++) {
						scv.ptr[i] = scalc.get(i+1).asDouble();
					}
				}
				else {
					printf("tried to set improper scaling factor\n");
					scm = 0;
				}
			}
		} else {
			//discrete specific params

		}

		//general optional parameters
		lr = rf.check("lefttoright");
		save = rf.check("save");
		if (save) {
			logname = rf.find("save").asString().c_str();
		}
		epochs = rf.check("epochs",Value(20),"number of training epochs").asInt();
		prior = rf.check("prior",Value(0.0001),"min values for parameters").asDouble();
		eps = rf.check("eps",Value(0.001),"learning rate").asDouble();
		decay = rf.check("decay",Value(1.0),"learning rate decay value (0.0-1.0)").asDouble();

		return true;

	}

	bool respond(const Bottle& command, Bottle& reply) {

		//handle information requests
		string msg(command.get(0).asString().c_str());

		//parameter gets
		if (msg == "get") {
			if (command.size() < 2) {
				reply.add(-1);
			}
			else {

				string arg(command.get(1).asString().c_str());
				if (arg == "n") {
					reply.add(S->nInitialized);
				}
				else if (arg == "r") {
					reply.add(r);
				}
				else if (arg == "d") {
					reply.add(d);
				}
				else if (arg == "a") {
					if (command.size() < 3) {
						reply.add(-1);
					} else {
						S->packA(reply,command.get(2).asInt());
					}
				}
				else if (arg == "pi") {
					if (command.size() < 3) {
						reply.add(-1);
					} else {
						S->packPi(reply,command.get(2).asInt());
					}
				}
				else if (arg == "b" || arg == "mu" || arg == "u") {
					if (command.size() < 3) {
						reply.add(-1);
					} else {
						S->packObs(reply,command.get(2).asInt());
					}
				}
				else {
					reply.add(-1);
				}
			}

		}
		else if (msg == "save") {
			if (command.size() < 3) {
				reply.add(-1);
			}
			else {
				string bname(command.get(1).asString().c_str());
				int tle = command.get(2).asInt();
				if (tle >= 0 && tle < S->nInitialized) {
					S->printToFile(bname, tle);
					reply.add(1);
				}
				else {
					reply.add(-1);
				}
			}
		}
		//allow certain running parameters to be set here
		else if (msg == "set") {

			if (command.size() < 3) {
				reply.add(-1);
			}
			else {
				string arg(command.get(1).asString().c_str());
				if (arg == "epochs") {
					S->setEpochs(command.get(2).asInt());
				}
				else if (arg == "prior") {
					S->setPrior(command.get(2).asDouble());
				}
				else if (arg == "eps") {
					S->setEps(command.get(2).asDouble());
				}
				else if (arg == "decay") {
					S->setEps_decay(command.get(2).asDouble());
				}
				else if (arg == "thresh") {
					S->setThresh(command.get(2).asDouble());
				}
				else {
					reply.add(-1);
				}
			}
		}
		else if (msg == "list") {
			reply.add("epochs");
			reply.add("prior");
			reply.add("eps");
			reply.add("decay");
			reply.add("thresh");
		}
		else if (msg == "eval") {
			if (command.size() < 2) {
				reply.add(-1);
			} else {
				double val;
				int ldx = command.get(1).asInt();
				if (ldx > S->nInitialized) {
					reply.add(-1);
				} else {
					int n = command.size()-2;
					int ** samplesD;
					samplesD = new int * [n];
					for (int i = 0; i < n; i++) {
						samplesD[i] = new int;
						samplesD[i][0] = command.get(i+2).asInt();
					}
					reply.add("log likelihood: ");
					if (ldx < 0) {
						for (int i = 0; i < S->nInitialized; i++) {
							val = S->evaluate(samplesD, n, i);
							reply.add(val);
						}
					} else {
						val = S->evaluate(samplesD, n, ldx);
						reply.add(val);
					}
					delete samplesD;
				}
			}
		}
		else if (msg == "gen") {
			if (command.size() < 2) {
				reply.add(-1);
			} else {
				int n = command.get(1).asInt();
				Bottle b = L->generateOutput(n, dt);
				reply.append(b);
			}
		}
		else {
			reply.add(-1);
		}


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
				C = new SequenceLearnerCont(r, d, b, epochs, thresh, prior, eps, alpha, xi, lr);
				S = C;
				S->eps_decay = decay;
			} else {
				C = new SequenceLearnerCont(r, d, b, epochs, thresh, prior, eps, lr);
				S = C;
				S->eps_decay = decay;
			}
			S->upobs = upobs;
			C->setScaling(scm, scs, &scv);
			D = NULL;

		} else {

			//discrete (fixed at one output obs for disc at this time
			D = new SequenceLearnerDisc(r, &d, 1, b, epochs, thresh, prior, eps, lr);
			S = D;
			S->eps_decay = decay;
			C = NULL;

		}

		//set up the rpc/observer port
		rpcPort.open(rpcName.c_str());
		attach(rpcPort);

		//pass everything off to the execution thread
		L = new LexiconThread(S, C, D, recvPort, sendPort, genPort);


		//set up the request port
		greqPort = new GenReqPort(L, dt);
		greqPort->useCallback();
		greqPort->open(greqName.c_str());

		return L->start();

	}

	virtual bool close() {

		if (save) {
			S->printToFile(logname);
		}
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


void dumbCSVReader(const char * fileName, IMat &tM, int length, int width) {

	//load the csv file into the target matrix
	FILE *fp;
	int z = length;
	char line[4096];
	char * coeff;

	fp = fopen(fileName, "r");

	if (fp == NULL) {

		printf("file %s not found, segfault inc\n",fileName);

	}

	if (z == 0) { //set length = 0 to find the file size
		while (!feof(fp)) {
			fscanf(fp,"%s",line);
			z++;
		}
		rewind(fp);
		z--;
	}

	tM.resize(z,width);
	for (int i = 0; i < z; i++) {
		fscanf(fp,"%s",line);
		coeff = strtok(line,",");
		tM.ptr[i][0] = atof(coeff);
		for (int j = 1; j < width; j++) {
			coeff = strtok(NULL,",");
			tM.ptr[i][j] = atof(coeff);
		}
	}

	fclose(fp);

}

void dumbCSVWriter(const char * fileName, IMat &tM) {

	FILE *fp = fopen(fileName,"w");

	for (int i = 0; i < tM.m; i++) {
		for (int j = 0; j < tM.n-1; j++) {
			fprintf(fp,"%f,",tM(i,j));
		}
		fprintf(fp,"%f\n",tM(i,tM.n-1));
	}
	fclose(fp);

}
