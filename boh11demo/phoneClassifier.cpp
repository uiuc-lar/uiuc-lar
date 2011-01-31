/*
 *  phoneClassifier.cpp
 *
 * 	Logan Niehaus
 * 	12/18/10
 * 	implementation of an HMM based phone classifier. a sequence of
 *  features (mfccs) goes in, and a symbol sequence comes out
 *
 * Module Args: (activity detection)
 * 	input 		-- input port name
 *  output		-- output port name
 *	afile		-- A matrix filename for the classifier (csv)
 *	mufile		-- mu vectors ''	''
 *	ufile		-- U matrices ''	''
 *	nphones		-- number of classes
 *	ncoeffs		-- obs. dimensionality
 *
 * PORTS:
 *	Inputs: /vad/words   	(Bottle of bottles. Each individual bottle a feature sample)
 *	Outputs: /phonetic		(Bottle of ints, corresponding to classes for each feature)
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
#include "../RMLE/HMM.hh"
#include "../RMLE/StochasticClassifier.hh"
#include "../RMLE/Gaussian.hh"
#include "../imatlib/IMatVecOps.hh"

//misc
#include <string>
#include <math.h>
#include <deque>

//defines
#define PRIOR 0.0001

//namespaces
using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

void dumbCSVReader(const char *, IMat &, int, int);
void dumbCSVWriter(const char *, IMat &);

class PhonePort : public BufferedPort<Bottle> {

protected:

	//classifier
	Allocator *allocator;
	HMM * p;
	Gaussian * obs_dist;

	//output port
	Port * oPort;
	Port * mPort; //input mirror port

	//verbosity
	bool debug;

public:

	PhonePort(HMM *& p_, Gaussian *& b_, Port *& oPort_, Port *& mPort_)
	: p(p_), obs_dist(b_), debug(false), oPort(oPort_), mPort(mPort_) {

	}

	//callback for incoming sequences
	virtual void onRead(Bottle& b) {

		//check for misclassifications
		bool badSample = false;
		int nBSamples = 0;

		//data, envelopes
		Bottle seqClass;
		Bottle tStamps;
		getEnvelope(tStamps);

		//reset the initial probs to uniform
		p->prob->fill(1.0/(real)p->r);

		//announce reception if verbose/debugging
		if (debug) {
			printf("Received sequence, length %d for classification\n", b.size());
		}

		for (int i = 0; i < b.size(); i++) {

			//unpack the sequence
			Bottle *item = b.get(i).asList();
			real *sample = new real[item->size()];
			for (int j = 0; j < item->size(); j++) {
				sample[j] = item->get(j).asDouble();
				if (debug) {
					printf("%f,",sample[j]);
				}
			}

			//make the classification
			int state = p->Classify(sample);

			//fill a new bottle with the sequence
			seqClass.add(state);

			//check to see if classifier is having problems
			if (state < 0 || state > obs_dist->d) {
				badSample = true;
				nBSamples++;
			}

			//print to screen if debugging desired
			if (debug) {
				printf("%d\n",state);
			}

			delete sample;

		}

		//pass along the timestamp
		oPort->setEnvelope(tStamps);

		//if valid, publish
		if (!badSample) {

			//only write if something is connected
			if (oPort->getOutputCount() > 0) {
				oPort->write(seqClass);
			}

			//mirror if we have a port connected
			if (mPort->getOutputCount() > 0) {
				mPort->write(b);
			}

		} else {
			printf("Phoneme stream could not be produced. Bad samples: %d/%d", nBSamples, b.size());
		}

	}

	virtual void debugOn() {

		debug = true;

	}

	virtual void debugOff() {

		debug = false;

	}

	virtual void close() {

		delete oPort;

	}

};

class PhoneModule : public RFModule {

protected:

	//ports
	string recvPort;
	string sendPort;
	string mirrorPort;
	Port * mPort;
	Port * oPort;
	PhonePort *iPort;
	Port rpcPort;

	//files
	Value afile;
	Value mufile;
	Value ufile;
	Value datafile;

	//classifier things
	int r;	//number of classifier states
	int d;	//observation dimensionality
	Allocator *allocator;
	HMM * p;
	Gaussian * obs_dist;

	//verbosity
	bool debug;


public:

	//override for respond method, handles rpc requests
	bool respond(const Bottle& command, Bottle& reply) {

		printf("Received rpc message, ");

		Bottle msg = command;

		printf("length %d.\n",msg.size());

		if (msg.get(0).asString() == "getn") {
			//return classifier size
			printf("Replying...\n");
			reply.add(r);
		}
		if (msg.get(0).asString() == "open") {

			if (msg.size() != 3) {
				reply.add(0);
			} else {

				//get the names of the two ports to connect
				string nmPort = msg.get(1).asString().c_str();
				string npPort = msg.get(2).asString().c_str();

				//make connection
				printf("Opening ports as requested...\t");
				if (!mPort->addOutput(nmPort.c_str()) ||
						!oPort->addOutput(npPort.c_str())) {
					printf("failed\n");
					reply.add(0);
				} else {
					printf("successful\n");
					reply.add(1);
				}
			}
		}

		return true;

	}

	virtual bool configure(ResourceFinder &rf)
	{
		IMat A;
		IMat MU;
		IMat U;
		IMat initData;

		//get port names
		recvPort = rf.check("input",Value("/phone:i"),"input port name").asString();
		sendPort = rf.check("output",Value("/phone:o"),"output port name").asString();
		mirrorPort = rf.check("mirror",Value("/phone:m"),"input mirror port name").asString();

		//get sizes
		r = rf.find("nphones").asInt();
		d = rf.find("ncoeffs").asInt();

		//get parameter files
		afile= rf.find("afile");
		mufile = rf.find("mufile");
		ufile = rf.find("ufile");
		datafile = rf.find("datafile");
		if (afile.isNull() || mufile.isNull() || ufile.isNull()) {
			if (datafile.isNull()) {
				printf("You must specify parameter files for A, MU, and U, or a data file for initialization\n");
				return false;
			} else {

				printf("initializing from data file...\n");
				//initialize from data file
				dumbCSVReader(datafile.asString().c_str(), initData, 0, d);
				//printf("with %d samples...\n",initData.m);

				//init obs
				allocator = new Allocator;
				p = new(allocator) HMM;
				obs_dist = new(allocator) Gaussian(r, d, NULL, &initData, 100);
				p->init(obs_dist, NULL, false, PRIOR, 1, false, false);

				//training
				real * sample;
				for (int i = 0; i < initData.m; i++) {
					sample = new real[initData.n];
					for (int j = 0; j < initData.n; j++) {
						sample[j] = initData(i,j);
					}
					p->Classify(sample);
					p->RMLEUpdate();
					delete sample;
					printf("%d\n",i);
				}

				p->print(200,10);

				if (rf.check("save")) {
					dumbCSVWriter("A.csv",*(p->A));
					dumbCSVWriter("MU.csv",*(obs_dist->MU));
					IMat U(r,d*d);
					IMat Rtmp, Utmp;
					for (int l = 0; l < r; l++)
					{
						obs_dist->R->getRow(l, 0, d, d, &Rtmp);
						U.getRow(l, 0, d, d, &Utmp);
						MatMatMult(&Rtmp, CblasTrans, &Rtmp, CblasNoTrans, &Utmp);
					}
					dumbCSVWriter("U.csv",U);
				}

				printf("initialization complete, phonetic classifier now online\n");

			}
		} else {

			printf("loading saved parameter set\n");

			//read parameters in
			dumbCSVReader(afile.asString().c_str(), A, r, r);
			dumbCSVReader(mufile.asString().c_str(), MU, r, d);
			dumbCSVReader(ufile.asString().c_str(), U, r, d*d);

			//init gaussian obs and HMM
			allocator = new Allocator;
			p = new(allocator) HMM;
			obs_dist = new(allocator) Gaussian(r, d, &MU, &U, PRIOR, true, false);
			p->init(obs_dist, &A, false, PRIOR, 1, true, false);
			p->print(200,10);
			printf("Phonetic model parameters successfully loaded\n");

		}

		//create and open ports
		oPort = new Port;
		oPort->open(sendPort.c_str());
		mPort = new Port;
		mPort->open(mirrorPort.c_str());
		mPort->enableBackgroundWrite(true);
		iPort = new PhonePort(p, obs_dist, oPort, mPort);
		iPort->useCallback();  // register callback
		iPort->open(recvPort.c_str());

		//setup rpc port
		rpcPort.open((recvPort+"/rpc").c_str());
		attach(rpcPort);

		//check for debug request
		debug = rf.check("debug");
		if (debug) {
			iPort->debugOn();
		}

		return true;
	}

	virtual bool close() {

		iPort->close();
		oPort->close();
		mPort->close();
		delete obs_dist;
		delete p;
		delete allocator;
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

	PhoneModule mod;

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
