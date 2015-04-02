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
 *  associativeMemory.cpp
 *
 * 	Logan Niehaus
 * 	12/23/10
 * 	Implementation of an HMM based associative memory. Must have at least
 * 	two symbolic inputs. Module uses synchrony detection to select one of two behaviors:
 *
 *  1: a_n and b_n presented simultaneously -- RMLE update using both observations
 *  2: a_b or b_n presented alone -- find single step MAP state estimate.
 *
 *  Case 2 will additionally emit a integer on the complementary output port
 *  (i.e. if a_n is presented alone, a b_n will be generated). The symbol
 *  emitted will be the most likely symbol for that modality given the internal
 *  state estimate.
 *
 * Module Args: name -- description (Required/Optional/Default given)
 * 	[module parameters]
 * 	inputA 		-- input A port name (D, /assocMem/in:a)
 *	inputB		-- input B port name (D, /assocMem/in:b)
 *	adA			-- activity A port name (D, /assocMem/act:a)
 *	adB			-- activity B port name (D, /assocMem/act:b)
 *  outputS		-- state output port name (D, /assocMem/out:s)
 *  outputA		-- generated A output port (D, /assocMem/out:a)
 *  outputB		-- generated B output port (D, /assocMem/out:b)
 *  rpc			-- rpc port name (O, D /assocMem/rpc)
 *  afile		-- CSV containing A matrix	(O)
 *  bfileA		-- CSV containing modality A observation matrix (O)
 *  bfileB		-- CSV containing modality B observation matrix	(O)
 *
 *
 *	[HMM parameters]
 *	nsymbA		-- number of possible observation symbols for input A (R)
 *	nsymbB		-- number of possible observation symbols for input B (R)
 *	nstates		-- number of states for associative memory (R)
 *	markov		-- flag to use markovian condition or not (O, D 1)
 *	epochs		-- number of epochs to train each symbol pair (O,D 1)
 *	prior		-- min. value for pmfs (O,D 0.0001)
 *	eps			-- learning rate (O,D 0.005, higher for assoc. mem.)
 *	decay		-- learning rate decay (O, D 1.0, D b/w 0.0 and 1.0)
 *
 * PORTS:
 *	Inputs: /lexAOut,/lexBOut  	(Bottles, w/ single integer. Symbolic modality observations.
 *								  These should be timestamped for synch detection. If not,
 *								  behavior is to wait some fixed amount of time for the other modality)
 *	Outputs: /assocMem/out:XXX	(Bottles w/ single int, corresponding to MAP estimates, generated symbols)
 *	RPCs: 	/assocMem/rpc		(RPC communication port for run-time interaction)
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
#include "../RMLE/IndepPMF.hh"

#undef min
#undef max

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

void packMat(Bottle &, IMat *);
void dumbCSVReader(const char *, IMat &, int, int);

class DataBuffer : public deque<int *> {

private:

	Semaphore mutex;

public:

	bool needsPair[2];

	DataBuffer() {
		needsPair[0] = false;
		needsPair[1] = false;
	}
	void lock()   { mutex.wait(); }
	void unlock() { mutex.post(); }

};

class MemoryPort : public BufferedPort<Bottle> {

protected:

	DataBuffer &buffer;
	Port * oppActPort;
	int m;	//should either be 0 (modality A) or 1 (modality B)

public:

	MemoryPort(DataBuffer &buf, Port * oppActPort_, int modality_)
	: buffer(buf), oppActPort(oppActPort_), m(modality_), BufferedPort<Bottle>() { }

	//callback for incoming sequences
	virtual void onRead(Bottle& b) {

		//first check if buffer is waiting for this
		if (buffer.needsPair[m]) {

			buffer.lock();
			int * item = buffer.back();
			item[m] = b.get(0).asInt();
			buffer.needsPair[m] = false;
			buffer.unlock();

		} else {

			//then check the port of the other modality for activity
			Bottle garbage;
			Bottle reply;
			oppActPort->write(garbage,reply);
			if (reply.get(0).asInt()) {

				//if activity detected, mark flag for other modality, and enter half obs
				int * item = new int[2];
				item[m] = b.get(0).asInt();
				item[1-m] = -1;
				buffer.lock();
				buffer.needsPair[1-m] = true;
				buffer.push_back(item);
				buffer.unlock();

			} else {

				//if no activity on opposite modality, enter observation as alone
				int * item = new int[2];
				item[m] = b.get(0).asInt();
				item[1-m] = -1;
				buffer.lock();
				buffer.push_back(item);
				buffer.unlock();

			}
		}

	}

};

class MemoryModule : public RFModule {

protected:

	//port names
	string recvPortA;
	string recvPortB;
	string actPortA;
	string actPortB;
	string sendPortS;
	string sendPortA;
	string sendPortB;
	string rpcName;

	//ports
	MemoryPort *aPort;
	MemoryPort *bPort;
	Port * activityA;
	Port * activityB;
	Port * outPortS;
	Port * outPortA;
	Port * outPortB;
	Port rpcPort;

	//files
	string afile;
	string bfileA;
	string bfileB;

	//associative memory parameters
	int r;			//number of classifier states
	int * d;		//symbol set sizes
	bool markov;	//treat associative mem as markoving (CHMM)
	int epochs;		//number of epochs to train
	double prior;	//min. value for pmfs
	double eps;		//learning rate
	double decay;	//learning rate decay

	//associative memory objects
	IMat * A;
	IMat * BA;
	IMat * BB;
	Allocator *allocator;
	HMM * p;
	IndepPMF * obs_dist;
	DataBuffer obs;


public:

	bool loadParams(ResourceFinder &rf) {

		//get observation alphabet sizes, and number of states
		if (!rf.check("nsymbA") || !rf.check("nsymbB")) {
			printf("Please specify observation alphabet sizes\n");
			return false;
		}
		d = new int[2];
		d[0] = rf.find("nsymbA").asInt();
		d[1] = rf.find("nsymbB").asInt();
		if (!rf.check("nstates")) {
			printf("Please specify associative memory size (# of concepts)\n");
			return false;
		}
		r = rf.find("nstates").asInt();

		//get port names
		recvPortA = rf.check("inputA",Value("/assocMem/in:a"),"modality A input port").asString();
		recvPortB = rf.check("inputB",Value("/assocMem/in:b"),"modality B input port").asString();
		actPortA = rf.check("adA",Value("/assocMem/act:a"),"modality A activity port").asString();
		actPortB = rf.check("adB",Value("/assocMem/act:b"),"modality B activity port").asString();
		sendPortS = rf.check("outputS",Value("/assocMem/out:s"),"internal state output port").asString();
		sendPortA = rf.check("outputA",Value("/assocMem/out:a"),"modality A output port").asString();
		sendPortB = rf.check("outputB",Value("/assocMem/out:b"),"modality B output port").asString();
		rpcName = rf.check("rpc",Value("/assocMem/rpc"),"rpc port name").asString();

		//check for saved parameters to be loaded
		A = NULL;
		BA = new IMat(d[0],r);
		BB = new IMat(d[1],r);
		BA->rand(0.3, 0.6);
		BB->rand(0.3, 0.6);
		if (rf.check("afile")) {
			A = new IMat(r,r);
			dumbCSVReader(rf.find("afile").asString().c_str(), *A, r, r);
			printf("A Matrix provided, loading params... \n");
			A->print(500,10);
		}
		if (rf.check("bfileA")) {
			dumbCSVReader(rf.find("bfileA").asString().c_str(), *BA, d[0], r);
			printf("B_a Matrix provided, loading params... \n");
			BA->print(500,10);
		}
		if (rf.check("bfileB")) {
			dumbCSVReader(rf.find("bfileB").asString().c_str(), *BB, d[1], r);
			printf("B_b Matrix provided, loading params... \n");
			BB->print(500,10);
		}

		//get optional parameters
		markov = (bool)rf.check("markov",Value(1),"use markovian property").asInt();
		epochs = rf.check("epochs",Value(1),"number of training epochs").asInt();
		prior = rf.check("prior",Value(0.0001),"min values for parameters").asDouble();
		eps = rf.check("eps",Value(0.005),"learning rate").asDouble();
		decay = rf.check("decay",Value(1.0),"learning rate decay value (0.0-1.0)").asDouble();

		return true;

	}

	virtual bool configure(ResourceFinder &rf)
	{

		//load in all the arguments/parameters
		if (!loadParams(rf)) {
			return false;
		}

		//initialize associative memory
		IMat ** B = new IMat * [2];
		B[0] = BA;
		B[1] = BB;
		IVecInt * D = new IVecInt(d,2);
		allocator = new Allocator;
		p = new(allocator) HMM;
		obs_dist = new(allocator) IndepPMF(D, B, prior, true, false);
		p->init(obs_dist, A, true, prior, 1, true, false);
		p->eps0 = eps;
		obs_dist->eps0 = eps;

		//create and open ports
		activityA = new Port;
		activityA->open(actPortA.c_str());
		activityB = new Port;
		activityB->open(actPortB.c_str());
		outPortS = new Port;
		outPortS->open(sendPortS.c_str());
		outPortA = new Port;
		outPortA->open(sendPortA.c_str());
		outPortB = new Port;
		outPortB->open(sendPortB.c_str());

		aPort = new MemoryPort(obs,activityB,0);
		bPort = new MemoryPort(obs,activityA,1);
		aPort->useCallback();
		aPort->open(recvPortA.c_str());
		bPort->useCallback();
		bPort->open(recvPortB.c_str());

		rpcPort.open(rpcName.c_str());
		attach(rpcPort);

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
				if (arg == "r") {
					reply.add(r);
				}
				else if (arg == "d") {
					if (command.size() < 3) {
						reply.add(-1);
					} else {
						reply.add(d[command.get(2).asInt()]);
					}
				}
				else if (arg == "a") {
					if (command.size() < 3) {
						reply.add(-1);
					} else {
						packMat(reply,p->A);
					}
				}
				else if (arg == "b") {
					if (command.size() < 3) {
						reply.add(-1);
					} else {
						packMat(reply,obs_dist->b[command.get(2).asInt()]);
					}
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
					epochs = command.get(2).asInt();
				}
				else if (arg == "prior") {
					prior = command.get(2).asDouble();
				}
				else if (arg == "eps") {
					eps = command.get(2).asDouble();
					p->eps0 = eps;
					obs_dist->eps0 = eps;
				}
				else if (arg == "decay") {
					decay = command.get(2).asDouble();
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
		}
		else {
			reply.add(-1);
		}

		return true;

	}

	virtual bool close() {

		activityA->close();
		activityB->close();
		outPortS->close();
		outPortA->close();
		outPortB->close();
		aPort->close();
		bPort->close();

		p->print(200,10);

		delete obs_dist;
		delete p;
		return true;

	}

	virtual double getPeriod() { return 0.0; }

	virtual bool updateModule() {

		//check if buffer is waiting on either input, or is empty
		if (obs.needsPair[0] || obs.needsPair[1] || obs.empty()) {

			//if it is, just keep waiting
			return true;

		} else {

			//if there is a sample for processing, process it
			int * pair = obs.front();
			IVecInt V;
			Bottle c, g;
			int concept;

			//check if modality B came in alone
			if (pair[0] == -1) {
				printf("B_n presented alone...\n");
				//if modality B is 'alone'
				concept = p->Classify(pair[1],1);
				obs_dist->b[0]->mmax(NULL,&V,1);
				g.add(V(concept));
				outPortA->write(g);
			}

			//check if modality A came in alone
			else if (pair[1] == -1) {
				printf("A_n presented alone...\n");
				//if modality A is alone
				concept = p->Classify(pair[0],0);
				obs_dist->b[1]->mmax(NULL,&V,1);
				g.add(V(concept));
				outPortB->write(g);
			}

			//check for simultaneous presentation
			else {
				printf("Both A and B presented together...\n");
				//if presented as a pair, do an RMLE step
				concept = p->Classify(pair);
				p->RMLEUpdate();
				if (!markov) {
					p->A->fill((real)1.0/r);
				}

			}

			//write out the conceptual classification, cleanup
			c.add(concept);
			outPortS->write(c);
			obs.lock();
			obs.pop_front();
			obs.unlock();

		}

		return true;

	}

};


int main(int argc, char *argv[])
{
	// we need to initialize the drivers list
	Network yarp;
	if (!yarp.checkNetwork())
		return -1;

	MemoryModule mod;

	ResourceFinder rf;

	rf.configure("ICUB_ROOT", argc, argv);

	return mod.runModule(rf);
}

void packMat(Bottle &b, IMat * M) {

	for (int i = 0; i < M->m; i++) {
		Bottle &tmp = b.addList();
		for (int j = 0; j < M->n; j++) {
			tmp.add(M->ptr[i][j]);
		}
	}

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
