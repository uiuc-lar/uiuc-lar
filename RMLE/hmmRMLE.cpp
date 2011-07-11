/*
 *  hmmRMLE.cpp
 *
 * 	Logan Niehaus
 * 	6/29/11
 * 	yarp module interface to a hidden markov model trained using the RMLE algorithm
 *
 * 	inputs:
 * 		/hmmRMLE/obs:i	-- observation data
 * 		/hmmRMLE/gen:i	-- state inputs for running model in reverse
 *
 *  params:
 *  	type	-- observation distribution type, either 'gaussian' or 'discrete' (R)
 *  	nstates	-- internal model order (R)
 *  	obsdim	-- observation distribution dimension, single int for gaussians, vector of ints for discrete (R)
 *  	eps		-- learning rate for training (D 0.001)
 *  	prior	-- min value for all pmfs (D 0.0005)
 *  	decay	-- learning rate decay; 1 = no decay (D 1.0)
 *  	initsamples	-- number of initialization samples to gather at beginning to use for kmeans initialization (O 0.0)
 *  	nkmiter -- number of iterations to run for k-means initialization (D 20)
 *  	nomark	-- flag to turn markov behavior off; all elements of trans. matrix are set equal (O)
 *  	lr		-- flag to enforce left-to-right condition on model (O)
 *  	afile	-- csv file containing A matrix data to be loaded
 *  	mufile	-- csv file containing mu vectors (O, gauss only)
 *  	rfile	--	""			""	   chol. decomp. cov matrices (O, gauss only)
 *  	bfile	--  ""			""     obs matrix data (O, disc only)
 *  	train	-- set to either 0 or 1, to give initial value for training flag (D 1)
 *  	log		-- flag to stream vector of all parameter values to port after each classification (O 0)
 *  	verbose	-- for now just enables echoing of the current state to stdout
 *  	name	-- module basename (D /hmmRMLE)
 *
 *  outputs:
 *  	/hmmRMLE/state:o	-- estimated (ML) internal state value; vector with one element
 *  	/hmmRMLE/gen:o		-- randomly generated observations; produced only when requested on /hmmRMLE/gen:i
 *  	/hmmRMLE/log:o		-- stream of all parameters values as a vector in form of [A B1 B2...] for disc and [A MU R] for gaussian
 *  	/hmmRMLE/rpc		-- rpc port for run-time access to model parameters/data
 *
 *  rpc commands:
 *  	set <param> <val>	-- set one of the run-time parameters to given value (n/a to type, nstates or obsdim)
 *  	get <param>			-- return the value of one of the run-time parameters
 *		get <model_param> <state> -- return the value of one of the HMM parameter sets (ex: 'get A', 'get mu 1') (not yet implemented)
 *		gen <state>			-- generate a random observation from <state>'s output distribution
 *
 *	TODO:
 *		online setting of some runtime parameters (eps, decay, train, etc...)
 *		better handling of dynamic deallocation on cleanup
 *		break csv, tab input file parsing out into new library
 *		shoehorn MC generation functions into proper classes or new aux function file
 *
 */


//yarp network
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/math/Math.h>

//internal libraries
#include "./HMM.hh"
#include "./Gaussian.hh"
#include "./IndepPMF.hh"
#include "../imatlib/IMatVecOps.hh"

//gsl includes (for randn functionality)
#include <gsl/gsl_randist.h>
#include <gsl/gsl_rng.h>

//misc
#include <string>
#include <math.h>
#include <deque>
#include <time.h>
#include <stdlib.h>

//namespaces
using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


class DataBuffer : public deque<yarp::sig::Vector *> {

private:

	Semaphore mutex;

public:

	void lock()   { mutex.wait(); }
	void unlock() { mutex.post(); }

};

class QueuerPort : public BufferedPort<yarp::sig::Vector> {

protected:

	DataBuffer &B;

public:

	QueuerPort(DataBuffer &_B) : B(_B) { }

	virtual void onRead(yarp::sig::Vector& data) {

		yarp::sig::Vector * f = new yarp::sig::Vector(data);

		B.lock();
		B.push_back(f);
		B.unlock();

	}


};


class HmmRmleThread : public Thread {

protected:

	//names
	ResourceFinder &rf;
	string name;

	//data ports
	QueuerPort * portObsIn;
	QueuerPort * portGenIn;
	BufferedPort<yarp::sig::Vector> * portStateOut;
	BufferedPort<yarp::sig::Vector> * portGenOut;
	BufferedPort<yarp::sig::Vector> * portLogOut;

	//data objects
	DataBuffer obsBuffer;
	DataBuffer genBuffer;
	IMat *Am, *MUm, *Rm;
	IMat *inData;
	IMat **Bm;

	//model objects
	Allocator *allocator;
	HMM * p;
	StochasticClassifier * obs_dist;
	Gaussian * g_dist;
	IndepPMF * d_dist;

	//model params
	int type;
	int r;
	double eps, prior, decay;
	bool nomark, ltr;
	int * d;
	int nmos;	//for discrete only

	//aux. model runtime params
	bool training;
	bool logparams;
	int initsamples;
	int nkmiter;
	bool verbose;

	//gsl rng vars
	const gsl_rng_type * T;
	gsl_rng * rg;


public:


	HmmRmleThread(ResourceFinder &_rf) : rf(_rf) { }

	HMM * getHMMHandle() {	return p; }
	Gaussian * getGaussHandle() { return g_dist; }
	IndepPMF * getDiscHandle() { return d_dist;	}

	bool getMatFromFile(IMat *& M, const char * matName, ResourceFinder &_rf) {
		ImageOf<PixelFloat> pD;
		if (rf.check(matName)) {
			if (file::read(pD, rf.find(matName).asString().c_str())) {
				M = new IMat(pD.height(),pD.width());
				printf("file contained matrix of size %d x %d\n", pD.height(), pD.width());
				for (int i = 0; i < pD.width(); i++) {
					for (int j = 0; j < pD.height(); j++) {
						M->ptr[j][i] = pD.pixel(i,j);
					}
				}
			} else {
				return false;
			}

		} else {
			return false;
		}
		return true;
	}

	bool writeMatToFile(IMat *& M, const char * fname) {

		FILE *fp;

		fp = fopen(fname, "w");

		if (M == NULL || fp == NULL) {
			return false;
		}

		for (int i = 0; i < M->m; i++) {
			for (int j = 0; j < M->n-1; j++) {
				fprintf(fp,"%f\t", M->ptr[i][j]);
			}
			fprintf(fp,"%f\n",M->ptr[i][M->n-1]);
		}

		fclose(fp);

		return true;

	}

	bool genFromModel(yarp::sig::Vector &Gout, int gstate) {

		yarp::sig::Vector &G = portGenOut->prepare();
		G.clear();

		if (type) {

			IVec *dc = new IVec;
			double rc;
			int wc;

			//generate a sample vector of symbols
			for (int i = 0; i < nmos; i++) {

				d_dist->b[i]->getCol(gstate, dc, true);
				rc = (double)rand()/(double)RAND_MAX;
				wc = 0;
				while (rc > 0.0) {
					rc = rc - dc->ptr[wc];
					wc++;
				}
				G.push_back(wc-1);

			}
			delete dc;


		} else {

			//generate a sample vector from the MVN using GSL
			IVec * xx = new IVec(d[0]);
			IVec * mutmp = new IVec(d[0]);
			IMat * Rtmp = new IMat(d[0],d[0]);

			//get mu, R matrix
			g_dist->R->getRow(gstate, 0, d[0], d[0], Rtmp, true);
			g_dist->MU->getRow(gstate, mutmp, true);

			//sample from distribution using GSL
			for (int i = 0; i < d[0]; i++) {
				//xx->ptr[i] = gsl_ran_gaussian (r, 1.0);
				xx->ptr[i] = gsl_ran_ugaussian_ratio_method(rg);
			}
			GenMatVecMult(1.0,Rtmp,CblasNoTrans,xx,1.0,mutmp);

			//copy to vector and clean up
			for (int i = 0; i < d[0]; i++) {
				G.push_back(mutmp->ptr[i]);
			}
			delete xx;
			delete mutmp;
			delete Rtmp;

		}


		Gout = G;
		portGenOut->write();


	}


	bool handleParams() {

		name=rf.check("name",Value("hmmRMLE")).asString().c_str();
		eps = rf.check("eps",Value(0.001),"learning rate").asDouble();
		prior = rf.check("prior",Value(0.0005),"min values for parameters").asDouble();
		decay = rf.check("decay",Value(0.0),"learning rate decay value (0.0-1.0)").asDouble();
		initsamples = rf.check("initsamples",Value(0),"number of kmeans init samples to gather before starting").asInt();
		nkmiter = rf.check("nkmiter",Value(20),"number of kmeans iters").asInt();
		nomark = rf.check("nomark");
		ltr = rf.check("lr");
		training = (bool)rf.check("train",Value(1)).asInt();
		logparams = (bool)rf.check("log");
		verbose = (bool)rf.check("verbose");

		//require number of states
		if (rf.check("nstates")) {
			r = rf.find("nstates").asInt();
		} else {
			printf("Please specify the number of states of internal model\n");
			return false;
		}

		//get obs distr type
		string typeName=rf.check("type",Value("gaussian")).asString().c_str();
		if (typeName == "gaussian") {
			type = 0;
		}
		else if (typeName == "discrete") {
			type = 1;
			initsamples = 0;
		}
		else {
			printf("Observation distribution type improperly specified, please use either '--type gaussian' or '--type discrete\n");
			return false;
		}

		//get number of observation distributions
		Bottle odim = rf.findGroup("obsdim");
		if (!odim.isNull()) {
			if (!type) {
				if (odim.size() > 2 || odim.size() < 2) {
					printf("Invalid observation dimension specified for given distribution type\n");
					return false;
				} else {
					d = new int;
					d[0] = odim.get(1).asInt();
				}
			} else {
				if (odim.size() < 2) {
					printf("Invalid observation dimension specified for given distribution type\n");
					return false;
				} else {
					nmos = odim.size()-1;
					d = new int[nmos];
					for (int i = 1; i < odim.size(); i++) {
						d[i-1] = odim.get(i).asInt();
					}
				}
			}
		}

		return true;

	}

	virtual bool threadInit() {

		//handle general model parameters
		if (!handleParams()) {
			return false;
		}


		//start up ports
		portObsIn=new QueuerPort(obsBuffer);
		string portObsIName="/"+name+"/obs:i";
		portObsIn->open(portObsIName.c_str());
		portObsIn->useCallback();

		portGenIn=new QueuerPort(genBuffer);
		string portGenIName="/"+name+"/gen:i";
		portGenIn->open(portGenIName.c_str());
		portGenIn->useCallback();

		portStateOut=new BufferedPort<yarp::sig::Vector>;
		string portStateOName="/"+name+"/state:o";
		portStateOut->open(portStateOName.c_str());

		portGenOut=new BufferedPort<yarp::sig::Vector>;
		string portGenOName="/"+name+"/gen:o";
		portGenOut->open(portGenOName.c_str());

		portLogOut=new BufferedPort<yarp::sig::Vector>;
		string portLogOName="/"+name+"/log:o";
		portLogOut->open(portLogOName.c_str());


		//check for previously loaded data
		Am = MUm = Rm = inData = NULL;
		Bm = NULL;

		//initialize model objects
		allocator = new Allocator;
		p = new(allocator) HMM;
		if (type) {

			//discrete output dist init
			d_dist = new(allocator) IndepPMF(r, nmos, d, prior, true);
			obs_dist = d_dist;
			g_dist = NULL;

		} else {

			//gaussian output dist init
			if (getMatFromFile(inData, "initdata", rf)) {

				g_dist = new(allocator) Gaussian(r, d[0], NULL, inData, nkmiter);

			} else {

				getMatFromFile(MUm, "mufile", rf);
				getMatFromFile(Rm, "rfile", rf);
				g_dist = new(allocator) Gaussian(r, d[0], MUm, NULL, 0.1, true);
				if (MUm) delete MUm;
				if (Rm) {
					g_dist->R->set(Rm->base, Rm->m, Rm->n, Rm->ld, true, true);
					delete Rm;
				}

			}
			obs_dist = g_dist;
			d_dist = NULL;
		}

		//initialize internal model
		if (getMatFromFile(Am, "afile", rf)) {
			p->init(obs_dist, Am, false, prior, 1, true);
			delete Am;
		} else {
			p->init(obs_dist, NULL, true, prior, 1);
		}

		//enforce constraints on A matrix if so desired
		if (nomark) {
			p->A->fill((real)1.0/r);
		}
		if (ltr) {
			for (int i = 0; i < r; i++) {
				for (int j = 0; j < r; j++) {
					if (j != i && j != (i+1)) {
						p->A->ptr[i][j] = 0;
					}
				}
			}
			p->ProbProject(p->A, prior, 2);
		}

		p->eps0 = eps;
		p->eps_exp = decay;
		obs_dist->eps0 = eps;
		obs_dist->eps_exp = decay;

		//setup rng
		gsl_rng_env_setup();
		T = gsl_rng_default;
		rg = gsl_rng_alloc (T);

		return true;

	}

	virtual void threadRelease() {

		portObsIn->interrupt();
		portGenIn->interrupt();
		portStateOut->interrupt();
		portGenOut->interrupt();

		delete portObsIn;
		delete portGenIn;
		delete portStateOut;
		delete portGenOut;

		//delete obs_dist;
		//delete p;
		delete allocator;

		gsl_rng_free (rg);

	}

	virtual void run() {

		while (isStopping() != true) {

			//check to see if watcher ports have received any new data
			if (!obsBuffer.empty()) {

				//if initialization samples still need to be gathered, then wait
				if (initsamples > 0) {

					//if required init samples gathered, then do init
					if (obsBuffer.size() == initsamples) {

						yarp::sig::Vector *vp;
						inData = new IMat(obsBuffer.size(), d[0]);
						inData->zero();
						for (int i = 0; obsBuffer.size() > 0; i++) {
							vp = obsBuffer.front();
							for (int j = 0; j < vp->size(); j++) {
								inData->ptr[i][j] = (*vp)[j];
							}
							obsBuffer.pop_front();
						}
						g_dist->KMeansInit(inData, true, nkmiter);
						initsamples = 0;
						printf("k-means initialization has been completed...\n");
					}

				}
				//if initialization already complete or no init required
				else {

					int cstate = -1;
					yarp::sig::Vector *vPtr = obsBuffer.front();
					yarp::sig::Vector X(*vPtr);
					obsBuffer.pop_front();
					delete vPtr;

					yarp::sig::Vector &lVec = portLogOut->prepare();
					lVec.clear();

					if (logparams) {
						for (int i = 0; i < r; i++) {
							for (int j = 0; j < r; j++) {
								lVec.push_back(p->A->ptr[i][j]);
							}
						}
					}

					//classify observation
					if (type) {

						int * data = new int[X.size()];
						for (int i = 0; i < X.size(); i++) {
							data[i] = (int)X[i];
						}
						cstate = p->Classify(data);
						delete data;

						if (logparams) {
							for (int k = 0; k < nmos; k++) {
								for (int i = 0; i < d[k]; i++) {
									for (int j = 0; j < r; j++) {
										lVec.push_back(d_dist->b[k]->ptr[i][j]);
									}
								}
							}
						}


					} else {

						real * data = new real[X.size()];
						for (int i = 0; i < X.size(); i++) {
							data[i] = (real)X[i];
						}
						cstate = p->Classify(data);
						delete data;

						if (logparams) {
							for (int i = 0; i < r; i++) {
								for (int j = 0; j < d[0]; j++) {
									lVec.push_back(g_dist->MU->ptr[i][j]);
								}
							}
							for (int i = 0; i < r; i++) {
								for (int j = 0; j < d[0]; j++) {
									for (int k = 0; k < d[0]; k++) {
										lVec.push_back(g_dist->R->ptr[i][k+d[0]*j]);
									}
								}
							}
						}

					}

					if (logparams) {
						portLogOut->write();
					} else {
						portLogOut->unprepare();
					}

					//train if it currently enabled, enforcing model constraints
					if (training) {
						p->RMLEUpdate();
					}
					if (nomark) {
						p->A->fill((real)1.0/r);
						p->reset();
					}
					if (ltr) {
						for (int i = 0; i < r; i++) {
							for (int j = 0; j < r; j++) {
								if (j != i && j != (i+1)) {
									p->A->ptr[i][j] = 0;
								}
							}
						}
						p->ProbProject(p->A, prior, 2);
					}

					//write out state
					yarp::sig::Vector &cs = portStateOut->prepare();
					cs.clear();
					cs.push_back(cstate);
					if (verbose) {
						printf("current model state: %d\n", cstate);
					}
					portStateOut->write();

				}
			}


			//check for generation requests
			if (!genBuffer.empty()) {

				int gstate = -1;
				yarp::sig::Vector *vPtr = genBuffer.front();
				yarp::sig::Vector greq(*vPtr);
				gstate = (int)greq[0];
				genBuffer.pop_front();
				delete vPtr;

				//allow the user to send any negative number to generate based on current model state
				if (gstate < 0 || gstate > r-1) {
					gstate = p->state;
				}

				//generate according to type
				yarp::sig::Vector G;
				G.clear();
				genFromModel(G, gstate);
			}
		}
	}
};


//module for handling inputs and objects
class HmmRmleModule : public RFModule {

protected:

	HmmRmleThread * thr;
	Port * rpcPort;
	string name;
	HMM * hs;
	Gaussian * gs;
	IndepPMF * ds;

public:


	bool respond(const Bottle& command, Bottle& reply) {

		//handle information requests
		string msg(command.get(0).asString().c_str());
		if (msg == "get") {
			if (command.size() < 2) {
				reply.add(-1);
			}
			else {
				string arg(command.get(1).asString().c_str());
				if (arg == "A") {
					hs = thr->getHMMHandle();
					for (int i = 0; i < hs->A->m; i++) {
						Bottle &tmp = reply.addList();
						for (int j = 0; j < hs->A->n; j++) {
							tmp.add(hs->A->ptr[i][j]);
						}
					}
				}
				else if (arg == "B") {
					int bn = command.get(2).asInt();
					ds = thr->getDiscHandle();
					if (ds) {
						for (int i = 0; i < ds->b[bn]->m; i++) {
							Bottle &tmp = reply.addList();
							for (int j = 0; j < ds->b[bn]->n; j++) {
								tmp.add(ds->b[bn]->ptr[i][j]);
							}
						}
					}
				}
				else if (arg == "MU") {
					gs = thr->getGaussHandle();
					if (gs) {
						for (int i = 0; i < gs->MU->m; i++) {
							Bottle &tmp = reply.addList();
							for (int j = 0; j < gs->MU->n; j++) {
								tmp.add(gs->MU->ptr[i][j]);
							}
						}
					}
				}
				else if (arg == "R") {
					gs = thr->getGaussHandle();
					if (gs) {
						for (int i = 0; i < gs->R->m; i++) {
							Bottle &tmp = reply.addList();
							for (int j = 0; j < gs->R->n; j++) {
								tmp.add(gs->R->ptr[i][j]);
							}
						}
					}
				}
				else {
					reply.add(-1);
				}
			}
		}
		if (msg == "save") {
			if (command.size() < 3) {
				reply.add(-1);
			}
			else {
				string arg(command.get(1).asString().c_str());
				string fname(command.get(2).asString().c_str());
				bool res = true;
				if (arg == "A") {
					hs = thr->getHMMHandle();
					res &= thr->writeMatToFile(hs->A, fname.c_str());
				}
				else if (arg == "B") {
					int bn = command.get(2).asInt();
					fname = command.get(3).asString().c_str();
					ds = thr->getDiscHandle();
					res &= thr->writeMatToFile(ds->b[bn], fname.c_str());
				}
				else if (arg == "MU") {
					gs = thr->getGaussHandle();
					res &= thr->writeMatToFile(gs->MU, fname.c_str());
				}
				else if (arg == "R") {
					gs = thr->getGaussHandle();
					res &= thr->writeMatToFile(gs->R, fname.c_str());
				}
				else {
					reply.add(-1);
				}
				if (res && reply.size() == 0) {
					reply.add(1);
				}
				else if (!res && reply.size() == 0) {
					reply.add(-1);
				}
			}
		}
		else if (msg == "gen") {
			if (command.size() != 2) {
				reply.add(-1);
			}
			else {
				int arg = command.get(1).asInt();
				yarp::sig::Vector gendSamp(0);
				thr->genFromModel(gendSamp, arg);
				for (int i = 0; i < gendSamp.size(); i++) {
					reply.addDouble(gendSamp[i]);
				}
			}
		}
		else if (msg == "reset") {
			hs = thr->getHMMHandle();
			hs->reset();
			reply.add(1);
		}
		else {
			reply.add(-1);
		}


		return true;

	}

	virtual bool configure(ResourceFinder &rf) {

		Time::turboBoost();

		//set up the rpc port
		name=rf.check("name",Value("hmmRMLE")).asString().c_str();
		rpcPort = new Port;
		string portRpcName="/"+name+"/rpc";
		rpcPort->open(portRpcName.c_str());
		attach(*rpcPort);

		thr=new HmmRmleThread(rf);
		if (!thr->start())
		{
			delete thr;
			return false;
		}

		return true;

	}

	virtual bool close() {

		thr->stop();
		delete thr;

		rpcPort->interrupt();
		delete rpcPort;

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

	srand(time(NULL));
	gsl_rng_default_seed = time(NULL);

	HmmRmleModule mod;

	ResourceFinder rf;

	rf.configure("ICUB_ROOT", argc, argv);

	return mod.runModule(rf);
}
