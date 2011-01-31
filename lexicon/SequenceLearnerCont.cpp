#include "SequenceLearnerCont.h"

SequenceLearnerCont::SequenceLearnerCont(int r_, int d_, int b_, int epochs_, double thresh_, double alpha_, double xi_, double prior_, double eps_)
	: d(d_), alpha(alpha_), xi(xi_), SequenceLearner(r_, b_, epochs_, thresh_, prior_, eps_) {

	//allocate everything
	allocator = new Allocator;
	p = new HMM * [b];
	obs_dist = new Gaussian * [b];
	nInitialized = 0;
	initData.resize(1,1);
	pi = new IVec * [b];

	//hyperparameter defaults
	stent = true;

	//initialize classifiers
	init();

}

SequenceLearnerCont::SequenceLearnerCont(int r_, int d_, int b_, int epochs_, double thresh_, double prior_, double eps_)
	: d(d_), SequenceLearner(r_, b_, epochs_, thresh_, prior_, eps_) {

	//allocate everything
	allocator = new Allocator;
	p = new HMM * [b];
	obs_dist = new Gaussian * [b];
	nInitialized = 0;
	initData.resize(1,1);
	pi = new IVec * [b];


	//hyperparameter defaults
	stent = false;

	//initialize classifiers
	init();

}

SequenceLearnerCont::~SequenceLearnerCont(){

}



void SequenceLearnerCont::init() {

	//initialize the banks randomly
	for (int i = 0; i < b; i++) {
		obs_dist[i] = new(allocator) Gaussian(r,d,true);
		p[i] = new(allocator) HMM;
		p[i]->init(obs_dist[i], NULL, false, prior, 1, false, false);
		pi[i] = new IVec;
		pi[i]->resize(r);
		obs_dist[i]->eps0 = eps;
		p[i]->eps0 = eps;
	}

}

int SequenceLearnerCont::train(real ** samples, int length) {

	int z = length;
	int lMaxIdx = classify(samples, z);
	double bestScore;
	if (lMaxIdx == -1) {
		bestScore = -1.7e+308;
	} else {
		bestScore = evaluate(samples,z,lMaxIdx);
	}

	//if no winners (thresholded) present, re-initialize a new HMM to that sequence
	if (bestScore <= lThresh && nInitialized < b) {

		initialize(samples, z, nInitialized);

		lMaxIdx = nInitialized;
		nInitialized++;

	}
	else {

		//if a winner is present, it trains on the sequence
		for (int i = 0; i < epochs; i++) {
			for (int j = 0; j < z; j++) {
				p[lMaxIdx]->Classify(samples[j]);
				p[lMaxIdx]->RMLEUpdate();
				if (stent) {
					obs_dist[lMaxIdx]->covReg(alpha,xi);
				}
			}
		}

	}

	//apply decays to eps after each model update
	p[lMaxIdx]->eps0 = p[lMaxIdx]->eps0*eps_decay;
	obs_dist[lMaxIdx]->eps0 = obs_dist[lMaxIdx]->eps0*eps_decay;

	return lMaxIdx;

}


int SequenceLearnerCont::classify(real ** samples, int length) {

	int lMaxIdx;
	IVec L;
	L.resize(nInitialized);

	//make an ML classification
	for (int i = 0; i < nInitialized; i++) {
		L[i] = evaluate(samples, length, i);
	}

	//find the max likelihood
	L.vmax(&lMaxIdx);

	return lMaxIdx;

}


double SequenceLearnerCont::evaluate(real ** samples, int length, int n) {

	int z = length;
	double likelihood;

	//reset the initial internal probabilities
	likelihood = 0;
	VecCopy(pi[n],p[n]->prob);

	//make an ML classification
	for (int j = 0; j < z; j++) {
		p[n]->Classify(samples[j]);
		likelihood +=log10(1.0/p[n]->scale)*(double)(1.0/(z+1));
	}

	return likelihood;

}

void SequenceLearnerCont::printAll() {

	for (int i = 0; i < nInitialized; i++) {
		p[i]->print(500,10);
		pi[i]->print(500,10);
	}
}

int SequenceLearnerCont::initialize(real ** samples, int length, int n) {

	int z = length;
	double thisLikelihood = -1.7e+308;


	while (isnan(thisLikelihood) || thisLikelihood <= lThresh) {

		//reinitialize until it doesnt return NAN again (hackish?)
		initData.resize(z,d);
		for (int j = 0; j < z; j++) {
			for (int l = 0; l < d; l++) {
				initData(j,l) = samples[j][l];
			}
		}
		obs_dist[n]->reset();
		obs_dist[n]->KMeansInit(&initData,true,1000);

		if (stent) {
			obs_dist[n]->covReg(alpha,xi);
		}

		p[n]->reset();
		p[n]->init(obs_dist[n], NULL, false, prior, 1, false, false);

		//reset the initial internal probabilities and train
		p[n]->prob->fill(1.0/(float)r);
		for (int i = 0; i < epochs; i++) {
			for (int j = 0; j < z; j++) {
				p[n]->Classify(samples[j]);
				p[n]->RMLEUpdate();
				if (stent) {
					obs_dist[n]->covReg(alpha,xi);
				}
			}
		}

		//set the pi vector accord to the first state's class
		int initClass = obs_dist[n]->Classify(samples[0]);
		for (int i = 0; i < r; i++) {
			if (initClass == i) {
				pi[n]->ptr[i] = 1-prior*(r-1);
			} else {
				pi[n]->ptr[i] = prior;
			}
		}

		//reset again
		VecCopy(pi[n],p[n]->prob);

		//make an ML classification
		thisLikelihood = evaluate(samples, z, n);

	}

	return 1;

}

void SequenceLearnerCont::printToFile(string baseName) {

	FILE * params;

	for (int i = 0; i < nInitialized; i++) {

		char tnum[10];
		string aFile(baseName);
		string muFile(baseName);
		string uFile(baseName);

		sprintf(tnum,"%d",i+1);

		//A
		aFile += tnum;
		aFile += ".A";
		params = fopen(aFile.c_str(),"w");
		for (int j = 0; j < p[i]->A->m; j++) {
			for (int k = 0; k < p[i]->A->n; k++) {
				fprintf(params,"%f,",p[i]->A->ptr[j][k]);
			}
			fprintf(params,"\n");
		}
		fclose(params);

		//mu
		muFile += tnum;
		muFile += ".MU";
		params = fopen(muFile.c_str(),"w");
		for (int j = 0; j < obs_dist[i]->MU->m; j++) {
			for (int k = 0; k < obs_dist[i]->MU->n; k++) {
				fprintf(params,"%f,",obs_dist[i]->MU->ptr[j][k]);
			}
			fprintf(params,"\n");
		}
		fclose(params);

		//U
		IMat U(r,d*d);

		IMat Rtmp;
		IMat Utmp;

		for (int l = 0; l < r; l++)
		{
			obs_dist[i]->R->getRow(l, 0, d, d, &Rtmp);
			U.getRow(l, 0, d, d, &Utmp);
			MatMatMult(&Rtmp, CblasTrans, &Rtmp, CblasNoTrans, &Utmp);
		}

		uFile += tnum;
		uFile += ".U";
		params = fopen(uFile.c_str(),"w");
		for (int j = 0; j < U.m; j++) {
			for (int k = 0; k < U.n; k++) {
				fprintf(params,"%f,",U.ptr[j][k]);
			}
			fprintf(params,"\n");
		}
		fclose(params);
	}
}

void SequenceLearnerCont::packObs(Bottle &dst, int n) {

	//pack MU
	Bottle &mu = dst.addList();
	for (int j = 0; j < obs_dist[n]->MU->m; j++) {
		Bottle &tmp = mu.addList();
		for (int k = 0; k < obs_dist[n]->MU->n; k++) {
			tmp.add(obs_dist[n]->MU->ptr[j][k]);
		}
	}

	//pack U
	IMat U(r,d*d);
	IMat Rtmp;
	IMat Utmp;
	Bottle &u = dst.addList();
	for (int l = 0; l < r; l++)
	{
		obs_dist[n]->R->getRow(l, 0, d, d, &Rtmp);
		U.getRow(l, 0, d, d, &Utmp);
		MatMatMult(&Rtmp, CblasTrans, &Rtmp, CblasNoTrans, &Utmp);
	}
	for (int j = 0; j < U.m; j++) {
		Bottle &tmp = u.addList();
		for (int k = 0; k < U.n; k++) {
			tmp.add(U.ptr[j][k]);
		}
	}

}
