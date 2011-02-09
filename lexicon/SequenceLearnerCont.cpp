#include "SequenceLearnerCont.h"

SequenceLearnerCont::SequenceLearnerCont(int r_, int d_, int b_, int epochs_, double thresh_, double prior_, double eps_, double alpha_, double xi_, bool makeLR_)
: d(d_), alpha(alpha_), xi(xi_), makeLR(makeLR_), SequenceLearner(r_, b_, epochs_, thresh_, prior_, eps_) {

	//allocate everything
	allocator = new Allocator;
	p = new HMM * [b];
	obs_dist = new Gaussian * [b];
	nInitialized = 0;
	initData.resize(1,1);
	pi = new IVec * [b];
	exemplar_initPos = new IVec * [b];
	exemplar_length = new int[b];


	//hyperparameter defaults
	stent = true;

	//initialize classifiers
	init();

}

SequenceLearnerCont::SequenceLearnerCont(int r_, int d_, int b_, int epochs_, double thresh_, double prior_, double eps_, bool makeLR_)
: d(d_), makeLR(makeLR_), SequenceLearner(r_, b_, epochs_, thresh_, prior_, eps_) {

	//allocate everything
	allocator = new Allocator;
	p = new HMM * [b];
	obs_dist = new Gaussian * [b];
	nInitialized = 0;
	initData.resize(1,1);
	pi = new IVec * [b];
	exemplar_initPos = new IVec * [b];
	exemplar_length = new int[b];


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
		exemplar_initPos[i] = new IVec;
		exemplar_initPos[i]->resize(d);
		exemplar_length[i] = 0;
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
				if (makeLR) {
					p[lMaxIdx]->RMLEUpdate(false);
				} else {
					p[lMaxIdx]->RMLEUpdate(true);
				}
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

		//this is only a pseudo-LR init. don't constrain the A matrix
		if (makeLR) {

			int spacing = length/r;
			for (int i = 0; i < r; i++) {
				for (int j = 0; j < d; j++) {
					obs_dist[n]->MU->ptr[i][j] = samples[i*spacing+(spacing/2)][j];
				}
			}

			obs_dist[n]->KMeansInit(&initData,false,1000);

		} else {

			obs_dist[n]->KMeansInit(&initData,true,1000);

		}

		if (stent) {
			obs_dist[n]->covReg(alpha,xi);
		}

		p[n]->reset();
		p[n]->init(obs_dist[n], NULL, false, prior, 1, false, false);

		//if set, initialize the A matrix to be slightly biased to L-R
		if (makeLR) {
			for (int i = 0; i < r; i++) {
				for (int j = 0; j < r; j++) {
					if (i == j) {
						p[n]->A->ptr[i][j] = 0.9;
					}
					else if (j == i+1) {
						p[n]->A->ptr[i][j] = 0.1;
					}
					else {
						p[n]->A->ptr[i][j] = 0.0;
					}
				}
			}
			p[n]->ProbProject(p[n]->A, prior, 2);
		}

		//reset the initial internal probabilities and train
		p[n]->prob->fill(1.0/(float)r);
		for (int i = 0; i < epochs; i++) {
			for (int j = 0; j < z; j++) {
				p[n]->Classify(samples[j]);
				if (makeLR) {
					p[n]->RMLEUpdate(false);
				} else {
					p[n]->RMLEUpdate(true);
				}
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

	exemplar_length[n] = length;
	exemplar_initPos[n]->set(samples[0],d,1,true);

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

/*
 * 	generates action from an HMM encoding, currently only works for
 * 		L-R HMMs, with observations coded as joint X, X_hat probs. based on:
 *
 * 	Calinon, S., D'halluin, F., Sauser, E.L., Caldwell, D.G. and Billard, A.G. (2010).
 * 	Learning and reproduction of gestures by imitation: An approach based on Hidden
 * 		Markov Model and Gaussian Mixture Regression.
 * 	IEEE Robotics and Automation Magazine, 17:2, 44-54.
 *
 * 	important note: this function assumes a particular structure about the data,
 * 	namely that the first d/2 samples are xyz, and next d/2 are their derivatives.
 *
 */
bool SequenceLearnerCont::generateSequence(IMat &data, int n, double dscale) {

	IVec cpos, mx, mxd, temp;
	IMat U(r,d*d);
	IMat ** Sx = new IMat *[r];
	IMat ** Sxi = new IMat *[r];
	IMat ** Sxdx = new IMat *[r];

	//sanity check
	if (n > nInitialized-1) {
		printf("Requested samples from an uninitialized bank\n");
		return false;
	}

	//set up data matrices
	IMat H, Xd;
	data.resize(exemplar_length[n],d/2);
	Xd.resize(exemplar_length[n],d/2);
	H.resize(exemplar_length[n],r);
	H.zero();
	data.zero();
	Xd.zero();

	//expand all the covariance matrices and stuff at the beginning
	// (sacrificing a bit of memory here)
	IMat Rtmp;
	IMat Utmp;
	IMat Stmp;
	for (int i = 0; i < r; i++)
	{

		obs_dist[n]->R->getRow(i, 0, d, d, &Rtmp);
		U.getRow(i, 0, d, d, &Utmp);
		MatMatMult(&Rtmp, CblasTrans, &Rtmp, CblasNoTrans, &Utmp);

		Sx[i] = new IMat;
		Sxi[i] = new IMat;
		Sxdx[i] = new IMat;

		U.getRow(i, 0, d, d, &Stmp);
		Stmp.getSubMat(0,0,d/2-1,d/2-1,Sx[i],true);
		Stmp.getSubMat(d/2,0,d-1,d/2-1,Sxdx[i],true);
		SymMatCholFact(Sx[i]);
		Sxi[i]->resize(Sx[i]->m,Sx[i]->n);
		MatCopy(Sx[i],Sxi[i]);
		MatCholInv(Sxi[i]);

	}

	//set initial conditions
	IVec initPos(d/2);
	initPos.set(exemplar_initPos[n]->ptr,d/2,1,true);
	data.setRow(0,&initPos);
	H.setRow(0,pi[n]);

	//recursively generate
	for (int i = 0; i < exemplar_length[n]-1; i++) {

		//calculate Xd
		IVec cvel(d/2);
		cvel.zero();

		for (int j = 0; j < r; j++) {

			data.getRow(i,&cpos,true);
			obs_dist[n]->MU->getRow(j,0,d/2,&mx,true);
			obs_dist[n]->MU->getRow(j,d/2,d/2,&mxd,true);
			VecSub(&mx,&cpos);
			temp.resize(d/2);
			SymMatVecMult(1.0,Sxi[j],&cpos,0.0,&temp);
			MatVecMult(Sxdx[j],CblasNoTrans,&temp,&cpos);
			VecAdd(&mxd,&cpos);
			VecAddScaled(H(i,j),&cpos,&cvel);

		}

		//update position
		data.getRow(i,&cpos,true);
		VecAddScaled(dscale,&cvel,&cpos);
		data.setRow(i+1,&cpos);

		//update pmf
		IVec prob(r);
		prob.zero();
		for (int j = 0; j < r; j++) {

			obs_dist[n]->MU->getRow(j,0,d/2,&mx,true);
			prob(j) = N(&cpos,&mx,Sx[j],NULL,NULL);

		}
		prob.normalize();
		H.setRow(i+1,&prob);

	}

	//cleanup
	for (int i = 0; i < r; i++) {
		delete Sx[i];
		delete Sxi[i];
		delete Sxdx[i];
	}
	delete Sx;
	delete Sxi;
	delete Sxdx;


}

