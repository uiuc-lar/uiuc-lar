#include "SequenceLearner.h"

SequenceLearner::SequenceLearner(int r_, int d_, int b_, int epochs_, double thresh_, double alpha_, double xi_)
	: r(r_), d(d_), b(b_), epochs(epochs_), lThresh(thresh_), alpha(alpha_), xi(xi_) {

	//allocate everything
	allocator = new Allocator;
	p = new HMM * [b];
	obs_dist = new Gaussian * [b];
	likelihood = new real[b];
	nInitialized = 0;
	initData.resize(1,1);
	pi = new IVec * [b];


	//hyperparameter defaults
	prior = 0.0000;
	//lThresh = -2.0;
	//epochs = 10;
	//alpha = 0.005;
	//xi = 1.0;

	//initialize classifiers
	init();

}


SequenceLearner::~SequenceLearner(){

}



void SequenceLearner::init() {

	//initialize the banks randomly
	for (int i = 0; i < b; i++) {
		obs_dist[i] = new(allocator) Gaussian(r,d,true);
		p[i] = new(allocator) HMM;
		p[i]->init(obs_dist[i], NULL, false, prior, 1, false, false);
		pi[i] = new IVec;
		pi[i]->resize(r);
	}

}

int SequenceLearner::train(real ** samples, int length) {
	FILE *fp;
	fp = fopen("probtrace.dat","w");
	int z = length;
	int lMaxIdx = classify(samples, z);

	//if no winners (thresholded) present, re-initialize a new HMM to that sequence
	if (lMaxIdx > b && nInitialized < b) {

		double thisLikelihood = -1.7e+308;
		while (isnan(thisLikelihood) || thisLikelihood <= lThresh) {

			//reinitialize until it doesnt return NAN again (hackish?)
			initData.resize(z,d);
			for (int j = 0; j < z; j++) {
				for (int l = 0; l < d; l++) {
					initData(j,l) = samples[j][l];
					//printf("%f ",samples[j][l]);
				}
				//printf("\n");
			}
			obs_dist[nInitialized]->reset();
			obs_dist[nInitialized]->KMeansInit(&initData,true,50);
			obs_dist[nInitialized]->covReg(alpha,xi);
			obs_dist[nInitialized]->print(5000,10);
			p[nInitialized]->reset();
			p[nInitialized]->init(obs_dist[nInitialized], NULL, false, prior, 1, false, false);


			//obs_dist[nInitialized]->print(100,10);
			//reset the initial internal probabilities and train
			for (int l = 0; l < r; l++) {
				//p[nInitialized]->prob->ptr[l] = 1.0/(double)r;
				p[nInitialized]->prob->ptr[l] = 1.0;
			}
			for (int i = 0; i < epochs; i++) {
				for (int j = 0; j < z; j++) {
					p[nInitialized]->Classify(samples[j]);
					for (int k = 0; k < r; k++) {
						fprintf(fp,"%f,",p[nInitialized]->f->ptr[k]);
					}
					fprintf(fp,"\n");
					p[nInitialized]->RMLEUpdate();
					obs_dist[nInitialized]->covReg(alpha,xi);
				}
			}

			fclose(fp);
			//set the pi vector accord to the first state's class
			int initClass = obs_dist[nInitialized]->Classify(samples[0]);
			for (int i = 0; i < r; i++) {
				if (initClass == i) {
					pi[nInitialized]->ptr[i] = 1-prior*r;
				} else {
					pi[nInitialized]->ptr[i] = prior;
				}
			}

			//reset again
			for (int l = 0; l < r; l++) {
				p[nInitialized]->prob->ptr[l] = pi[nInitialized]->ptr[l];
			}

			//make an ML classification
			thisLikelihood = 0;
			for (int j = 0; j < z; j++) {
				p[nInitialized]->Classify(samples[j]);
				thisLikelihood +=log10(1.0/p[nInitialized]->scale)*(double)(1.0/(z+1));
			}
			//printf("%f \n",thisLikelihood);
			//obs_dist[nInitialized]->print();

		}
		lMaxIdx = nInitialized;
		nInitialized++;
		//reset again
		for (int l = 0; l < r; l++) {
			p[lMaxIdx]->prob->ptr[l] = 1.0/(double)r;
		}
		for (int i = 0; i < epochs; i++) {
			for (int j = 0; j < z; j++) {
				//p[nInitialized]->Classify(samples[j]);
				//p[nInitialized]->RMLEUpdate();
				p[lMaxIdx]->Classify(samples[j]);
				p[lMaxIdx]->RMLEUpdate();
				obs_dist[lMaxIdx]->covReg(alpha,xi);
			}
		}

	}
	else {
		//if a winner is present, it trains on the sequence
		for (int i = 0; i < epochs; i++) {
			for (int j = 0; j < z; j++) {
				p[lMaxIdx]->Classify(samples[j]);
				p[lMaxIdx]->RMLEUpdate();
				obs_dist[lMaxIdx]->covReg(alpha,xi);
			}
		}
	}

	return lMaxIdx;

}


int SequenceLearner::classify(real ** samples, int length) {

	int lMaxIdx = b+1;
	double lMax = -1.7e+300;
	int z = length;

	//reset the initial internal probabilities
	for (int k = 0; k < nInitialized; k++) {
		likelihood[k] = 0;
		for (int l = 0; l < r; l++) {
			p[k]->prob->ptr[l] = pi[k]->ptr[l];
		}
	}
	//this->printAll();
	//make an ML classification
	for (int j = 0; j < z; j++) {
		//over all HMMs in the bank
		for (int k = 0; k < nInitialized; k++) {
			//TODO:
			p[k]->Classify(samples[j]);
			//if (j == 0) { printf("%f ",log10(1.0/p[k]->scale)*(double)(1.0/(z+1))); }
			likelihood[k] +=log10(1.0/p[k]->scale)*(double)(1.0/(z+1));
		}
	}

	for (int k = 0; k < nInitialized; k++) {
		printf("%f ",likelihood[k]);
		if (likelihood[k] > lMax && likelihood[k] > lThresh) {
			lMax = likelihood[k];
			lMaxIdx = k;
		}
	}
	printf("%d \n", lMaxIdx);

	return lMaxIdx;

}


double SequenceLearner::evaluate(real ** samples, int length, int n) {

	int lMaxIdx = b+1;
	double lMax = -1.7e+300;
	int z = length;

	//reset the initial internal probabilities
	for (int k = 0; k < nInitialized; k++) {
		likelihood[k] = 0;
		for (int l = 0; l < r; l++) {
			p[k]->prob->ptr[l] = 1.0/(double)r;
		}
	}

	//make an ML classification
	for (int j = 0; j < z; j++) {
		//over all HMMs in the bank
		for (int k = 0; k < nInitialized; k++) {
			p[k]->Classify(samples[j]);
			likelihood[k] +=log10(1.0/p[k]->scale)*(double)(1.0/(z+1));
		}
	}

	for (int k = 0; k < nInitialized; k++) {
		printf("%f ",likelihood[k]);
		if (likelihood[k] > lMax && likelihood[k] > lThresh) {
			lMax = likelihood[k];
			lMaxIdx = k;
		}
	}
	printf("%d \n", lMaxIdx);

	return lMax;

}

void SequenceLearner::printAll() {

	for (int i = 0; i < nInitialized; i++) {
		p[i]->print(500,10);
		pi[i]->print(500,10);
	}
}

int SequenceLearner::initialize(real ** samples, int length, int n) {

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
		obs_dist[n]->covReg(alpha,xi);
		//obs_dist[n]->print(100,10);
		p[n]->reset();
		p[n]->init(obs_dist[n], NULL, false, prior, 1, false, false);



		//reset the initial internal probabilities and train
		for (int l = 0; l < r; l++) {
			p[n]->prob->ptr[l] = 1.0/(double)r;
		}
		for (int i = 0; i < epochs; i++) {
			for (int j = 0; j < z; j++) {
				p[n]->Classify(samples[j]);
				p[n]->RMLEUpdate();
				obs_dist[n]->covReg(alpha,xi);
			}
		}

		//obs_dist[n]->print(100,10);

		//set the pi vector accord to the first state's class
		int initClass = obs_dist[n]->Classify(samples[0]);
		for (int i = 0; i < r; i++) {
			if (initClass == i) {
				pi[n]->ptr[i] = 1-prior*r;
			} else {
				pi[n]->ptr[i] = prior;
			}
		}

		//reset again
		for (int l = 0; l < r; l++) {
			p[n]->prob->ptr[l] = pi[n]->ptr[l];
		}

		//make an ML classification
		thisLikelihood = 0;
		for (int j = 0; j < z; j++) {
			p[n]->Classify(samples[j]);
			thisLikelihood +=log10(1.0/p[n]->scale)*(double)(1.0/(z+1));
		}
	}

	return 1;

}

void SequenceLearner::printToFile(string baseName) {

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
