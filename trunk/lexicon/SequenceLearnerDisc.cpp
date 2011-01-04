#include "SequenceLearnerDisc.h"

SequenceLearnerDisc::SequenceLearnerDisc(int r_, int * d_, int n_, int b_, int epochs_, double thresh_, bool makeLR_, double prior_, double eps_)
	: d(d_), nOuts(n_), makeLR(makeLR_), SequenceLearner(r_, b_, epochs_, thresh_, prior_, eps_) {

	//allocate everything
	allocator = new Allocator;
	p = new HMM * [b];
	//obs_dist = new Gaussian * [b];
	obs_dist = new IndepPMF * [b];
	//likelihood = new real[b];
	nInitialized = 0;
	initData.resize(1,1);
	pi = new IVec * [b];

	//initialize classifiers
	init();

}


SequenceLearnerDisc::~SequenceLearnerDisc(){

}



void SequenceLearnerDisc::init() {

	//initialize the banks randomly
	for (int i = 0; i < b; i++) {
		obs_dist[i] = new(allocator) IndepPMF(r, nOuts, d, prior, true);
		p[i] = new(allocator) HMM;
		p[i]->init(obs_dist[i], NULL, false, prior, 1, false, false);
		pi[i] = new IVec;
		pi[i]->resize(r);
		pi[i]->fill((float)(1.0/r));
		obs_dist[i]->eps0 = eps;
		obs_dist[i]->history = 10;
		p[i]->eps0 = eps;
	}

}

int SequenceLearnerDisc::train(int ** samples, int length) {

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

	} else {

		//if a winner is present, it trains on the sequence
		for (int i = 0; i < epochs; i++) {
			VecCopy(pi[lMaxIdx],p[lMaxIdx]->prob);
			for (int j = 0; j < z; j++) {
				p[lMaxIdx]->Classify(samples[j]);
				p[lMaxIdx]->RMLEUpdate();
				if (makeLR) {
					makeALR(lMaxIdx);
				}
			}
		}

	}

	//apply decays to eps after each model update
	p[lMaxIdx]->eps0 = p[lMaxIdx]->eps0*eps_decay;
	obs_dist[lMaxIdx]->eps0 = obs_dist[lMaxIdx]->eps0*eps_decay;

	return lMaxIdx;

}


int SequenceLearnerDisc::classify(int ** samples, int length) {

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


double SequenceLearnerDisc::evaluate(int ** samples, int length, int n) {

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

void SequenceLearnerDisc::printAll() {

	for (int i = 0; i < nInitialized; i++) {
		p[i]->print(500,10);
		pi[i]->print(500,10);
	}
}

int SequenceLearnerDisc::initialize(int ** samples, int length, int n) {

	int z = length;
	double thisLikelihood = -1.7e+308;

	while (isnan(thisLikelihood) || thisLikelihood <= lThresh) {

		//reinitialize until it doesnt return NAN again (hackish?)
		obs_dist[n]->reset();
		p[n]->reset();
		p[n]->init(obs_dist[n], NULL, false, prior, 1, false, false);

		//do some special things if this should be treated as a LtR model
		if (makeLR) {
			//start the parameter values in a friendly spot to make L-R model converge properly
			for (int i = 0; i < r; i++) {

				//use a sliding window histogram method for B matrix
				int * symbs = new int[d[0]];
				for (int j = 0; j < d[0]; j++) {
					symbs[j] = 0;
				}
				int dl = i*(length/r);
				int dh = min((i+1)*(length/r),length);

				for (int j = dl; j < dh; j++) {
					symbs[samples[j][0]] = symbs[samples[j][0]] + 1;
				}
				for (int j = 0; j < d[0]; j++) {
					//update probs properly
					obs_dist[n]->b[0]->ptr[j][i] = ((double)symbs[j])/((double)(dh-dl));
				}
				//enforce conditions for A matrix
				for (int j = 0; j < r; j++) {
					if (j != i && j != (i+1)) {
						p[n]->A->ptr[i][j] = 0;
					}
				}
				delete symbs;
			}
			//enforce condition on pi
			pi[n]->ptr[0] = 1-(r-1)*prior;
			for (int i = 1; i < r; i++) {
				pi[n]->ptr[i] = prior;
			}
			//fix matrices back up (enforce constraints, fix priors)
			ProbProject(p[n]->A, 2);
			ProbProject(obs_dist[n]->b[0],1);
		}

		//reset the initial internal probabilities and train
		for (int i = 0; i < epochs; i++) {
			//after each epoch, reinit the pi vector
			VecCopy(pi[n],p[n]->prob);
			for (int j = 0; j < z; j++) {
				p[n]->Classify(samples[j]);
				p[n]->RMLEUpdate();
				if (makeLR) {
					makeALR(n);
				}
			}
		}

		//reset again
		VecCopy(pi[n],p[n]->prob);

		//make an ML classification
		thisLikelihood = evaluate(samples, z, n);

	}

	return 1;

}

void SequenceLearnerDisc::makeALR(int n) {

	for (int i = 0; i < r; i++) {
		//enforce conditions for A matrix
		for (int j = 0; j < r; j++) {
			if (j != i && j != (i+1)) {
				p[n]->A->ptr[i][j] = 0;
			}
		}
	}
	ProbProject(p[n]->A, 2);

}


int SequenceLearnerDisc::ProbProject(IMat *Q_m,
                                      int dim)
{
   int i,j;

   real **Q = Q_m->ptr;

   int rows = Q_m->m;
   int cols = Q_m->n;

   int min_count;
//   int *min_pos;
   IVecInt min_pos;

   int repeat, rep_count;

   if (dim == 1)
   {
      real col_sum;
      real col_add;

      if (prior > ((real)1.0)/rows)
         prior = ((real)1.0)/rows;

//      min_pos = (int *)allocator->alloc(sizeof(int)*rows);
      min_pos.resize(rows);

      for (j = 0; j < cols; j++)
      {
         min_count = 0;
//         memset((void *)min_pos, 0, rows*sizeof(int));
         min_pos.zero();

         rep_count = 0;

         do
         {
            repeat = 0;
            col_sum = 0;

            for (i = 0; i < rows; i++)
               col_sum += Q[i][j];

            col_add = (1.0-col_sum)/(rows-min_count);

            for (i = 0; i < rows; i++)
            {
               if (!min_pos(i))
               {
                  //Q[i][j] /= col_sum;
            	  Q[i][j] += col_add;
                  if (Q[i][j] < prior)
                  {
                     Q[i][j] = prior;
                     min_pos(i) = 1;
                     min_count++;
                     repeat = 1;
                  }
               }
            }


            rep_count++;  // idiot checking

         } while (repeat && rep_count <= cols);
      }
   }
   else
   {
      real row_sum;
      real row_add;

      if (prior > ((real)1.0)/cols)
         prior = ((real)1.0)/cols;

//      min_pos = (int *)allocator->alloc(sizeof(int)*cols);
      min_pos.resize(cols);

      for (i = 0; i < rows; i++)
      {
         min_count = 0;
//         memset((void *)min_pos, 0, cols*sizeof(int));
         min_pos.zero();

         rep_count = 0;

         do
         {
            repeat = 0;
            row_sum = 0;

            for (j = 0; j < cols; j++)
               row_sum += Q[i][j];

            row_add = (1.0-row_sum)/(cols-min_count);

            for (j = 0; j < cols; j++)
            {
               if (!min_pos(j))
               {
                  Q[i][j] += row_add;
                  if (Q[i][j] < prior)
                  {
                     Q[i][j] = prior;
                     min_pos(j) = 1;
                     min_count++;
                     repeat = 1;
                  }
               }
            }

            rep_count++;  // idiot checking

         } while (repeat && rep_count <= cols);

      }
   }

   return 0;
}

void SequenceLearnerDisc::printToFile(string baseName) {

	FILE * params;

	for (int i = 0; i < nInitialized; i++) {

		char tnum[10];
		string aFile(baseName);
		string bFile(baseName);
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

		//B
		bFile += tnum;
		bFile += ".B";
		params = fopen(bFile.c_str(),"w");
		for (int j = 0; j < obs_dist[i]->b[0]->m; j++) {
			for (int k = 0; k < obs_dist[i]->b[0]->n; k++) {
				fprintf(params,"%f,",obs_dist[i]->b[0]->ptr[j][k]);
			}
			fprintf(params,"\n");
		}
		fclose(params);

	}

}
