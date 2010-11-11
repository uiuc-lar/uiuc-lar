/**
 * @file   Gaussian.cc
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Thu Jul  8 14:43:44 2004
 * 
 * @brief  Gaussian Mixture Model
 * 
 * 
 * $Log: Gaussian.cc,v $
 * Revision 1.1  2005/05/09 20:52:57  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.5.2.2  2004/11/07 23:49:40  k-squire
 * Renamed epsilon to eps0.
 * Added epsilon decay value eps_exp.
 * Removed projection code; data dimension should be reduced before it
 *    gets to us.
 * Added kmeans-based constructor.
 * Fixed naming of U and R; U usually refers to the covariance matrix,
 *    whereas R refers to the Cholesky decomposition of this matrix.
 *
 * Revision 1.5.2.1  2004/10/24 21:11:10  k-squire
 * Changed all reference of "rotation" to "projection" to more accurately
 *   describe the projection file. :-)
 * Fixed an off-by-one bug when setting the lower triangle of a matrix to zero.
 * Added debugging to Classify()
 * Modified KMeans init to allow use of projection file.
 *
 * Revision 1.5  2004/08/11 03:09:14  k-squire
 * Merged changes from KEVIN_DEVEL branch
 *
 * Revision 1.4.2.1  2004/08/11 03:00:13  k-squire
 * Changed IMatLib to imatlib.
 *
 * Revision 1.4  2004/08/11 01:15:28  k-squire
 * Changed "matrix/*.h" to "IMatLib/*.h"
 *
 * Revision 1.3  2004/08/10 23:49:11  k-squire
 * Backported changes from illy/hmm.
 * Changed hmm_run to conditionally depend on libIBase, IServer stuff.
 *
 * Revision 1.1.2.3  2004/08/10 22:12:00  k-squire
 * Fixed bug: force data_len to be set even if not using projection.
 *
 * Revision 1.1.2.2  2004/08/08 22:49:12  k-squire
 * Added ability to set class labels.
 * Added reset() function.
 *
 * Revision 1.1.2.1  2004/08/04 06:48:40  k-squire
 * Changed "rotation" to "projection" when referring to projection matrix.
 *   (Semantics, but it was incorrect before.)
 * Added a local allocator to make it easy to delete all locally allocated
 *   material.
 * Added private function addOptions(), to eliminate redundant code.
 * Set lower triangle of matrix to zero on init after Cholesky factorization.
 * Fixed save and load functions to actually save and load covariance matrix.
 *
 * Revision 1.1  2004/08/03 06:38:38  k-squire
 * Import and initial checkin.
 *
 * Revision 1.2  2004/08/03 03:40:28  k-squire
 * Added default class constructor.
 * Added ability to rotate input data according to a (pre-calculated)
 *    rotation/dimension reduction vector
 * Constructors/initializers now take covariance matrix U instead of its
 *    Cholesky decomposition R (though R is stored internally).
 * Added initialization from a filelist of classes and optional rotation
 *    matrix file.
 * Added kmeans init.
 * Added debug print functions.
 * Created and moved constructor stuff into init().
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 *
 */


static char *Gaussian_CC_Id = "$Id: Gaussian.cc,v 1.1 2005/05/09 20:52:57 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <torch/general.h>
#include <torch/Object.h>
#include <torch/Random.h>
#include <torch/DiskXFile.h>

#include "../imatlib/IMat.hh"
#include "../imatlib/IVec.hh"
#include "../imatlib/IVecInt.hh"

#include "Gaussian.hh"
#include "N.hh"
#include "gen_defines.h"

using namespace Torch;

Gaussian::Gaussian() : 
    		  StochasticClassifier(),
    		  MU(NULL),
    		  R(NULL),
    		  u_min(0.1),
    		  S_MU(NULL),
    		  S_MU_new(NULL),
    		  S_R(NULL),
    		  S_R_new(NULL),
    		  u(NULL),
    		  w_MU(NULL),
    		  w_MU_next(NULL),
    		  w_R(NULL),
    		  w_R_next(NULL),
    		  best_class(-1),
    		  R2_MU(NULL),
    		  R2_R(NULL),
    		  df_MU(NULL),
    		  df_R(NULL),
    		  scale(0.0),
    		  silence_state(-1),
    		  MU_temp(NULL),
    		  R_temp(NULL),
    		  df_MU_temp(NULL),
    		  df_R_temp(NULL)
    		  {
	G_allocator = new Allocator;
	addOptions();
    		  }



Gaussian::Gaussian(int r_, 
		int d_,
		bool rand_init,
		real mu_min,
		real mu_max,
		real u_min_):
		StochasticClassifier(r_, d_)
		{
	G_allocator = new Allocator;

	IMat *MU_ = new(G_allocator) IMat(r_, d_);
	IMat *U_ = new(G_allocator) IMat(r_, d_*d_);

	if (rand_init)
	{
		MU_->rand(mu_min, mu_max);
		U_->rand(u_min_, 10.0*u_min_);
	}
	else  // assume k-means or user init
	{
		MU_->zero();
		U_->zero();
	}

	addOptions();

	init(MU_, U_, u_min_);

		}


Gaussian::Gaussian(int r_,
		int d_,
		IMat *MU_,
		IMat *U_,
		real u_min_,
		bool do_copy,
		bool retain):
		StochasticClassifier(r_, d_)
		{
	G_allocator = new Allocator;

	addOptions();

	init(MU_, U_, u_min_, do_copy, retain);
		}

Gaussian::Gaussian(int r_,
		int d_,
		real *mu_,
		real *uu_,
		real u_min_,
		bool do_copy,
		bool retain):
		StochasticClassifier(r_, d_)
		{
	G_allocator = new Allocator;

	IMat *MU_ = new(G_allocator) IMat(mu_, r_, d_, do_copy, retain);
	IMat *U_ = new(G_allocator) IMat(uu_, r_, d_*d_, do_copy, retain);

	addOptions();

	init(MU_, U_, u_min_, false, false);
		}

Gaussian::Gaussian(int r_,
		int d_,
		IMat *MU__,
		IMat *kmeans_data,
		int max_iter,
		real u_min_,
		bool do_copy,
		bool retain):
		StochasticClassifier(r_, d_)
		{
	G_allocator = new Allocator;

	IMat *MU_;

	if (do_copy)
	{
		MU_ = new(G_allocator) IMat;
		*MU_ = *MU__;
	}
	else
	{
		MU_ = MU__;
		if (retain)
			G_allocator->retain(MU_, 1);
	}

	IMat *U_ = new(G_allocator) IMat(r_, d_*d_);

	addOptions();

	init(MU_, U_, u_min_, false, false);

	//KMeansInit(kmeans_data, false, max_iter);
	KMeansInit(kmeans_data, true, max_iter);
		}

Gaussian::~Gaussian()
{
	delete G_allocator;
}

void Gaussian::addOptions()
{
	// ** From StochasticClassifier.cc

	//    // learning options

	//    addIOption("history", &history, 1, "averaging history");

	//    addBOption("average S", &avg_obs, false, "average observations");
	//    addBOption("average parms", &avg_iters, false, "average iterages");

	//    addROption("eps0", &eps0, 0.001, "learning rate");
	//    addROption("eps_exp", &eps_exp, 0.000, "learning rate decay");
	//    addROption("prior", &prior, 0.0, "prior on weights");

	addROption("u min", &u_min, 0.01, "minimum value for diag. variances.");
	addIOption("silence_state", &silence_state, -1, "state to use for silence (not updated)");

}


int Gaussian::init(IMat *MU_,
		IMat *U_,
		real u_min_,
		bool do_copy,
		bool retain)
{
	int i, j;

	u_min = u_min_;

	// ***

	if (MU_)
	{
		if (do_copy)
			MU = new(G_allocator) IMat(*MU_);
		else
		{
			MU = MU_;
			if (retain)
				G_allocator->retain(MU,1);
		}
	}
	else
		MU = new(G_allocator) IMat(r,d);  // must be initialized!

		S_MU = new(G_allocator) IMat(r,d);
		S_MU->zero();

		S_MU_new = new(G_allocator) IMat(r,d);
		S_MU_new->zero();

		w_MU = new(G_allocator) IMat(r,r*d);
		w_MU->zero();
		w_MU_next = new(G_allocator) IMat(r,r*d);
		w_MU_next->zero();

		// ***

		if (U_)
		{
			if (do_copy)
				R = new(G_allocator) IMat(*U_);
			else
			{
				R = U_;
				if (retain)
					G_allocator->retain(R,1);
			}

			IMat U;
			IVec ltri;

			for (i = 0; i < r; i++)
			{
				R->getRow(i, 0, d, d, &U);
				SymMatCholFact(&U);

				// Set lower triangle to zero

				for (int j = -1; j >= -(d-1); j--)
				{
					U.getDiag(j, &ltri);
					ltri.zero();
				}
			}

		}
		else
			R = new(G_allocator) IMat(r,d*d); // must be initialized!

		S_R = new(G_allocator) IMat(r,d*d);
		S_R->zero();

		S_R_new = new(G_allocator) IMat(r,d*d);
		S_R_new->zero();

		w_R = new(G_allocator) IMat(r,r*d*d);
		w_R->zero();
		w_R_next = new(G_allocator) IMat(r,r*d*d);
		w_R_next->zero();

		log_prob = 0;
		best_class = -1;

		Rs = new(G_allocator) IMat(r,r);
		R1 = new(G_allocator) IMat(r,r);
		R2_MU = new(G_allocator) IMat(r,r*d);
		R2_MU->zero();
		R2_R = new(G_allocator) IMat(r,r*d*d);
		R2_R->zero();

		df_MU = new(G_allocator) IMat(r,r*d);
		df_MU->zero();
		df_R = new(G_allocator) IMat(r,r*d*d);
		df_R->zero();

		df_MU_temp = new(G_allocator) IMat(r,r*d);
		df_MU_temp->zero();
		df_R_temp = new(G_allocator) IMat(r,r*d*d);
		df_R_temp->zero();

		u = NULL;  // Later set to point to the u in the corresponding HMM

		MU_temp = new(G_allocator) IMat(r,d);
		R_temp = new(G_allocator) IMat(r,d*d);

		return 0;
}

int Gaussian::init(XFile *filelist,
		real u_min_)
{
	int i;
	char line[81],fn[81], cls_name[81];
	int r_ = 0, d_ = 0;
	int row1, col1, row2, col2;
	int cls;

	IMat **data;
	IMat data_tmp;

	IVec mu_tmp;
	IMat u_tmp;

	IVec mu;
	IMat u;

	IMat *MU_ = new(G_allocator) IMat;
	IMat *U_ = new(G_allocator) IMat;

	if (filelist->scanf("%d", &r_) != 1)
		error("Error: can't read number of classes from file to initialize Gaussians.");

	if (filelist->scanf("%d", &d_) != 1)
		error("Error: can't read dimensionality from file to initialize Gaussians.");

	data_len = d_;
	StochasticClassifier::init(r_,d_);  // this sets r and d

	MU_->reshape(r,d);
	U_->reshape(r,d*d);

	Allocator dallocator;  // memory created with dallocator will be
	// freed when routine exits

	data = (IMat **)dallocator.alloc(sizeof(IMat *)*r);

	for (i = 0; i < r; i++)
		data[i] = new(&dallocator) IMat;

	while (1)
	{
		tryagain:
		fn[0] = '\0';

		if (filelist->gets(line,81) == NULL)
			break;

		if (strlen(line) <= 1)
			goto tryagain;

		int count;

		if ((count = sscanf(line,"%s %d %s", fn, &cls, cls_name)) < 2)
			error("Bad line read: %s\n",line);

		// TODO: check to make sure cls is not too big!

		if (count == 3)
			strncpy(label[cls], cls_name, 81);

		// ********

		{
			DiskXFile datafile(fn, "r");
			data[cls]->appendXFile(&datafile);
		}

		// ********
	}

	for (i = 0; i < r; i++)
	{
		MU_->getRow(i, &mu);
		U_->getRow(i, 0, d, d, &u);

		data[i]->mean(&mu);
		data[i]->cov(&mu, &u);

	}

	init(MU_, U_, u_min_, false, false);
}

void Gaussian::reset()
{
	S_MU->zero();
	S_MU_new->zero();
	w_MU->zero();
	w_MU_next->zero();
	S_R->zero();
	S_R_new->zero();
	w_R->zero();
	w_R_next->zero();
	R2_R->zero();

	//LAN 9-08-10
	R1->zero();
	R2_MU->zero();

	df_MU->zero();
	df_R->zero();
	df_MU_temp->zero();
	df_R_temp->zero();

	StochasticClassifier::reset();
}

int Gaussian::Classify(real *y)
{
	int i;

	//   static IVec *data = new IVec;
	static IVec data;
	static IVec d2;

	data.set(y, d);

	real max_lik = 0;

	static IVec *df_mu = new IVec;
	static IMat *df_rr = new IMat;
	static IVec *mu = new IVec;
	static IMat *rr = new IMat;

	if (debug)
		printf("\n");

	for (i = 0; i < r; i++)
	{
		MU->getRow(i, mu);
		df_MU->getRow(i, i*d, d, df_mu);

		R->getRow(i, 0, d, d, rr);
		df_R->getRow(i, i*d*d, d, d, df_rr);

		(*prob)(i) = N(&data, mu, rr, df_mu, df_rr);

		if (debug)
			printf("%8.6g ", (*prob)(i));

		N(&data, mu, rr, df_mu, df_rr);

		if ((*prob)(i) > max_lik)
		{
			max_lik = (*prob)(i);
			best_class = i;
		}
	}

	if (debug)
		printf("\n");

	return best_class;
}

int Gaussian::Updatew()
{
	// Update w = du/d(phi(l))

	// w(t+1) = R1*w(t) + R2;

	IMat *tmp;

	MatMatMult(R1, CblasNoTrans,
			w_MU, CblasNoTrans,
			w_MU_next);            // w(t+1) = R1*w(t)
	MatAdd(R2_MU, w_MU_next);         // w(t+1) += R2
	swap(w_MU, w_MU_next, tmp);       // w(t) <=> w(t+1)

	MatMatMult(R1, CblasNoTrans,
			w_R, CblasNoTrans,
			w_R_next);           // w(t+1) = R1*w(t)
	MatAdd(R2_R, w_R_next);         // w(t+1) += R2
	swap(w_R, w_R_next, tmp);       // w(t) <=> w(t+1)

	return 0;
}

int Gaussian::UpdateR(IMat *Rs_,
		IMat *R1_,
		IVec *u_,
		real scale_)
{
	Rs = Rs_;
	R1 = R1_;
	u = u_;
	scale = scale_;

	//   %%%%%%%%%%%%%%%%%%%%%%%%%%
	//   %
	//   % For observation probabilities
	//   %
	//   %       R2(:,m) = A' * (eye(r,r) - F*u*ones(1,r)*scale) ...
	//   %                 * (dF(m)*u)*scale;
	//   %
	//   % We'd like to calculate this range of R2 as single
	//   % matrix multiplication.  Here, the main obstacle here is the
	//   % (dF(m)*u) = (df(m).*u), which becomes
	//   %
	//   %               (df .* (repmat(u,1,sizeof(df))))
	//   %
	//   % where repmat() replicates a matrix, and df is the derivative
	//   % of column vector f with respect to each density parameter.
	//   %
	//   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	static IVec *df_sub = new IVec;
	static IVec *df2_sub = new IVec;

	int i;

	for (i = 0; i < r; i++)
	{
		df_MU->getRow(i, i*d, d, df_sub);
		df_MU_temp->getRow(i, i*d, d, df2_sub);

		VecCopy(df_sub, df2_sub);
		VecScale(scale * (*u)(i), df2_sub);
	}

	MatMatMult(Rs, CblasNoTrans,
			df_MU_temp, CblasNoTrans,
			R2_MU);

	for (i = 0; i < r; i++)
	{
		df_R->getRow(i, i*d*d, d*d, df_sub);
		df_R_temp->getRow(i, i*d*d, d*d, df2_sub);

		VecCopy(df_sub, df2_sub);
		VecScale(scale * (*u)(i), df2_sub);
	}

	MatMatMult(Rs, CblasNoTrans,
			df_R_temp, CblasNoTrans,
			R2_R);

	return 0;
}


int Gaussian::UpdateGradient()
{
	iter++;

	// Update the score vector

	// Get a vector view of the S matrices

	static IVec *S_MU_vec = new(allocator) IVec;  // TODO: change to static!
	static IVec *S_R_vec = new(allocator) IVec;

	S_MU_new->getAll(S_MU_vec);
	S_R_new->getAll(S_R_vec);

	// S_new = scale * (w'f + df'u)

	GenMatVecMult(scale,
			w_MU, CblasTrans,
			prob,               // prob == f
			0.0, S_MU_vec);
	GenMatVecMult(scale,
			w_R, CblasTrans,
			prob,
			0.0, S_R_vec);

	GenMatVecMult(scale,
			df_MU, CblasTrans,
			u,
			1.0, S_MU_vec);
	GenMatVecMult(scale,
			df_R, CblasTrans,
			u,
			1.0, S_R_vec);

	// Set lower triangle of S_R to zero

	static IMat *S_R_mat = new(allocator) IMat;

	for (int i = 0; i < r; i++)
	{
		S_R_new->getRow(i, 0, d, d, S_R_mat);

		for (int j = -1; j >= -d; j--)
		{
			S_R_mat->getDiag(j, S_R_vec);
			S_R_vec->zero();
		}
	}

	if (silence_state >= 0)
	{
		S_MU_new->getRow(silence_state, S_MU_vec);
		S_MU_vec->zero();

		S_R_new->getRow(silence_state, S_R_vec);
		S_R_vec->zero();
	}

	// Average gradients if desired

	int n = min(iter, history);

	if (avg_obs && n > 1)
	{
		// S = S + (1/n)*(S_new - S)
		//
		//   = ((n-1)/n)*S + (1/n)*S_new

		MatScale((n-1.0)/n, S_MU);
		MatAddScaled(1.0/n, S_MU_new, S_MU);

		MatScale((n-1.0)/n, S_R);
		MatAddScaled(1.0/n, S_R_new, S_R);

	}
	else
	{
		IMat *tmp;

		swap(S_MU, S_MU_new, tmp);
		swap(S_R, S_R_new, tmp);
	}

	return 0;
}


int Gaussian::UpdateParms()
{
	int n = min(iter, history);

	real epsilon;

	if (eps_exp != 0.0)
		epsilon = eps0/((real)pow((double)iter,(double)eps_exp));
	else
		epsilon = eps0;

	// Update mu, R

	if (avg_iters && n > 1)
	{
		// mu_temp = mu + epsilon*n*S_MU
		// R_temp = R + epsilon*n*S_R

		MatCopy(MU, MU_temp);
		MatCopy(R, R_temp);

		MatAddScaled(epsilon*n, S_MU, MU_temp);
		MatAddScaled(epsilon*n, S_R, R_temp);

		// mu = mu + (1/n)*(mu_temp - mu)

		MatScale((n-1.0)/n, MU);
		MatAddScaled(1.0/n, MU_temp, MU);

		// R = R + (1/n)*(R_temp - R)

		MatScale((n-1.0)/n, R);
		MatAddScaled(1.0/n, R_temp, R);
	}
	else
	{
		MatAddScaled(epsilon, S_MU, MU);
		MatAddScaled(epsilon, S_R, R);
	}

	return 0;
}

int Gaussian::RMLEUpdate()
{
	// use only if not within an HMM
	return (UpdateParms());
}

int Gaussian::KMeansInit(IMat *x,
		bool rand_init,
		int max_iter)
{
	int i,j;

	int n;     // number of data points

	IMat y;

	IMat MU_old;         // previous mean vecs
	IMat dist;           // sum((x-mu)^2)

	IVec    minvals;     // min values
	IVecInt minlocs;     // corresponding min value locations

	IVecInt counts;      // number of points in each class
	IVecInt order;       // sorted order of counts
	IVecInt I;           // indices of rows of x

	bool finished = false;

	y = *x;


	////////////////////
	// Idiot checking
	//////////////////////

	if (d != y.n)
		error ("KMeansInit: data has wrong dimension (dim(y) == %d, should be %d)\n",
				y.n, d);

	////////////////////
	// Initialization
	//////////////////////

	n = y.m;

	MU_old.reshape(r,d);

	counts.resize(r);

	////////////////////
	// Pick means
	//////////////////////

	if (r == 1)
	{
		IVec mean;
		IMat cov;

		MU->getRow(0,&mean);               // mean = alias for mu
		R->getRow(0, 0, d, d, &cov);   // cov  = alias for U

		y.mean(&mean);       //  mean
		y.cov(&mean, &cov);  //  cov

		SymMatCholFact(&cov);

		return 0;
	}

	if (rand_init)
	{
		IVec row;

		for (i = 0; i < r; i++)
		{
			j = Random::random() % n;

			y.getRow(j,&row);      // alias row j of x
			MU->setRow(i, &row);   // copy to row i of mu
		}
	}

	// else use MU as is

	////////////////////
	// Calculate x.^2
	//////////////////////

	IMat xdotx;
	IMat x2;             // sum(x.^2) * ones (1,r)

	IMat mdotm;
	IMat m2;             // ones(n,1) * sum(mu.^2)

	xdotx = y;
	MatDotTimes(1.0, &xdotx, &xdotx);

	IVec x_sum;
	xdotx.sum(&x_sum, 2);         // x_sum = sum(x.^2) along each row

	x2.reshape(n, r);             // x2 is an n by r array
	x2.vecFillCols(&x_sum);

	m2.reshape(n, r);

	////////////////////
	// Main loop
	//////////////////////

	for (i = 0; i < max_iter; i++)
	{
		////////////////////
		// Save old means
		//////////////////////

		MatCopy(MU, &MU_old);              // MU_old = MU_

		////////////////////
		// Calculate distances
		//////////////////////

		mdotm = *MU;
		MatDotTimes(1.0, &mdotm, &mdotm);  // mdotm = mu.^2

		IVec MU_sum;
		mdotm.sum(&MU_sum, 2);             // MU_sum = sum(mu2,2)
		m2.vecFillRows(&MU_sum);

		dist = x2;
		//      MatCopy(&x2, &dist);
		MatAdd(&m2, &dist);

		GenMatMatMult(-2.0,
				&y, CblasNoTrans,
				MU, CblasTrans,
				1.0,
				&dist);              // dist = x^2 + m2 - 2x*m


				////////////////////
				// Assign each point to a center
		//////////////////////

		dist.mmin(&minvals, &minlocs, 1);  // find the min of each row

		////////////////////
		// Count numbers in each bin
		//////////////////////

		counts.zero();

		for (j = 0; j < n; j++)
			counts[minlocs[j]]++;

		// *****

		//printf("Counts/class:\n");
		//counts.print();

		int cls;
		counts.sort(&order);           // order is the sorted order of
		// the class counts (indices only)
		for (j = 0; j < r; j++)
		{
			cls = order[j];
			minlocs.find(cls, &I);

			if (I.n == 0)
			{
				int index;               // this class has no members!
				IVec row;

				minvals.vmax(&index);    // use an outlier to start a new
				y.getRow(index, &row);  // class....
				MU->setRow(cls, &row);

				minlocs[index] = cls;
				minvals[index] = 0.0;
			}
			else
			{
				IVec mean;

				MU->getRow(cls, &mean);    // mean is an alias into MU_
				y.mean(&mean, 1, &I);     // mu(cls) = avg of x's in
				// that class
			}

		}

		////////////////////
		// Are we done yet?
		//////////////////////

		//hackish (fix)
		//finished = true;
		for (int idx = 0; idx < counts.n; idx++) {
			if (counts[idx] == 1) {
				finished = false;
			}
		}
		if (finished)
			break;

		MatSub(MU, &MU_old);
		MU_old.abs();
		if (MU_old.mmin() < REAL_EPSILON)
			break;
	}

	IVec mean;
	IMat U;

	for (i = 0; i < r; i++)
	{
		minlocs.find(i, &I);
		MU->getRow(i,&mean);               // mean = alias for mu
		R->getRow(i, 0, d, d, &U);       // U  = alias for R

		if (I.n <= 1)
		{
			warning("Tried calculating covariance with less than 2 points!\n");
			warning("Setting cov. matrix to eye(%d)!\n", d);
			U.eye();
		}
		else
		{
			y.cov(&mean, &U, 1, &I);       // calc covariance of class i
			SymMatCholFact(&U);
		}
	}

	return 0;
}


void Gaussian::print(int scrn_width, int col_width, char *num_format)
{
	IMat U(r,d*d);

	IMat Rtmp;
	IMat Utmp;

	for (int i = 0; i < r; i++)
	{
		R->getRow(i, 0, d, d, &Rtmp);
		U.getRow(i, 0, d, d, &Utmp);
		MatMatMult(&Rtmp, CblasTrans, &Rtmp, CblasNoTrans, &Utmp);
	}


	printf("mu (%d by %d):\n\n", r, d);
	MU->print(scrn_width, col_width, num_format);

	printf("U (%d by %d):\n\n", r, d*d);
	U.print(scrn_width, col_width, num_format);

}



void Gaussian::print_w(int scrn_width, 
		int col_width,
		char *num_format)
{
	printf ("w_MU:\n");
	w_MU->print(scrn_width, col_width, num_format);
	printf ("w_R:\n");
	w_R->print(scrn_width, col_width, num_format);
}

void Gaussian::print_df(int scrn_width, 
		int col_width,
		char *num_format)
{
	printf("df_MU:\n");
	df_MU->print(scrn_width, col_width, num_format);
	printf("df_R:\n");
	df_R->print(scrn_width, col_width, num_format);
}

void Gaussian::print_R(int scrn_width, 
		int col_width,
		char *num_format)
{
	printf("R2_MU:\n");
	R2_MU->print(scrn_width, col_width, num_format);
	printf("R2_R:\n");
	R2_R->print(scrn_width, col_width, num_format);
}

void Gaussian::print_S(int scrn_width, 
		int col_width,
		char *num_format)
{
	printf("S_MU:\n");
	S_MU->print(scrn_width, col_width, num_format);
	printf("S_R:\n");
	S_R->print(scrn_width, col_width, num_format);
}

void Gaussian::saveXFile(XFile *file)
{
	int i;

	IMat U(r,d*d);
	IMat rr;
	IMat sigma;

	for (i = 0; i < r; i++)
	{
		R->getRow(i, 0, d, d, &rr);
		U.getRow(i, 0, d, d, &sigma);
		MatMatMult(&rr, CblasTrans, &rr, CblasNoTrans, &sigma);
	}

	file->printf("\nGaussian Parms:\n");

	StochasticClassifier::saveXFile(file);

	file->printf("\nMU:\n");
	MU->saveXFile(file);

	file->printf("\nU:\n");
	U.saveXFile(file);

	file->printf("\nu_min: " REAL_FORMAT_OUT "\n", u_min);

}

void Gaussian::loadXFile(XFile *file)
{
	G_allocator->freeAll();

	IMat *MU_ = new(G_allocator) IMat;
	IMat *U_ = new(G_allocator) IMat;

	real u_min_;

	file->scanf("\nGaussian Parms:\n", NULL);

	StochasticClassifier::loadXFile(file);

	file->scanf("\nMU:\n", NULL);
	MU_->loadXFile(file);

	file->scanf("\nU:\n", NULL);
	U_->loadXFile(file);

	file->scanf("\nu_min: " REAL_FORMAT_IN "\n", &u_min_);

	init(MU_, U_, u_min_, false, false);
}

int Gaussian::covReg(real lambda_, real xi_) {

	//unpacking
	IMat rr;
	IMat L,C1,C2;

	//gsl things
	gsl_matrix * U = gsl_matrix_alloc(d,d);
	gsl_vector * eigs = gsl_vector_alloc(d);
	gsl_matrix * V = gsl_matrix_alloc(d,d);
	gsl_eigen_symmv_workspace * uwk =  gsl_eigen_symmv_alloc(d);

	//R->print(100,10);

	for (int i = 0; i < r; i++)
	{
		//unpack each covariance matrix
		IMat sigma(d,d);
		R->getRow(i, 0, d, d, &rr);
		MatMatMult(&rr, CblasTrans, &rr, CblasNoTrans, &sigma);

		for (int j = 0; j < d; j++) {
			for (int k = 0; k < d; k++) {
				U->data[j * U->tda + k] = sigma(j,k);
			}
		}

		//find the eigendecomposition
		gsl_eigen_symmv(U, eigs, V, uwk);


		//add lambda*eye(d) to the eigenvalue matrix
		L.resize(d,d);
		L.eye();
		for (int j = 0; j < d; j++) {
			if (eigs->data[j*eigs->stride] > lambda_ && eigs->data[j*eigs->stride] < xi_) {
				L(j,j) = eigs->data[j*eigs->stride];
			}
			else if (eigs->data[j*eigs->stride] > xi_) {
				L(j,j) = xi_;
			}
			else {
				L(j,j) = lambda_;
			}
		}
		C1.resize(d,d);
		for (int j = 0; j < d; j++) {
			for (int k = 0; k < d; k++) {
				C1(j,k) = V->data[j * U->tda + k];
			}
		}
		C2.resize(d,d);

		//recompose
		R->getRow(i, 0, d, d, &sigma);
		MatMatMult(&L,CblasNoTrans,&C1,CblasTrans,&C2);
		MatMatMult(&C1,CblasNoTrans,&C2,CblasNoTrans,&sigma);


		//take cholesky decomp and restore
		SymMatCholFact(&sigma);

		// Set lower triangle to zero
		IVec ltri;
		for (int j = -1; j >= -(d-1); j--)
		{
			sigma.getDiag(j, &ltri);
			ltri.zero();
		}

	}

	//R->print(100,10);
	gsl_matrix_free(U);
	gsl_matrix_free(V);
	gsl_vector_free(eigs);
	gsl_eigen_symmv_free(uwk);

	return 0;

}
