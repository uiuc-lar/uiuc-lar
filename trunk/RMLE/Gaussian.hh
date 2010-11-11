/**
 * @file   Gaussian.hh
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Thu Jul  8 13:41:24 2004
 * 
 * @brief  Gaussian Distribuion classifier definition
 * 
 * 
 * $Log: Gaussian.hh,v $
 * Revision 1.1  2005/05/09 20:52:57  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.3.6.2  2004/11/07 23:49:40  k-squire
 * Renamed epsilon to eps0.
 * Added epsilon decay value eps_exp.
 * Removed projection code; data dimension should be reduced before it
 *    gets to us.
 * Added kmeans-based constructor.
 * Fixed naming of U and R; U usually refers to the covariance matrix,
 *    whereas R refers to the Cholesky decomposition of this matrix.
 *
 * Revision 1.3.6.1  2004/10/24 21:11:10  k-squire
 * Changed all reference of "rotation" to "projection" to more accurately
 *   describe the projection file. :-)
 * Fixed an off-by-one bug when setting the lower triangle of a matrix to zero.
 * Added debugging to Classify()
 * Modified KMeans init to allow use of projection file.
 *
 * Revision 1.3  2004/08/10 23:49:11  k-squire
 * Backported changes from illy/hmm.
 * Changed hmm_run to conditionally depend on libIBase, IServer stuff.
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

#ifndef Gaussian_HH
#define Gaussian_HH

static char *Gaussian_HH_Id = "$Id: Gaussian.hh,v 1.1 2005/05/09 20:52:57 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <torch/general.h>

#include "StochasticClassifier.hh"
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
//#include <gsl/gsl_blas.h>


using namespace Torch;

   
class Gaussian : public StochasticClassifier
{
public:

// *** From StochasticClassifier:

//    int                d;   // num inputs
//    int                r;   // num classes
//    int       best_class;
//    IVec           *prob;

//    char         **label;

//    IVec              *S;
//    IVec          *S_new;

//    int             iter;
//    int          history;

//    bool         avg_obs;
//    bool       avg_iters;

//    real         eps0;
//    real           prior;

//    real        log_prob;

//    IMat             *Rs;
//    IMat             *R1;
//    IMat             *R2;

   IMat                 *MU;
   IMat                  *R;

   real               u_min;

   IMat               *S_MU;
   IMat           *S_MU_new;

   IMat                *S_R;
   IMat            *S_R_new;

   IVec                  *u;

   IMat               *w_MU;
   IMat          *w_MU_next;
   IMat                *w_R;
   IMat           *w_R_next;

   int           best_class;

   IMat              *R2_MU;
   IMat               *R2_R;

   IMat              *df_MU;
   IMat               *df_R;

   real               scale;

   int        silence_state;

private:
   Allocator   *G_allocator;
   
   IMat            *MU_temp;
   IMat             *R_temp;
   IMat         *df_MU_temp;
   IMat          *df_R_temp;

public:

   Gaussian();

   Gaussian(int r_, 
            int d_,
            bool rand_init = false,
            real mu_min_ = -1.0,
            real mu_max_ = 1.0,
            real u_min_ = 0.1);

   Gaussian(int r_, 
            int d_,
            IMat *MU_,
            IMat *U_,
            real u_min_ = 0.1, 
            bool do_copy = false, 
            bool retain = false);
   
   Gaussian(int r_, 
            int d_,
            real *MU_,
            real *U_,
            real u_min_ = 0.1, 
            bool do_copy = false, 
            bool retain = false);
   
   Gaussian(int r_, 
            int d_,
            IMat *MU_,
            IMat *kmeans_data,
            int max_iter,
            real u_min_ = 0.1, 
            bool do_copy = false, 
            bool retain = false);

   ~Gaussian();
   
private:
   void addOptions();
   
public:
   int init(XFile *filelist,
            real u_min = 0.1);

   virtual void reset();

   virtual int Classify(real *y);

   virtual int Updatew();

   virtual int UpdateR(IMat *Rs_,
                       IMat *R1_, 
                       IVec *u_,
                       real scale_);

   virtual int UpdateGradient();

   virtual int UpdateParms();

   virtual int RMLEUpdate();

   int covReg(real lambda_, real xi_);

   virtual int KMeansInit(IMat *x,
                          bool rand_init = false,
                          int max_iter = 20);

   virtual void print(int scrn_width=80, 
                      int col_width=12, 
                      char *num_format="%12.8f");

   // debugging stuff

   virtual void print_w(int scrn_width=80, 
                        int col_width=12, 
                        char *num_format="%12.8f");

   virtual void print_df(int scrn_width=80, 
                         int col_width=12, 
                         char *num_format="%12.8f");

   virtual void print_R(int scrn_width=80, 
                        int col_width=12, 
                        char *num_format="%12.8f");

   virtual void print_S(int scrn_width=80, 
                        int col_width=12, 
                        char *num_format="%12.8f");

   ////////////////////
   // File stuff
   //////////////////////

   virtual void saveXFile(XFile *file);  ///< save the Gaussian to a file
   virtual void loadXFile(XFile *file);  ///< load the Gaussian from a file

private:

   int init(IMat *MU_ = NULL,
            IMat *U_ = NULL,
            real u_min_ = 0.1, 
            bool do_copy = false, 
            bool retain = false);
};


#endif // Gaussian_HH

   
