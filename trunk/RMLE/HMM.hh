/**
 * @file   hmm.hh
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Mon Jul  5 21:11:15 2004
 * 
 * @brief  hidden Markov level class definition
 * 
 * $Log: HMM.hh,v $
 * Revision 1.1  2005/05/09 20:52:59  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.6.2.2  2005/02/09 00:16:44  k-squire
 * Added Viterbi stuff, classification of integers.
 *
 * Revision 1.6.2.1  2004/11/07 23:55:49  k-squire
 * Renamed epsilon to eps0.
 * Added decay parameter eps_exp.
 * Added classification for a single discrete input using the marginal pmf
 *    for that input. (Classify (int y, int pos)
 * Added function to get the remaining viterbi sequence.
 * Fixed scale test so that it actually tests for zero and not REAL_EPSILON.
 * *** NOTE: VITERBI CLASSIFICATION IS BROKEN! ***
 *
 * Revision 1.6  2004/08/11 03:09:14  k-squire
 * Merged changes from KEVIN_DEVEL branch
 *
 * Revision 1.5.2.1  2004/08/11 03:00:13  k-squire
 * Changed IMatLib to imatlib.
 *
 * Revision 1.5  2004/08/11 01:15:29  k-squire
 * Changed "matrix/*.h" to "IMatLib/*.h"
 *
 * Revision 1.4  2004/08/10 23:49:11  k-squire
 * Backported changes from illy/hmm.
 * Changed hmm_run to conditionally depend on libIBase, IServer stuff.
 *
 * Revision 1.1.2.3  2004/08/08 22:51:06  k-squire
 * Added ability to set labels.
 * Added reset() function.
 * Implemented Viterbi algorithm with fixed history backtracking.
 *
 * Revision 1.1.2.2  2004/08/04 06:52:01  k-squire
 * Added a local allocator to make it easy to delete all locally allocated
 *   material.
 * Added default empty constructor;
 * Added init() functions, for init after empty construction.
 * Added private function addOptions(), to eliminate redundant code.
 * Cleanups/error checking in save and load functions.
 *
 * Revision 1.1.2.1  2004/08/03 07:07:10  k-squire
 * Removed obsolete dependence on Frame.hh.
 *
 * Revision 1.1  2004/08/03 06:38:39  k-squire
 * Import and initial checkin.
 *
 * Revision 1.2  2004/08/03 03:46:41  k-squire
 * Added debug print functions.
 * Added setObsDist() function.
 * Added saveXFile(), loadXFile() functions.
 * Fixed to work with modified IMat class functions:
 *    getDiag, getRow, getCol.
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 *
 */

#ifndef HMM_HH
#define HMM_HH

//static char *HMM_HH_Id = "$Id: HMM.hh,v 1.1 2005/05/09 20:52:59 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <torch/general.h>

#include "../imatlib/IMat.hh"
#include "../imatlib/IVec.hh"
#include "../imatlib/IMatVecOps.hh"

#include "StochasticClassifier.hh"

#define HMM_START_STR "HMM_uiuc"

using namespace Torch;

class HMM : public StochasticClassifier
{
public:
      
// *** From StochasticClassifier:

//    int                r;   // num classes
//    int                d;   // num inputs
//    int       best_class;
//    IVec           *prob;

//    char         **label;

//    IVec              *S;
//    IVec          *S_new;

//    int             iter;
//    int          history;

//    bool         avg_obs;
//    bool       avg_iters;

//    real            eps0;
//    real         eps_exp;
//    real           prior;

//    real        log_prob;

//    IMat             *Rs;
//    IMat             *R1;
//    IMat             *R2;

   IMat                  *A;
   StochasticClassifier  *b;

   IMat                *S_A;
   IMat            *S_A_new;

   IVec                  *f;
   
   IVec                  *u;
   IMat                  *w;
   IMat             *w_next;

   real               scale;

   // viterbi variables
   IVec             **delta;
   IVecInt            **psi;
   int         viterbi_hist;
   int              vit_pos;
   int            vit_count;
      
//   IVec             **probs;

   int                state;

private:
   Allocator *HMM_allocator;

   IMat           *prob_mat;
   IMat                *eye;
   IMat             *tmp_rr;
   IMat             *A_temp;

   IVec              *tmp_r;

public:

   HMM();

   HMM(StochasticClassifier *b_,
       bool rand_A = false,
       real prior_ = 0.0,
       int viterbi_hist_ = 1);
      
   HMM(StochasticClassifier *b_, 
       IMat *A_, 
       bool rand_A = false,
       real prior_ = 0.0, 
       int viterbi_hist_ = 1,
       bool do_copy = false, 
       bool retain = false);
      
   HMM(StochasticClassifier *b_, 
       real *A_, 
       real prior_ = 0.0, 
       int viterbi_hist_ = 1,
       bool do_copy = false, 
       bool retain = false);
      
   ~HMM();

private:
   void addOptions();
   
public:
   void init(int r_,
             bool rand_A = false,
             int viterbi_hist_ = 1);
   
   void init(StochasticClassifier *b_, 
             IMat *A_ = NULL, 
             bool rand_A = false,
             real prior_ = 0.0, 
             int viterbi_hist_ = 1,
             bool do_copy = false, 
             bool retain = false);

   virtual void reset();

   void setObsDist(StochasticClassifier *b_, bool retain=false);

   int setViterbiHist(int viterbi_hist_);

   virtual int Classify(real *y);

   virtual int Classify(int *y);

   virtual int Classify(int y, int pos);

   int getViterbiSeq(IVecInt *seq);
   
   virtual int Updatew();

   virtual int UpdateR(IMat *Rs_=NULL,
                       IMat *R1_=NULL, 
                       IVec *u_=NULL,
                       real scale_ = 1.0);

   virtual int UpdateGradient();

   virtual int UpdateParms();

   virtual int RMLEUpdate();

   int InitTransProb(IMat *A_,
                     real prior = 0.0, 
                     bool rand_init = false);

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

   virtual void saveXFile(XFile *file); ///< save the classifier to a file
   virtual void loadXFile(XFile *file); ///< load the classifier from a file

};

#endif // HMM_HH
