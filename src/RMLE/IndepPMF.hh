/**
 * @file   IndepPMF.hh
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Thu Jul  8 13:41:24 2004
 * 
 * @brief  IndepPMF Distribuion classifier definition
 * 
 * 
 * $Log: IndepPMF.hh,v $
 * Revision 1.1  2005/05/09 20:52:57  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.5.2.2  2005/02/09 00:18:30  k-squire
 * Fixed bug in IndepPMF constructor.
 * Changed getRow to take three inputs.
 * Added Generate(), GenerateMax().
 *
 * Revision 1.5.2.1  2004/11/07 23:52:27  k-squire
 * Renamed epsilon to eps0.
 * Added decay parameter eps_exp.
 * Added classification for a single input using the marginal pmf for that
 *    input. (Classify (int y, int pos)
 *
 * Revision 1.5  2004/08/11 03:09:14  k-squire
 * Merged changes from KEVIN_DEVEL branch
 *
 * Revision 1.4.2.1  2004/08/11 03:00:13  k-squire
 * Changed IMatLib to imatlib.
 *
 * Revision 1.4  2004/08/11 01:15:29  k-squire
 * Changed "matrix/*.h" to "IMatLib/*.h"
 *
 * Revision 1.3  2004/08/10 23:49:12  k-squire
 * Backported changes from illy/hmm.
 * Changed hmm_run to conditionally depend on libIBase, IServer stuff.
 *
 * Revision 1.1.2.1  2004/08/10 22:13:06  k-squire
 * Renamed class_labels to labels
 *
 * Revision 1.1  2004/08/03 06:38:39  k-squire
 * Import and initial checkin.
 *
 * Revision 1.2  2004/08/03 03:49:34  k-squire
 * Added debug print functions.
 * Added saveXFile(), loadXFile() functions.
 * Fixed to work with modified IMat class functions:
 *    getDiag, getRow, getCol.
 * Changed current observation type to IVecInt
 * Overloaded Classify() function to allow vector of integers.
 * Changed init() to use IVecInt.
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 *
 */

#ifndef IndepPMF_HH
#define IndepPMF_HH

static char *IndepPMF_HH_Id = "$Id: IndepPMF.hh,v 1.1 2005/05/09 20:52:57 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <torch/general.h>
#include "../imatlib/IVecInt.hh"

#include "StochasticClassifier.hh"

using namespace Torch;

   
class IndepPMF : public StochasticClassifier
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

   IVecInt               *y;  // the current observation

   IVecInt               *s;  // number of possible obs. / input
                              // this is a d-dim vector

   IMat                 **b;  // d matrices, each is s(d) x r

   IMat               **S_b;  // d matrices, each is s(d) x r
   IMat           **S_b_new;
   
   IVec                  *u;  // r length vector

   IMat               **w_b;  // d matrices, each is r x (r * s(d))
   IMat          **w_b_next;

   int           best_class;

   IMat              **R2_b;  // d matrices, each is r x (r * s(d))

   IMat              **df_b;  // d matrices, each is r x (r * s(d))

   IMat                 *fy;  // d x r; prob = prod(fy, 1)

   real               scale;

private:
   
   IMat            **b_temp;
   IMat         **df_b_temp;

public:

// TODO: add support for IVecInt's

   IndepPMF();
   
   IndepPMF(int r_,
            int d_,
            int *s_,
            real prior_ = 0.0,
            bool rand_init = true,
            bool do_copy = true, 
            bool retain = false);
   
   IndepPMF(int r_, 
            int d_,
            int *s_,
            real **b_,
            real prior_ = 0.0,
            bool do_copy = true, 
            bool retain = false);
   
   IndepPMF(IVecInt *s_,
            IMat **b_,
            real prior_ = 0.0,
            bool do_copy = false, 
            bool retain = false);
   
   ~IndepPMF();
   
   virtual int Classify(real *y);

   virtual int Classify(int y, int pos);

   virtual int Classify(int *y);

   virtual int Updatew();

   virtual int UpdateR(IMat *Rs_,
                       IMat *R1_, 
                       IVec *u_,
                       real scale_);

   virtual int UpdateGradient();

   virtual int UpdateParms();

   virtual int RMLEUpdate();

//   virtual int KMeansInit(Sequence *s);

   virtual int Generate(int rr, int ddim, real rn);
   virtual int GenerateMax(int rr, int ddim);

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

private:
   void init(IVecInt *s_,
             IMat **b_,
             bool do_copy = false, 
             bool retain = false);

};


#endif // IndepPMF_HH

   
