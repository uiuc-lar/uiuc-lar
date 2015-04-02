/**
 * @file   StochasticClassifier.hh
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Tue Jul  6 14:47:41 2004
 * 
 * @brief  classify an input according to classes with probability
 * distributions 
 * 
 *
 * $Log: StochasticClassifier.hh,v $
 * Revision 1.1  2005/05/09 20:52:57  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.6.2.1  2004/11/07 23:57:44  k-squire
 * Renamed epsilon to eps0.
 * Added decay parameter eps_exp.
 * Added function to to set string labels.
 * Fixed parsing of "end labels" string when reading in labels during file load.
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
 * Revision 1.4  2004/08/10 23:49:12  k-squire
 * Backported changes from illy/hmm.
 * Changed hmm_run to conditionally depend on libIBase, IServer stuff.
 *
 * Revision 1.1.2.3  2004/08/08 22:50:26  k-squire
 * Added ability to set labels.
 * Added reset() function.
 *
 * Revision 1.1.2.2  2004/08/04 06:53:38  k-squire
 * Added debug variable, option.
 * Added private function addOptions(), to eliminate redundant code.
 * Moved REAL_FORMAT defines to gen_defines.
 *
 * Revision 1.1.2.1  2004/08/03 07:07:10  k-squire
 * Removed obsolete dependence on Frame.hh.
 *
 * Revision 1.1  2004/08/03 06:38:39  k-squire
 * Import and initial checkin.
 *
 * Revision 1.2  2004/08/03 03:36:04  k-squire
 * Changed variable name max_hist-->history.
 * Added init() function
 * Changed ProbProject to allow projection along either dimension.
 * Added debug print functions: print_w(), print_df(), print_R(), print_S()
 * Added saveXFile(), loadXFile() virtual functions
 * Re-added default class constructor.
 * Changed other constructors to always initialize to zero/NULL, so init()
 *    functions properly
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 * 
 */

#ifndef STOCHCLASSIFIER_HH
#define STOCHCLASSIFIER_HH

static char *StochasticClassifier_HH_ID = "$Id: StochasticClassifier.hh,v 1.1 2005/05/09 20:52:57 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <torch/general.h>
#include <torch/Object.h>

#include "../imatlib/IMat.hh"
#include "../imatlib/IVec.hh"

#include "label_list.hh"

//#include "Frame.hh"

using namespace Torch;

class StochasticClassifier : public Object
{
public:

   int                r;   // num classes
   int                d;   // dim of obs pdf
   int         data_len;   // dim of data (may differ from d)
   int                p;   // number of parms
   int       best_class;
   IVec           *prob;

   char         **label;

   IVec              *S;
   IVec          *S_new;

   int             iter;
   int          history;

   bool         avg_obs;
   bool       avg_iters;

   real            eps0;
   real         eps_exp;
   real           prior;

   real        log_prob;

   IMat             *Rs;
   IMat             *R1;
   IMat             *R2;

   Allocator *lblallocator;

   bool           debug;

public:

   StochasticClassifier();
   StochasticClassifier(int r_, int d_, real prior = 0.0);
   ~StochasticClassifier();

private:
   void addOptions();
   
public:
   int init(int r_, int d_, real prior = 0.0);

   virtual void reset();

   virtual int Classify(real *y);

   virtual int Updatew();

   virtual int UpdateR(IMat *Rs_=NULL,
                       IMat *R1_=NULL, 
                       IVec *u_=NULL,
                       real scale_ = 1.0);

   virtual int UpdateGradient();

   virtual int UpdateParms();

   virtual int RMLEUpdate();

   int setLabels(str_list *labels_);

   int ProbProject(IMat *Q, real prior = 0.0, int dim = 1);

   virtual void print(int scrn_width=80, 
                      int col_width=12, 
                      char *num_format="%12.8f");

   // Debugging stuff

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

   // file saving

   virtual void saveXFile(XFile *file);  ///< save the classifier to a file
   virtual void loadXFile(XFile *file);  ///< load the classifier from a file

};



#endif // STOCHCLASSIFIER_HH
