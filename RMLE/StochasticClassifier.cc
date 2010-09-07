/**
 * @file   StochasticClassifier.cc
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Tue Jul  6 15:38:58 2004
 * 
 * @brief  classify an input according to classes with probability
 * distributions
 *
 * 
 * $Log: StochasticClassifier.cc,v $
 * Revision 1.1  2005/05/09 20:52:57  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.6.2.2  2004/11/07 23:57:44  k-squire
 * Renamed epsilon to eps0.
 * Added decay parameter eps_exp.
 * Added function to to set string labels.
 * Fixed parsing of "end labels" string when reading in labels during file load.
 *
 * Revision 1.6.2.1  2004/10/24 21:06:24  k-squire
 * Fixed comparison bug: white space at beginning of line seems to be ignored
 *   by gets. :-(
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

static char *StochasticClassifier_CC_ID = "$Id: StochasticClassifier.cc,v 1.1 2005/05/09 20:52:57 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <torch/Object.h>

#include <imatlib/IVecInt.hh>
#include "StochasticClassifier.hh"

#include "gen_defines.h"
//#include "Frame.hh"

using namespace Torch;


StochasticClassifier::StochasticClassifier():
      d(0),
      r(0),
      data_len(0),
      prob(NULL),
      label(NULL),
      S(NULL),
      S_new(NULL),
      iter(1),
      history(1),
      avg_obs(false),
      avg_iters(false),
      eps0(0.001),
      eps_exp(0.0),
      prior(0.0),
      log_prob(0.0),
      Rs(NULL),
      R1(NULL),
      R2(NULL),
      debug(false)
{
   addOptions();
   lblallocator = new Allocator;
}

StochasticClassifier::StochasticClassifier(int r_, int d_, real prior_):
      r(0),
      d(0),
      data_len(0),
      prob(NULL),
      prior(0.0),
      label(NULL),
      S(NULL),
      S_new(NULL),
      iter(0),
      history(1),
      avg_obs(false),
      avg_iters(false),
      eps0(0.001),
      eps_exp(0.0),
      log_prob(0.0),
      Rs(NULL),
      R1(NULL),
      R2(NULL),
      debug(false)
{
   addOptions();
   lblallocator = new Allocator;

   init(r_, d_, prior_);
}


StochasticClassifier::~StochasticClassifier()
{
   delete lblallocator;
}

void StochasticClassifier::addOptions()
{
   // Allow setting of various options

   addIOption("history",       &history,   1,     "averaging history");

   addBOption("average S",     &avg_obs,   false, "average observations");
   addBOption("average parms", &avg_iters, false, "average iterages");
   
   addROption("eps0",          &eps0,      0.001, "initial learning rate");
   addROption("eps_exp",       &eps_exp,   0.000, "learning rate decay");
   addROption("prior",         &prior,     0.0,   "prior on weights");

   addBOption("debug",         &debug,     false, "print debug info");
   
}

int StochasticClassifier::init(int r_, int d_, real prior_)
{
   int i;
   
   r = r_;
   d = d_;
   data_len = d_;
   prior = prior_;
   
   if (prob == NULL)
      prob = new(allocator) IVec(r);
   else
      prob->resize(r);

   if (!label)
   {
      label = (char **)lblallocator->alloc(r*sizeof(char *));
      for (i = 0; i < r; i++)
      {
         label[i] = (char *)lblallocator->alloc(81);
         *label[i] = '\0';
      }
   }

   if (prob)
      prob->fill(1.0/r);
}

void StochasticClassifier::reset()
{
   if (prob && r > 0)
      prob->fill(1.0/r);
}

int StochasticClassifier::Classify(real *y)
{
   return -1;
}

int StochasticClassifier::Updatew()
{
   return -1;
}

int StochasticClassifier::UpdateR(IMat *Rs_,
                                  IMat *R1_, 
                                  IVec *u_,
                                  real scale_)
{
   return -1;
}


int StochasticClassifier::UpdateGradient()
{
   return -1;
}


int StochasticClassifier::UpdateParms()
{
   return -1;
}

int StochasticClassifier::RMLEUpdate()
{
   return -1;
}

int StochasticClassifier::setLabels(str_list *labels_)
{
   int i;
   int i_max = r;
   
   if (labels_->count < r)
      i_max = labels_->count;

   for (i = 0; i < i_max; i++)
      strcpy(label[i],(*labels_)[i]);
   
   return 0;
}

int StochasticClassifier::ProbProject(IMat *Q_m,
                                      real prior,
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

//   allocator->free(min_pos);
      
   return 0;
}

void StochasticClassifier::print(int scrn_width, 
                                 int col_width, 
                                 char *num_format)
{
   printf("Info:\n\n");
   
   printf("number of inputs  (d) = %8d\n", d);
   printf("number of classes (r) = %8d\n", r);
   printf("eps0                  = %8.4f\n", eps0);
   printf("eps_exp               = %8.4f\n", eps_exp);
   printf("prior                 = %8.4f\n", prior);
   printf("log probability       = %8.4f\n", log_prob);
}


void StochasticClassifier::print_w(int scrn_width, 
                                   int col_width, 
                                   char *num_format)
{
}

void StochasticClassifier::print_df(int scrn_width, 
                         int col_width, 
                         char *num_format)
{
}

void StochasticClassifier::print_R(int scrn_width, 
                        int col_width, 
                        char *num_format)
{
}

void StochasticClassifier::print_S(int scrn_width, 
                        int col_width, 
                        char *num_format)
{
}

void StochasticClassifier::saveXFile(XFile *file)
{
   file->printf("\nClassifier Parms:\n");
   file->printf(" num_classes: %d\n", r);
   file->printf(" num_inputs: %d\n", d);
   // TODO: add distinction between this and data_len!

   file->printf(" average S: %d\n", (int)avg_obs);
   file->printf(" average parms: %d\n", (int)avg_iters);
   file->printf(" history: %d\n", history);
   file->printf(" eps0: " REAL_FORMAT_OUT "\n", eps0);
   file->printf(" eps_exp: " REAL_FORMAT_OUT "\n", eps_exp);
   file->printf(" prior: " REAL_FORMAT_OUT "\n", prior);

   file->printf(" class labels:\n");
   
   int i;
   for (i = 0; i < r; i++)
      if (strlen(label[i]) > 0)
         file->printf("%4d %15s\n", i, label[i]);

   file->printf(" end labels\n");

   // TODO other parms???

}

void StochasticClassifier::loadXFile(XFile *file)
{
   int i;
   
   file->scanf("\nClassifier Parms:\n", NULL);
   file->scanf(" num_classes: %d\n", &r);
   file->scanf(" num_inputs: %d\n", &d);

   if (!prob)
      prob = new IVec;

   prob->resize(r);
   prob->fill(1.0/r);

   file->scanf(" average S: %d\n", (int*)&avg_obs);
   file->scanf(" average parms: %d\n", (int*)&avg_iters);
   file->scanf(" history: %d\n", &history);
   file->scanf(" eps0: " REAL_FORMAT_IN "\n", &eps0);
   file->scanf(" eps_exp: " REAL_FORMAT_IN "\n", &eps_exp);
   file->scanf(" prior: " REAL_FORMAT_IN "\n", &prior);

   file->scanf(" class labels:\n", NULL);

   lblallocator->freeAll();

   label = (char **)lblallocator->alloc(r*sizeof(char *));
   for (i = 0; i < r; i++)
   {
      label[i] = (char *)lblallocator->alloc(81);
      *label[i] = '\0';
   }

   char line[81];
   int dummy, cls;
   
   int done = 0;

   do
   {
      file->gets(line, 81);
      if (strncmp(line, " end labels", 11) != 0 &&
          strncmp(line, "end labels", 10) != 0)
      {
         sscanf(line, " %d", &cls);
         sscanf(line, " %d %s", &dummy, label[cls]);
      }
      else
         done = 1;
   } while (!done);
   
   // TODO other parms???

}
