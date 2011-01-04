/**
 * @file   IndepPMF.cc
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Thu Jul  8 14:43:44 2004
 * 
 * @brief  IndepPMF Mixture Model
 * 
 * 
 * $Log: IndepPMF.cc,v $
 * Revision 1.1  2005/05/09 20:52:57  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.5.2.2  2005/02/09 00:18:29  k-squire
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
 * Revision 1.3  2004/08/10 23:49:11  k-squire
 * Backported changes from illy/hmm.
 * Changed hmm_run to conditionally depend on libIBase, IServer stuff.
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


static char *IndepPMF_CC_Id = "$Id: IndepPMF.cc,v 1.1 2005/05/09 20:52:57 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <torch/general.h>
#include <torch/Object.h>

#include <imatlib/IMat.hh>
#include <imatlib/IVec.hh>

#include "IndepPMF.hh"
#include "N.hh"
#include "gen_defines.h"

using namespace Torch;

IndepPMF::IndepPMF() :
      StochasticClassifier(),
      y(NULL),
      s(NULL),
      b(NULL),
      S_b(NULL),
      S_b_new(NULL),
      u(NULL),
      w_b(NULL),
      w_b_next(NULL),
      best_class(-1),
      R2_b(NULL),
      df_b(NULL),
      fy(NULL),
      scale(1.0),
      b_temp(NULL),
      df_b_temp(NULL)
{

// ** From StochasticClassifier.cc

//    // learning options

//    addIOption("history", &history, 1, "averaging history");

//    addBOption("average S", &avg_obs, false, "average observations");
//    addBOption("average parms", &avg_iters, false, "average iterages");
   
//    addROption("eps0", &eps0, 0.001, "learning rate");
//    addROption("eps_exp", &eps_exp, 0.000, "learning rate decay");
//    addROption("prior", &prior, 0.0, "prior on weights");

}

IndepPMF::IndepPMF(int r_,
                   int d_,
                   int *s_,
                   real prior_,
                   bool rand_init,
                   bool do_copy, 
                   bool retain):
      StochasticClassifier(r_, d_, prior_)
{
   IVecInt *s2_ = new(allocator) IVecInt(s_, d_, 1, do_copy, retain);

   // initialize b

   IMat **b_;
   
   b_ = (IMat **)allocator->alloc(sizeof(IMat *)*d);

   for (int i = 0; i < d; i++)
   {
      b_[i] = new(allocator)IMat(s_[i], r);
      
      if (rand_init)
      {
         b_[i]->rand(prior_, 1.0);
         
         ProbProject(b_[i], prior_, 1);
         //b_[i]->normalize();
      }
      //else not initialized!!!
   }
   
   init(s2_, b_, false, false);

// ** From StochasticClassifier.cc

//    // learning options

//    addIOption("history", &history, 1, "averaging history");

//    addBOption("average S", &avg_obs, false, "average observations");
//    addBOption("average parms", &avg_iters, false, "average iterages");
   
//    addROption("eps0", &eps0, 0.001, "learning rate");
//    addROption("eps_exp", &eps_exp, 0.000, "learning rate decay");
//    addROption("prior", &prior, 0.0, "prior on weights");

}

   
IndepPMF::IndepPMF(int r_, 
                   int d_,
                   int *s_,
                   real **b_,
                   real prior_,
                   bool do_copy, 
                   bool retain):
      StochasticClassifier(r_, d_, prior_)
{
   IVecInt *s2_ = new(allocator) IVecInt(s_, d_, 1, do_copy, retain);
   
   IMat **b2_ = (IMat **)allocator->alloc(sizeof(IMat **) * d);

   for (int i = 0; i < d; i++)
   {
      b2_[i] = new(allocator)IMat(b_[i], s_[i], r, do_copy, retain);
      ProbProject(b2_[i], prior_, 1);
   }
   
   init(s2_, b2_, false, false);

// ** From StochasticClassifier.cc

//    // learning options

//    addIOption("history", &history, 1, "averaging history");

//    addBOption("average S", &avg_obs, false, "average observations");
//    addBOption("average parms", &avg_iters, false, "average iterages");
   
//    addROption("eps0", &eps0, 0.001, "learning rate");
//    addROption("eps_exp", &eps_exp, 0.000, "learning rate decay");
//    addROption("prior", &prior, 0.0, "prior on weights");

}

IndepPMF::IndepPMF(IVecInt *s_,
                   IMat **b_,
                   real prior_,
                   bool do_copy, 
                   bool retain):
      StochasticClassifier(b_[0]->n, s_->n, prior_)
{

   for (int i = 0; i < d; i++){
      ProbProject(b_[i], prior_, 1);
   }

   init(s_, b_, do_copy, retain);

// ** From StochasticClassifier.cc

//    // learning options

//    addIOption("history", &history, 1, "averaging history");

//    addBOption("average S", &avg_obs, false, "average observations");
//    addBOption("average parms", &avg_iters, false, "average iterages");
   
//    addROption("eps0", &eps0, 0.001, "learning rate");
//    addROption("eps_exp", &eps_exp, 0.000, "learning rate decay");
//    addROption("prior", &prior, 0.0, "prior on weights");

}

IndepPMF::~IndepPMF()
{
}


#define MULTI_IMAT_CALLOC(VAR, D, X, Y, iter) \
   { \
      VAR = (IMat **)allocator->alloc(sizeof(IMat **) * (D)); \
      for ((iter) = 0; (iter) < (D); (iter)++) \
      { \
         VAR[(iter)] = new(allocator) IMat((X),(Y)); \
         VAR[(iter)]->zero(); \
      } \
   }


void IndepPMF::init(IVecInt *s_,
                    IMat **b_,
                    bool do_copy, 
                    bool retain)
{
   int i;

   y = new(allocator) IVecInt(d);

   if (do_copy)
   {

      s = new(allocator) IVecInt(*s_);

      b = (IMat **)allocator->alloc(sizeof(IMat **) * d);
      for (i = 0; i < d; i++)
         b[i] = new(allocator) IMat(*b_[i]);
   }
   else
   {
      s = s_;
      b = b_;

      if (retain)
      {
         allocator->retain(s);
         allocator->retain(b,1);
      }
   }

   MULTI_IMAT_CALLOC(S_b, d, (*s)[i], r, i);
   MULTI_IMAT_CALLOC(w_b, d, r, r*(*s)[i], i);
   MULTI_IMAT_CALLOC(df_b, d, r, r*(*s)[i], i);

   Rs = new(allocator) IMat(r,r);
   R1 = new(allocator) IMat(r,r);
   MULTI_IMAT_CALLOC(R2_b, d, r, r*(*s)[i], i);

   fy = new(allocator) IMat(d,r);

   u = NULL;  // Later set to point to the u in the corresponding HMM

   MULTI_IMAT_CALLOC(b_temp, d, (*s)[i], r, i);
   MULTI_IMAT_CALLOC(S_b_new, d, (*s)[i], r, i);
   MULTI_IMAT_CALLOC(w_b_next, d, r, r*(*s)[i], i);
   MULTI_IMAT_CALLOC(df_b_temp, d, r, r*(*s)[i], i);
   
// ***

   log_prob = 0;
   best_class = -1;

}

int IndepPMF::Classify(real *y_)
{
   int i;

   static IVec row;

   y->set(y_,d);

   for (i = 0; i < d; i++)
   {
      b[i]->getRow(y_[i], &row);   // row points to row "y_[i]" of b[i]
      fy->setRow(i, &row);
   }

   fy->prod(prob,1);   // prob = product along dimension 1 of fy

   prob->vmax(&best_class);

   return best_class;
}


int IndepPMF::Classify(int y_, int pos)
{
   int i;

   static IVec row;

   b[pos]->getRow(y_, prob, true);  // FRED: check me

   prob->vmax(&best_class);

   return best_class;
}

int IndepPMF::Classify(int *y_)
{
   int i;

   static IVec row;

   y->set(y_,d,1,true);

   for (i = 0; i < d; i++)
   {
      b[i]->getRow(y_[i], &row);   // row points to row "y_[i]" of b[i]
      fy->setRow(i, &row);
   }
   
   fy->prod(prob,1);   // prob = product along dimension 1 of fy

   prob->vmax(&best_class);

   return best_class;
}

int IndepPMF::Updatew()
{
   // Update w = du/d(phi(l))

   // w(t+1) = R1*w(t) + R2;

   IMat **tmp;

   for (int i = 0; i < d; i++)
   {
      MatMatMult(R1, CblasNoTrans,
                 w_b[i], CblasNoTrans,
                 w_b_next[i]);            // w(t+1) = R1*w(t)
      MatAdd(R2_b[i], w_b_next[i]);       // w(t+1) += R2
   }
   
   swap(w_b, w_b_next, tmp);              // w(t) <=> w(t+1)
   
   return 0;
}

int IndepPMF::UpdateR(IMat *Rs_,
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

   static IVec df_diag;
   static IVec df2_diag;

//    % we only need to calculated the derivative for
//    % obs. probabilities corresponding to actual observations

   int i, dd, offset;

   for (dd = 0; dd < d; dd++)
   {
      df_b[dd]->zero();
      df_b_temp[dd]->zero();

      offset = y->ptr[dd]*r;
      df_b[dd]->getDiag(offset, &df_diag);
      df_b_temp[dd]->getDiag(offset, &df2_diag);
      for (i = 0; i < r; i++)
         if ((*fy)(dd,i) > 0.0)
         {
            df_diag(i) = (*prob)(i)/(*fy)(dd,i);
            df2_diag(i) = df_diag(i) * (*u)(i) * scale;
         }
      
      // TODO: change to use sub-matrices when available
      // (... but make sure that the rest of R2 is properly zeroed out.)

      MatMatMult(Rs, CblasNoTrans,
                 df_b_temp[dd], CblasNoTrans,
                 R2_b[dd]);
   }

   return 0;
}


int IndepPMF::UpdateGradient()
{
   iter++;

// Update the score vector

   // Get a vector view of the S matrices

   static IVec S_b_vec;

   int i;

   for (i = 0; i < d; i++)
   {
      S_b_new[i]->getAll(&S_b_vec);
   
      // S_new = scale * (w'f + df'u)

      GenMatVecMult(scale, 
                    w_b[i], CblasTrans,
                    prob,               // prob == f
                    0.0, &S_b_vec);

      GenMatVecMult(scale, 
                    df_b[i], CblasTrans,
                    u, 
                    1.0, &S_b_vec);
   }

   // Average gradients if desired

   int n = min(iter, history);

   if (avg_obs && n > 1)
   {
      // S = S + (1/n)*(S_new - S)
      //
      //   = ((n-1)/n)*S + (1/n)*S_new

      for (i = 0; i < d; i++)
      {
         MatScale((n-1.0)/n, S_b[i]);
         MatAddScaled(1.0/n, S_b_new[i], S_b[i]);
      }
   }
   else
   {
      IMat **tmp;

      swap(S_b, S_b_new, tmp);
   }

   return 0;
}


int IndepPMF::UpdateParms()
{
   int i;
   int n = min(iter, history);
   
   real epsilon;
   
   if (eps_exp != 0.0)
      epsilon = eps0/((real)pow((double)iter,(double)eps_exp));
   else
      epsilon = eps0;
   
// Update all b[i]

   if (avg_iters && n > 1)
   {
      for (i = 0; i < d; i++)
      {
         // b_temp =  b + epsilon*n*S_b;

         MatCopy(b[i], b_temp[i]);
         MatAddScaled(epsilon*n, S_b[i], b_temp[i]);
         
         // Projection

         ProbProject(b_temp[i], prior, 1);

         // b = b + (1/n)*(b_temp - b)
      
         MatScale((n-1.0)/n, b[i]);
         MatAddScaled(1.0/n, b_temp[i], b[i]);
      }
   }
   else
   {
      // b = b + epsilon*S_b;

      for (i = 0; i < d; i++)
      {
         MatAddScaled(epsilon, S_b[i], b[i]);
         ProbProject(b[i], prior, 1);
      }

   }
   
   return 0;
}

int IndepPMF::RMLEUpdate()
{
   // use only if not within an HMM
   return (UpdateParms());
}

int IndepPMF::Generate(int rr, int ddim, real rn)
{
   int i, s_d;
   real sum = 0.0;
   
   if (rr < 0 || rr >= r || ddim < 0 || ddim >= d)
      return -1;
   
   s_d = (*s)[ddim];

   for (i = 0; i < s_d; i++)
   {
      sum = sum + (*b[ddim])(i,rr);
      if (rn < sum)
         return i;
   }

   // shouldn't reach here!
   warning("Couldn't generate a discrete output with"
           " IndepPMF::Generate(%d, %d, %f)", rr, ddim, rn);
   
   return -1;
}

int IndepPMF::GenerateMax(int rr, int ddim)
{
   int i, s_d;
   real mx = 0.0;
   int mx_pos = -1;
   
   if (rr < 0 || rr >= r || ddim < 0 || ddim >= d)
      return -1;
   
   s_d = (*s)[ddim];

   for (i = 0; i < s_d; i++)
   {
      if ((*b[ddim])(i,rr) > mx)
      {
         mx = (*b[ddim])(i,rr);
         mx_pos = i;
      }
   }
   
   return mx_pos;
}

void IndepPMF::print(int scrn_width, int col_width, char *num_format)
{
   int i;
   
   for (i = 0; i < d; i++)
   {
      printf("b{%d}(%d x %d):\n\n", i, (*s)[i], r);
      b[i]->print(scrn_width, col_width, num_format);
   }
   
}


void IndepPMF::print_w(int scrn_width, 
                  int col_width, 
                  char *num_format)
{
   printf ("w_b:\n");
   for (int i = 0; i < d; i++)
      w_b[i]->print(scrn_width, col_width, num_format);
}

void IndepPMF::print_df(int scrn_width, 
                   int col_width, 
                   char *num_format)
{
   printf ("df_b:\n");
   for (int i = 0; i < d; i++)
      df_b[i]->print(scrn_width, col_width, num_format);
}

void IndepPMF::print_R(int scrn_width, 
                  int col_width, 
                  char *num_format)
{
   printf("R2_b:\n");
   for (int i = 0; i < d; i++)
      R2_b[i]->print(scrn_width, col_width, num_format);
}

void IndepPMF::print_S(int scrn_width, 
                  int col_width, 
                  char *num_format)
{
   printf("S_b:\n");
   for (int i = 0; i < d; i++)
      S_b[i]->print(scrn_width, col_width, num_format);
}

void IndepPMF::saveXFile(XFile *file)
{
   int i;
   char b_str[10];

   file->printf("\nIndepPMF Parms:\n");   
   
   StochasticClassifier::saveXFile(file);

   file->printf("\ns:\n");
   s->saveXFile(file);

   for (i = 0; i < d; i++)
   {
      snprintf(b_str, 10, "\nb{%d}:\n", i);
      file->printf(b_str);

      b[i]->saveXFile(file);
   }
   
}

void IndepPMF::loadXFile(XFile *file)
{
   allocator->freeAll();

   IVecInt *s_;
   IMat **b_;
   char b_str[10];
   
   file->scanf("\nIndepPMF Parms:\n", NULL);

   StochasticClassifier::loadXFile(file);

   s_ = new IVecInt;

   file->scanf("\ns:\n",NULL);
   s_->loadXFile(file);

   d = s_->n;
   
   b_ = (IMat **) allocator->alloc(sizeof(IMat *)*d);

   for (int i = 0; i < d; i++)
   {
      b_[i] = new(allocator) IMat;

      snprintf(b_str, 10, "\nb{%d}:\n", i);
      file->scanf(b_str, NULL);

      b_[i]->loadXFile(file);
   }

   init(s_, b_, false, false);
}
