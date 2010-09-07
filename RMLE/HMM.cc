/**
 * @file   HMM.cc
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Tue Jul  6 15:47:37 2004
 * 
 * @brief  Hidden Markov Model
 * 
 * $Log: HMM.cc,v $
 * Revision 1.1  2005/05/09 20:52:59  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.6.2.3  2005/02/09 00:16:44  k-squire
 * Added Viterbi stuff, classification of integers.
 *
 * Revision 1.6.2.2  2004/11/07 23:55:48  k-squire
 * Renamed epsilon to eps0.
 * Added decay parameter eps_exp.
 * Added classification for a single discrete input using the marginal pmf
 *    for that input. (Classify (int y, int pos)
 * Added function to get the remaining viterbi sequence.
 * Fixed scale test so that it actually tests for zero and not REAL_EPSILON.
 * *** NOTE: VITERBI CLASSIFICATION IS BROKEN! ***
 *
 * Revision 1.6.2.1  2004/10/24 21:17:50  k-squire
 * Commented out some debugging in Classify.
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
 * Revision 1.1  2004/08/03 06:38:38  k-squire
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

//static char *hmm_CC_Id = "$Id: HMM.cc,v 1.1 2005/05/09 20:52:59 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <string.h>

#include <torch/general.h>



#include "HMM.hh"
#include "StochasticClassifier.hh"
#include "IndepPMF.hh"
#include "gen_defines.h"

HMM::HMM():
      StochasticClassifier(),
      A(NULL),
      b(NULL),
      S_A(NULL),
      S_A_new(NULL),
      f(NULL),
      u(NULL),
      w(NULL),
      w_next(NULL),
      scale(1.0),
      delta(NULL),
      psi(NULL),
      viterbi_hist(0),
      vit_count(0),
      vit_pos(0),
      state(-1),
      prob_mat(NULL),
      eye(NULL),
      tmp_rr(NULL),
      A_temp(NULL),
      tmp_r(NULL)
{
   HMM_allocator = new Allocator;
}


HMM::HMM(StochasticClassifier *b_, 
         bool rand_A,
         real prior_, 
         int viterbi_hist_) :
      StochasticClassifier (b_->r, b_->d, prior_),
      A(NULL),
      b(NULL),
      S_A(NULL),
      S_A_new(NULL),
      f(NULL),
      u(NULL),
      w(NULL),
      w_next(NULL),
      scale(1.0),
      delta(NULL),
      psi(NULL),
      viterbi_hist(0),
      vit_count(0),
      vit_pos(0),
      state(-1),
      prob_mat(NULL),
      eye(NULL),
      tmp_rr(NULL),
      A_temp(NULL),
      tmp_r(NULL)
{
   HMM_allocator = new Allocator;

   init(b_, NULL, rand_A, viterbi_hist_);
}
   
HMM::HMM(StochasticClassifier *b_, 
         IMat *A_, 
         bool rand_A,
         real prior_, 
         int viterbi_hist_,
         bool do_copy, 
         bool retain):
      StochasticClassifier (b_->r, b_->d, prior_),
      A(NULL),
      b(NULL),
      S_A(NULL),
      S_A_new(NULL),
      f(NULL),
      u(NULL),
      w(NULL),
      w_next(NULL),
      scale(1.0),
      delta(NULL),
      psi(NULL),
      viterbi_hist(0),
      vit_count(0),
      vit_pos(0),
      state(-1),
      prob_mat(NULL),
      eye(NULL),
      tmp_rr(NULL),
      A_temp(NULL),
      tmp_r(NULL)
{
   HMM_allocator = new Allocator;

   init(b_, A_, rand_A, viterbi_hist_, do_copy, retain);
}
   
HMM::HMM(StochasticClassifier *b_, 
         real *a_, 
         real prior_, 
         int viterbi_hist_,
         bool do_copy, 
         bool retain):
      StochasticClassifier (b_->r, b_->d, prior_),
      A(NULL),
      b(NULL),
      S_A(NULL),
      S_A_new(NULL),
      f(NULL),
      u(NULL),
      w(NULL),
      w_next(NULL),
      scale(1.0),
      delta(NULL),
      psi(NULL),
      viterbi_hist(0),
      vit_count(0),
      vit_pos(0),
      state(-1),
      prob_mat(NULL),
      eye(NULL),
      tmp_rr(NULL),
      A_temp(NULL),
      tmp_r(NULL)
{
   HMM_allocator = new Allocator;

   IMat *A_ = new(HMM_allocator) IMat(a_, r, r, do_copy, retain);

   init(b_, A_, false, viterbi_hist_, false, false);
}
   
HMM::~HMM()
{
   delete HMM_allocator;
}

void HMM::setObsDist(StochasticClassifier *b_, bool retain)
{
   init(b_, A, false, prior, viterbi_hist, false, false);

   if (retain)
      HMM_allocator->retain(b_,1);
}

int HMM::setViterbiHist(int viterbi_hist_)
{
   int i;
   
   if (viterbi_hist_ == viterbi_hist)
      return 0;
   
   if (viterbi_hist_ < viterbi_hist)
   {
      for (i = viterbi_hist_; i < viterbi_hist; i++)
      {
         HMM_allocator->free(delta[i]);
         HMM_allocator->free(psi[i]);
      }
   }

   delta = (IVec **)HMM_allocator->realloc(delta, 
                                           sizeof(IVec *) * viterbi_hist_);
   psi = (IVecInt **)HMM_allocator->realloc(psi, 
                                            sizeof(IVecInt *) * viterbi_hist_);

   if (viterbi_hist_ > viterbi_hist)
   {
      for (i = viterbi_hist; i < viterbi_hist_; i++)
      {
         delta[i] = new(HMM_allocator) IVec(r);
         delta[i]->zero();
      }
      
      for (i = viterbi_hist; i < viterbi_hist_; i++)
      {
         psi[i] = new(HMM_allocator) IVecInt(r);
         psi[i]->zero();
      }
   }

   viterbi_hist = viterbi_hist_;
   vit_count = 0;
   vit_pos = 0;

   *delta[vit_pos] = *prob;

   return 0;
}

void HMM::addOptions()
{
   // These are set in StochasticClassifier

//    addIOption("history", &history, 1, "averaging history");

//    addBOption("average S", &avg_obs, false, "average S (log probabilities, aka observations)");
//    addBOption("average parms", &avg_iters, false, "average parameters");
   
//    addROption("eps0", &eps0, 0.001, "learning rate");
//    addROption("eps_exp", &eps_exp, 0.000, "learning rate decay");
//    addROption("prior", &prior, 0.0, "prior on probabilities");

//    addBOption("debug", &debug, false, "print debug info");

}

void HMM::init(StochasticClassifier *b_, 
               IMat *A_, 
               bool rand_A,
               real prior_,
               int viterbi_hist_,
               bool do_copy, 
               bool retain)
{
   int i;

   b = b_;
   f = b->prob;

   d = b->d;
   r = b->r;

   StochasticClassifier::init(r,d, prior_);

   label = b->label;

   if (A_)
   {
      if (do_copy)
         A = new(HMM_allocator) IMat(*A_);
      else
      {
         A = A_;
         if (retain)
            HMM_allocator->retain(A,1);
      }
   }
   else
   {
      A = new(HMM_allocator) IMat(r,r);
      InitTransProb(A, prior, rand_A);
   }
   
   p = r*r;                             // Number of parameters

   S_A = new(HMM_allocator) IMat(r,r);
   S_A->zero();

   S_A_new = new(HMM_allocator) IMat(r,r);
   S_A_new->zero();

   u = new(HMM_allocator) IVec(r);
   u->zero();

   w = new(HMM_allocator) IMat(r,p);
   w->zero();
   w_next = new(HMM_allocator) IMat(r,p);
   w_next->zero();

   log_prob = 0;                        // log probability
   state = -1;                          // HMM state

   Rs = new(HMM_allocator) IMat(r,r);
   R1 = new(HMM_allocator) IMat(r,r);
   R2 = new(HMM_allocator) IMat(r,p);
   R2->zero();

   prob_mat = new(HMM_allocator) IMat(r,r);
   eye = new(HMM_allocator) IMat(r,r);
   eye->eye();
   tmp_rr = new(HMM_allocator) IMat(r,r);
   A_temp = new(HMM_allocator) IMat(r,r);

   tmp_r = new(HMM_allocator) IVec(r);

   // Viterbi stuff

   viterbi_hist = viterbi_hist_;
   
//   probs = (IVec **)HMM_allocator->alloc(sizeof(IVec *) * viterbi_hist);
   delta = (IVec **)HMM_allocator->alloc(sizeof(IVec *) * viterbi_hist);
   psi = (IVecInt **)HMM_allocator->alloc(sizeof(IVecInt *) * viterbi_hist);

   // the next three for loops are separate in order to keep them
   // contiguous in memory.

//   for (i = 0; i < viterbi_hist; i++)
//      probs[i] = new(HMM_allocator) IVec(r);
   
   for (i = 0; i < viterbi_hist; i++)
      delta[i] = new(HMM_allocator) IVec(r);

   for (i = 0; i < viterbi_hist; i++)
      psi[i] = new(HMM_allocator) IVecInt(r);

   *delta[0] = *prob;

   vit_count = 0;
   vit_pos = 0;
}

void HMM::reset()
{
   int i;

   StochasticClassifier::reset();

   S_A->zero();
   S_A_new->zero();
   u->zero();
   w->zero();
   w_next->zero();
   R2->zero();
   
   for (i = 1; i < viterbi_hist; i++)
   {
      delta[i]->zero();
      psi[i]->zero();
   }

   *delta[0] = *prob;
   vit_count = 0;
   vit_pos = 0;
   
   b->reset();

}


int HMM::Classify(real *y)
{
   int i;

   /*
   if (debug)
   {
      printf ("Classifying");
      for (i = 0; i < d; i++)         // TODO: d is not set in HMM!
         printf("  %20.14f",y[i]);
   }
   */
   
   // Update u = P(X(t)|Y(1)...Y(t-1))

   MatVecMult(A, CblasTrans, prob, u);  // u = A'prob

   // Classify according to prob. density function

   b->Classify(y);
   f = b->prob;

   // HMM probabilities = f.*u (normalized) = P(X(t)|Y(1)...Y(t))

   scale = VecDot(f,u);               // inner product

//   if (scale < REAL_EPSILON)
   if (scale == 0.0)
   {
      if (debug)
      {
         printf("Cannot classify input! Likelihood is zero!\n");
         printf("Resetting...\n");
      }
      //reset();
      //return -2;
      scale = REAL_EPSILON;
   }
   else
      scale = 1/scale;

   VecCopy(f, prob);
   VecDotTimes(scale, u, prob);         // prob = scale * u .* f

   if (viterbi_hist <= 1)
   {
      prob->vmax(&state);  // the current state will be the maximum
                           // probability 
   }
   else
   {
      if (vit_count == 0)
      {
         *delta[vit_pos] = *prob;
         vit_pos = (vit_pos+1)%viterbi_hist;
         vit_count++;
      }
      else
      {
         static IMat A_tmp;
         static IVec row;

         int vit_prev = vit_pos - 1;

         if (vit_prev < 0)
            vit_prev = viterbi_hist-1;
      
         A_tmp = *A;
   
         for (i = 0; i < r; i++)
         {
            A_tmp.getRow(i, &row);
            VecScale((*delta[vit_prev])[i], &row);
         }
      
         A_tmp.mmax(delta[vit_pos],psi[vit_pos],1);
      
         VecDotTimes(1.0,f,delta[vit_pos]);

         delta[vit_pos]->normalize();

         //backtrack

         if (vit_count+1 < viterbi_hist)
         {
            state = -1;
            vit_count++;
         }
         else
         {
            int vit_backtrack = vit_pos;
            
            delta[vit_pos]->vmax(&state);
            
            for (i = 0; i < viterbi_hist-1; i++)
            {
               state = (*psi[vit_backtrack])[state];

               vit_backtrack = vit_backtrack -1;
               if (vit_backtrack < 0)
                  vit_backtrack = viterbi_hist - 1;
            }
         }
         
         vit_pos = (vit_pos + 1)%viterbi_hist;
      }
      // TODO: if desired, we could also calculate backward probs here
   }
   
   return state;
}

int HMM::Classify(int *y)
{
   int i;

   /*
   if (debug)
   {
      printf ("Classifying");
      for (i = 0; i < d; i++)         // TODO: d is not set in HMM!
         printf("  %20.14f",y[i]);
   }
   */
   
   // Update u = P(X(t)|Y(1)...Y(t-1))

   MatVecMult(A, CblasTrans, prob, u);  // u = A'prob

   // Classify according to prob. density function

   ((IndepPMF *)b)->Classify(y);
   f = b->prob;

   // HMM probabilities = f.*u (normalized) = P(X(t)|Y(1)...Y(t))

   scale = VecDot(f,u);               // inner product

//   if (scale < REAL_EPSILON)
   if (scale == 0.0)
   {
      if (debug)
      {
         printf("Cannot classify input! Likelihood is zero!\n");
         printf("Resetting...\n");
      }
      reset();
      return -1;
   }
   else
      scale = 1/scale;

   VecCopy(f, prob);
   VecDotTimes(scale, u, prob);         // prob = scale * u .* f

   if (viterbi_hist <= 1)
   {
      prob->vmax(&state);  // the current state will be the maximum
                           // probability 
   }
   else
   {
      // TODO: change to a circular buffer 

      // run the viterbi algorith back (viterbi_hist) steps      
   
      // shift everything down...

      IVec *delta_temp = delta[viterbi_hist-1];
      IVecInt *psi_temp = psi[viterbi_hist-1];
      
      for (i = viterbi_hist-1; i >= 1; i--)
      {
//         probs[i] = probs[i-1];
         delta[i] = delta[i-1];    // not the most efficient. :-(
         psi[i] = psi[i-1];
      }

      delta[0] = delta_temp;
      psi[0] = psi_temp;

//      probs[0] = prob;

      static IMat A_tmp;
      static IVec row;
   
      A_tmp = *A;
   
      for (i = 0; i < r; i++)
      {
         A_tmp.getRow(i, &row);
         VecScale((*delta[1])[i], &row);
      }
   
      A_tmp.mmax(delta[0],psi[0],1);
   
      VecDotTimes(1.0,f,delta[0]);

      // need to normalize deltas

      real s = delta[0]->sum();
      VecScale(1.0/s, delta[0]);

      // backtrack
   
      delta[0]->vmax(&state);

      for (i = 1; i < viterbi_hist; i++)
         state = (*psi[i-1])[state];

      // TODO: if desired, we could also calculate backward probs here
   }

   return state;
}

int HMM::Classify(int y, int pos)
{
   int i;

   // Update u = P(X(t)|Y(1)...Y(t-1))

   MatVecMult(A, CblasTrans, prob, u);  // u = A'prob

   // Classify according to prob. density function

   ((IndepPMF *)b)->Classify(y,pos);
   f = b->prob;

   // HMM probabilities = f.*u (normalized) = P(X(t)|Y(1)...Y(t))

   scale = VecDot(f,u);               // inner product

//   if (scale < REAL_EPSILON)
   if (scale == 0.0)
   {
      if (debug)
      {
         printf("Cannot classify input! Likelihood is zero!\n");
         printf("Resetting...\n");
      }
      reset();
      return -1;
   }
   else
      scale = 1/scale;

   VecCopy(f, prob);
   VecDotTimes(scale, u, prob);         // prob = scale * u .* f

   if (viterbi_hist == 1)
   {
      prob->vmax(&state);  // the current state will be the maximum
                           // probability 
   }
   else
   {
      // TODO: change to a circular buffer 

      // run the viterbi algorith back (viterbi_hist) steps      
   
      // shift everything down...

      IVec *delta_temp = delta[viterbi_hist-1];
      IVecInt *psi_temp = psi[viterbi_hist-1];
      
      for (i = viterbi_hist-1; i >= 1; i--)
      {
//         probs[i] = probs[i-1];
         delta[i] = delta[i-1];    // not the most efficient. :-(
         psi[i] = psi[i-1];
      }

      delta[0] = delta_temp;
      psi[0] = psi_temp;

//      probs[0] = prob;

      static IMat A_tmp;
      static IVec row;
   
      A_tmp = *A;
   
      for (i = 0; i < r; i++)
      {
         A_tmp.getRow(i, &row);
         VecScale((*delta[1])[i], &row);
      }
   
      A_tmp.mmax(delta[0],psi[0],1);
   
      VecDotTimes(1.0,f,delta[0]);

      // need to normalize deltas

      real s = delta[0]->sum();
      VecScale(1.0/s, delta[0]);

      // backtrack
   
      delta[0]->vmax(&state);

      for (i = 1; i < viterbi_hist; i++)
         state = (*psi[i-1])[state];

      // TODO: if desired, we could also calculate backward probs here
   }
   
   return state;
}

int HMM::getViterbiSeq(IVecInt *seq)
{
   int i;
   int state;

   seq->resize(viterbi_hist);

   delta[0]->vmax(&state);
   (*seq)[viterbi_hist-1] = state;

   for (i = 1; i < viterbi_hist; i++)
   {
      state = (*psi[i-1])[state];
      (*seq)[viterbi_hist-1-i] = state;
   }

   return 0;
}

int HMM::Updatew()
{
   // Update w = du/d(phi(l))

   MatMatMult(R1, CblasNoTrans, 
              w, CblasNoTrans, 
              w_next);
   MatAdd(R2, w_next);                  // w(t+1) = R1*w(t) + R2

   IMat *tmp;
   swap(w, w_next, tmp);

   b->Updatew();
}


int HMM::UpdateR(IMat *Rs_,
                 IMat *R1_, 
                 IVec *u_,
                 real scale_)
{
   int i,j;

   // Update Rs, R1, and R2

   // NOTE: THIS MUST BE DONE AFTER UPDATING w!!!

//  Rs = A' * (eye(r,r) - F*u*ones(1,r)*scale)
//     = A' * (eye(r,r) - prob*ones(1,r));

   prob_mat->vecFillCols(prob);
   tmp_rr->eye();
   MatSub(prob_mat, tmp_rr);          // tmp = (eye - prob*ones)
   MatMatMult(A, CblasTrans,
              tmp_rr, CblasNoTrans,
              Rs);                    // Rs = A' * tmp

//  R1 = Rs * F * scale 
//     = Rs .* repmat(f',r,1) * scale;
   
   static IVec *rowIndex = new(allocator) IVec;

   MatCopy(Rs,R1);   // R1 = Rs
   
   for (i = 0; i < r; i++)
   {
      R1->getRow(i, rowIndex);
      VecDotTimes(scale, f, rowIndex);
   }

//   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//   %
//   % R2 calculation:
//   %
//   % For transition probabilities (A = a(ij), we need to calculate
//   %
//   %   R2(:,m) = dA'*F*u*scale   (m = pos of a(ij) in phi, column order)
//   %
//   % which, assuming column major storage, is equivalent to
//   %
//   %        [g(1) g(2) ... g(r)   0    0  ...   0        0    0  ...   0   ]
//   %        [  0    0  ...   0  g(1) g(2) ... g(r)       .       .     .   ]
//   %   R2 = [  .       .     .    0    0  .     0  ...   .         .   .   ]
//   %        [  .         .   .    .         .   .        0    0  ...   0   ]
//   %        [  0  ...  ...   0    0   ..  ...   0      g(1) g(2) ... g(r)  ]
//   %
//   % where 
//   %
//   %   g(i) = f(i)*u(i)*scale = Prob(X = i) = probability of being
//   %                                          in state i
//   %
//   % If m=a_ij were in row major order, then
//   %
//   %        [g(1)            g(2)                   g(r)                ]
//   %   R2 = [    g(1)            g(2)         ...       g(r)        ... ]
//   %        [        ...             ...                    ...         ]
//   %        [           g(1)            g(2)                   g(r)     ]
//   %
//   % with zeros on the off diagonals.
//   %
//   %%%%%%%%%%%%%%%%%%%%%%%%%%

   // R2 should already have zeros on off diagonals, so no need to
   // clear it

   static IVec *R2diag = new IVec;

   // C/C++ uses row-major storage

   for (i = 0, j = 0; i < r; i++, j+=r)
   {
      R2->getDiag(j, R2diag);
      R2diag->fill((*prob)(i));
   }
   
   b->UpdateR(Rs, R1, u, scale);

   return 0;
}

int HMM::UpdateGradient()
{
   iter++;

#ifdef DEBUG
   printf("HMM iter: %d\n\n", iter);
#endif

   // Update the score vector

   // S_A_new = scale * w'*f

   static IVec *S_A_vec = new IVec;
   
   S_A_new->getAll(S_A_vec);
   GenMatVecMult(scale, w, CblasTrans, f, 0.0, S_A_vec);
   
   // Average gradients if desired

   int n = min(iter, history);

   if (avg_obs && n > 1)
   {
      // S_A = S_A + (1/n)*(S_A_new - S_A)
      //   = ((n-1)/n)*S_A + (1/n)*S_A_new

      MatScale((n-1.0)/n, S_A);
      MatAddScaled(1.0/n, S_A_new, S_A);
      
   }
   else
   {
      IMat *tmp;
      swap(S_A, S_A_new, tmp);
   }

   b->UpdateGradient();

   return 0;
}
   
int HMM::UpdateParms()
{
// Parameter update of the observation density functions is
// independent of whatever happens here.

   int n = min(iter, history);

   real epsilon;
   
   if (eps_exp != 0.0)
      epsilon = eps0/((real)pow((double)iter,(double)eps_exp));
   else
      epsilon = eps0;
   
// Update A

   static IVec *A_vec = new(allocator) IVec;

   if (avg_iters && n > 1)
   {
      // A_temp =  A + epsilon*n*S_A;

      MatCopy(A, A_temp);
      MatAddScaled(epsilon*n, S_A, A_temp);

      // Projection

      ProbProject(A_temp, prior, 2);

      // A = A + (1/n)*(A_temp - A)

      MatScale((n-1.0)/n, A);
      MatAddScaled(1.0/n, A_temp, A);
   }
   else
   {
      // A =  A + epsilon*S_A;

      MatAddScaled(epsilon, S_A, A);
      ProbProject(A, prior, 2);
   }

   b->UpdateParms();

   return 0;
}
   
int HMM::RMLEUpdate()
{
   if (debug)
   {
      printf("*******************\n\n");
 
      printf("prob:\n");
      prob->print(90,20,"%20.14f");
      b->prob->print(90,20,"%20.14f");
   }
   
   Updatew();

   if (debug)
   {
      print_w(90,20,"%20.14f");
      b->print_w(90,20,"%20.14f");
      
      printf("*******************\n\n");
   }

   UpdateR();

   if (debug)
   {
      b->print_df(90,20,"%20.14f");

      printf("*******************\n\n");

      print_R(90,20,"%20.14f");
      b->print_R(90,20,"%20.14f");

      printf("*******************\n\n");
   }

   UpdateGradient();

   if (debug)
   {
      print_S(90,20,"%20.14f");
      b->print_S(90,20,"%20.14f");

      printf("*******************\n\n");
   }

   UpdateParms();

   return 0;
}

int HMM::InitTransProb(IMat *A_, 
                       real prior,
                       bool rand_init)
{
   int i;

   if (rand_init)
   {
      A_->rand();
         
      ProbProject(A_, prior, 2);
   }
   else
   {
      real prob = 1.0/r;

      A_->fill(prob);
   }
   
   return 0;
}

void HMM::print(int scrn_width, int col_width, char *num_format)
{
   printf ("A:\n");
   A->print(scrn_width, col_width, num_format);
   
   b->print(scrn_width, col_width, num_format);
}

void HMM::print_w(int scrn_width, 
                  int col_width, 
                  char *num_format)
{
   printf ("w_A:\n");
   w->print(scrn_width, col_width, num_format);
}

void HMM::print_df(int scrn_width, 
                   int col_width, 
                   char *num_format)
{
}

void HMM::print_R(int scrn_width, 
                  int col_width, 
                  char *num_format)
{
   printf("Rs:\n");
   Rs->print(scrn_width, col_width, num_format);
   printf("R1:\n");
   R1->print(scrn_width, col_width, num_format);
   printf("R2_A:\n");
   R2->print(scrn_width, col_width, num_format);
}

void HMM::print_S(int scrn_width, 
                  int col_width, 
                  char *num_format)
{
   printf("S_A:\n");
   S_A->print(scrn_width, col_width, num_format);
}

void HMM::saveXFile(XFile *file)
{
   file->printf("HMM Parms:\n");

   StochasticClassifier::saveXFile(file);
   
   file->printf("A:\n");
   A->saveXFile(file);

   file->printf("prior: " REAL_FORMAT_OUT "\n", prior);
}

void HMM::loadXFile(XFile *file)
{
   char buffer[256];
   real prior_;

   file->gets(buffer,256);
   if (strcmp(buffer, "HMM Parms:\n") != 0)
      error("Could not read \"HMM Parms:\"");
   
   StochasticClassifier::loadXFile(file);

   file->gets(buffer,256);
   if (strcmp(buffer, "A:\n") != 0)
      error("Could not read \"A:\"");

   IMat *A_ = new IMat;
   A_->loadXFile(file);

   if (file->scanf("prior: " REAL_FORMAT_IN "\n", &prior_) < 1)
      prior_ = 0.0;

   HMM_allocator->freeAll();

   // TODO: cludge! fix me... assume it will be followed by a call to
   // setObsDist

   A = A_;
   prior = prior_;
   viterbi_hist = 1;

   return;
//   return init(NULL, A_, false, prior_, 1, false, true);
}
