/*
 * Copyright (C) 2000-2003 Matt Kleffner <kleffner@ifp.uiuc.edu>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA
 * or visit http://www.fsf.org/licenses/gpl.html
 */

/**
 * @file   filter.c
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Sat Feb 02 2002
 *
 * @brief  Structures and functions necessary for filtering data using both
 *         traditional and warped methods. See Matt Kleffner's MS thesis for a
 *         discussion of the warped algorithms
 *
 * $Log: filter.c,v $
 * Revision 1.1  2005/05/09 20:58:08  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2005/03/18 17:13:20  mrmcclai
 * Imported Sources
 *
 * Revision 1.5  2004/03/29 21:57:42  k-squire
 * Changed alloc.h to malloc.h, to avoid compilation error with gcc.
 *
 * Revision 1.4  2003/02/03 19:36:18  kleffner
 * Added comments, license
 *
 * Revision 1.3  2003/01/29 01:19:55  kleffner
 * Fixed bug: filter bug in FIR filters
 * Fixed bug: NULL check bug in create_filter
 *
 * Revision 1.2  2002/12/10 20:33:43  kleffner
 * Fixed bug: removed pole-zero cancellation from warped filter, thereby
 * reducing the order of the filter
 *
 * Revision 1.1  2002/05/14 21:46:03  kleffner
 * Initial checkin
 *
 */

#include "filter.h"
#include <malloc.h>
#include <math.h>
#include <string.h>

#define MAX(X,Y) ((X)<(Y)?(Y):(X))
#define MIN(X,Y) ((X)<(Y)?(X):(Y))

/**
 * Filtering process is
 *    y[n] = b0*x[n] + b1*x[n-1] + ... + bN*x[n-N] +
 *                     a1*y[n-1] + a2*y[n-2] + ... + aM*y[n-M]
 * States are saved in filter structure
 * Note that a0=1 and is not stored in the filter structure
 * Minimum N, M is 0, but not both because that is just y[n]=b0*x[n]
 * filter->numcoeffs is pointer to bi's, ascending in i from 0 to N
 * filter->dencoeffs is pointer to ai's, ascending in i from 1 to N
 * filter->states    is pointer to states, initialized to 0
 * 
 * @param  sfilter pointer to filter structure containing
 *                    filter coefficients and states
 * @param  input   pointer to input array
 * @param  length  length of input and output arrays
 * @param  output  pointer to array where filtered input is to be stored
 */
void filter_data(filter *sfilter, double *input, int length, double *output)
{
   register int i, j, istate, itemp, limit;
   int Omax = sfilter->Omax, Omin = sfilter->Omin;
   int N = sfilter->N, M = sfilter->M;
   double *stateptr = sfilter->states;
   double *numcoeffs = sfilter->numcoeffs, *dencoeffs = sfilter->dencoeffs;

   istate = sfilter->istate;
   for(i=0; i<length; ++i)
   {
      // generate output
      output[i] = numcoeffs[0] * input[i] + stateptr[istate];

      // update states
      for(j=0; j<Omin; ++j)
      {
         itemp = (istate + 1) % Omax;
         stateptr[istate] = dencoeffs[j] * output[i] +
                            numcoeffs[j+1] * input[i] + stateptr[itemp];
         istate = itemp;
      }

      // update remaining states
      if( N == Omax )
      {
         for(j=Omin, limit=N-1; j<limit; ++j)
         {
            itemp = (istate + 1) % Omax;
            stateptr[istate] = numcoeffs[j+1] * input[i] + stateptr[itemp];
            istate = itemp;
         }
         stateptr[istate] = numcoeffs[N] * input[i];
         istate = (istate + 1) % Omax;
      }
      else
      {
         for(j=Omin, limit=M-1; j<limit; ++j)
         {
            itemp = (istate + 1) % Omax;
            stateptr[istate] = dencoeffs[j] * output[i] + stateptr[itemp];
            istate = itemp;
         }
         stateptr[istate] = dencoeffs[limit] * output[i];
         istate = (istate + 1) % Omax;
      }
   }
   sfilter->istate = istate;
}

/**
 * Resets filter states to 0
 * 
 * @param  sfilter pointer to filter structure
 */
void reset_filter(filter *sfilter)
{
   sfilter->istate = 0;
   memset(sfilter->states, 0, sfilter->Omax*sizeof(double));
}

/**
 * Create filter structure. Filtering process is
 *    y[n] = b0*x[n] + b1*x[n-1] + ... + bN*x[n-N] +
 *                     a1*y[n-1] + a2*y[n-2] + ... + aM*y[n-M]
 * States are saved in filter structure
 * Note that a0=1 and is not stored in the filter structure
 * Minimum N, M is 0, but not both because that is just y[n]=b0*x[n]
 * filter->numcoeffs is pointer to bi's, ascending in i from 0 to N
 * filter->dencoeffs is pointer to ai's, ascending in i from 1 to N
 * filter->states    is pointer to states, initialized to 0
 * 
 * @param  numorder order of feedforward coefficients
 * @param  denorder order of feedback coefficients
 *
 * @return new filter structure on success, NULL on error
 */
filter* create_filter(int numorder, int denorder)
{
   filter *sfilter;
   sfilter            = (filter *)malloc(sizeof(filter));
   sfilter->N         = numorder;
   sfilter->M         = denorder;
   sfilter->Omax      = MAX(numorder, denorder);
   sfilter->Omin      = MIN(numorder, denorder);
   sfilter->numcoeffs = (double *)malloc((numorder+1)*sizeof(double));
   if(denorder>0)
   {
      sfilter->dencoeffs = (double *)malloc(denorder*sizeof(double));
   }
   else sfilter->dencoeffs = NULL;
   if(sfilter->Omax>0)
   {
      sfilter->states    = (double *)malloc(sfilter->Omax*sizeof(double));
   }
   // degenerate case of N=M=0, keep dummy state so filter_data still works
   else sfilter->states  = (double *)malloc(sizeof(double));
   
   if( (sfilter==NULL) || (sfilter->states==NULL) ||
       (sfilter->numcoeffs==NULL && numorder>=0)  ||
       (sfilter->dencoeffs==NULL && denorder>0)   )
   {
      destroy_filter(sfilter);
      return(NULL);
   }
   else
   {
      reset_filter(sfilter);
      return(sfilter);
   }
}

/**
 * @param  sfilter pointer to filter structure
 */
void destroy_filter(filter *sfilter)
{
   if(sfilter!=NULL)
   {
      free(sfilter->numcoeffs);
      free(sfilter->dencoeffs);
      free(sfilter->states);
      free(sfilter);
   }
}

/**
 * @param  swarparrays pointer to warparrays structure
 * @param  warpcoeffs  pointer to array of warped coefficients
 * @param  ingain      gain to apply to input of filter
 * @param  sfilter     pointer to filter structure where coefficients are placed
 */
void calculate_warpfilter(warparrays *swarparrays, double *warpcoeffs,
                          double ingain, filter *sfilter)
{
   register int i, j, k;
   int O = swarparrays->order;
   double *filtnum = sfilter->numcoeffs, *filtden = sfilter->dencoeffs;
   double *wdmat = swarparrays->denmatrix, *wnvec = swarparrays->numvector;
   double sum, scale;
   int Op1 = O + 1;

   // normalize a0 term to 1, scale remaining ai's and bi's by 1/a0
   for(j=0, sum=wdmat[0]; j<O; ++j)
   {
      sum += wdmat[j+1] * warpcoeffs[j];
   }

   scale = 1.0 / sum;

   k = Op1;
   for(i=0; i<O; ++i)
   {
      for(j=0, sum=wdmat[k++]; j<O; ++j)
      {
         sum += wdmat[k++] * warpcoeffs[j];
      }
      filtden[i] = - scale * sum;
   }

   scale = ingain * scale;
   for(j=0; j<O; ++j)
   {
      filtnum[j] = scale * wnvec[j];
   }
}

/**
 * @param  order order of warped coefficients to be generated
 * @param  alpha warp parameter
 */
warparrays* create_warparrays(int order, double alpha)
{
   double *temp;            // temp array freed after all arrays created
   warparrays *swarparrays;
   swarparrays            = (warparrays *)malloc(sizeof(warparrays));
   swarparrays->order     = order;
   swarparrays->alpha     = alpha;
   swarparrays->numvector = (double *)malloc(order*sizeof(double));
   swarparrays->denmatrix = (double *)malloc(
                                      (order+1)*(order+1)*sizeof(double));
   temp                   = (double *)malloc((order+1)*sizeof(double));

   if( (swarparrays==NULL) || (swarparrays->numvector==NULL) ||
       (swarparrays->denmatrix==NULL) || (temp==NULL) )
   {
      destroy_warparrays(swarparrays);
      free(temp);
      return(NULL);
   }
   else
   {
      register int i, j, k, l, op1;
      double eq1[2], eq2[2], tempcurr, tempprev, scalefactor;

      // do polynomial multiplication to construct denmatrix and numvector
      eq1[0] = 1; eq1[1] = -alpha; // flipped version of (-alpha+z^(-1))
      eq2[0] = -alpha; eq2[1] = 1; // flipped version of (1-alpha*z^(-1))
      scalefactor = sqrt(1-alpha*alpha);

      temp[0] = eq2[1]; temp[1] = eq2[0];
      op1 = order + 1;
      for(j=1; j<order; ++j)
      {
         tempprev = temp[0];
         temp[0] = eq2[1] * temp[0];
         for(k=1; k<(j+1); ++k)
         {
            tempcurr = temp[k];
            temp[k] = eq2[1] * tempcurr + eq2[0] * tempprev;
            tempprev = tempcurr;
         }
         temp[j+1] = eq2[0] * tempprev;

         // at j=order-2, numvector=scalefactor*temp[0 to order-1];
         if(j==(order-2))
         {
            for(k=0; k<order; ++k)
            {
               swarparrays->numvector[k] = temp[k] * scalefactor;
            }
         }
      }
      // move temp to 0th column of denmatrix
      for(j=0; j<op1; ++j)
      {
         swarparrays->denmatrix[op1*j] = temp[j];
      }

      // calculate rest of columns of denmatrix
      for(i=1; i<op1; ++i)
      {
         temp[0] = eq2[1]; temp[1] = eq2[0];
         for(j=1; j<(order-i); ++j)
         {
            tempprev = temp[0];
            temp[0] = eq2[1] * temp[0];
            for(k=1; k<(j+1); ++k)
            {
               tempcurr = temp[k];
               temp[k] = eq2[1] * tempcurr + eq2[0] * tempprev;
               tempprev = tempcurr;
            }
            temp[j+1] = eq2[0] * tempprev;
         }
         for(j=(order-i); j<order; ++j)
         {
            tempprev = temp[0];
            temp[0] = eq1[1] * temp[0];
            for(k=1; k<(j+1); ++k)
            {
               tempcurr = temp[k];
               temp[k] = eq1[1] * tempcurr + eq1[0] * tempprev;
               tempprev = tempcurr;
            }
            temp[j+1] = eq1[0] * tempprev;
         }
         // move temp to ith column of denmatrix
         for(j=0; j<op1; ++j)
         {
            swarparrays->denmatrix[ op1 * j + i ] = -temp[j];
         }
      }

      free(temp);
      return(swarparrays);
   }
}

/**
 * @param  sfilter pointer to filter structure
 */
void destroy_warparrays(warparrays *swarparrays)
{
   if(swarparrays!=NULL)
   {
      free(swarparrays->numvector);
      free(swarparrays->denmatrix);
      free(swarparrays);
   }
}
