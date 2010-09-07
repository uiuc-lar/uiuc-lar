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
 * @file   lpc.c
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Fri Jan 24 2002
 *
 * @brief  lpc structure and associated functions. Notation borrowed from
 * L.R. Rabiner and R.W. Schafer, "Digital Processing of Speech Signals,"
 * Prentice-Hall, 1978
 *
 * $Log: lpc.c,v $
 * Revision 1.1  2005/05/09 20:58:09  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2005/03/18 17:13:20  mrmcclai
 * Imported Sources
 *
 * Revision 1.5  2004/03/29 21:57:42  k-squire
 * Changed alloc.h to malloc.h, to avoid compilation error with gcc.
 *
 * Revision 1.4  2003/02/03 19:43:21  kleffner
 * Added comments, license
 *
 * Revision 1.3  2002/12/18 22:36:00  kleffner
 * Added function to convert reflection coefficients into LPCs
 *
 * Revision 1.2  2002/12/09 21:22:57  kleffner
 * Fixed bug: variable "order" in autocorr_lpc is supposed to be int, not double
 *
 * Revision 1.1  2002/05/14 21:46:07  kleffner
 * Initial checkin
 *
 */

#include "lpc.h"
#include <malloc.h>
#include <math.h>

/**
 * @param  slpc pointer to lpc structure
 */
void calculate_lpc(lpc *slpc)
{
   double sum, *E, *acorr, *alast, *lpc, *refl;
   register int i,j,im1,jm1;
   int acorrlen, order;
   order = slpc->order;

   E = slpc->E;      acorr = slpc->acorr;  alast = slpc->alast;
   lpc = slpc->lpc;  refl  = slpc->refl;
   acorrlen = order + 1;
   if(acorr[0]==0) E[0]=1e-9;
   else         E[0]=acorr[0];
   refl[0]=acorr[1]/E[0];
   lpc[0]=refl[0];
   alast[0]=lpc[0];
   E[1]=(1-refl[0]*refl[0])*E[0];

   for(i=2,im1=1;i<=order;i++,im1++){
      sum=0;
      for(j=1; j<i; j++)
      {
         sum+=alast[j-1]*acorr[i-j];
      }
      refl[im1]=(acorr[i]-sum)/E[im1];
      lpc[im1]=refl[im1];
      for(j=1,jm1=0;j<i;j++,jm1++)
      {
         lpc[jm1]=alast[jm1]-refl[im1]*alast[im1-j];
      }
      for(jm1=0;jm1<(i-1);jm1++)
      {
         alast[jm1]=lpc[jm1];
      }
      alast[im1]=lpc[im1];
      E[i]=(1-refl[im1]*refl[im1])*E[im1];
   }

   for(i=1, im1=0, sum=acorr[0]; i<acorrlen; ++i, ++im1)
   {
      sum-=lpc[im1]*acorr[i];
   }
   slpc->gain = sqrt(sum);
}

/**
 * @param  slpc pointer to lpc structure
 */
void refl2lpc(lpc *slpc)
{
   double *alast, *lpc, *refl;
   register int i,j,im1,jm1;
   int order;

   order = slpc->order;
   alast = slpc->alast; lpc = slpc->lpc; refl  = slpc->refl;
   lpc[0]=refl[0];
   alast[0]=lpc[0];

   for(i=2,im1=1;i<=order;i++,im1++){
      lpc[im1]=refl[im1];
      for(j=1,jm1=0;j<i;j++,jm1++)
      {
         lpc[jm1]=alast[jm1]-refl[im1]*alast[im1-j];
      }
      for(jm1=0;jm1<(i-1);jm1++)
      {
         alast[jm1]=lpc[jm1];
      }
      alast[im1]=lpc[im1];
   }
}

/**
 * Useful for finding magnitude squared of FIR filter 1-sum(k=1:P,lpc[k]*z^-k)
 * to get magnitude squared, apply fft to lpcacorr
 *
 * @param  lpc structure used in calculating autocorrelation of lpc's
 * @param  length order+1 array to hold resulting autocorrelation coefficients
 */
void autocorr_lpc(lpc *slpc, double *lpcacorr)
{
   register int i, j, k;
   double *ptr1, *ptr2, sum;
   int order;
   order = slpc->order;
   ptr1 = slpc->lpc;

   for(i=0, sum=1; i<order; ++i)
   {
      sum += ptr1[i] * ptr1[i];
   }
   lpcacorr[0] = sum;

   for(j=1, ptr2=ptr1+1; j<order; ++j, ++ptr2)
   {
      sum = -ptr1[j-1];
      for(i=0; i<(order-j); ++i)
      {
         sum += ptr1[i] * ptr2[i];
      }
      lpcacorr[j] = sum;
   }
   lpcacorr[order] = -ptr1[order-1];
   return;
}

/**
 * @param  order the order of the LPCs
 *
 * @return new lpc structure on success, NULL on error
 */
lpc* create_lpc(int order)
{
   lpc *slpc;
   slpc        = (lpc *)malloc(sizeof(lpc));
   slpc->order = order;
   slpc->lpc   = (double *)malloc(order*sizeof(double));
   slpc->refl  = (double *)malloc(order*sizeof(double));
   slpc->acorr = (double *)malloc((order+1)*sizeof(double));
   slpc->E     = (double *)malloc((order+1)*sizeof(double));
   slpc->alast = (double *)malloc(order*sizeof(double));

   if( (slpc==NULL) || (slpc->lpc==NULL) || (slpc->refl==NULL) ||
       (slpc->acorr==NULL) || (slpc->E==NULL) || (slpc->alast==NULL) )
   {
      destroy_lpc(slpc);
      return(NULL);
   }
   else return(slpc);
}

/**
 * @param  slpc pointer to lpc structure
 */
void destroy_lpc(lpc *slpc)
{
   if(slpc!=NULL)
   {
      free(slpc->lpc);
      free(slpc->refl);
      free(slpc->acorr);
      free(slpc->E);
      free(slpc->alast);
      free(slpc);
   }
}
