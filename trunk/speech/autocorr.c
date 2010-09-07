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
 * @file   autocorr.c
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Fri Jan 24 2002
 *
 * @brief  Standard and warped autocorrelation functions. See Matt Kleffner's
 *         MS thesis for more info on warped autocorrelation
 *
 * $Log: autocorr.c,v $
 * Revision 1.1  2005/05/09 20:58:08  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2005/03/18 17:13:20  mrmcclai
 * Imported Sources
 *
 * Revision 1.2  2003/02/03 19:31:58  kleffner
 * Added comments, license
 *
 * Revision 1.1  2002/05/14 21:46:01  kleffner
 * Initial checkin
 *
 */

#include "autocorr.h"
#include <math.h>

/**
 * @param data    pointer to data array used to calculate autocorrelation
 * @param length  length of data array
 * @param lolag   lowest lag value of autocorrelation 
 * @param hilag   highest lag value of autocorrelation
 * @param acorr   array for autocorrelation values
 */
void autocorrN(double *data, int length, int lolag, int hilag, double *acorr)
{
   register int i, j, k;
   double *ptr1, *ptr2, sum, invlen;
   invlen = 1.0 / length;
   ptr1 = data;

   for(j=lolag, k=0, ptr2=ptr1+lolag; j<=hilag; ++j, ++k, ++ptr2)
   {
      for(i=0, sum=0; i<(length-j); ++i)
      {
         sum += ptr1[i] * ptr2[i];
      }
      acorr[k] = sum * invlen;
   }
   return;
}

/**
 * @param data1     pointer to data array that will be "fixed"
 *                     must have length(data1) >= length + hilag
 * @param data2     pointer to data array that will "slide" -
 *                     must have length(data2) >= length
 * @param length    length which crosscorr will be calculated over
 * @param lolag     lowest lag value of cross-correlation 
 * @param hilag     highest lag value of cross-correlation
 * @param normalize zero for scaling by 1/length
 *                  nonzero if scaling by
 *                     1/sqrt[(xcorr(0) starting at data1[0], data2[0]) * 
 *                     (xcorr(0) starting at data[lag], data2[lag])]
 *                     is desired
 *                  if normalize is nonzero, must have
 *                     length(datai) >= length + hilag for i=1,2
 * @param xcorr     array for cross-correlation values
 */
void crosscorrN(double *data1, double *data2, int length,
                int lolag, int hilag, int normalize, double *xcorr)
{
   register int i, j, k;
   double *ptr1, *ptr2, sum;

   if(normalize)
   {
      double *ptr3, *ptr4; // for calculating normalization parameter
      double nlag0, nlagL; // for storing normalization components
      ptr1 = data1 + lolag; ptr2 = data2;

      //calculate normalization parameter at lag 0
      ptr3 = data1; ptr4 = data2;
      for(i=0, nlag0=0; i<length; ++i)
      {
         nlag0 += ptr3[i] * ptr4[i];
      }

      for(j=lolag, k=0; j<=hilag; ++j, ++k, ++ptr1)
      {
         //calculate normalization parameter at lag 0
         ptr3 = data1 + j; ptr4 = data2 + j;
         for(i=0, nlagL=0; i<length; ++i)
         {
            nlagL += ptr3[i] * ptr4[i];
         }

         for(i=0, sum=0; i<length; ++i)
         {
            sum += ptr1[i] * ptr2[i];
         }

         if ( (nlag0==0) || (nlagL==0) )
         {
            xcorr[k] = 0;
         }
         else
         {
            xcorr[k] = sum / sqrt( nlag0 * nlagL );
         }
      }
   }
   else
   {
      double invlen = 1.0 / length;
      ptr1 = data1 + lolag; ptr2 = data2;

      for(j=lolag, k=0; j<=hilag; ++j, ++k, ++ptr1)
      {
         for(i=0, sum=0; i<length; ++i)
         {
            sum += ptr1[i] * ptr2[i];
         }
         xcorr[k] = sum * invlen;
      }
   }
   return;
}

/**
 * @param data    pointer to data array used to calculate autocorrelation
 * @param length  length of data array
 * @param lag     lag value of autocorrelation
 * @return        autocorrelation value
 */
double autocorr(double *data, int length, int lag)
{
   register int i;
   double *ptr1, *ptr2, sum, invlen;
   ptr1 = data;
   ptr2 = data + lag;
   sum = 0;
   for(i=0; i<(length-lag); ++i)
   {
      sum += ptr1[i] * ptr2[i];
   }
   return(sum / (double)length);
}

/**
 * @param data    pointer to data array used to
 *                calculate warped autocorrelation
 * @param length  length of data array
 * @param alpha   warp parameter
 * @param hilag   highest lag value of warped autocorrelation
 * @param temp    scratch array, must be at least as long as data
 * @param acorr   array for warped autocorrelation values
 */
void warped_autocorr(double *data, int length, double alpha, int hilag,
                     double *temp, double *acorr)
{
   register int i, j, k, jm1;
   double sum, t, tm1, invlen;
   invlen = 1.0 / length;
   for(j=0, sum=0; j<length; ++j) { sum += data[j] * data[j]; }
   acorr[0] = sum * invlen;

   temp[0] = -alpha * data[0];
   for(j=1, jm1=0, sum=temp[0]*data[0]; j<length; ++j, ++jm1)
   {
      temp[j]  = data[jm1] + alpha * ( temp[jm1] - data[j] );
      sum     += temp[j] * data[j];
   }
   acorr[1] = sum * invlen;

   for(i=2; i<(hilag+1); ++i)
   {
      tm1     = temp[0];
      temp[0] = -alpha * temp[0];
      for(j=1, jm1=0, sum=temp[0]*data[0]; j<length; ++j, ++jm1)
      {
        t       = temp[j];
        temp[j] = tm1 + alpha * ( temp[jm1] - t );
        tm1     = t;
        sum    += temp[j] * data[j];
      }
      acorr[i] = sum * invlen;
   }
}

/**
 * @param samplerate sample rate
 * @return           warp parameter
 */
double warp_parameter(double samplerate)
{
   return( 1.0674 * sqrt( 2.0/M_PI * atan(0.00006583*samplerate) ) - 0.1916 );
}
