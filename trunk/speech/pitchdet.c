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
 * @file   pitchdet.c
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Tue Feb 05 2002
 *
 * @brief  Pitch detector and voicing detector. See Matt Kleffner's MS thesis
 *         for a discussion of the algorithms
 *
 * $Log: pitchdet.c,v $
 * Revision 1.1  2005/05/09 20:58:08  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2005/03/18 17:13:20  mrmcclai
 * Imported Sources
 *
 * Revision 1.3  2003/02/03 19:46:51  kleffner
 * Added comments, license
 *
 * Revision 1.2  2002/12/09 21:19:23  kleffner
 * Increased search range so that pitch periods of hilag and lolag are possible,
 * changed pitch spectrum parameters from samples to normalized frequency
 * (1 is the sampling rate)
 *
 * Revision 1.1  2002/05/14 21:46:10  kleffner
 * Initial checkin
 *
 */

#include "pitchdet.h"
#include "autocorr.h"
#include <math.h>
#include <string.h>

#define MAX(X,Y) ((X)<(Y)?(Y):(X))

/**
 * @param  spitchdetector pointer to pitchdetector structure
 * @param  audio          pointer to audio data of length L
 */
void estimate_pitch(pitchdetector *spitchdetector, double *audio)
{
   register int i, j, k;
   int ilargest;
   double sum, largest, max;
   int Lp             = spitchdetector->Lp;
   int Olpc           = spitchdetector->Olpc;
   int S              = spitchdetector->S;
   int Sc             = spitchdetector->Sc;
   int lolag          = spitchdetector->lolag;
   int hilag          = spitchdetector->hilag;
   double *confarr    = spitchdetector->confarr;
   double *s2a        = spitchdetector->spec2acorr;
   double *a2s        = spitchdetector->acorr2spec;
   double *spec       = spitchdetector->spectrum;
   double *subsp      = spitchdetector->subspec;
   double *acorr      = spitchdetector->slpc->acorr;
   double *lpcacorr   = spitchdetector->lpcacorr;
   double *pitchacorr = spitchdetector->pitchacorr;

   memcpy(spitchdetector->zeropadin, audio, spitchdetector->L*sizeof(double) );
   rfftw_one(spitchdetector->fftpl, spitchdetector->zeropadin, spec);

   subsp[0] = spec[0] * spec[0];
   for(i=1, j=(S-1); i<=Sc; ++i, --j)
   {
      subsp[i] = spec[i] * spec[i] + spec[j] * spec[j];
   }

   for(i=0, k=0; i<=Olpc; ++i)
   {
      for(j=0, sum=0; j<=Sc; ++j)
      {
         sum += s2a[k] * subsp[j];
         k++;
      }
      acorr[i] = sum;
   }

   calculate_lpc(spitchdetector->slpc);
   autocorr_lpc(spitchdetector->slpc, lpcacorr);

   for(i=0, k=0; i<=Sc; ++i)
   {
      for(j=0, sum=0; j<=Olpc; ++j)
      {
         sum += a2s[k] * lpcacorr[j];
         k++;
      }
      spec[i] = subsp[i] * sum;
   }

   memset( spec + (Sc+1), 0, (S-Sc-1)*sizeof(double) );
   rfftw_one(spitchdetector->ifftpl, spec, pitchacorr);

   //find peak
   for(i=lolag, largest = 0, ilargest = lolag; i<=hilag; ++i)
   {
      if ( (pitchacorr[i-1] < pitchacorr[i]) &&
           (pitchacorr[i] > pitchacorr[i+1]) )
      {
         if (pitchacorr[i] >= largest)
         {
            largest  = pitchacorr[i];
            ilargest = i;
         }
      }
   }
   spitchdetector->pitchper = ilargest;

   //find voicing confidence from first half of array
   crosscorrN(audio, audio, Lp, ilargest-CONF_OFFSET,
              ilargest+CONF_OFFSET, -1, confarr);

   //find max confidence
   for(i=0, max=0; i<CONF_SEARCH; ++i)
   {
      if(confarr[i]>max) max = confarr[i];
   }
   spitchdetector->vconf = max;

   //find voicing confidence from last half of array
   crosscorrN(audio+Lp, audio+Lp, Lp, -ilargest-CONF_OFFSET,
              -ilargest+CONF_OFFSET, 1, confarr);   

   //find max confidence
   for(i=0, max=0; i<CONF_SEARCH; ++i)
   {
      if(confarr[i]>max) max = confarr[i];
   }
   spitchdetector->vconf = MAX(max, spitchdetector->vconf);

}

/**
 * @param  audiolength     Length of data which pitch is extracted from
 * @param  lolag           Lowest lag value to search for pitch peak
 * @param  hilag           Highest lag value to search for pitch peak
 * @param  lpcorder        Order of linear predictor used in finding pitch
 * @param  pitchspeccutoff Normalized low-pass whitening filter cutoff,
 *                         where 0.5 corresponds to half the sampling rate
 *
 * @return new pitchdetector structure on success, NULL on error
 */
pitchdetector* create_pitchdetector(int audiolength, int lolag, int hilag,
                                    int lpcorder, double pitchspeccutoff)
{
   int spectrumlength = 1 << (int)(log(hilag+2*audiolength)/log(2) + 1);
                        //2^ceil(log2(hilag+2*audiolength))
   int spectrumcutoff = (int)(spectrumlength*pitchspeccutoff + 0.5);
   pitchdetector *spd = (pitchdetector *)malloc(sizeof(pitchdetector));
   if ( spd==NULL ) return(NULL);

   spd->L        = audiolength;
   spd->Lp       = audiolength / 2;
   spd->lolag    = lolag;
   spd->hilag    = hilag;
   spd->Olpc     = lpcorder;
   spd->S        = spectrumlength;
   spd->Sc       = spectrumcutoff;

   spd->fftpl      = rfftw_create_plan(spd->S,  FFTW_FORWARD, FFTW_ESTIMATE);
                                                            //FFTW_MEASURE
   spd->ifftpl     = rfftw_create_plan(spd->S, FFTW_BACKWARD, FFTW_ESTIMATE);
   spd->slpc       = create_lpc(lpcorder);
   spd->zeropadin  = (double *)malloc((spd->S)*sizeof(double));
   spd->spec2acorr = (double *)malloc((lpcorder+1)*(spd->Sc+1)*sizeof(double));
   spd->acorr2spec = (double *)malloc((spd->Sc+1)*(lpcorder+1)*sizeof(double));
   spd->spectrum   = (double *)malloc((spd->S)*sizeof(double));
   spd->subspec    = (double *)malloc((spd->Sc+1)*sizeof(double));
   spd->lpcacorr   = (double *)malloc((lpcorder+1)*sizeof(double));
   spd->pitchacorr = (double *)malloc((spd->S)*sizeof(double));

   if( (spd->fftpl==NULL)      || (spd->ifftpl==NULL)     ||
       (spd->slpc==NULL)       || (spd->zeropadin==NULL)  ||
       (spd->spec2acorr==NULL) || (spd->acorr2spec==NULL) ||
       (spd->spectrum==NULL)   || (spd->subspec==NULL)    ||
       (spd->lpcacorr==NULL)   || (spd->pitchacorr==NULL) )
   {
      destroy_pitchdetector(spd);
      return(NULL);
   }
   else
   {
      int i, j, k;
      double pi_Sc = M_PI / (double)spectrumcutoff;
      double wi, *s2a = spd->spec2acorr, *a2s = spd->acorr2spec;

      memset( spd->zeropadin, 0, spd->S*sizeof(double) );

      //calculate matrices
      for(i=0, k=0; i<=lpcorder; ++i)
      {
         for(j=0; j<=spectrumcutoff; ++j)
         {
            wi = j * pi_Sc;
            s2a[k] = cos(i * wi) * ( 2 - ( (j==0)||(j==spectrumcutoff) ) );
                                   //factor of 2 needed for most entries
            k++;
         }
      }

      for(i=0, k=0; i<=spectrumcutoff; ++i)
      {
         wi = i * pi_Sc;
         for(j=0; j<=lpcorder; ++j)
         {
            a2s[k] = cos(j * wi) * ( 2 - (j==0) );
                                   //factor of 2 needed for most entries
            k++;
         }
      }

      return(spd);
   }
}

/**
 * @param  spitchdetector pointer to pitchdetector structure
 */
void destroy_pitchdetector(pitchdetector *spitchdetector)
{
   rfftw_destroy_plan(spitchdetector->fftpl);
   rfftw_destroy_plan(spitchdetector->ifftpl);
   free(spitchdetector->slpc);
   free(spitchdetector->zeropadin);
   free(spitchdetector->spec2acorr);
   free(spitchdetector->acorr2spec);
   free(spitchdetector->spectrum);
   free(spitchdetector->subspec);
   free(spitchdetector->lpcacorr);
   free(spitchdetector->pitchacorr);
   free(spitchdetector);
}
