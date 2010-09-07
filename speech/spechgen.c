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
 * @file   spechgen.c
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Wed Mar 20 2002
 *
 * @brief  Speech generation structure. See Matt Kleffner's MS thesis
 *         for a discussion of the algorithms
 *
 * $Log: spechgen.c,v $
 * Revision 1.2  2006/02/20 17:19:39  mrmcclai
 * no change
 *
 * Revision 1.1  2005/05/09 20:58:08  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2005/03/18 17:13:20  mrmcclai
 * Imported Sources
 *
 * Revision 1.5  2004/08/03 02:02:45  k-squire
 * Changed synthfeature structure to use all doubles, and rearranged the order
 *    slightly so that the features the HMM uses for recognition are all
 *    together. (gain, pitchper, and timestamp are now at the end of the
 *    structure.)
 *
 * Revision 1.4  2003/02/03 19:48:34  kleffner
 * Added comments, license
 *
 * Revision 1.3  2003/01/29 01:27:07  kleffner
 * Set (swa = NULL) for traditional-lpc generation so that destroy function
 * does not seg fault
 *
 * Revision 1.2  2002/12/18 22:19:01  kleffner
 * Modified to use quantized log area ratios, pitch and gain and voicing
 * confidence
 * Fractional pitch periods are allowed and are no longer truncated
 * Removed pole-zero cancellation from warped filter
 *
 * Revision 1.1  2002/05/14 21:46:12  kleffner
 * Initial checkin
 *
 */

#include "spechgen.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#define NOISEFILE    "noise.dat"
#define NOISEFILELEN 16384

/**
 * @param  sspg  pointer to speech_generator structure
 * @param  ssf   pointer to synthfeature structure
 * @param  audio pointer to audio data of length framelength
 */
void generate_speech(speech_generator *sspg, synthfeature *ssf, double *audio)
{
   register int i;
   register int nseindex = sspg->noiseindex;
   register int flen     = sspg->framelength;
   double       imppos   = sspg->impulseindex;
   double       gaingl;
   double       gainnse;
   double       pper     = ssf->pitchper;
   double       vconf    = ssf->vconf;
   double       *glwv    = sspg->glottalwvfm;
   double       *nse     = sspg->noise;

   lar2refl(sspg->slpc->refl, ssf->lar, sspg->slpc->order);
   refl2lpc(sspg->slpc);

   if( sspg->warpedcoeffs )
   {
      calculate_warpfilter(sspg->swa, sspg->slpc->lpc,
                             (double)(ssf->gain), sspg->sfilter);
   }
   else
   {
      sspg->sfilter->numcoeffs[0] = (double)(ssf->gain);
      memcpy(sspg->sfilter->dencoeffs, sspg->slpc->lpc,
             sspg->sfilter->M*sizeof(double));
   }

   //fill glottal waveform
   gaingl  = sqrt( vconf * pper );
   gainnse = sqrt( 1 - vconf );

   for(i=0; i<flen; ++i)
   {
      glwv[i] = gainnse * nse[nseindex++];
   }
   sspg->noiseindex += flen;
   if( sspg->noiseindex > ( NOISEFILELEN - flen ) )
   {
      sspg->noiseindex = 0;
   }

   while( imppos < flen )
   {
      glwv[(int)(imppos+0.5)] += gaingl;
      imppos                  += pper;
   }

   //set impulse for next pitch impulse after current frame
   sspg->impulseindex = imppos - flen;

   filter_data(sspg->sfilter, glwv, flen, audio);
}

/**
 * @param  sspg  pointer to speech_generator structure
 */
void reset_spgenerator(speech_generator *sspg)
{
   sspg->impulseindex = 0;
   sspg->noiseindex   = 0;
   reset_filter(sspg->sfilter);
}

/**
 * @param  samplingrate  sampling rate in Hz
 * @param  lpcorder      order of LPCs in synthfeature structures
 * @param  framelength   length of audio arrays used in speech generation
 * @param  warpedcoeffs  zero for traditional LP, non-zero for warped LP
 * @param  alpha         warp parameter
 *
 * @return new pitchdetector structure on success, NULL on error
 */
speech_generator* create_spgenerator(double samplingrate,
                                     int lpcorder,
                                     int framelength,
                                     int warpedcoeffs,
                                     double alpha)
{
   FILE *noisefile;
   register int i;
   int returnval;
   double *nse;
   speech_generator *sspg =
      (speech_generator *)malloc(sizeof(speech_generator));
   if( sspg==NULL ) return(NULL);

   noisefile = fopen( NOISEFILE, "r" );
   if( noisefile == NULL )
   {
      free(sspg);
      return(NULL);
   }

   sspg->noise = (double *)malloc(NOISEFILELEN*sizeof(double));
   if(sspg->noise==NULL)
   {
      destroy_spgenerator(sspg);
   }

   nse = sspg->noise;
   for(i=0; i<NOISEFILELEN; ++i)
   {
      returnval = fscanf(noisefile, "%30le\n", nse + i);
      if( returnval != 1 )
      {
         destroy_spgenerator(sspg);
         return(NULL);
      }
   }

   sspg->framelength  = framelength;
   sspg->warpedcoeffs = warpedcoeffs;
   sspg->samplingrate = samplingrate;

   sspg->glottalwvfm  = (double *)malloc(framelength*sizeof(double));

   if( warpedcoeffs )
   {
      sspg->sfilter = create_filter(lpcorder-1, lpcorder);
      sspg->swa = create_warparrays(lpcorder, alpha);

      if( (sspg->sfilter==NULL) || (sspg->glottalwvfm==NULL) ||
          (sspg->swa==NULL) )
      {
         destroy_spgenerator(sspg);
         return(NULL);
      }
   }
   else
   {
      sspg->sfilter = create_filter(0, lpcorder);
      sspg->swa = NULL;

      if( (sspg->sfilter==NULL) || (sspg->glottalwvfm==NULL) )
      {
         destroy_spgenerator(sspg);
         return(NULL);
      }
   }

   sspg->slpc = create_lpc(lpcorder);
   if( sspg->slpc==NULL )
   {
      destroy_spgenerator(sspg);
      return(NULL);
   }

   reset_spgenerator(sspg);
   return(sspg);
}

/**
 * @param  sspg  pointer to speech_generator structure
 */
void destroy_spgenerator(speech_generator *sspg)
{
   if(sspg!=NULL)
   {
      free(sspg->glottalwvfm);
      free(sspg->noise);
      destroy_filter(sspg->sfilter);
      destroy_warparrays(sspg->swa);
      destroy_lpc(sspg->slpc);
      free(sspg);
   }
}
