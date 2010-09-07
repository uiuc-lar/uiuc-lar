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
 * @file   synthftr.c
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Wed Feb 20 2002
 *
 * @brief  Structures and functions for generating speech features suitable
 *         for synthesis. See Matt Kleffner's MS thesis for a discussion
 *         of the algorithms
 *
 * $Log: synthftr.c,v $
 * Revision 1.1  2005/05/09 20:58:09  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.2  2005/04/19 19:47:16  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1.2.1  2005/04/01 19:00:20  mrmcclai
 * KEVIN_DEVEL branch files added
 *
 * Revision 1.8.2.2  2005/02/17 06:43:54  k-squire
 * Rearranged synthfeature struct again:
 *   moved voice_active after gain
 *   added energy after voice_active
 *
 * Revision 1.8.2.1  2004/08/08 23:16:37  k-squire
 * Removed timestamp parameter from extract_synthfeature, as it wasn't being
 *    used.
 * Removed tindex, timestamp from synthfeature generator for same reason.
 * Added a voice_active field to synthfeature, and rearranged yet again so
 *    that the features most likely used by the HMM are first.
 * Added copy_synthfeature() function.
 *
 * Revision 1.8  2004/08/03 02:02:45  k-squire
 * Changed synthfeature structure to use all doubles, and rearranged the order
 *    slightly so that the features the HMM uses for recognition are all
 *    together. (gain, pitchper, and timestamp are now at the end of the
 *    structure.)
 *
 * Revision 1.7  2004/07/31 03:55:42  k-squire
 * Reverted change: free(NULL) is actually okay.
 *
 * Revision 1.6  2004/07/30 06:25:27  k-squire
 * Dummy me didn't fix bugs before checking the last file in. :-(
 *
 * Revision 1.5  2004/07/30 05:46:09  k-squire
 * Fixed free of NULL variable in destroy_synthfeature.
 * Added create_synthfeature_list, destroy_synthfeature_list,
 *    save_synthfeature.
 *
 * Revision 1.4  2003/02/03 19:54:39  kleffner
 * Added comments, license
 * Added call to reset_frameholder in reset_sfgenerator
 * Check for (wacorrtemp == NULL) (traditional LPC) in destroy_sfgenerator
 *
 * Revision 1.3  2002/12/18 22:29:58  kleffner
 * Modified to generate and store quantized log area ratios, pitch, gain and
 * voicing confidence
 * Added functions to quantize and "unquantize" features
 *
 * Revision 1.2  2002/12/09 21:53:16  kleffner
 * Implemented clipping/scaling of final voicing confidence score,
 * changed spectral parameters that are input to pitch detector,
 * compute gain for "junk features",
 * ppX variables in extract_synthfeature changed to type double
 *
 * Revision 1.1  2002/05/14 21:46:14  kleffner
 * Initial checkin
 *
 */

#include "synthftr.h"
#include "autocorr.h"
#include <math.h>
#include <string.h>

//use logic for 3-point median instead of sorting algorithm
#define  MEDIAN(X,Y,Z) ((X)<(Y) ? ( (X)<(Z) ? ((Y)<(Z)?(Y):(Z)) : (X) ) : \
                                  ( (Y)<(Z) ? ((X)<(Z)?(X):(Z)) : (Y) ))
#define  CLIP0(X)      ((X)<0  ?  0 : (X))
#define  CLIPmax       0.999
#define  CLIPmag(X)    ((X)<-CLIPmax ? -CLIPmax : ((X)>CLIPmax ? CLIPmax : (X)))

#define  MAX_UINT16_T  65536
#define  PITCH_SCALE   64

/**
 * @param  ssfg      pointer to synthfeature_generator structure
 * @param  ssf       pointer to synthfeature structure
 * @param  audio     pointer to audio data of length framelength
 * @param  timestamp timestamp to be applied to feature - note that
 *                   numberofframes / 2 + 1 dummy features, with dummy stamps,
 *                   are assigned after reset
 */
void extract_synthfeature(synthfeature_generator *ssfg, synthfeature *ssf,
                          double *audio)
{
   register int i;
   double pp0, pp1, pp2;
   double vc0, vc1, vc2;
   int totallen = ssfg->framelength * ssfg->numberofframes;
   int order    = ssfg->slpc->order;
   double *frameptr    = ssfg->sframeholder->framedata;
   double *wwinptr     = ssfg->wwin;
   double *wwintempptr = ssfg->wwintemp;

   add_frame(ssfg->sframeholder, audio);
   estimate_pitch(ssfg->spd, frameptr);
   ssfg->pitchper[ssfg->index] = ssfg->spd->pitchper;
   ssfg->vconf[ssfg->index]    = ssfg->spd->vconf;
   pp0 = ssfg->pitchper[0]; pp1 = ssfg->pitchper[1]; pp2 = ssfg->pitchper[2];
   vc0 = ssfg->vconf[0];    vc1 = ssfg->vconf[1];    vc2 = ssfg->vconf[2];

   for(i=0; i<totallen; ++i)
   {
      wwintempptr[i] = wwinptr[i] * frameptr[i];
   }

   if( ssfg->warpedcoeffs )
   {
      warped_autocorr(wwintempptr, totallen, ssfg->alpha, order,
                      ssfg->wacorrtemp, ssfg->slpc->acorr);
   }
   else
   {
      autocorrN(wwintempptr, totallen, 0, order, ssfg->slpc->acorr);
   }

   calculate_lpc(ssfg->slpc);

   if(ssfg->resetctr==0)
   {
      ssf->pitchper  = MEDIAN(pp0,pp1,pp2);
      ssf->vconf     = ssfg->vconfscale *
                          CLIP0(MEDIAN(vc0,vc1,vc2)-ssfg->vconfmin);
      refl2lar(ssf->lar,  ssfg->refltemp, order);
      ssf->gain = ssfg->gaintemp;
      memcpy(ssfg->refltemp, ssfg->slpc->refl,
                             ssfg->slpc->order*sizeof(double));
      ssfg->gaintemp = ssfg->slpc->gain;
   }
   else
   {
      // number of "junk features" = delay are put out after reset
      ssf->pitchper  = ssfg->pitchper[ssfg->index];
      ssf->vconf     = ssfg->vconfscale *
                          CLIP0(ssfg->vconf[ssfg->index]-ssfg->vconfmin);
      refl2lar(ssf->lar,  ssfg->slpc->refl, order);
      ssf->gain = ssfg->slpc->gain;
      memcpy(ssfg->refltemp, ssfg->slpc->refl,
                             ssfg->slpc->order*sizeof(double));
      ssfg->gaintemp = ssfg->slpc->gain;
      ssfg->resetctr--;
   }

   ssf->voice_active = 0.0;
   ssfg->index = (ssfg->index + 1) % 3;

}

/**
 * @param  lar       pointer to log area ratios of length order
 * @param  refl      pointer to reflection coefficients of length order
 * @param  order     order of LARs and reflection coefficients
 */
void refl2lar(double *lar, double *refl, int order)
{
   register int i; double fast;
   for(i=0; i<order; ++i)
   {
      fast   = CLIPmag(refl[i]);
      lar[i] = log((1 - fast) / (1 + fast));
   }
}

/**
 * @param  refl      pointer to reflection coefficients of length order
 * @param  lar       pointer to log area ratios of length order
 * @param  order     order of LARs and reflection coefficients
 */
void lar2refl(double *refl, double *lar, int order)
{
   register int i; double dfast; double  ffast;

   for(i=0; i<order; ++i)
   {
      ffast   = exp(lar[i]);
      dfast   = (1 - ffast) / (1 + ffast);
      refl[i] = (double)CLIPmag(dfast);
   }
}

/**
 * @param  pitch pitch period to be quantized
 *
 * @return quantized pitch period
 */
unsigned short Q_pitch(double pitch)
{
   int pitchout = (int)(pitch*PITCH_SCALE + 0.5);
   if(pitchout == MAX_UINT16_T) pitchout = MAX_UINT16_T - 1;
   return((unsigned short)pitchout);
}

/**
 * @param  vconf voicing confidence to be quantized
 *
 * @return quantized voicing confidence
 */
unsigned short Q_vconf(double vconf)
{
   int vconfout = (int)(vconf*MAX_UINT16_T + 0.5);
   if(vconfout == MAX_UINT16_T) vconfout = MAX_UINT16_T - 1;
   return((unsigned short)vconfout);
}

/**
 * @param  pitch pitch period to be un-quantized
 *
 * @return un-quantized pitch period
 */
double UQ_pitch(unsigned short pitch)
{
   return((double)(pitch) / PITCH_SCALE);
}

/**
 * @param  vconf voicing confidence to be un-quantized
 *
 * @return un-quantized voicing confidence
 */
double UQ_vconf(unsigned short vconf)
{
   return((double)(vconf) / MAX_UINT16_T);
}

/**
 * @param  ssfg pointer to synthfeature_generator structure
 */
void reset_sfgenerator(synthfeature_generator *ssfg)
{
   memset(ssfg->pitchper, 0, 3*sizeof(double));
   memset(ssfg->vconf, 0, 3*sizeof(double));
   memset(ssfg->refltemp, 0, ssfg->slpc->order*sizeof(double));

   reset_frameholder(ssfg->sframeholder);

   ssfg->index    = 0;
   ssfg->resetctr = ssfg->delay;
}

/**
 * @param  wwin   pointer to window of length samples
 * @param  length length of weighting window
 */
void wwin_hamming(double* wwin, int length)
{
   register int i;
   double fast = 2 * M_PI / (double)(length - 1);

   for(i=0; i<length; ++i)
   {
      wwin[i] = 0.54 - 0.46 * cos( fast * i );
   }
}

/**
 * @param  lpcorder order of LPCs to be stored in synthfeature
 *
 * @return new synthfeature structure on success, NULL on error
 */
synthfeature* create_synthfeature(int lpcorder)
{
   synthfeature *ssf = (synthfeature *)malloc(sizeof(synthfeature));
   if ( ssf==NULL ) return(NULL);

   ssf->lar  = (double *)malloc(lpcorder*sizeof(double));

   if( ssf->lar == NULL )
   {
      destroy_synthfeature(ssf);
      return(NULL);
   }
   else return(ssf);
}

/**
 * @param  lpcorder order of LPCs to be stored in synthfeature
 * @param  length length of the list
 *
 * @return new synthfeature structure on success, NULL on error
 */
synthfeature* create_synthfeature_list(int lpcorder, int length)
{
   register int i;
   double *lars, *p;
   synthfeature *ssf = (synthfeature *)malloc(sizeof(synthfeature)*length);
   if ( ssf==NULL ) return(NULL);

   lars = (double *)malloc(lpcorder*sizeof(double)*length);
   if (lars == NULL)
   {
      destroy_synthfeature_list(ssf);
      return(NULL);
   }

   p = lars;
   
   for (i = 0; i<length; ++i)
   {
      ssf[i].lar = p;
      p+= lpcorder;
   }

   return(ssf);
}

/**
 * @param ssynthfeature pointer to synthfeature structure
 */
void destroy_synthfeature(synthfeature *ssynthfeature)
{
   free(ssynthfeature->lar);
   free(ssynthfeature);
}

/**
 * @param ssynthfeature pointer to synthfeature list
 */
void destroy_synthfeature_list(synthfeature *ssynthfeature)
{
   free(ssynthfeature->lar);
   free(ssynthfeature);      // yes, this is really the same as
                             // destroy_synthfeature!
}

/**
 * @param dest destination of copy
 * @param src  src of copy
 */

void copy_synthfeature(synthfeature *dest, synthfeature *src, int lpcorder)
{
   dest->pitchper = src->pitchper;
   dest->voice_active = src->voice_active;
   dest->energy = src->energy;
   dest->gain = src->gain;
   dest->vconf = src->vconf;
   memcpy(dest->lar, src->lar, sizeof(double)*lpcorder);
}

/** 
 * @param featurefile file to save synthfeature to
 * @param lpcorder lpc order
 * @param ssynthfeature the synthfeature to save
 * 
 * @return 0 on success, -1 on error
 */

int save_synthfeature(FILE *featurefile,
                      int lpcorder,
                      synthfeature *ssynthfeature)
{
   register int i;

   fprintf(featurefile, " %12.8g", ssynthfeature->pitchper);
   fprintf(featurefile, " %12.8g", ssynthfeature->gain);
   fprintf(featurefile, " %12.8g", ssynthfeature->voice_active);
   fprintf(featurefile, " %12.8g", ssynthfeature->energy);
   fprintf(featurefile, " %12.8g", ssynthfeature->vconf);

   for (i = 0; i<lpcorder; ++i)
      fprintf(featurefile, " %12.8g", ssynthfeature->lar[i]);

   fprintf(featurefile, "\n");
}

/**
 * @param  samplingrate    sampling rate of audio
 * @param  lpcorder        order of LPCs in features
 * @param  warpedcoeffs    zero for traditional LP, non-zero for warped LP
 * @param  framelength     length of audio arrays used in feature generation
 * @param  numberofframes  number of frames used in generating each feature
 * @param  lolag           lowest correlation lag used in pitch detection
 * @param  hilag           highest correlation lag used in pitch detection
 * @param  pitchlpcorder   order of LPCs used in pitch detection
 * @param  pitchspeccutoff normalized low-pass cutoff for pitch detection,
 *                         where 0.5 corresponds to half the sampling rate
 * @param  vconfmin        lower threshold at which voicing confidence is
 *                         clipped to zero
 *
 * @return new synthfeature_generator structure on success, NULL on error
 */
synthfeature_generator* create_sfgenerator(double samplingrate, 
                                           int lpcorder,
                                           int warpedcoeffs,
                                           int framelength,
                                           int numberofframes,
                                           int lolag, int hilag,
                                           int pitchlpcorder,
                                           double pitchspeccutoff,
                                           double vconfmin)
{
   int totallen = framelength * numberofframes;
   synthfeature_generator *ssfg =
      (synthfeature_generator *)malloc(sizeof(synthfeature_generator));
   if( ssfg==NULL ) return(NULL);

   ssfg->warpedcoeffs   = warpedcoeffs;
   ssfg->framelength    = framelength;
   ssfg->numberofframes = numberofframes;
   ssfg->delay          = numberofframes / 2 + 1; //1 accounts for median delay
   ssfg->resetctr       = ssfg->delay;
   ssfg->index          = 0;

   ssfg->samplingrate   = samplingrate;
   ssfg->alpha          = warp_parameter(samplingrate);
   ssfg->vconfmin       = vconfmin;
   ssfg->vconfscale     = 1.0 / (1 - vconfmin);

   ssfg->refltemp       = (double *)malloc(lpcorder*sizeof(double));
   ssfg->wwin           = (double *)malloc(totallen*sizeof(double));
   ssfg->wwintemp       = (double *)malloc(totallen*sizeof(double));

   ssfg->sframeholder   = create_frameholder(framelength, numberofframes);
   ssfg->slpc           = create_lpc(lpcorder);
   ssfg->spd            = create_pitchdetector(totallen, lolag, hilag,
                                               pitchlpcorder, pitchspeccutoff);

   if( warpedcoeffs )
   {
      ssfg->wacorrtemp  = (double *)malloc(totallen*sizeof(double));
      if( ssfg->wacorrtemp==NULL )
      {
         destroy_sfgenerator(ssfg);
         return(NULL);
      }
   }
   else ssfg->wacorrtemp = NULL;

   if( (ssfg->refltemp==NULL)     || (ssfg->wwin==NULL)         ||
       (ssfg->wwintemp==NULL)     ||
       (ssfg->sframeholder==NULL) || (ssfg->slpc==NULL)         ||
       (ssfg->spd==NULL) )
   {
      destroy_sfgenerator(ssfg);
      return(NULL);
   }
   else
   {
      wwin_hamming(ssfg->wwin, totallen);
      reset_sfgenerator(ssfg);
      return(ssfg);
   }
}

/**
 * @param  ssfg pointer to synthfeature_generator structure
 */
void destroy_sfgenerator(synthfeature_generator *ssfg)
{
   if(ssfg!=NULL)
   {
      if(ssfg->wacorrtemp != NULL) free(ssfg->wacorrtemp);
      free(ssfg->refltemp);
      free(ssfg->wwin);
      free(ssfg->wwintemp);
      destroy_frameholder(ssfg->sframeholder);
      destroy_lpc(ssfg->slpc);
      destroy_pitchdetector(ssfg->spd);
      free(ssfg);
   }
}
