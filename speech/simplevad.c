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
 * @file   simplevad.c
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Tue Dec 31 2002
 *
 * @brief  Simple voice activity detector structure and functions.
 *         Designed for 10 msec frames, 3 frames used for each energy value.
 *         See Matt Kleffner's MS thesis for a discussion of the algorithm.
 *
 * $Log: simplevad.c,v $
 * Revision 1.1  2005/05/09 20:58:08  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.2  2005/04/19 19:47:16  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1.2.1  2005/04/01 19:00:20  mrmcclai
 * KEVIN_DEVEL branch files added
 *
 * Revision 1.2.6.2  2005/02/17 06:42:53  k-squire
 * Added calc_energy(): calculate the energy in a frame of audio
 *
 * Revision 1.2.6.1  2004/08/08 23:18:08  k-squire
 * Added create_simplevad2, reset_simplevad2, and apply_simplevad2, which
 *    mimic the behavior of create_simplevad, reset_simplevad, and
 *    apply_simplevad, respectively, but are meant for use with IServer.
 *
 * Revision 1.2  2003/02/08 23:17:04  kleffner
 * Added missing Log: from initial checkin
 *
 *
 */

#include "simplevad.h"
#include <math.h>
#include <stdlib.h>

#define ABS(X) ((X)<0?(-(X)):(X))

double simplevad_filtercoeffs[]= {0.11764705882353, 0.23529411764706,
                                  0.29411764705882, 0.23529411764706,
                                  0.11764705882353};
int simplevad_filterorder = 4;

double calc_energy(double *audio, int len, filter *en_filter)
{
   int i;
   double energy = 0.0;
   double t_energy = 0.0;
   
   for (i = 0; i < len; ++i)
      t_energy += audio[i]*audio[i];

   if (en_filter)
      filter_data(en_filter, &t_energy, 1, &energy);
   else
      energy = t_energy;

   energy = sqrt(energy / len);
   
   return energy;
}

/**
 * @param  ssvad       pointer to ssvad structure
 * @param  leftenergy  pointer to variable to be filled with left chan. energy
 * @param  rightenergy pointer to variable to be filled with right chan. energy
 *
 * @return status of VAD
 */
int apply_simplevad(simplevad *ssvad, double *leftenergy, double *rightenergy)
{
   register int i;
   int indexdiff;
   int flen        = ssvad->framelength;
   int totframes   = ssvad->totalframes;
   double *lptr    = ssvad->leftchannel;
   double *rptr    = ssvad->rightchannel;
   double lothresh = ssvad->lothresh;
   double hithresh = ssvad->hithresh;
   double prevtestenergy = ssvad->prevtestenergy;
   double lenergy, renergy, totalenergy, testenergy;

   if(ssvad->phraseready)
   {
      return( -( (ssvad->phraseready << SIMPLEVAD_PhraseFramesShift) |
                  ssvad->soundcount ) );
   }

   lptr = ssvad->leftchannel  + ( ssvad->frameindex * flen );
   rptr = ssvad->rightchannel + ( ssvad->frameindex * flen );

   for(i=0, lenergy=0, renergy=0; i<flen; ++i)
   {
      lenergy += lptr[i]*lptr[i];
      renergy += rptr[i]*rptr[i];
   }

   filter_data(ssvad->sleftfilter,  &lenergy, 1, leftenergy);
   filter_data(ssvad->srightfilter, &renergy, 1, rightenergy);
   *leftenergy  = sqrt(*leftenergy  / flen);
   *rightenergy = sqrt(*rightenergy / flen);
   
   ssvad->leftfenergy[ssvad->energyframeindex]  = *leftenergy;
   ssvad->rightfenergy[ssvad->energyframeindex] = *rightenergy;

   testenergy = ssvad->leftfenergy[ssvad->energyframeindex];

   if(ssvad->predelaycount == 0)
   {
      if(ssvad->phraseaddpostpad)
      {
         ssvad->postdelaycount--;
         if(ssvad->postdelaycount == 0)
         {
            ssvad->phraseend   = ssvad->frameindex;
            ssvad->phraseready = ( (ssvad->phraseend - ssvad->phrasestart +
                                    totframes) % totframes ) + 1;
         }
      }
      else
      {
         // check if first sound (start of phrase)
         if( (testenergy>=lothresh) && (prevtestenergy<lothresh) &&
             (!ssvad->incomingphrase) )
         {
            ssvad->incomingphrase = -1;
            ssvad->lastsndend = ssvad->frameindex;
         }
         // lower threshold indicates end of sound
         else if( (testenergy<=lothresh) && (prevtestenergy>lothresh) &&
                  (ssvad->incomingsnd) )
         {
            ssvad->incomingsnd = 0;
            ssvad->soundcount++;
            ssvad->lastsndend = ssvad->frameindex;
            if(ssvad->soundcount == ssvad->maxsndcnt)
            {
               ssvad->phraseaddpostpad = -1;
            }
         }
         // higher threshold indicates beginning of sound
         if( (testenergy>=hithresh) && (prevtestenergy<hithresh) &&
                  (!ssvad->incomingsnd) && ssvad->incomingphrase )
         {
            ssvad->incomingsnd = -1;
         }
         ssvad->prevtestenergy = testenergy;
      } // else phraseaddpostpad


      if(ssvad->incomingphrase)
      {
         if( (ssvad->usedframes == ssvad->maxframes) ||
             ((ssvad->frameindex - ssvad->lastsndend + totframes)%totframes + 1
                > ssvad->maxframesdist) )
         {
            // first threshold crossed, no sounds detected so reset and cont.
            if(!ssvad->soundcount)
            {
               reset_simplevad(ssvad);
               return(0);
            }
            // nonzero number of sounds, but maxsndcnt not reached
            else if( (ssvad->soundcount < ssvad->maxsndcnt) &&
                     (!ssvad->phraseaddpostpad) )
            {
               indexdiff = ssvad->frameindex - ssvad->lastsndend;
               ssvad->phraseaddpostpad = -1;
               // need to post-pad frames
               if( ABS(indexdiff) < ssvad->postpadsize )
               {
                  ssvad->postdelaycount = ssvad->postpadsize - abs(indexdiff);
               }
               // already have enough frames for complete "sub-phrase"
               else
               {
                  ssvad->postdelaycount = 0;
                  ssvad->phraseend = (ssvad->lastsndend + ssvad->postpadsize) %
                                      ssvad->totalframes;
                  ssvad->phraseready = ( (ssvad->phraseend -
                                          ssvad->phrasestart +
                                          ssvad->totalframes) %
                                          ssvad->totalframes ) + 1;
               }
            } // else if soundcount
         } // if usedframes
         ssvad->usedframes++;
      } // if incomingphrase
      else if(!ssvad->incomingphrase)
      {
         ssvad->phrasestart = (ssvad->phrasestart + 1) % totframes;
      } // else incomingsnd
   } // if predelaycount
   else
   {
      ssvad->predelaycount--;
   } // else predelaycount

   if(ssvad->phraseready)
   {
      ssvad->frameindex = ssvad->phrasestart;
   }
   else
   {
      ssvad->frameindex       = (ssvad->frameindex + 1) % totframes;
      ssvad->energyframeindex = (ssvad->energyframeindex + 1) % totframes;
   }

   return( (ssvad->phraseready << SIMPLEVAD_PhraseFramesShift) |
            ssvad->soundcount );
}

/**
 * @param  ssvad        pointer to ssvad structure
 * @param  leftchannel  pointer to left chan. input frame
 * @param  rightchannel pointer to right chan. input frame
 * @param  frameindex   frame index, used to calc. start and end index
 *                      of phrases
 *
 * @return status of VAD
 */
int apply_simplevad2(simplevad *ssvad, double *leftchannel,
                     double *rightchannel, double *leftenergy,
                     double *rightenergy, int frameindex)
{
   register int i;
   int indexdiff;
   double *lptr;
   double *rptr;
   int flen        = ssvad->framelength;
   double lothresh = ssvad->lothresh;
   double hithresh = ssvad->hithresh;
   double prevtestenergy = ssvad->prevtestenergy;
   double lenergy, renergy, lenergyf, renergyf, totalenergy, testenergy;

   if(ssvad->phraseready)
      reset_simplevad2(ssvad, frameindex);

   lptr = leftchannel;
   rptr = rightchannel;

   for(i=0, lenergy=0; i<flen; ++i)
      lenergy += lptr[i]*lptr[i];

   filter_data(ssvad->sleftfilter,  &lenergy, 1, leftenergy);
   *leftenergy  = sqrt(*leftenergy  / flen);

   if (rightchannel)
   {
      for(i=0, renergy=0; i<flen; ++i)
         renergy += rptr[i]*rptr[i];
      filter_data(ssvad->srightfilter, &renergy, 1, rightenergy);
      *rightenergy = sqrt(*rightenergy / flen);
   }
   
   testenergy = *leftenergy;

   if(ssvad->predelaycount == 0)
   {
      if(ssvad->phraseaddpostpad)
      {
         ssvad->postdelaycount--;
         if(ssvad->postdelaycount == 0)
         {
            ssvad->phraseend   = frameindex;
            ssvad->phraseready = ssvad->phraseend - ssvad->phrasestart + 1;
         }
      }
      else
      {
         // check if first sound (start of phrase)
         if( (testenergy>=lothresh) && (prevtestenergy<lothresh) &&
             (!ssvad->incomingphrase) )
         {
            ssvad->incomingphrase = -1;
            ssvad->lastsndend = frameindex;
         }
         // lower threshold indicates end of sound
         else if( (testenergy<=lothresh) && (prevtestenergy>lothresh) &&
                  (ssvad->incomingsnd) )
         {
            ssvad->incomingsnd = 0;
            ssvad->soundcount++;
            ssvad->lastsndend = frameindex;
            if(ssvad->soundcount == ssvad->maxsndcnt)
            {
               ssvad->phraseaddpostpad = -1;
            }
         }
         // higher threshold indicates beginning of sound
         if( (testenergy>=hithresh) && (prevtestenergy<hithresh) &&
                  (!ssvad->incomingsnd) && ssvad->incomingphrase )
         {
            ssvad->incomingsnd = -1;
         }
         ssvad->prevtestenergy = testenergy;
      } // else phraseaddpostpad


      if(ssvad->incomingphrase)
      {
         if( (frameindex - ssvad->lastsndend > ssvad->maxframesdist) )
         {
            // first threshold crossed, no sounds detected so reset and cont.
            if(!ssvad->soundcount)
            {
               reset_simplevad2(ssvad, frameindex);
//               return(-1);
               return(0);
            }
            // nonzero number of sounds, but maxsndcnt not reached
            else if( (ssvad->soundcount < ssvad->maxsndcnt) &&
                     (!ssvad->phraseaddpostpad) )
            {
               indexdiff = frameindex - ssvad->lastsndend;
               ssvad->phraseaddpostpad = -1;
               // need to post-pad frames
               if( ABS(indexdiff) < ssvad->postpadsize )
               {
                  ssvad->postdelaycount = ssvad->postpadsize - abs(indexdiff);
               }
               // already have enough frames for complete "sub-phrase"
               else
               {
                  ssvad->postdelaycount = 0;
                  ssvad->phraseend = ssvad->lastsndend + ssvad->postpadsize;
                  ssvad->phraseready = ssvad->phraseend - ssvad->phrasestart
                                       + 1;
               }
            } // else if soundcount
         } // if usedframes
      } // if incomingphrase
      else if(!ssvad->incomingphrase)
      {
         ssvad->phrasestart = ssvad->phrasestart + 1;
      } // else incomingsnd
   } // if predelaycount
   else
   {
      ssvad->predelaycount--;
   } // else predelaycount

   return( (ssvad->phraseready << SIMPLEVAD_PhraseFramesShift) |
            ssvad->soundcount );
}

/**
 * @param  ssvad        pointer to simplevad structure
 * @param  leftchannel  double pointer to returned left channel frame
 * @param  rightchannel double pointer to returned right channel frame
 * @param  leftenergy   pointer to variable to be filled with left chan. energy
 * @param  rightenergy  pointer to variable to be filled with right chan. energy
 *
 * @return 0 on success, -1 on error or if detection not complete,
 *         1 if all frames pointers have been successfully transferred
 */
int simplevad_getphraseframes(simplevad *ssvad,
                              double **leftchannel, double **rightchannel,
                              double  *leftenergy,  double  *rightenergy)
{
   int frameindex   = ssvad->frameindex;

   if(!ssvad->phraseready)
   {
      return(-1);
   }

   if(leftchannel != NULL)
   {
      *leftchannel  = ssvad->leftchannel  + ( frameindex * ssvad->framelength );
      *leftenergy   = ssvad->leftfenergy[frameindex];
   }
   else if(rightchannel == NULL) return(-1);

   if(rightchannel != NULL)
   {
      *rightchannel = ssvad->rightchannel + ( frameindex * ssvad->framelength );
      *rightenergy  = ssvad->rightfenergy[frameindex];
   }

   ssvad->frameindex = (ssvad->frameindex + 1) % ssvad->totalframes;

   if(frameindex == ssvad->phraseend)
   {
      reset_simplevad(ssvad);
      return(1);
   }
   else return(0);
}

/**
 * @param  ssvad        pointer to simplevad structure
 * @param  leftchannel  double pointer to returned left channel frame
 * @param  rightchannel double pointer to returned right channel frame
 *
 * @return 0 on success, -1 if phrase has been detected
 */
int simplevad_getnextframes(simplevad *ssvad,
                            double **leftchannel, double **rightchannel)
{
   if(ssvad->phraseready) return(-1);

   else
   {
      if(leftchannel != NULL)
      {
         *leftchannel  = ssvad->leftchannel  +
                         ( ssvad->frameindex * ssvad->framelength );
      }
      else if(rightchannel == NULL) return(-1);

      if(rightchannel != NULL)
      {
         *rightchannel = ssvad->rightchannel +
                         ( ssvad->frameindex * ssvad->framelength );
      }
      return(0);
   }
}

/**
 * @param  ssvad pointer to simplevad structure
 */
void reset_simplevad(simplevad *ssvad)
{
   int i;

   ssvad->soundcount=0;     ssvad->phrasestart=0;    ssvad->phraseend=0;
   ssvad->lastsndend=0;     ssvad->incomingsnd=0;    ssvad->incomingphrase=0;
   ssvad->phraseready=0;    ssvad->usedframes=1;     ssvad->frameindex = 0;
   ssvad->prevtestenergy=0;
   ssvad->phraseaddpostpad=0; 

   ssvad->energyframeindex = ssvad->totalframes - ssvad->totalframedelay;

   ssvad->predelaycount  = ssvad->prepadsize;
   ssvad->postdelaycount = ssvad->postpadsize;

   reset_filter(ssvad->sleftfilter);
   reset_filter(ssvad->srightfilter);
}

/**
 * @param  ssvad      pointer to simplevad structure
 * @param  frameindex frame index for external frame
 */
void reset_simplevad2(simplevad *ssvad, int frameindex)
{
   int i;

   ssvad->soundcount=0;
   
   ssvad->frameindex = frameindex;   ssvad->phrasestart=frameindex;
   ssvad->phraseend=frameindex;      ssvad->lastsndend=frameindex;

   ssvad->incomingsnd=0;
   ssvad->incomingphrase=0;
   ssvad->phraseready=0;

   ssvad->usedframes=1;
   ssvad->prevtestenergy=0;
   ssvad->phraseaddpostpad=0; 

   ssvad->energyframeindex = frameindex - ssvad->totalframedelay;

   ssvad->predelaycount  = ssvad->prepadsize;
   ssvad->postdelaycount = ssvad->postpadsize;

   reset_filter(ssvad->sleftfilter);
   reset_filter(ssvad->srightfilter);
}

/**
 * @param  samplingrate  sampling rate of audio
 * @param  framelength   length of audio arrays used in detection
 * @param  maxsounds     max number of sounds to be detected in phrase
 * @param  maxframes     max number of frames allowed in phrase
 * @param  maxframesdist max allowed distance between sounds, in frames
 * @param  padframes     number of frames to add to beginning and end of phrase
 * @param  lothresh      low RMS threshold to use in detecting a sound
 * @param  hithresh      high RMS threshold to use in detecting a sound
 *
 * @return new simplevad structure on success, NULL on error
 */
simplevad* create_simplevad(double samplingrate, int framelength,
                            int maxsounds, int maxframes, int maxframesdist,
                            int padframes, double lothresh, double hithresh)
{
   int i;
   simplevad *ssvad = (simplevad *)malloc(sizeof(simplevad));
   if( ssvad==NULL ) return(NULL);

   ssvad->samplingrate    = samplingrate;
   ssvad->framelength     = framelength;
   ssvad->maxframes       = maxframes;
   ssvad->maxframesdist   = maxframesdist;
   ssvad->padframes       = padframes;
   ssvad->totalframes     = maxframes + 2*padframes;
   ssvad->prepadsize      = padframes + simplevad_filterorder/2;
   ssvad->postpadsize     = padframes - simplevad_filterorder/2 - 1;
   ssvad->totalframedelay = simplevad_filterorder/2;

   if (ssvad->postpadsize < 1)
      ssvad->postpadsize = 1;

   ssvad->lothresh        = lothresh;
   ssvad->hithresh        = hithresh;
   ssvad->maxsndcnt       = maxsounds;

   ssvad->sleftfilter     = create_filter(simplevad_filterorder,0);
   ssvad->srightfilter    = create_filter(simplevad_filterorder,0);

   ssvad->leftchannel  =
             (double *)malloc(ssvad->totalframes*framelength*sizeof(double));
   ssvad->rightchannel =
             (double *)malloc(ssvad->totalframes*framelength*sizeof(double));
   ssvad->leftfenergy  = (double *)malloc(ssvad->totalframes*sizeof(double));
   ssvad->rightfenergy = (double *)malloc(ssvad->totalframes*sizeof(double));

   if( (ssvad->sleftfilter==NULL) || (ssvad->srightfilter==NULL) ||
       (ssvad->leftchannel==NULL) || (ssvad->rightchannel==NULL) ||
       (ssvad->leftfenergy==NULL) || (ssvad->rightfenergy==NULL) )
   {
      destroy_simplevad(ssvad);
      return(NULL);
   }
   else
   {
      for(i=0; i<=simplevad_filterorder; ++i)
      {
         ssvad->sleftfilter->numcoeffs[i]  = simplevad_filtercoeffs[i];
         ssvad->srightfilter->numcoeffs[i] = simplevad_filtercoeffs[i];
      }

      reset_simplevad(ssvad);
      return(ssvad);
   }
}

/**
 * @param  samplingrate  sampling rate of audio
 * @param  framelength   length of audio arrays used in detection
 * @param  maxsounds     max number of sounds to be detected in phrase
 * @param  maxframesdist max allowed distance between sounds, in frames
 * @param  padframes     number of frames to add to beginning and end of phrase
 * @param  lothresh      low RMS threshold to use in detecting a sound
 * @param  hithresh      high RMS threshold to use in detecting a sound
 *
 * @return new simplevad structure on success, NULL on error
 */
simplevad* create_simplevad2(double samplingrate, int framelength,
                            int maxsounds, int maxframesdist,
                            int padframes, double lothresh, double hithresh)
{
   int i;
   simplevad *ssvad = (simplevad *)malloc(sizeof(simplevad));
   if( ssvad==NULL ) return(NULL);

   ssvad->samplingrate    = samplingrate;
   ssvad->framelength     = framelength;
   ssvad->maxframesdist   = maxframesdist;
   ssvad->padframes       = padframes;
   ssvad->prepadsize      = padframes + simplevad_filterorder/2;
   ssvad->postpadsize     = padframes - simplevad_filterorder/2 - 1;
   ssvad->totalframedelay = simplevad_filterorder/2;

   if (ssvad->postpadsize < 1)
      ssvad->postpadsize = 1;

   ssvad->lothresh        = lothresh;
   ssvad->hithresh        = hithresh;
   ssvad->maxsndcnt       = maxsounds;

   ssvad->sleftfilter     = create_filter(simplevad_filterorder,0);
   ssvad->srightfilter    = create_filter(simplevad_filterorder,0);

   ssvad->leftchannel  = NULL;
   ssvad->rightchannel = NULL;
   ssvad->leftfenergy  = NULL;
   ssvad->rightfenergy = NULL;

   if( (ssvad->sleftfilter==NULL) || (ssvad->srightfilter==NULL))
   {
      destroy_simplevad(ssvad);
      return(NULL);
   }
   else
   {
      for(i=0; i<=simplevad_filterorder; ++i)
      {
         ssvad->sleftfilter->numcoeffs[i]  = simplevad_filtercoeffs[i];
         ssvad->srightfilter->numcoeffs[i] = simplevad_filtercoeffs[i];
      }

      reset_simplevad2(ssvad, 0);
      return(ssvad);
   }
}

/**
 * @param  ssvad pointer to simplevad structure
 */
void destroy_simplevad(simplevad *ssvad)
{
   if(ssvad!=NULL)
   {
      free(ssvad->leftchannel);
      free(ssvad->rightchannel);
      free(ssvad->leftfenergy);
      free(ssvad->rightfenergy);
      destroy_filter(ssvad->sleftfilter);
      destroy_filter(ssvad->srightfilter);
      free(ssvad);
   }
}
