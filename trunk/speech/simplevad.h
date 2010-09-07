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
 * @file   simplevad.h
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Wed Dec 18 2002
 *
 * @brief  Simple voice activity detector structure and functions.
 *         Designed for 10 msec frames, 3 frames used for each energy value.
 *         See Matt Kleffner's MS thesis for a discussion of the algorithm.
 *
 * $Log: simplevad.h,v $
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
 * Revision 1.2  2003/02/08 23:16:23  kleffner
 * Added missing Log: from initial checkin
 *
 * 
 */

#ifndef SIMPLEVAD_H
#define SIMPLEVAD_H

#include "filter.h"

#define SIMPLEVAD_SoundCountMask       0x000000FF
#define SIMPLEVAD_PhraseFramesMask     0xFFFFFF00
#define SIMPLEVAD_PhraseFramesShift    0x8

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Structure containing information necessary to detect vocal activity in audio
 */
typedef struct
{
   int framelength;       ///< length of each frame of audio, samples per chan.
   int maxframes;         ///< maximum number of frames in phrase
   int maxframesdist;     ///< maximum distance between sounds, in frames
   int padframes;         ///< number of frames to keep before and after phrase
   int totalframes;       ///< maxframes + 2*padframes
   int prepadsize;        ///< padding before phrase
   int postpadsize;       ///< padding after phrase
   int totalframedelay;   ///< delay of filter plus one for windowing delay
   int maxsndcnt;         ///< maximum number of "syllables" in phrase
   int soundcount;        ///< current number of "syllables" in phrase
   int phrasestart;       ///< beginning of phrase
   int phraseend;         ///< end of phrase
   int lastsndend;        ///< position of last sound/syllable detected
   int incomingsnd;       ///< indicates a sound/syllable is occurring
   int incomingphrase;    ///< indicates whether any sounds have been detected
   int phraseready;       ///< indicates ending of phrase, ready for processing
   int phraseaddpostpad;  ///< indicates phrase captured, add padding to end
   int usedframes;        ///< number of frames used in phrase
   int predelaycount;     ///< delay start of VAD detection by prepadsize
   int postdelaycount;    ///< delay end   of VAD detection by postpadsize
   int frameindex;        ///< index to next frame to be filled
   int energyframeindex;  ///< index to place next filtered energy sample

   double samplingrate;   ///< sampling rate of audio
   double lothresh;       ///< lower energy threshold
   double hithresh;       ///< higher energy threshold
   double prevtestenergy; ///< test energy from previous frame
   double *leftchannel;   ///< buffer for left  channel frames
   double *rightchannel;  ///< buffer for right channel frames
   double *leftfenergy;   ///< array for left  channel filtered energy values
   double *rightfenergy;  ///< array for right channel filtered energy values

   filter *sleftfilter;   ///< filter for left  energy data
   filter *srightfilter;  ///< filter for right energy data
} simplevad;

/** 
 * Calculates the energy in a frame of audio data.
 * 
 * @param audio audio data
 * @param len length of audio buffer
 * @param en_filter pointer to a filter (NULL if not used)
 * 
 * @return RMS energy in audio buffer
 */
   
double calc_energy(double *audio, int len, filter *en_filter);
   
/**
 * Applies VAD to a frame of speech in internal buffer.
 * Use simplevad_getnextframes to get pointer to next internal frame.
 * Currently only left channel is used in detection, and returned energy
 * values have delay of 2 frames.
 *
 * While VAD is still detecting a phrase, the number of sounds detected
 * is returned. When detection is complete,
 * (phraseframes << SIMPLEVAD_PhraseFramesShift) | soundcount
 * is returned, where phraseframes is the number of frames in the phrase.
 * If this function is called again before the internal buffer has been
 * drained with simplevad_getphraseframes,
 * -((phraseframes << SIMPLEVAD_PhraseFramesShift) | soundcount)
 * is returned
 *
 * Soundcount and phraseframes can be decoded from returned value retval by:
 * soundcount   =  retval & SIMPLEVAD_SoundCountMask;
 * phraseframes = (retval & SIMPLEVAD_PhraseFramesMask) >>
 *                    SIMPLEVAD_PhraseFramesShift;
 */
int apply_simplevad(simplevad *ssvad, double *leftenergy, double *rightenergy);
int apply_simplevad2(simplevad *ssvad, double *leftchannel,
                     double *rightchannel, double *leftenergy,
                     double *rightenergy, int frameindex);

/**
 * Returns pointers to next internal frame of detected speech through
 * leftchannel and rightchannel. DO NOT allocate memory to these variables!
 * Use this function after apply_simplevad has detected activity.
 * Returned energy values are compensated for delay.
 */
int simplevad_getphraseframes(simplevad *ssvad,
                              double **leftchannel, double **rightchannel,
                              double  *leftenergy,  double  *rightenergy);

/**
 * Returns pointers to next internal frame of speech to be filled through
 * leftchannel and rightchannel. DO NOT allocate memory to these variables!
 * Use this function to get the next frame, fill it with audio, then call
 * apply_simplevad.
 */
int simplevad_getnextframes(simplevad *ssvad,
                            double **leftchannel, double **rightchannel);

/**
 * Resets a simplevad structure
 */
void reset_simplevad(simplevad *ssvad);
void reset_simplevad2(simplevad *ssvad, int frameindex);

/**
 * Creates a simplevad structure
 */
simplevad* create_simplevad(double samplingrate, int framelength,
                            int maxsounds, int maxframes, int maxframesdist,
                            int padframes, double lothresh, double hithresh);

simplevad* create_simplevad2(double samplingrate, int framelength,
                             int maxsounds, int maxframesdist, int padframes,
                             double lothresh, double hithresh);

/**
 * Frees resources used by a simplevad structure
 */
void destroy_simplevad(simplevad *ssvad);

#ifdef __cplusplus
}
#endif //extern "C"
#endif //SIMPLEVAD_H
