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
 * @file   synthftr.h
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Wed Feb 20 2002
 *
 * @brief  Structures and functions for generating speech features suitable
 *         for synthesis. See Matt Kleffner's MS thesis for a discussion
 *         of the algorithms
 *
 * $Log: synthftr.h,v $
 * Revision 1.1  2005/05/09 20:58:09  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.2  2005/04/19 19:47:16  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1.2.1  2005/04/01 19:00:20  mrmcclai
 * KEVIN_DEVEL branch files added
 *
 * Revision 1.6.2.2  2005/02/17 06:43:54  k-squire
 * Rearranged synthfeature struct again:
 *   moved voice_active after gain
 *   added energy after voice_active
 *
 * Revision 1.6.2.1  2004/08/08 23:16:37  k-squire
 * Removed timestamp parameter from extract_synthfeature, as it wasn't being
 *    used.
 * Removed tindex, timestamp from synthfeature generator for same reason.
 * Added a voice_active field to synthfeature, and rearranged yet again so
 *    that the features most likely used by the HMM are first.
 * Added copy_synthfeature() function.
 *
 * Revision 1.6  2004/08/03 02:02:45  k-squire
 * Changed synthfeature structure to use all doubles, and rearranged the order
 *    slightly so that the features the HMM uses for recognition are all
 *    together. (gain, pitchper, and timestamp are now at the end of the
 *    structure.)
 *
 * Revision 1.5  2004/07/30 05:46:09  k-squire
 * Fixed free of NULL variable in destroy_synthfeature.
 * Added create_synthfeature_list, destroy_synthfeature_list,
 *    save_synthfeature.
 *
 * Revision 1.4  2003/02/03 19:51:36  kleffner
 * Added comments, license
 *
 * Revision 1.3  2002/12/18 22:31:28  kleffner
 * Modified to generate and store quantized log area ratios, pitch, gain and
 * voicing confidence
 * Added functions to quantize and "unquantize" features
 *
 * Revision 1.2  2002/12/09 21:27:36  kleffner
 * Implemented clipping/scaling of final voicing confidence score,
 * changed spectral parameters that are input to pitch detector
 *
 * Revision 1.1  2002/05/14 21:46:15  kleffner
 * Initial checkin
 *
 */

#ifndef SYNTHFTR_H
#define SYNTHFTR_H

#include "lpc.h"
#include "pitchdet.h"
#include "frame.h"
/*
#ifdef HAVE_INTTYPES_H
#include <inttypes.h>
#else
#include "istdint.h"
#endif
*/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * Structure containing speech features suitable for synthesis
 *
 * NOTE: the HMM code is dependent on this variable order!
 */
typedef struct
{
   double   *lar;                ///< log area ratios
   double   pitchper;            ///< pitch period
   double   gain;                ///< lp prediction error gain
   double   voice_active;        ///< set to 1 if voice is active
   double   energy;              ///< energy
   double   vconf;               ///< voicing confidence
} synthfeature;

/**
 * Structure containing necessary information for generating synthfeatures
 */
typedef struct
{
   int           framelength;    ///< length of single frame
   int           warpedcoeffs;   ///< nonzero if warped coefficients are used
   int           numberofframes; ///< number of frames used in generation
   int           delay;          ///< delay in frames between input and ouput
   int           resetctr;       ///< counter to handle delay between input
                                 ///<    and output after resetting generator
   int           index;          ///< index into pitchper and vconf arrays
   double        samplingrate;   ///< sampling rate of audio
   double        alpha;          ///< warp parameter
   double        gaintemp;       ///< gain storage for aligning lar's and pitch
   double        *refltemp;      ///< temp storage for aligning lar's and pitch
   double        *wwin;          ///< pointer to weighting window
   double        *wacorrtemp;    ///< temporary array for warped autocorr
   double        *wwintemp;      ///< temporary array for windowed audio
   double        pitchper[3];    ///< buffer of pitch values for median op
   double        vconf[3];       ///< buffer of confidence values for median op
   double        vconfmin;       ///< value of vconf that is mapped to zero
   double        vconfscale;     ///< 1.0 / (1 - vconfmin)
   frameholder   *sframeholder;  ///< array for buffering multiple frames
   lpc           *slpc;          ///< lpc structure
   pitchdetector *spd;           ///< pitch detector structure
} synthfeature_generator;

/**
 * Creates synthfeature structure from overlapped windows of speech
 */
void extract_synthfeature(synthfeature_generator *ssfg, synthfeature *ssf,
                          double *audio);

/**
 * Converts reflection coefficients to log area ratios
 */
void refl2lar(double *lar, double *refl, int order);

/**
 * Converts log area ratios to reflection coefficients
 */
void lar2refl(double *refl, double *lar, int order);

/**
 * Quantizes pitch period for storage in synthfeature structure
 */
unsigned short Q_pitch(double pitch);

/**
 * Quantizes voicing confidence for storage in synthfeature structure
 */
unsigned short Q_vconf(double vconf);

/**
 * Un-quantizes pitch period in synthfeature storage format
 */
double UQ_pitch(unsigned short pitch);

/**
 * Un-quantizes voicing confidence in synthfeature storage format
 */
double UQ_vconf(unsigned short vconf);

/**
 * Resets a synthfeature_generator structure
 */
void reset_sfgenerator(synthfeature_generator *ssfg);

/**
 * Calculates a hamming weighting window
 */
void wwin_hamming(double* wwin, int length);

/**
 * Creates a synthfeature structure
 */
synthfeature* create_synthfeature(int lpcorder);

synthfeature* create_synthfeature_list(int lpcorder, int length);

/**
 * Frees resources used by a synthfeature structure
 */
void destroy_synthfeature(synthfeature *ssynthfeature);

void destroy_synthfeature_list(synthfeature *ssynthfeature);

/**
 * Copy a synthfeature
 *
 * Assumes structures are already allocated!
 */

void copy_synthfeature(synthfeature *dest, synthfeature *src, int lpcorder);

/**
 * Save a synthfeature to a file
 */

int save_synthfeature(FILE *featurefile,
                      int lpcorder,
                      synthfeature *ssynthfeature);

///> pitchspeccutoff is fcutoff/fs
/**
 * Creates a synthfeature_generator structure
 */
synthfeature_generator* create_sfgenerator(double samplingrate, 
                                           int lpcorder,
                                           int warpedcoeffs,
                                           int framelength,
                                           int numberofframes,
                                           int lolag, int hilag,
                                           int pitchlpcorder,
                                           double pitchspeccutoff,
                                           double vconfmin);

/**
 * Frees resources used by a synthfeature structure
 */
void destroy_sfgenerator(synthfeature_generator *ssfg);

#ifdef __cplusplus
}
#endif //extern "C"
#endif //SYNTHFTR_H
