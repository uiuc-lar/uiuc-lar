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
 * @file   spechgen.h
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Wed Mar 20 2002
 *
 * @brief  Speech generation structure. See Matt Kleffner's MS thesis
 *         for a discussion of the algorithms
 *
 * $Log: spechgen.h,v $
 * Revision 1.1  2005/05/09 20:58:08  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2005/03/18 17:13:20  mrmcclai
 * Imported Sources
 *
 * Revision 1.3  2003/02/03 19:48:06  kleffner
 * Added comments, license
 *
 * Revision 1.2  2002/12/18 22:12:32  kleffner
 * Modified to use quantized log area ratios, pitch and gain and voicing
 * confidence
 * Fractional pitch periods are allowed and are no longer truncated
 *
 * Revision 1.1  2002/05/14 21:46:13  kleffner
 * Initial checkin
 *
 */

#ifndef SPECHGEN_H
#define SPECHGEN_H

#include "filter.h"
#include "synthftr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Structure containing necessary information to perform speech generation
 */
typedef struct
{
   int        framelength;  ///< length of single frame
   int        warpedcoeffs; ///< nonzero if warped coefficients are used
   int        noiseindex;   ///< index into noise waveform
   double     impulseindex; ///< index for placing pitch impulses
   double     samplingrate; ///< sampling rate of audio
   double     *glottalwvfm; ///< storage for glottal waveform
   double     *noise;       ///< storage for white noise
   filter     *sfilter;     ///< filter structure
   warparrays *swa;         ///< arrays for filtering with warped coeffs
   lpc        *slpc;        ///< scratch LPC structure, used in LARs->LPCs
} speech_generator;

/**
 * Generates speech based on information in synthfeature structure
 */
void generate_speech(speech_generator *sspg, synthfeature *ssf, double *audio);

/**
 * Resets a speech_generator structure
 */
void reset_spgenerator(speech_generator *sspg);

/**
 * Creates a speech_generator structure
 */
speech_generator* create_spgenerator(double samplingrate,
                                     int lpcorder,
                                     int framelength,
                                     int warpedlpc,
                                     double alpha);

/**
 * Free resources used by a speech_generator structure
 */
void destroy_spgenerator(speech_generator *sspg);

#ifdef __cplusplus
}
#endif //extern "C"
#endif //SPECHGEN_H
