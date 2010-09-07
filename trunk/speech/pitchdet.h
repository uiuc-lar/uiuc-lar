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
 * @file   pitchdet.h
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Tue Feb 05 2002
 *
 * @brief  Pitch detector and voicing detector. See Matt Kleffner's MS thesis
 *         for a discussion of the algorithms
 *
 * $Log: pitchdet.h,v $
 * Revision 1.1  2005/05/09 20:58:09  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2005/03/18 17:13:20  mrmcclai
 * Imported Sources
 *
 * Revision 1.4  2003/02/03 19:46:22  kleffner
 * Added comments, license
 *
 * Revision 1.3  2002/12/09 21:21:00  kleffner
 * Changed pitch spectrum parameters from samples to normalized cutoff frequency
 * (1 is the sampling rate)
 *
 * Revision 1.2  2002/12/07 00:08:59  kleffner
 * Fixed bug: rfftw_plan is a pointer, so rfftw_plans are stored in structure
 * instead of pointers to rfftw_plans
 *
 * Revision 1.1  2002/05/14 21:46:11  kleffner
 * Initial checkin
 *
 */

#ifndef PITCHDET_H
#define PITCHDET_H

#include "lpc.h"
#include <rfftw.h>

#define CONF_SEARCH 5 // must be odd number
#define CONF_OFFSET CONF_SEARCH / 2

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Structure containing necessary information to perform
 * pitch and voicing estimation
 */
typedef struct
{
   int L;              ///< length of data which pitch is extracted from
   int Lp;             ///< length of data used to calculate vconf
   int lolag;          ///< lowest lag value (highest pitch) in pitch range
   int hilag;          ///< highest lag value (lowest pitch) in pitch range
   int Olpc;           ///< order of spectral whitening LPC's
   int S;              ///< length of spectrum used, should be power of 2
   int Sc;             ///< use spectral indeces of 0-Sc for pitch analysis
   double pitchper;    ///< estimated pitch period
   double vconf;       ///< voicing confidence ( 0: unvoiced, 1: voiced )
   double confarr[CONF_SEARCH];
                       ///< vconf estimates for consecutive lags
   lpc *slpc;          ///< pointer to LPC struct used for spectral whitening
   rfftw_plan fftpl;   ///< fft structure, from FFTW library
   rfftw_plan ifftpl;  ///< ifft structure, from FFTW library
   double *zeropadin;  ///< array to hold audio and padded zeros
   double *spec2acorr; ///< pointer to spectrum to autocorr coeff. matrix
   double *acorr2spec; ///< pointer to autocorr coeff. to spectrum matrix
   double *spectrum;   ///< array containing audio spectrum
   double *subspec;    ///< array containing spectrum from 0 to Sc
   double *lpcacorr;   ///< array containing autocorrelation of LPC coeffs
   double *pitchacorr; ///< autocorrelation used directly in calculating pitch
} pitchdetector;

/**
 * Estimates pitch and voicing from input buffer
 */
void estimate_pitch(pitchdetector *spitchdetector, double *audio);

/**
 * Creates a pitchdetector structure
 */
pitchdetector* create_pitchdetector(int audiolength, int lolag, int hilag,
                                    int lpcorder, double pitchspeccutoff);

/**
 * Free resources used by a pitchdetector structure
 */
void destroy_pitchdetector(pitchdetector *spitchdetector);

#ifdef __cplusplus
}
#endif //extern "C"
#endif //PITCHDET_H
