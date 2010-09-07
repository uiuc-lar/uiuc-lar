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
 * @file   autocorr.h
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Fri Jan 24 2002
 *
 * @brief  Standard and warped autocorrelation functions. See Matt Kleffner's
 *         MS thesis for more info on warped autocorrelation
 *
 * $Log: autocorr.h,v $
 * Revision 1.1  2005/05/09 20:58:09  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2005/03/18 17:13:20  mrmcclai
 * Imported Sources
 *
 * Revision 1.2  2003/02/03 19:30:00  kleffner
 * Added comments, license
 *
 * Revision 1.1  2002/05/14 21:46:02  kleffner
 * Initial checkin
 *
 */

#ifndef AUTOCORR_H
#define AUTOCORR_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Autocorrelation over lag values ranging from lolag to hilag
 */
void autocorrN(double *data, int length, int lolag, int hilag, double *acorr);

/**
 * Cross-correlation over lag values ranging from lolag to hilag with
 * optional normalization
 */
void crosscorrN(double *data1, double *data2, int length,
                int lolag, int hilag, int normalize, double *xcorr);

/**
 * Autocorrelation at a single lag value
 */
double autocorr(double *data, int length, int lag);

/**
 * Warped autocorrelation over lag values ranging from 0 to hilag
 */
void warped_autocorr(double *data, int length, double alpha, int hilag,
                     double *temp, double *acorr);

/**
 * Sampling rate to warp parameter conversion based on the Bark scale
 */
double warp_parameter(double samplerate);

#ifdef __cplusplus
}
#endif //extern "C"
#endif //AUTOCORR_H
