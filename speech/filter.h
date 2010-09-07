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
 * @file   filter.h
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Sat Feb 02 2002
 *
 * @brief  Structures and functions necessary for filtering data using both
 *         traditional and warped methods. See Matt Kleffner's MS thesis for a
 *         discussion of the warped algorithms
 *
 * $Log: filter.h,v $
 * Revision 1.1  2005/05/09 20:58:09  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2005/03/18 17:13:20  mrmcclai
 * Imported Sources
 *
 * Revision 1.2  2003/02/03 19:35:02  kleffner
 * Added comments, license
 *
 * Revision 1.1  2002/05/14 21:46:04  kleffner
 * Initial checkin
 *
 */

#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Structure containing information necessary for filtering data
 */
typedef struct
{
   int N;             ///< order of numerator
   int M;             ///< order of denominator
   int Omax;          ///< MAX(N,M)
   int Omin;          ///< MIN(N,M)
   int istate;        ///< circular index to states
   double *numcoeffs; ///< numerator coefficients
   double *dencoeffs; ///< denominator coefficients
   double *states;    ///< states of the filter
} filter;

/**
 * Structure containing additional information for warped filtering
 */
typedef struct
{
   int order;         ///< order of the warped filter
   double alpha;      ///< warp parameter
   double *numvector; ///< numerator coefficients, (1-alpha*Z^(-1))^N
   double *denmatrix; ///< matrix for calculating denominator coefficients
} warparrays;

/**
 * Filter data
 */
void filter_data(filter *sfilter, double *data, int length, double *output);

/**
 * Resets a filter structure
 */
void reset_filter(filter *sfilter);

/**
 * Creates a filter structure
 */
filter* create_filter(int numorder, int denorder);

/**
 * Free resources used by a filter 
 */
void destroy_filter(filter *sfilter);

/**
 * Converts warped coefficients to filter structure coefficients
 */
void calculate_warpfilter(warparrays *swarparrays, double *warpcoeffs,
                          double ingain, filter *sfilter);

/**
 * Creates a warparrays structure
 */
warparrays* create_warparrays(int order, double alpha);

/**
 * Free resources used by a warparrays structure
 */
void destroy_warparrays(warparrays *swarparrays);

#ifdef __cplusplus
}
#endif //extern "C"
#endif //FILTER_H
