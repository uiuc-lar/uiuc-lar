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
 * @file   lpc.h
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Fri Jan 24 2002
 *
 * @brief  lpc structure and associated functions. Notation borrowed from
 * L.R. Rabiner and R.W. Schafer, "Digital Processing of Speech Signals,"
 * Prentice-Hall, 1978
 *
 * $Log: lpc.h,v $
 * Revision 1.1  2005/05/09 20:58:08  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2005/03/18 17:13:20  mrmcclai
 * Imported Sources
 *
 * Revision 1.3  2003/02/03 19:42:27  kleffner
 * Added comments, license
 *
 * Revision 1.2  2002/12/18 22:36:52  kleffner
 * Added function to convert reflection coefficients into LPCs
 *
 * Revision 1.1  2002/05/14 21:46:08  kleffner
 * Initial checkin
 *
 */

#ifndef LPC_H
#define LPC_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Structure containing arrays and values necessary to calculate LPCs.
 */
typedef struct
{
   int order;     ///< order of the LPC's
   double *lpc;   ///< lpc coefficients
   double *refl;  ///< array of reflection coefficients
   double *acorr; ///< array of autocorrelation values
   double gain;   ///< gain extracted from residual error power
   double *E;     ///< temporary array for calculating LPC's
   double *alast; ///< temporary array for calculating LPC's
} lpc;

/**
 * Calculates LPCs and reflection coefficients
 */
void calculate_lpc(lpc *slpc);

/**
 * Calculates LPCs from reflection coefficients
 */
void refl2lpc(lpc *slpc);

/**
 * Function for calculating autocorrelation of lpc coefficients.
 */
void autocorr_lpc(lpc *slpc, double *lpcacorr);

/**
 * Create an lpc structure
 */
lpc* create_lpc(int order);

/**
 * Free resources used by an lpc structure
 */
void destroy_lpc(lpc *slpc);

#ifdef __cplusplus
}
#endif //extern "C"
#endif //LPC_H
