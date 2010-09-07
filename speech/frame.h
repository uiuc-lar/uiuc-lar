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
 * @file   frame.h
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Wed Feb 06 2002
 *
 * @brief  Structure and functions for processing 1-D signals in overlapping
 *         windows.
 *
 * $Log: frame.h,v $
 * Revision 1.1  2005/05/09 20:58:09  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2005/03/18 17:13:20  mrmcclai
 * Imported Sources
 *
 * Revision 1.2  2003/02/03 19:38:57  kleffner
 * Added comments, license
 *
 * Revision 1.1  2002/05/14 21:46:06  kleffner
 * Initial checkin
 *
 */

#ifndef FRAME_H
#define FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Structure for processing 1-D signals in overlapping windows.
 * Each frameholder contains multiple frames and each frame is added to the
 * end of the buffer pointed to by framedata
 */
typedef struct
{
   int L;             ///< Length of individual frame
   int N;             ///< Number of consecutive frames required for processing
   int M;             ///< Number of samples to save when adding frames: L(N-1)
   int T;             ///< Total number of samples in N frames: L*N
   double *framedata; ///< Pointer to array of consecutive frames
   double *fdplusL;   ///< Pointer to framedata[L];
   double *fdplusM;   ///< Pointer to framedata[M];
} frameholder;

/**
 * Add a frame to the end of the buffer
 */
void add_frame(frameholder *sframeholder, double *newframe);

/**
 * Reset buffer to zero
 */
void reset_frameholder(frameholder *sframeholder);

/**
 * Creates a frameholder
 */
frameholder* create_frameholder(int framelength, int numberofframes);

/**
 * Free resources used by a frameholder
 */
void destroy_frameholder(frameholder *sframeholder);

#ifdef __cplusplus
}
#endif //extern "C"
#endif //FRAME_H
