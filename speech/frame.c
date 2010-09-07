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
 * @file   frame.c
 * @author Matt Kleffner <kleffner@ifp.uiuc.edu>
 * @date   Wed Feb 06 2002
 *
 * @brief  Structure and functions for processing 1-D signals in overlapping
 *         windows.
 *
 * $Log: frame.c,v $
 * Revision 1.1  2005/05/09 20:58:08  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2005/03/18 17:13:20  mrmcclai
 * Imported Sources
 *
 * Revision 1.3  2004/03/29 21:57:42  k-squire
 * Changed alloc.h to malloc.h, to avoid compilation error with gcc.
 *
 * Revision 1.2  2003/02/03 19:40:19  kleffner
 * Added comments, license
 * Added function reset_frameholder
 *
 * Revision 1.1  2002/05/14 21:46:05  kleffner
 * Initial checkin
 *
 */

#include "frame.h"
#include <malloc.h>
#include <string.h>

/**
 * @param  sframeholder   pointer to frameholder
 * @param  newframe       pointer to frame data to copy to end of buffer,
 *                           must contain at least L samples
 */
void add_frame(frameholder *sframeholder, double *newframe)
{
   memcpy(sframeholder->framedata, sframeholder->fdplusL,
          sframeholder->M * sizeof(double));
   memcpy(sframeholder->fdplusM, newframe, sframeholder->L * sizeof(double));
}

/**
 * @param  sframeholder   pointer to frameholder
 */
void reset_frameholder(frameholder *sframeholder)
{
   memset(sframeholder->framedata, 0, sframeholder->T * sizeof(double));
}

/**
 * @param  framelength    length of individual frame
 * @param  numberofframes number of frames 
 *
 * @return new frameholder on success, NULL on error
 */
frameholder* create_frameholder(int framelength, int numberofframes)
{
   frameholder *sframeholder = (frameholder *)malloc(sizeof(frameholder));
   if( sframeholder==NULL ) return(NULL);

   sframeholder->L         = framelength;
   sframeholder->N         = numberofframes;
   sframeholder->M         = framelength * (numberofframes - 1);
   sframeholder->T         = framelength * numberofframes;
   sframeholder->framedata = (double *)malloc(sframeholder->T*sizeof(double));
   sframeholder->fdplusL   = sframeholder->framedata + framelength;
   sframeholder->fdplusM   = sframeholder->framedata + sframeholder->M;

   if( sframeholder->framedata == NULL )
   {
      destroy_frameholder(sframeholder);
      return(NULL);
   }
   else
   {
      memset(sframeholder->framedata, 0, sframeholder->T*sizeof(double));
      return(sframeholder);
   }
}

/**
 * @param  sframeholder   pointer to frameholder
 */
void destroy_frameholder(frameholder *sframeholder)
{
   free(sframeholder->framedata);
   free(sframeholder);
}
