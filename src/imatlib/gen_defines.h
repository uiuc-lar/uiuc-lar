/**
 * @file   gen_defines.h
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Mon Jul 12 18:21:33 2004
 * 
 * @brief  useful macros
 * 
 *
 * $Log: gen_defines.h,v $
 * Revision 1.1  2005/05/09 20:53:20  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.2  2004/08/10 23:03:06  k-squire
 * Backported changes from illy/matrix
 *
 * Revision 1.1.2.2  2004/08/08 23:07:45  k-squire
 * Moved REAL_FORMAT_* definitions to gen_defines.h.
 *
 * Revision 1.1.2.1  2004/08/03 07:04:15  k-squire
 * Initial checkin.
 *
 * Revision 1.1  2004/08/03 07:02:31  k-squire
 * Initial checkin.
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 */

#ifndef GEN_DEFINES_H
#define GEN_DEFINES_H

static char *gen_defines_H_Id = "$Id: gen_defines.h,v 1.1 2005/05/09 20:53:20 mrmcclai Exp $";

#define swap(a__,b__,tmp__) ((tmp__)=(a__),(a__)=(b__), (b__)=(tmp__))

#ifdef USE_DOUBLE
#define REAL_FORMAT_OUT "%20.16lf"
#define REAL_FORMAT_IN "%lf"
#else
#define REAL_FORMAT_OUT "%12.8f"
#define REAL_FORMAT_IN "%f"
#endif


#endif // GEN_DEFINES_H
