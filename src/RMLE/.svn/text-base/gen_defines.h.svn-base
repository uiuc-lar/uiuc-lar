/**
 * @file   gen_defines.h
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Mon Jul 12 18:21:33 2004
 * 
 * @brief  useful macros
 * 
 *
 * $Log: gen_defines.h,v $
 * Revision 1.1  2005/05/09 20:52:59  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2005/03/18 17:11:38  mrmcclai
 * Imported Sources
 *
 * Revision 1.2  2004/08/10 23:49:12  k-squire
 * Backported changes from illy/hmm.
 * Changed hmm_run to conditionally depend on libIBase, IServer stuff.
 *
 * Revision 1.1.2.1  2004/08/04 06:43:20  k-squire
 * Added REAL_FORMAT definitions.
 *
 * Revision 1.1  2004/08/03 06:38:39  k-squire
 * Import and initial checkin.
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 */

#ifndef GEN_DEFINES_H
#define GEN_DEFINES_H

static char *gen_defines_H_Id = "$Id: gen_defines.h,v 1.1 2005/05/09 20:52:59 mrmcclai Exp $";

#define swap(a__,b__,tmp__) ((tmp__)=(a__),(a__)=(b__), (b__)=(tmp__))

#ifdef USE_DOUBLE
#define REAL_FORMAT_OUT "%20.16g"
#define REAL_FORMAT_IN "%lf"
#else
#define REAL_FORMAT_OUT "%12.8g"
#define REAL_FORMAT_IN "%f"
#endif


#endif // GEN_DEFINES_H
