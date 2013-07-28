/**
 * @file   N.hh
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Thu Jul  8 20:49:36 2004
 * 
 * @brief  calculation of Gaussian and derivatives at y
 * 
 * $Log: N.hh,v $
 * Revision 1.1  2005/05/09 20:52:58  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.4  2004/08/11 03:09:14  k-squire
 * Merged changes from KEVIN_DEVEL branch
 *
 * Revision 1.3.2.1  2004/08/11 03:00:13  k-squire
 * Changed IMatLib to imatlib.
 *
 * Revision 1.3  2004/08/11 01:15:29  k-squire
 * Changed "matrix/*.h" to "IMatLib/*.h"
 *
 * Revision 1.2  2004/08/10 23:49:12  k-squire
 * Backported changes from illy/hmm.
 * Changed hmm_run to conditionally depend on libIBase, IServer stuff.
 *
 * Revision 1.1  2004/08/03 06:38:39  k-squire
 * Import and initial checkin.
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 *
 */

#ifndef N_H
#define N_H

#include <torch/general.h>
   
#include "../imatlib/IVec.hh"
#include "../imatlib/IMat.hh"
#include "../imatlib/IMatVecOps.hh"

static char *N_H_Id = "$Id: N.hh,v 1.1 2005/05/09 20:52:58 mrmcclai Exp $";

#ifndef USE_DOUBLE
#define pow(x,y) powf((x),(y))
#define sqrt(x)  sqrtf(x)
#define exp(x)   expf(x)
#endif

real N(IVec *y, IVec *mu, IMat *U, IVec *df_mu = NULL, IMat *df_U = NULL);


#endif /* N_H */
