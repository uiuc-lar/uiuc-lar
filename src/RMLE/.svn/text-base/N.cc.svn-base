/**
 * @file   N.cc
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Thu Jul  8 20:55:32 2004
 * 
 * @brief  calculation of Gaussian and derivatives at y
 * 
 * $Log: N.cc,v $
 * Revision 1.1  2005/05/09 20:52:58  mrmcclai
 * *** empty log message ***
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

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <torch/general.h>
#include "N.hh"


real N(IVec *y, IVec *mu, IMat *R, IVec *df_mu, IMat *df_R)
{
   static IVec *yy = new IVec;
   static IMat *SigmaInv = new IMat;
   static IVec *SigmaInv_yy = new IVec;

   real nn;

   int d = y->n;

   // resize local vectors/matrices if necessary

   if (yy->n != d)
   {
      yy->resize(d);
      SigmaInv->reshape(d,d);
      SigmaInv_yy->resize(d);
   }
   
   VecCopy(y,yy);
   VecSub(mu,yy);                         // yy = y-mu;

   real det_Sigma = CholDet(R);

   MatCopy(R,SigmaInv);
   MatCholInv(SigmaInv);                  // SigmaInv = inv(Sigma) = inv(R'R)

//   SigmaInv_yy->zero();
   SymMatVecMult(1.0, SigmaInv, yy, 0.0, SigmaInv_yy); // SigmaInv_yy = SigmaInv * yy
   
   nn = (1/(pow(2.0*M_PI,(d/2.0)) * sqrt(det_Sigma))) *
      exp(-0.5 * VecDot(yy, SigmaInv_yy));

   // Calc derivatives

   if (df_mu)
   {
      VecCopy(SigmaInv_yy, df_mu);
      VecScale(nn, df_mu);
   }
   
   if (df_R)
   {
      // SigmaInv = SigmaInv - SigmaInv_yy*SigmaInv_yy'

      SymMatRankUpdate(-1, SigmaInv_yy, SigmaInv);
   
      // df_R = -nn * R * (SigmaInv - SigmaInv_yy*SigmaInv_yy')
   
      MatSymMatMult(-nn, R, SigmaInv, 0.0, df_R);

   }

   return nn;
}

