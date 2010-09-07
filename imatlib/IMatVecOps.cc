/**
 * @file   IMatVecOps.cc
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Sat Jul 10 15:21:37 2004
 * 
 * @brief  BLAS and LAPACK operations on Mat and Vec classes
 * 
 * $Log: IMatVecOps.cc,v $
 * Revision 1.1  2005/05/09 20:53:20  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 * 
 */

static char * IBLAS_OPS_CC_Id = "$Id: IMatVecOps.cc,v 1.1 2005/05/09 20:53:20 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include "IMat.hh"
#include "IMatVecOps.hh"

/**
 * clapack-like interface to sgesvd_. See IMatVecOps.hh for details.
 * 
 */

int clapack_sgesvd(const enum CBLAS_ORDER Order, const enum CLAPACK_JOB jobu,
                   const enum CLAPACK_JOB jobvt, const int M, 
                   const int N, const float *A, const int lda, 
                   const float *S, const float *U, const int ldu, 
                   const float *VT, const int ldvt, const float *work,
                   const int lwork)
{
   int info;
   char jobu_c;
   char jobvt_c;

   switch (jobu)
   {
      case ClapackAll:
         jobu_c = 'A';
         break;
      case ClapackNone:
         jobu_c = 'N';
         break;
      case ClapackMin:
         jobu_c = 'S';
         break;
      case ClapackMinOverwrite:
         jobu_c = 'O';
         break;
   }
   
   switch (jobvt)
   {
      case ClapackAll:
         jobvt_c = 'A';
         break;
      case ClapackNone:
         jobvt_c = 'N';
         break;
      case ClapackMin:
         jobvt_c = 'S';
         break;
      case ClapackMinOverwrite:
         jobvt_c = 'O';
         break;
   }
   
   if (Order == CblasRowMajor)
      sgesvd_(&jobu_c, &jobvt_c, &N, &M, A, &lda, S, VT, &ldvt, U, &ldu, 
              work, &lwork, &info);
   else
      sgesvd_(&jobu_c, &jobvt_c, &M, &N, A, &lda, S, U, &ldu, VT, &ldvt, 
              work, &lwork, &info);

   return info;
}

/**
 * clapack-like interface to dgesvd_. See IMatVecOps.hh for details.
 * 
 */

int clapack_dgesvd(const enum CBLAS_ORDER Order, const enum CLAPACK_JOB jobu,
                   const enum CLAPACK_JOB jobvt, const int M, 
                   const int N, const double *A, const int lda, 
                   const double *S, const double *U, const int ldu, 
                   const double *VT, const int ldvt, const double *work,
                   const int lwork)
{
   int info;
   char jobu_c;
   char jobvt_c;

   switch (jobu)
   {
      case ClapackAll:
         jobu_c = 'A';
         break;
      case ClapackNone:
         jobu_c = 'N';
         break;
      case ClapackMin:
         jobu_c = 'S';
         break;
      case ClapackMinOverwrite:
         jobu_c = 'O';
         break;
   }
   
   switch (jobvt)
   {
      case ClapackAll:
         jobvt_c = 'A';
         break;
      case ClapackNone:
         jobvt_c = 'N';
         break;
      case ClapackMin:
         jobvt_c = 'S';
         break;
      case ClapackMinOverwrite:
         jobvt_c = 'O';
         break;
   }
   
   if (Order == CblasRowMajor)
      dgesvd_(&jobu_c, &jobvt_c, &N, &M, A, &lda, S, VT, &ldvt, U, &ldu, 
              work, &lwork, &info);
   else
      dgesvd_(&jobu_c, &jobvt_c, &M, &N, A, &lda, S, U, &ldu, VT, &ldvt, 
              work, &lwork, &info);

   return info;
}

/**
 * Determinant of A, calculated as the product of the diagonal of the
 * cholesky decomposition of A (from SymMatCholFact)
 * 
 * See IMatVecOps.hh for details.
 * 
 */


real CholDet(IMat *chol_A)
{
   int i;
   real prod = 1.0;
   real **p = chol_A->ptr;
   
   for (i = 0; i < chol_A->m; i++)
      prod *= p[i][i];

   prod *= prod;

   return prod;
}

/**
 * Element by element multiply of two vectors (real *)
 * 
 * See IMatVecOps.hh for details.
 *
 */

      
void DotTimes(int n, real alpha, real *x, int incX, real *y, int incY)
{
   if (alpha == 1.0)
      for (int i = 0; i < n; i++, x+=incX, y+=incY)
         *y *= *x;
   else
      for (int i = 0; i < n; i++, x+=incX, y+=incY)
         *y *= (*x * alpha);
}
