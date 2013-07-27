/**
 * @file   IMatVecOps.hh
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Fri Jul  9 22:37:55 2004
 * 
 * @brief  BLAS and LAPACK interface for use with IMat classes
 * 
 * $Log: IMatVecOps.hh,v $
 * Revision 1.1  2005/05/09 20:53:20  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.3.2.1  2004/10/26 10:48:55  k-squire
 * Added comment to Matrix copy: does not work when ld > n
 * Fixed GenMatVecMult to use A->ld
 * Fixed comments in Cholesky factorization: A=U'U (not UU')
 *
 * Revision 1.3  2004/07/30 03:57:04  k-squire
 * Fixed gemm call to properly check for tranpose of A.
 *
 * Revision 1.2  2004/07/29 07:08:38  k-squire
 * Added rank K update of a symmetric matrix.
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 * 
 */

#ifndef IBLAS_OPS_HH
#define IBLAS_OPS_HH

static char * IBLAS_OPS_HH_Id = "$Id: IMatVecOps.hh,v 1.1 2005/05/09 20:53:20 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include "IMat.hh"
#include <cblas.h>

extern "C"
{
#include <atlas/clapack.h>
}

#ifdef USE_DOUBLE
# define CBLAS(fn) cblas_d##fn
# define CLAPACK(fn) clapack_d##fn
#else
# define CBLAS(fn) cblas_s##fn
# define CLAPACK(fn) clapack_s##fn
#endif

/**
 * @defgroup BLAS1 BLAS level 1 wrappers
 * 
 */
//@{

/** 
 * Vector copy
 * 
 * @param x vector to copy from
 * @param y vector to copy to
 * 
 * @return y = x
 */
#define VecCopy(x,y) \
     CBLAS(copy)((y)->n,(x)->ptr,(x)->stride,(y)->ptr,(y)->stride)

/** 
 * Vector scale
 * 
 * @param alpha scale factor
 * @param x vector
 * 
 * @return x = alpha*x
 */

#define VecScale(alpha, x) \
     CBLAS(scal)((x)->n, alpha, (x)->ptr, (x)->stride)

/** 
 * Scaled vector addition
 * 
 * @param alpha scale factor
 * @param x vector
 * @param y output vector
 * 
 * @return y = alpha*x + y
 */

#define VecAddScaled(alpha,x,y) \
     CBLAS(axpy)((x)->n,(alpha),(x)->ptr,(x)->stride,(y)->ptr,(y)->stride)

/** 
 * Vector addition
 * 
 * @param x input vector
 * @param y output vector
 * 
 * @return y = x+y
 */

#define VecAdd(x,y) \
     VecAddScaled(1.0,(x),(y))

/** 
 * Vector subtraction
 * 
 * @param x input vector
 * @param y output vector
 * 
 * @return y = -x+y
 */

#define VecSub(x,y) \
     VecAddScaled(-1.0,(x),(y))

/** 
 * Vector dot product
 * 
 * @param x input vector
 * @param y output vector
 * 
 * @return x dot y (scalar)
 */

#define VecDot(x,y) \
     CBLAS(dot)((x)->n,(x)->ptr,(x)->stride,(y)->ptr,(y)->stride)

/** 
 * Matrix scaling
 * 
 * @param alpha scale factor
 * @param A matrix
 * 
 * @return A = alpha*A
 */

#define MatScale(alpha, A) \
     CBLAS(scal)((A)->m*(A)->n, (alpha), (A)->base, 1)

/** 
 * Matrix copy  (DOES NOT WORK WHEN LD > N!!!!)
 * 
 * @param A input matrix
 * @param B output matrix
 * 
 * @return B = A
 */

#define MatCopy(A,B) \
     CBLAS(copy)((B)->m*(B)->n, (A)->base, 1, (B)->base, 1)

/** 
 * Scaled matrix addition
 * 
 * @param alpha scale factor
 * @param A input matrix
 * @param B output matrix
 * 
 * @return B = alpha*A + B
 */

#define MatAddScaled(alpha,A,B) \
        CBLAS(axpy)((B)->m*(B)->n, (alpha), (A)->base, 1, (B)->base, 1)


/** 
 * Matrix addition
 * 
 * @param A input matrix
 * @param B output matrix
 * 
 * @return B = A + B
 */

#define MatAdd(A,B) \
     MatAddScaled(1.0,(A),(B))

/** 
 * Matrix subtraction
 * 
 * @param A input matrix
 * @param B output matrix
 * 
 * @return B = B - A
 */

#define MatSub(A,B) \
     MatAddScaled(-1.0,(A),(B))
//@}

/**
 * @defgroup BLAS2 BLAS level 2 wrappers
 * 
 */
//@{

/** 
 * General matrix-vector multiplication
 * 
 * @param alpha scale factor
 * @param A input matrix
 * @param transA CblasTrans or CblasNoTrans
 * @param x input vector
 * @param beta scale factor
 * @param y output vector
 * 
 * @return y = alpha * Ax + beta*y
 */

#define GenMatVecMult(alpha,A,transA,x,beta,y) \
     CBLAS(gemv)(CblasRowMajor, (transA), \
                 (A)->m, (A)->n, \
                 (alpha), \
                 (A)->base, (A)->ld, \
                 (x)->ptr, (x)->stride, \
                 (beta), \
                 (y)->ptr, (y)->stride)

/** 
 * Matrix-vector multiplication
 * 
 * @param A input matrix
 * @param transA CblasTrans or CblasNoTrans
 * @param x input vector
 * @param y output vector
 * 
 * @return y = Ax
 */

#define MatVecMult(A,transA,x,y) \
     GenMatVecMult(1.0,(A),(transA),(x),0.0,(y))

/** 
 * Symmetric matrix-vector multiplication
 * 
 * @param alpha scale factor
 * @param A input matrix (symmetric, stored in upper triangle)
 * @param x input vector
 * @param beta scale factor
 * @param y output vector
 * 
 * @return y = alpha * Ax + beta*y (A symmetric, stored in upper triangle)
 */

#define SymMatVecMult(alpha,A,x,beta,y) \
     CBLAS(symv)(CblasRowMajor, CblasUpper, (A)->m, (alpha), \
                 (A)->base, (A)->m, \
                 (x)->ptr, (x)->stride, (beta), (y)->ptr, (y)->stride)

/** 
 * Rank 1 update of a symmetric matrix
 * 
 * @param alpha scale factor
 * @param x input vector
 * @param A output matrix (symmetric, stored in upper triangle)
 * 
 * @return A = alpha*x*x' + A
 */

#define SymMatRankUpdate(alpha, x, A) \
     CBLAS(syr)(CblasRowMajor, CblasUpper, (A)->m, (alpha), \
                (x)->ptr, (x)->stride, (A)->base, (A)->m)

//@}

/**
 * @defgroup BLAS3 BLAS level 3 wrappers
 * 
 */
//@{


/** 
 * General matrix-matrix multiplication (gemm)
 * 
 * @param alpha scale factor
 * @param A matrix
 * @param transA CblasTrans or CblasNoTrans
 * @param B matrix
 * @param transB CblasTrans or CblasNoTrans
 * @param beta scale factor
 * @param C output matrix
 * 
 * @return C = alpha * A * B + beta * C
 */
#define GenMatMatMult(alpha,A,transA,B,transB,beta,C) \
      CBLAS(gemm)(CblasRowMajor,  \
                 (transA), (transB), \
                 (C)->m, (C)->n, \
                 ((transA) == CblasTrans ? (A)->m : (A)->n), \
                 (alpha), \
                 (A)->base, (A)->n, \
                 (B)->base, (B)->n, \
                 (beta), \
                 (C)->base, (C)->n)

/** 
 * Simple matrix-matrix multiplication
 * 
 * @param A input matrix
 * @param transA CblasTrans or CblasNoTrans
 * @param B input matrix
 * @param transB CblasTrans or CblasNoTrans
 * @param C output matrix
 * 
 * @return C = A*B
 */

#define MatMatMult(A,transA,B,transB,C) \
     GenMatMatMult(1.0,(A),(transA),(B),(transB),0.0,(C))

/** 
 * Matrix-symmetric matrix multiplication (symm)
 * 
 * @param alpha scale factor
 * @param A input matrix
 * @param B symmetric input matrix (stored in upper triangle)
 * @param beta scale factor
 * @param C output matrix
 * 
 * @return C = alpha * A * B + beta * C  (B symmetric)
 */

#define MatSymMatMult(alpha, A, B, beta, C) \
     CBLAS(symm)(CblasRowMajor, CblasRight, CblasUpper, (C)->m, (C)->n, \
                 (alpha), (B)->base, (B)->n, (A)->base, (A)->n, beta, \
                 (C)->base, (C)->n)

/** 
 * Rank k update of a symmetric matrix
 * 
 * @param alpha scale factor
 * @param x input vector
 * @param A output matrix (symmetric, stored in upper triangle)
 * 
 * @return A = alpha*x*x' + A
 */

#define SymMatRankKUpdate(alpha, A, transA, beta, C) \
     CBLAS(syrk)(CblasRowMajor, CblasUpper, (transA), \
                 (C)->m, \
                 ((transA) == CblasNoTrans ? (A)->n : (A)->m), \
                 (alpha), \
                 (A)->base, (A)->ld, \
                 (beta), \
                 (C)->base, (C)->ld)

//@}

/**
 * @defgroup CLAPACK CLAPACK wrappers
 * 
 */
//@{

/** 
 * Cholesky factorization of a (symmetric) matrix
 * 
 * @param A input matrix (symmetric, stored in upper triangle)
 * 
 * @return upper triangular matrix U (in A), for A=U'U
 */

#define SymMatCholFact(A) \
     CLAPACK(potrf)(CblasRowMajor, CblasUpper, (A)->m, (A)->base, (A)->m)


/** 
 * Matrix inverse of A, after cholesky factorization (SymMatCholFact())
 * 
 * @param A upper triangular matrix U (in A), for A=U'U 
 *          (cholesky factorization of A)
 * 
 * @return inv(A)
 */

#define MatCholInv(A) \
     CLAPACK(potri)(CblasRowMajor, CblasUpper, (A)->m, (A)->base, (A)->m)


/** 
 * Matrix inverse of upper triangular matrix A
 * 
 * @param A upper triangular matrix
 * 
 * @return inv(A)
 */

#define MatUpTriInv(A) \
     CLAPACK(trtri)(CblasRowMajor, CblasUpper, CblasNonUnit, \
                                               (A)->m, (A)->base, (A)->m)

/** 
 * C wrapper for fortran code: single precision svd(A)
 * 
 * Used by clapack_sgesvd below.
 *
 * @param jobu whether/how to calculate U
 * @param jobvt whether/how to calculate VT
 * @param m rows of A
 * @param n columns of A
 * @param a matrix (float *)
 * @param lda length of first dimension of A (=columns for CblasRowMajorOrder)
 * @param s eigenvalues from svd(A)
 * @param u left eigenvectors of svd(A)
 * @param ldu length of first dimension of U
 * @param vt right eigenvectors of svd(A)
 * @param ldvt length of first dimension of VT
 * @param work work area 
 * @param lwork size of work area (must be >= MAX(3*MIN(M,N)+MAX(M,N),5*MIN(M,N)))
 * @param info returned information from calculation
 */
extern "C" void sgesvd_(const char *jobu, const char *jobvt, const int *m, 
                        const int *n, const float *a, const int *lda, 
                        const float *s, const float *u, const int *ldu, 
                        const float *vt, const int *ldvt, const float *work,
			const int *lwork, const int *info);


/** 
 * C wrapper for fortran code: double precision svd(A)
 * 
 * Used by clapack_dgesvd below.
 * 
 * @param jobu whether/how to calculate U
 * @param jobvt whether/how to calculate VT
 * @param m rows of A
 * @param n columns of A
 * @param a matrix (double *)
 * @param lda length of first dimension of A (=columns for CblasRowMajorOrder)
 * @param s eigenvalues from svd(A)
 * @param u left eigenvectors of svd(A)
 * @param ldu length of first dimension of U
 * @param vt right eigenvectors of svd(A)
 * @param ldvt length of first dimension of VT
 * @param work work area 
 * @param lwork size of work area (must be >= MAX(3*MIN(M,N)+MAX(M,N),5*MIN(M,N)))
 * @param info returned information from calculation
 */
extern "C" void dgesvd_(const char *jobu, const char *jobvt, const int *m, 
                        const int *n, const double *a, const int *lda, 
                        const double *s, const double *u, const int *ldu, 
                        const double *vt, const int *ldvt, const double *work,
			const int *lwork, const int *info);


/**
 * Enumerated type for the job parameter in svd
 * 
 */

enum CLAPACK_JOB 
{ 
   ClapackAll=201,           ///< All cols/rows returned
   ClapackNone=202,          ///< No calculation is done for this matrix
   ClapackMin=203,           ///< The first min(M,N) rows/cols are returned
   ClapackMinOverwrite=204   ///< The first min(M,N) rows/cols are
                             ///< returned in the input matrix A
};

/** 
 * clapack-like interface to sgesvd
 * 
 * @param Order CblasRowMajor or CblasColMajor
 * @param jobu whether/how to calculate U
 * @param jobvt whether/how to calculate VT
 * @param M rows of A
 * @param N n columns of A
 * @param A matrix (float *)
 * @param lda length of first dimension of A (=columns for CblasRowMajorOrder)
 * @param S eigenvalues from svd(A)
 * @param U left eigenvectors of svd(A)
 * @param ldu length of first dimension of U
 * @param VT right eigenvectors of svd(A)
 * @param ldvt length of first dimension of VT
 * @param work work area 
 * @param lwork size of work area (must be >= MAX(3*MIN(M,N)+MAX(M,N),5*MIN(M,N)))
 * 
 * @return info parameter from sgesvd_
 */

int clapack_sgesvd(const enum CBLAS_ORDER Order, const enum CLAPACK_JOB jobu,
                   const enum CLAPACK_JOB jobvt, const int M, 
                   const int N, const float *A, const int lda, 
                   const float *S, const float *U, const int ldu, 
                   const float *VT, const int ldvt, const float *work,
                   const int lwork);

/** 
 * clapack-like interface to dgesvd
 * 
 * @param Order CblasRowMajor or CblasColMajor
 * @param jobu whether/how to calculate U
 * @param jobvt whether/how to calculate VT
 * @param M rows of A
 * @param N n columns of A
 * @param A matrix (double *)
 * @param lda length of first dimension of A (=columns for CblasRowMajorOrder)
 * @param S eigenvalues from svd(A)
 * @param U left eigenvectors of svd(A)
 * @param ldu length of first dimension of U
 * @param VT right eigenvectors of svd(A)
 * @param ldvt length of first dimension of VT
 * @param work work area 
 * @param lwork size of work area (must be >= MAX(3*MIN(M,N)+MAX(M,N),5*MIN(M,N)))
 * 
 * @return info parameter from dgesvd_
 */

int clapack_dgesvd(const enum CBLAS_ORDER Order, const enum CLAPACK_JOB jobu,
                   const enum CLAPACK_JOB jobvt, const int M, 
                   const int N, const double *A, const int lda, 
                   const double *S, const double *U, const int ldu, 
                   const double *VT, const int ldvt, const double *work,
                   const int lwork);

/** 
 * Singular value decomposition of a matrix
 *
 * Note: THE INPUT MATRIX IS NOT PRESERVED!
 * 
 * @param A input matrix
 * @param D eigenvalues from svd(A)
 * @param jobU whether/how to calculate U
 * @param U left eigenvectors of svd(A)
 * @param jobVT whether/how to calculate VT
 * @param VT right eigenvectors of svd(A) (matrix is transposed!)
 * @param work work area (IVec) (length must be >= MAX(3*MIN(M,N)+MAX(M,N),5*MIN(M,N)))
 * 
 * @return singular value decomposition: U*diag(D)*VT' = A
 */

#define SVD(A,D,jobU,U,jobVT,VT,work) \
     CLAPACK(gesvd)(CblasRowMajor, (jobU), (jobVT), (A)->m, (A)->n, \
                    (A)->base, (A)->n, (D)->ptr, (U)->base, (U)->n, \
                    (VT)->base, (VT)->n, (work)->ptr, (work)->n)

//@}  // clapack wrappers



/**
 * @defgroup MiscMatrix miscellaneous matrix/vector operations
 * 
 */

//@{

/** 
 * Determinant of A, calculated as the product of the diagonal of the
 * cholesky decomposition of A (from SymMatCholFact)
 * 
 * Note that this function just calculates the product of the diagonal
 * of the matrix.  For a matrix factored with cholesky factorization,
 * this gives the determinant of the original matrix, but it could
 * have other uses (and if so, should probably be renamed).
 *
 * @param chol_A input matrix (cholesky factorization of matrix A)
 * 
 * @return determinant of A
 */

real CholDet(IMat *chol_A);

/** 
 * Element by element multiply of two vectors (real *)
 * 
 * @param n length of inputs
 * @param alpha scale factor
 * @param x input vector
 * @param incX stride of x
 * @param y input vector
 * @param incY stride of y
 *
 * @return y = x *. y
 */

void DotTimes(int n, real alpha, real *x, int incX, real *y, int incY);


/** 
 * Element by element multiply of two vectors (IVec)
 *
 * Note: no checking is done on the vector lengths!
 * 
 * @param alpha scale factor
 * @param x input vector
 * @param y input/output vector
 * 
 * @return y = x .* y
 */

#define VecDotTimes(alpha,x,y) \
     DotTimes((x)->n, (alpha), (x)->ptr, (x)->stride, \
              (y)->ptr, (y)->stride)

/** 
 * Element by element multiply of two matrices (IMat)
 *
 * Note: no checking is done on the matrix sizes!
 * 
 * @param alpha scale factor
 * @param x input vector
 * @param y input/output vector
 * 
 * @return B = alpha * A .* B 
 */

#define MatDotTimes(alpha,A,B) \
     DotTimes((B)->m*(B)->n, (alpha), (A)->base, 1, (B)->base, 1)

//@}

#endif // IBLAS_OPS_HH
