/**
 * @file   IMat.cc
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Fri Jul  9 13:26:38 2004
 * 
 * @brief  Matrix class
 * 
 * $Log: IMat.cc,v $
 * Revision 1.2  2006/08/04 19:27:11  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.1  2005/05/09 20:53:20  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.7.2.2  2005/02/09 00:33:30  k-squire
 * Added copy constructor from IMatInt.
 * Added normalize(): normalize row sums 1.
 * Added quantization code.
 * Bug fixes.
 *
 * Revision 1.7.2.1  2004/10/26 10:46:51  k-squire
 * Fixed bug in addCol: when base is reallocated, we need to call fillPtr()
 *    before using ptr.  Also need to update ld.
 * Added appendRows()
 * Changed variable names in mean and cov to be more understandable.
 * Added std(): calcs std. dev. for each column/row in a matrix.
 * Added mean1(): scales mean by (n-1); used by std.
 * Added scatter(): calcs scatter of a matrix.
 * Added standardize(): subtracts mean, scales by std. dev.
 * Changed saveXFile to no longer print "IMat:"
 *
 * Revision 1.7  2004/08/10 23:03:06  k-squire
 * Backported changes from illy/matrix
 *
 * Revision 1.1.2.1  2004/08/08 23:07:00  k-squire
 * Moved REAL_FORMAT_* definitions to gen_defines.h.
 *
 * Revision 1.1  2004/08/03 06:39:07  k-squire
 * Import and initial checkin.
 *
 * Revision 1.6  2004/08/03 04:18:33  k-squire
 * Now uses ld as a stride from the beginning of one row to the next.
 *    (* WARNING: this is not yet tested thoroughly! *)
 * Added getSubMat() (using ld)
 * Fixed loadXFile() so that "IMat:" is not required when loading a matrix.
 * Added appendXFile() to append a file to this matrix.  Matrix sizes should
 *    be the same (although the matrix is resized appropriately if not).
 *
 * Revision 1.5  2004/07/30 03:55:21  k-squire
 * Changed file format slightly for ease of editing.
 *
 * Revision 1.4  2004/07/29 07:07:39  k-squire
 * Added copy constructor.
 * Changed getRow, getCol, getDiag order.
 * Added setRow, setCol, setDiag.
 * Added sum, mean, cov, mmin, mmax, abs.
 * Added saveXFile, loadXFile.
 *
 * Revision 1.3  2004/07/26 16:59:10  k-squire
 * Added Set*, prod functions.
 *
 * Revision 1.2  2004/07/23 20:49:08  k-squire
 * Minor formatting changes.
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 * 
 */
static char *IMAT_CC_Id = "$Id: IMat.cc,v 1.2 2006/08/04 19:27:11 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <torch/general.h>
#include <torch/Object.h>
#include <torch/Random.h>
#include <torch/Allocator.h>

#include <cblas.h>

#include "IMat.hh"
#include "IMatVecOps.hh"
#include "gen_defines.h"

////////////////////
// Constructors/Destructor
//////////////////////

/** 
 * Empty Constructor
 * 
 */
IMat::IMat():
      m(0),
      n(0),
      ld(0),
      real_size(0),
      real_ptr_len(0),
      base(NULL),
      ptr(NULL),
      is_alias(false)
{
}

/** 
 * Constructor
 * 
 * @param m_ rows
 * @param n_ cols
 */

IMat::IMat(int m_, int n_):
      m(0),
      n(0),
      ld(0),
      real_size(0),
      real_ptr_len(0),
      base(NULL),
      ptr(NULL),
      is_alias(false)
{
   init(m_, n_);
}


/** 
 * Constructor
 * 
 * @param base_ data/memory to use/copy
 * @param m_ rows
 * @param n_ cols
 * @param do_copy true if data should be copied from base_
 * @param retain if true and do_copy is false, retain and manage the
 *               memory pointed to by base_
 */
IMat::IMat(real *base_, int m_, int n_, int ld_, bool do_copy, bool retain):
//       m(m_),
//       n(n_),
//       ld(n_),
      m(0),
      n(0),
      ld(0),
      real_size(0),
      real_ptr_len(0),
      base(NULL),
      ptr(NULL),
      is_alias(false)
{
   
   set(base_, m_, n_, ld_, do_copy, retain);

//    if (do_copy)
//    {
//       if (init(m_,n_) < 0)
//          return;

//       if (ld_ <= n_)
//          memcpy(base, base_, m*n*sizeof(real));
//       else
//       {
//          // need to use ld_ as a stride in base_

//          real *p = base_;
         
//          for (int i = 0; i < m; i++, p+=ld_)
//             memcpy(ptr[i], p, n*sizeof(real));
//       }
//    }
//    else
//    {
//       base = base_;
//       real_size = m*n;
//       if (ld_ > n_)
//          ld = ld_;

//       fillPtr();
   
//       if (retain)
//          allocator->retain(base);
//    }
   
}

/** 
 * Copy constructor
 * 
 * @param A IMat matrix used to initialize this matrix.
 */

IMat::IMat(IMat &A):
      m(0),
      n(0),
      ld(0),
      real_size(0),
      real_ptr_len(0),
      base(NULL),
      ptr(NULL),
      is_alias(false)
{
   init(A.m, A.n);

   for (int i = 0; i < m; i++)
      memcpy(ptr[i],A.ptr[i],n*sizeof(real));
}

/** 
 * Destructor
 * 
 */

IMat::~IMat()
{
}

////////////////////
// Matrix Initialization
//////////////////////

/** 
 * Initializer.
 * 
 * @param m_ rows
 * @param n_ cols
 * 
 * @return 0 on success, -1 on error
 */

int IMat::init(int m_, int n_)
{
   m = m_;
   n = n_;
   ld = n_;

   is_alias = false;

   base = (real *)allocator->alloc(sizeof(real) * m*n);
   if (base)
      real_size = m*n;
   else
      return -1;
   
   fillPtr();

   return 0;
}


/** 
 * Fill ptr with pointers into base, for easy access.
 * 
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::fillPtr()
{
   real *b;
   real **p;
   
   if (m > real_ptr_len)
   {
      real **new_ptr = (real **)allocator->realloc(ptr, sizeof(real *)*m);
      if (new_ptr)
      {
         ptr = new_ptr;
         real_ptr_len = m;
      }
      else
      {
         error("BAD ERROR: could not allocate memory for matrix row pointers!!!\n");
         return -1;
      }
   }
   
   b = base;
   p = ptr;

   for (int i = 0; i < m; i++, b+=ld)
      *p++ = b;

   return 0;
}

/** 
 * Copy/alias data from base_ to this matrix, resizing as necessary.
 * 
 * @param base_ data source
 * @param m_ rows of data
 * @param n_ cols of data
 * @param ld_ length from start of one row to the next (for submatrices)
 * @param do_copy if true, copy the matrix
 * @param retain retain the pointer to *base_
 *
 * @return 0 on success, -1 on error
 */

int IMat::set(real *base_, int m_, int n_, int ld_, bool do_copy, bool retain)
{
   if (do_copy)
   {
      if (reshape(m_,n_) != 0)
         return -1;
      
      if (ld_ <= n_)
         memcpy(base, base_, m*n*sizeof(real));
      else
      {
         // need to use ld_ as a stride in base_

         real *p = base_;
         
         for (int i = 0; i < m; i++, p+=ld_)
            memcpy(ptr[i], p, n*sizeof(real));
      }
   }
   else
   {
      if (!is_alias)
         allocator->free(base); // it's okay if base is NULL

      m = m_;
      n = n_;
      if (ld_ >= n_)
         ld = ld_;
      else
         ld = n_;
   
      real_size = m*n;
      base = base_;
      is_alias = true;
      
      fillPtr();

      if (retain)
         allocator->retain(base);
   }
   

   return 0;
}

////////////////////
// Operators
//////////////////////


/** 
 * Reference operator
 * 
 * 
 * @return reference to location (r,c)
 */

real &IMat::operator()(int r, int c)
{
   return ptr[r][c];
}

/** 
 * Copy data from matrix A to this matrix
 * 
 * @param A matrix
 * 
 * @return reference to this
 */

IMat &IMat::operator=(const IMat &A)
{
   if (&A == this)
      return *this;

   if (m != A.m || n != A.n)
      reshape(A.m, A.n);
   
   MatCopy(&A, this);

   return *this;
}

/** 
 * Copy data from matrix A to this matrix
 * 
 * @param A matrix
 * 
 * @return reference to this
 */

IMat &IMat::operator=(IMatInt &A)
{
   int i,j;
   
   if (m != A.m || n != A.n)
      reshape(A.m, A.n);
   
   for (i = 0; i < m; i++)
      for (j = 0; j < n; j++)
         ptr[i][j] = (real)(A(i,j));

   return *this;
}

////////////////////
// Matrix Fill Ops
//////////////////////


/** 
 * Fill the matrix with a particular value.
 * 
 * @param val value to fill the matrix with.
 */

void IMat::fill(real val)
{
   int i,j;

   for (i = 0; i < m; i++)
      for (j = 0; j < n; j++)
         ptr[i][j] = val;
}

/** 
 * zero the matrix
 * 
 */

void IMat::zero()
{
   if (base == NULL)
      return;

   if (ld == n)
      memset(base, 0, m*n*sizeof(real));
   else
      for (int i = 0; i < m; i++)
         memset(ptr[i],0, n*sizeof(real));
}

/** 
 * Fill the matrix with ones
 * 
 */

void IMat::ones()
{
   fill(1.0);
}

/** 
 * Fill the matrix with uniform real random values on [a,b]
 * 
 * @param a minimum random value
 * @param b maximum random value
 */

void IMat::rand(real a, real b)
{
   int i,j;
   real *p = base;

   for (i = 0; i < m; i++)
      for (j = 0; j < n; j++)
         ptr[i][j] = Torch::Random::boundedUniform(a,b);
}	

/** 
 * Fill ones on the major diagonal, zeros on the off diagonals.
 * 
 */

void IMat::eye()
{
   zero();

   IVec d;
   
   getDiag(&d);
   d.ones();
}

void IMat::normalize()
{
   int i;
   IVec col;
   
   for (i = 0; i < n; i++)
   {
      getCol(i, &col);
      col.normalize();
   }
}

/** 
 * Fills the columns of the matrix with vec.
 *
 * Note: no checking is done on the length of vec. If it is too long,
 * only the first m elements are copied.  If it is too short, random
 * values after the end of the array appear in the matrix, and the
 * program may core dump.
 * 
 * @param vec vector to fill columns of matrix with
 */

void IMat::vecFillCols(IVec *vec)
{
   IVec row;
   
   for (int i = 0; i < m; i++)
   {
      getRow(i,&row);
      row.fill((*vec)(i));
   }
}

/** 
 * Fills the rows of the matrix with vec.
 *
 * See note in vecFillCols above
 * 
 * @param vec vector to fill rows of matrix with
 */

void IMat::vecFillRows(IVec *vec)
{
   IVec row;
   
   for (int i = 0; i < m; i++)
      setRow(i,vec);
}

////////////////////
// Matrix shape manipulation
//////////////////////

/** 
 * Transpose the matrix
 *
 * Note: this is a rather expensive operation; use sparingly.
 * Most of the BLAS and many of the LAPACK function calls in MathOps
 * will allow you to use the transpose of the matrix in the call, so
 * you won't have to transpose the matrix here first.
 * 
 */

void IMat::transpose()
{
   real tmp;
      
   int i,j;

   int m_new = n;
   int n_new = m;
   int ms = max(m,n);

   resize(ms,ms);

   for (i = 0; i < ms; i++)
      for (j = i+1; j < ms; j++)
      {
         tmp = ptr[i][j];
         ptr[i][j] = ptr[j][i];
         ptr[j][i] = tmp;
      }
      
   resize(m_new,n_new);
}

/** 
 * Reshape the current matrix, resizing as necessary.  Note that no
 * data is moved around when resizing.  You might lose data if you're
 * not careful...
 * 
 * @param m_ rows
 * @param n_ cols
 * @param clear if true, any added memory is cleared
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::reshape(int m_, int n_, bool clear)
{
   real *new_base;

   if (m == m_ && n == n_)
      return 0;

   // Does base need to be bigger?
      
   if (real_size < m_*n_)
   {
      if (!is_alias)
      {
         new_base = (real *)allocator->realloc(base, sizeof(real) * m_*n_);
   
         if (new_base != NULL)
         {
            base = new_base;
            real_size = m_*n_;
         }
         else
            return -1;
      }
      else
         return -1;  // can't resize an alias that's too small!

      if (clear)
         memset(base+m*n, 0, (m_*n_-m*n)*sizeof(real));
   }

   // if (m*n <= m_*n_), we're good with memory allocations 

   m = m_;
   n = n_;
   ld = n_;

   fillPtr();

   return 0;
}

/** 
 * Resize the matrix by adding or removing rows/cols.  Data is
 * preserved.
 * 
 * @param m_ rows
 * @param n_ cols
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::resize(int m_, int n_)
{
   if (m == 0 || n == 0)
      return(reshape(m_,n_));

   // Do resize in a smart way

   if (m_ < m)
      delRow(m_, m-m_);
   
   if (n_ < n)
      delCol(n_,n-n_);
   else if (n_ > n)
      addCol(n_-n);
   
   if (m_ > m)
      addRow(m_-m);

   return 0;
}

/** 
 * Add rows to the bottom of the matrix
 * 
 * @param rows number of rows to add
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::addRow(unsigned int rows)
{
   if ((m+rows)*n > real_size)
   {
      if (is_alias)
      {
         error("Tried adding a row to an aliased matrix which is too small!\n");
         return -1;
      }

      real *new_base;

      new_base = (real *)allocator->realloc(base, (m+rows)*n*sizeof(real));
      if (new_base != NULL)
      {
         base = new_base;
         real_size = (m+rows)*n;
      }
      else
         return -1;
   }

   memset(base+m*n, 0, (n*rows)*sizeof(real));
   m+=rows;

   fillPtr();

   return 0;
}

/** 
 * Add columns to the right-hand side of the matrix
 * 
 * @param cols number of cols to add
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::addCol(unsigned int cols)
{
   if ((m*(n+cols)) > real_size)
   {
      if (is_alias)
      {
         error("Tried adding a column to an aliased matrix which is too small!\n");
         return -1;
      }

      real *new_base;

      new_base = (real *)allocator->realloc(base, m*(n+cols)*sizeof(real));
      if (new_base != NULL)
      {
         base = new_base;
         real_size = m*(n+cols);
         fillPtr();
      }
      else
         return -1;
   }

   for (int i = m-1; i >= 1; i--)
   {
      memmove(base + i*(n+cols), ptr[i], n*sizeof(real));
      ptr[i] = base + i*(n+cols);
      memset(ptr[i]+n, 0, cols*sizeof(real));
   }
   if (base != NULL)
      memset(base+n, 0, cols*sizeof(real));

   n+=cols;
   ld+=cols;
      
   return 0;
}

/** 
 * Add rows and cols to the matrix
 * 
 * @param rows number of rows to add
 * @param cols number of cols to add
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::addRowCol(unsigned int rows, unsigned int cols)
{
   if (addCol(cols) < 0)
      return -1;
   if (addRow(rows) < 0)
      return -1;

   return 0;
}

/** 
 * Delete rows from the matrix 
 * 
 * @param row row number to begin deleting at
 * @param count number of rows to delete
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::delRow(unsigned int row, unsigned int count)
{
   if (row > m-count)
      return -1;

   if (row == m-count)
   {
      m-=count;
      return 0;
   }
   
   if (is_alias)
   {
      warning("Deleting rows from aliases not yet supported.\n");
      return -1;
   }

   real *target = ptr[row];
   real *source = ptr[row+count];
   int len = m-(row+count);
   
   memmove(target, source, n*len*sizeof(real));
   m-= count;

   return 0;
}


/** 
 * Delete columns from the matrix
 * 
 * @param col col number to start deleting at
 * @param count number of columns to delete
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::delCol(unsigned int col, unsigned int count)
{
   int i, len;
   
   if (col > n-count)
      return -1;

   if (is_alias)
   {
      warning("Deleting columns from aliases not yet supported.\n");
      return -1;
   }

   real *new_loc = base + col;
   real *old_loc = base + (col+count);
   len = n-count;
   
   for (i = 0; i < m-1; i++, new_loc+=len, old_loc+=n)
      memmove(new_loc, old_loc, len*sizeof(real));

   if (col+count < n)
      memmove(new_loc, old_loc, (n-(col+count))*sizeof(real));

   n -= count;
   ld -= count;

   fillPtr();

   return 0;
}

/** 
 * Delete 1 row and 1 col from the matrix
 * 
 * @param row row to delete
 * @param col col to delete
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::delRowCol(unsigned int row, unsigned int col)
{
   if (delRow(row) < 0)
      return -1;
   if (delCol(col) < 0)
      return -1;

   return 0;
}

/** 
 * Append rows from matrix A to this matrix
 * 
 * @param A Matrix from which to copy rows
 * @param start_row first row to copy
 * @param stop_row last row to copy
 * 
 * @return 0 on success, -1 on failure
 */
int IMat::appendRows(IMat *A,
                     int start_row,
                     int stop_row)
{
   if (stop_row == -1)          // copy to end of matrix if stop_row
      stop_row = A->m-1;        // is -1

   if (start_row > stop_row)    // idiot checking
      return -1;

   int old_rows = m;
   int copy_rows = (stop_row-start_row+1);
   int copy_len = copy_rows*A->n*sizeof(real);

   if (m == 0 || n == 0)
      init(copy_rows, A->n);
   else
   {
      if (A->n != n)               // row lengths should be the same
         return -1;

      addRow(copy_rows);
   }
   
   memcpy(ptr[old_rows], A->ptr[start_row], copy_len);

   return 0;
}

////////////////////
// Copy/Alias Matrix Parts
//////////////////////

/** 
 * Get an alias for a row of the matrix
 * 
 * @param row row to alias
 * @param vec vector to use as an alias for row
 * @param do_copy if true, the row is copied; if not, it's
 *                aliased 
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::getRow(unsigned int row,
                 IVec *vec, 
                 bool do_copy)
{
   if (!vec || row >= m)
      return -1;

   return (vec->set(ptr[row], n, 1, do_copy));
}

/** 
 * Get an alias for a row of the matrix
 * 
 * @param row row to alias
 * @param col column to start at for alias
 * @param len length of vector
 * @param vec vector to use as an alias for row
 * @param do_copy if true, the row is copied; if not, it's
 *                aliased 
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::getRow(unsigned int row,
                 unsigned int col,
                 int len,
                 IVec *vec, 
                 bool do_copy)
{
   if (!vec || row >= m || col >= n)
      return -1;

   if (len < 0)
      len = n - col;
   else if (len > n - col)
      return -1;

   return (vec->set(&ptr[row][col], len, 1, do_copy));
}

/** 
 * Get an alias for a row of the matrix
 * 
 * @param vec vector to use as an alias for row
 * @param row row to alias
 * @param col column to start at for alias
 * @param len length of vector
 * @param do_copy if true, the row is copied; if not, it's
 *                aliased 
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::getRow(unsigned int row,
                 unsigned int col,
                 int m_,
                 int n_,
                 IMat *A,
                 bool do_copy)
{
   if (!A || row >= m || col >= n)
      return -1;

   if (m_*n_ > n-col)
      return -1;
   
   return (A->set(&ptr[row][col], m_, n_, do_copy));
}


/** 
 * Get an alias for a column of the matrix
 * 
 * @param col column to alias
 * @param vec vector to use as an alias for col
 * @param do_copy if true, the column is copied; if not, it's
 *                aliased 
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::getCol(unsigned int col,
                 IVec *vec, 
                 bool do_copy)
{
   if (!vec || col >= n)
      return -1;

   return (vec->set(&ptr[0][col], m, n, do_copy));
}

/** 
 * Get an alias for a column of the matrix
 * 
 * @param col column to alias
 * @param row row start at for alias
 * @param len length of vector
 * @param vec vector to use as an alias for col
 * @param do_copy if true, the column is copied; if not, it's
 *                aliased 
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::getCol(unsigned int col,
                 unsigned int row,
                 int len,
                 IVec *vec, 
                 bool do_copy)
{
   if (!vec || col >= n || row >= m)
      return -1;

   if (len < 0)
      len = m - row;
   else if (len > m - row)
      return -1;

   return (vec->set(&ptr[row][col], len, n, do_copy));
}

/** 
 * Get an alias for the main diagonal of the matrix
 * 
 * @param vec vector to use as an alias for col
 * @param do_copy if true, the diagonal is copied; if not, it's
 *                aliased 
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::getDiag(IVec *vec, 
                  bool do_copy)
{
   return (getDiag(0,vec,do_copy));
}

/** 
 * Get an alias for a diagonal of the matrix
 * 
 * @param vec vector to use as an alias for col
 * @param offset offset from main diagonal; negative offsets indicate
 *               lower diagonals, positive offsets indicate upper
 *               diagonals.
 * @param do_copy if true, the diagonal is copied; if not, it's
 *                aliased 
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::getDiag(int offset, 
                  IVec *vec, 
                  bool do_copy)
{
   unsigned int row = 0, col = 0;
   
   if (!vec)
      return -1;

   if (offset < 0)
   {
      row = -offset;
      if (row >= m)
         return -1;
   }
   else
   {
      col = offset;
      if (col >= n)
         return -1;
   }

   return (vec->set(&ptr[row][col], 
                    min(m - row, n - col),
                    n+1, do_copy));
}

/** 
 * Get an alias for all of the matrix.  Does not work properly if this
 * Matrix is a submatrix of another matrix.
 * 
 * @param vec vector to use as an alias for row
 * @param do_copy if true, the row is copied; if not, it's
 *                aliased 
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::getAll(IVec *vec, 
                 bool do_copy)
{
   if (!vec)
      return -1;

   return (vec->set(base, m*n, 1, do_copy));
}

/** 
 * Copy or get an alias for a sub-matrix
 * 
 * @param A destination matrix
 * @param row1 first row to start submatrix
 * @param col1 first column to start submatrix
 * @param row2 last row of submatrix
 * @param col2 last column of submatrix
 * @param do_copy copy the matrix
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::getSubMat(unsigned int row1,
                    unsigned int col1,
                    unsigned int row2,
                    unsigned int col2,
                    IMat *A,
                    bool do_copy)
{
   if (!A)
      return -1;

   if (row1 >= m || row2 >= m || col1 >= n || col2 >= n)
      return -1;

   return (A->set(&ptr[row1][col1], row2-row1+1, col2-col1+1, n, do_copy));
}



   ////////////////////
   // Set Matrix Rows/Cols/Diags
   //////////////////////

// int IMat::setRow(IVec *vec,
//                  unsigned int row,
//                  unsigned int col = 0)
// {
//    IVec matRow;
   
//    getRow(&matRow,row,col,vec->n);
//    matRow = *vec;
// }

// int IMat::setCol(IVec *vec,
//                  unsigned int col,
//                  unsigned int row = 0)
// {
//    IVec matCol;
   
//    getCol(&matCol,col,row,vec->n);
//    matCol = *vec;
// }

int IMat::setRow(unsigned int row,
                 IVec *vec)
{
   IVec matRow;
   
   getRow(row, &matRow);
   matRow = *vec;
}

int IMat::setCol(unsigned int col,
                 IVec *vec)
{
   IVec matCol;
   
   getCol(col, &matCol);
   matCol = *vec;
}


int IMat::setDiag(IVec *vec)
{
   return (setDiag(0,vec));
}

int IMat::setDiag(int offset,
                  IVec *vec)
{
   IVec matDiag;

   getDiag(&matDiag,offset);
   matDiag = *vec;
}


////////////////////
// Misc
//////////////////////

/** 
 * Print out the matrix
 * 
 * @param scrn_width screen width
 * @param col_width width to use for matrix columns
 * @param num_format format to print with
 * 
 * @return 0 on success, -1 on failure
 */

int IMat::print(int scrn_width, int col_width, char *num_format)
{
   int cols = (scrn_width-6)/col_width;

   static char *format = new char[10];
   static char *formatd = new char[10];

   char *output = new char[col_width+1];
   
   snprintf(format, 10, "%%%ds", col_width);
   snprintf(formatd, 10, "%%%dd", col_width);

   for (int i = 0; i < n; i+=cols)
   {
      printf ("       Columns %d through %d\n\n", i, min(i+cols, n)-1);

      printf ("      ");
      for (int k = i; k < min(i+cols, n); k++)
         printf (formatd, k);

      printf("\n");

      for (int j = 0; j < m; j++)
      {
         printf ("%5d ", j);

         for (int k = i; k < min(i+cols, n); k++)
         {
            snprintf(output, col_width+1, num_format, ptr[j][k]);
            printf(format, output);
         }
         printf("\n");
      }
      printf("\n");
   }

   printf("\n");

   delete[] output;

   return 0;
}

int IMat::prod(IVec *out, int dim)
{
   int i;
   
   IVec tmp;

   if (dim == 1)
   {
      out->resize(n);
      out->ones();
      
      for (i = 0; i < m; i++)
      {
         getRow(i, &tmp);
         VecDotTimes(1.0,&tmp,out);
      }
   }
   else
   {
      out->resize(m);
      out->ones();
      
      for (i = 0; i < n; i++)
      {
         getCol(i, &tmp);
         VecDotTimes(1.0,&tmp,out);
      }
   }

   return 0;
}

int IMat::sum(IVec *out, int dim, IVecInt *ind)
{
   int i;
   
   static IVec tmp;

   if (dim == 1)
   {
      out->resize(n);
      out->zero();

      if (ind)
      {
         for (i = 0; i < ind->n; i++)
         {
            getRow((*ind)[i], &tmp);
            VecAdd(&tmp,out);
         }
      }
      else
      {
         for (i = 0; i < m; i++)
         {
            getRow(i, &tmp);
            VecAdd(&tmp,out);
         }
      }
   }
   else
   {
      out->resize(m);
      out->zero();
      
      if (ind)
      {
         for (i = 0; i < ind->n; i++)
         {
            getCol((*ind)[i], &tmp);
            VecAdd(&tmp,out);
         }
      }
      else
      {
         for (i = 0; i < n; i++)
         {
            getCol(i, &tmp);
            VecAdd(&tmp,out);
         }
      }
   }

   return 0;
}

int IMat::mean(IVec *mu, int dim, IVecInt *ind)
{
   int i;
   real size;
   
   sum(mu, dim, ind);

   if (ind)
      size = (real)ind->n;
   else if (dim==1)
      size = (real)m;
   else
      size = (real)n;

   VecScale(1.0/size, mu);

   return 0;
}

int IMat::mean1(IVec *mu, int dim, IVecInt *ind)
{
   int i;
   real size;
   
   sum(mu, dim, ind);

   if (ind)
      size = (real)ind->n;
   else if (dim==1)
      size = (real)m;
   else
      size = (real)n;

   size = max(size,2);

   VecScale(1.0/(size-1), mu);

   return 0;
}

int IMat::std(IVec *mu, IVec *sigma, int dim, IVecInt *ind)
{
   real scale;
   IVec vec;
   IMat tmp;

   sigma->resize(n);
   
   if (dim == 1)
   {
      int i;
      
      if (ind)
      {
         tmp.reshape(ind->n,n);
         for (i = 0; i < ind->n; i++)
         {
            tmp.getRow(i, &vec);        // alias row i of tmp
            getRow((*ind)[i], &vec, true); // copy row ind[i] of this
                                        // matrix into row i of tmp
            VecSub(mu, &vec);
         }
         
      }
      else
      {
         tmp.reshape(m,n);
         MatCopy(this, &tmp);
         for (i = 0; i < m; i++)
         {
            tmp.getRow(i, &vec);
            VecSub(mu, &vec);
         }
      }
      
      MatDotTimes(1.0, &tmp, &tmp);
      tmp.mean1(sigma, dim);
      sigma->sqrt();

   }
   else  // variance of columns
   {
      int j;

      if (ind)
      {
         tmp.reshape(n,ind->n);
         for (j = 0; j < ind->n; j++)
         {
            tmp.getCol(j, &vec);        // alias row i of tmp
            getCol((*ind)[j], &vec, true); // copy col ind[i] of this
                                        // matrix into row i of tmp
            VecSub(mu, &vec);
         }
         
      }
      else
      {
         tmp.reshape(m,n);
         MatCopy(this, &tmp);
         for (j = 0; j < m; j++)
         {
            tmp.getCol(j, &vec);
            VecSub(mu, &vec);
         }
      }

      MatDotTimes(1.0, &tmp, &tmp);
      tmp.mean1(sigma, dim);
      sigma->sqrt();
   }
   
   return 0;
}


int IMat::cov(IVec *mu, IMat *Sigma, int dim, IVecInt *ind)
{
   real scale;
   IVec vec;
   IMat tmp;

   Sigma->reshape(mu->n, mu->n);
   Sigma->zero();
   Sigma->eye();
   
   if (dim == 1)
   {
      int i;
      
      if (ind)
      {
         scale = 1.0/(ind->n - 1.0);
         tmp.reshape(ind->n,n);
         for (i = 0; i < ind->n; i++)
         {
            tmp.getRow(i, &vec);        // alias row i of tmp
            getRow((*ind)[i], &vec, true); // copy row ind[i] of this
                                        // matrix into row i of tmp
            VecSub(mu, &vec);
         }
         
      }
      else
      {
         scale = 1.0/(m - 1.0);
         tmp.reshape(m,n);
         MatCopy(this, &tmp);
         for (i = 0; i < m; i++)
         {
            tmp.getRow(i, &vec);
            VecSub(mu, &vec);
         }
      }
      
      //SymMatRankKUpdate(scale,
       //                 &tmp, CblasTrans,
      //                  0.0, Sigma);
      SymMatRankKUpdate(scale,
                        &tmp, CblasTrans,
                        0.00001, Sigma);

      //COMPLETE HACK FIX THIS

   }
   else  // variance of columns
   {
      int j;

      if (ind)
      {
         scale = 1.0/(ind->n - 1.0);
         tmp.reshape(n,ind->n);
         for (j = 0; j < ind->n; j++)
         {
            tmp.getCol(j, &vec);        // alias row i of tmp
            getCol((*ind)[j], &vec, true); // copy col ind[i] of this
                                        // matrix into row i of tmp
            VecSub(mu, &vec);
         }
         
      }
      else
      {
         scale = 1.0/(n - 1.0);
         tmp.reshape(m,n);
         MatCopy(this, &tmp);
         for (j = 0; j < m; j++)
         {
            tmp.getCol(j, &vec);
            VecSub(mu, &vec);
         }
      }
      SymMatRankKUpdate(scale,
                        &tmp, CblasNoTrans,
                        0.0, Sigma);

   }
   
   return 0;
}

int IMat::scatter(IVec *mu, IMat *scat, int dim, IVecInt *ind)
{
   real scale;
   IVec vec;
   IMat tmp;

   scat->reshape(mu->n, mu->n);
   scat->zero();
   
   if (dim == 1)
   {
      int i;
      
      if (ind)
      {
         scale = 1.0/(ind->n - 1.0);
         tmp.reshape(ind->n,n);
         for (i = 0; i < ind->n; i++)
         {
            tmp.getRow(i, &vec);        // alias row i of tmp
            getRow((*ind)[i], &vec, true); // copy row ind[i] of this
                                        // matrix into row i of tmp
            VecSub(mu, &vec);
         }
         
      }
      else
      {
         scale = 1.0/(m - 1.0);
         tmp.reshape(m,n);
         MatCopy(this, &tmp);
         for (i = 0; i < m; i++)
         {
            tmp.getRow(i, &vec);
            VecSub(mu, &vec);
         }
      }
      
      for (i = 0; i < tmp.m; i++)
      {
         tmp.getRow(i, &vec);
         SymMatRankUpdate(1.0, &vec, scat);
      }
   }
   else  // scatter of columns
   {
      int j;

      if (ind)
      {
         scale = 1.0/(ind->n - 1.0);
         tmp.reshape(n,ind->n);
         for (j = 0; j < ind->n; j++)
         {
            tmp.getCol(j, &vec);        // alias row i of tmp
            getCol((*ind)[j], &vec, true); // copy col ind[i] of this
                                        // matrix into row i of tmp
            VecSub(mu, &vec);
         }
         
      }
      else
      {
         scale = 1.0/(n - 1.0);
         tmp.reshape(m,n);
         MatCopy(this, &tmp);
         for (j = 0; j < m; j++)
         {
            tmp.getCol(j, &vec);
            VecSub(mu, &vec);
         }
      }

      for (j = 0; j < tmp.m; j++)
      {
         tmp.getCol(j, &vec);
         SymMatRankUpdate(1.0, &vec, scat);
      }
   }
   
   return 0;
}


real IMat::mmin(int *pos)
{
   IVec x;
   
   getAll(&x);
   
   return x.vmin(pos);
}

int IMat::mmin(IVec *out, IVecInt *pos, int dim)
{
   real *p1 = NULL;
   int *p2 = NULL;
   real mm;
   
   if (dim == 1)
   {
      int i;
      IVec row;

      if (out)
      {
         out->resize(m);
         p1 = out->ptr;
      }
      if (pos)
      {
         pos->resize(m);
         p2 = pos->ptr;
      }
      
      for (i = 0; i < m; i++)
      {
         getRow(i, &row);
         mm = row.vmin(p2);

         if (out)
            *p1++ = mm;

         if (pos)
            p2++;
      }
   }
   else
   {
      int j;
      IVec col;
      
      if (out)
      {
         out->resize(n);
         p1 = out->ptr;
      }
      if (pos)
      {
         pos->resize(n);
         p2 = pos->ptr;
      }
      
      for (j = 0; j < m; j++)
      {
         getCol(j, &col);
         mm = col.vmin(p2);

         if (out)
            *p1++ = mm;

         if (pos)
            p2++;
      }
   }

   return 0;
}

real IMat::mmax(int *pos)
{
   IVec x;
   
   getAll(&x);
   
   return x.vmax(pos);
}

int IMat::mmax(IVec *out, IVecInt *pos, int dim)
{
   real *p1 = NULL;
   int *p2 = NULL;
   real mm;
   
   if (dim == 2)
   {
      int i;
      IVec row;

      if (out)
      {
         out->resize(m);
         p1 = out->ptr;
      }
      if (pos)
      {
         pos->resize(m);
         p2 = pos->ptr;
      }
      
      for (i = 0; i < m; i++)
      {
         getRow(i, &row);
         mm = row.vmax(p2);

         if (out)
            *p1++ = mm;

         if (pos)
            p2++;
      }
   }
   else
   {
      int j;
      IVec col;
      
      if (out)
      {
         out->resize(n);
         p1 = out->ptr;
      }
      if (pos)
      {
         pos->resize(n);
         p2 = pos->ptr;
      }
      
      for (j = 0; j < n; j++)
      {
         getCol(j, &col);
         mm = col.vmax(p2);

         if (out)
            *p1++ = mm;

         if (pos)
            p2++;
      }
   }

   return 0;
}

#ifndef USE_DOUBLE
#define fabs fabsf
#endif

void IMat::abs()
{
   int i;
   real *b = base;
   
   for (i = 0; i < m*n; i++, b++)
      *b = fabs(*b);
}

void IMat::standardize(IVec *mu,
                       IVec *sigma,
                       int dim)
{
   int i,j;
   IVec vec;

   if (mu == NULL)
      mean(mu, dim);

   if (sigma == NULL)
      std(mu,sigma,dim);

   if (dim == 1)
   {
      for (i = 0; i < m; i++)
      {
         getRow(i,&vec);
         VecSub(mu,&vec);
         for (j = 0; j < n; j++)
            vec[j] = vec[j]/(*sigma)[j];
      }
   }
   else // dim == 2
   {
      for (j = 0; j < n; j++)
      {
         getCol(j,&vec);
         VecSub(mu,&vec);
         for (i = 0; i < m; i++)
            vec[i] = vec[i]/(*sigma)[i];
      }
   }
   
         
}

void IMat::getQuantLevels(IMat *q_levels, int bins)
{
   int i,j;
   IVec col;
   IVecInt ind;
   IVec q_row;
   
//   IVec sv;

   q_levels->resize(n, bins-1);

   for (i = 0; i < n; i++)
   {
      char ch;

      getCol(i, &col);

      col.sort(&ind);
//      col.getInd(&sv, &ind);

      q_levels->getRow(i, &q_row);

      for (j = 1; j < bins; j++)
	q_row[j-1] = col[ind[(int)(j*m/(real)bins)]];
   }

   return;
}

int IMat::quantize(IMatInt *out, IMat *q_levels)
{
   int i;
   IVec row;
   IVecInt out_row;
   
   out->resize(m,n);

   for (i = 0; i < m; i++)
   {
      getRow(i, &row);
      out->getRow(&out_row, i);

      row.quantize(&out_row, q_levels);
   }

   return 0;
}

void IMat::saveXFile(XFile *file)
{
   file->printf("%d %d\n", m, n);
   for (int i = 0; i < m; i++)
   {
      for (int j = 0; j < n; j++)
         file->printf("  " REAL_FORMAT_OUT, ptr[i][j]);
      file->printf("\n");
   }

   return;
}

void IMat::loadXFile(XFile *file)
{
   int m_, n_;
   char inp[81];
   
   if ((file->gets(inp,80)) == NULL)
      error ("Error reading from file!");
   
   if (sscanf(inp, "IMat: %d %d", &m_, &n_) != 2)
      if (sscanf(inp, "%d %d", &m_, &n_) != 2)
         error("Error: input string \"%s\" does not have matrix size!\n",inp);

   if (reshape(m_,n_) < 0)
      error("Tried reading data into an array that is not big enough and cannot be resized.\n");
  
   for (int i = 0; i < m; i++)
      for (int j = 0; j < n; j++)
      {
         if (file->scanf(REAL_FORMAT_IN, &ptr[i][j]) != 1)
            error("Error reading matrix from file!\n");
         file->scanf("\n",NULL);
      }

   return;

}


void IMat::appendXFile(XFile *file)
{
   int m_, n_;
   int old_m;
   char inp[80];
   
   if ((file->gets(inp,80)) == NULL)
      error ("Error reading from file!");
   
   if (sscanf(inp, "IMat: %d %d", &m_, &n_) != 2)
      if (sscanf(inp, "%d %d", &m_, &n_) != 2)
         error("Error: input string \"%s\" does not have matrix size!\n", inp);

//   if (n_ != n)
//      warning("Input matrix has different number of columns (%d) than\n"
//              " the matrix reading it (%d).\n", n_, n);

   old_m = m;

   if (resize(m+m_,max(n_,n)) < 0)
      error("Tried reading data into an array that is not big enough and cannot be resized.\n");

   for (int i = old_m; i < m; i++)
      for (int j = 0; j < n_; j++)
      {
         if (file->scanf(REAL_FORMAT_IN, &ptr[i][j]) != 1)
            error("Error reading matrix from file!\n");
         file->scanf("\n",NULL);
      }

   return;

}


