/**
 * @file   IMatInt.cc
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Fri Jul  9 13:26:38 2004
 * 
 * @brief  Matrix class
 * 
 * $Log: IMatInt.cc,v $
 * Revision 1.1  2005/05/09 20:53:19  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.3.2.1  2005/02/09 00:38:12  k-squire
 * Now initialize to zero size until after everything is set up.
 * Now uses ld to specify length from start of one row to next
 *    (for submatrices).
 * Copy constructor from IMat.
 * Check more often for NULL base.
 * Ones fills with 1 instead of 1.0
 * Handle resize of 0x0 matrix.
 * Make a histogram.
 * Bug fixes.
 *
 * Revision 1.3  2004/07/30 03:55:32  k-squire
 * Changed file format slightly for ease of editing.
 *
 * Revision 1.2  2004/07/29 07:08:10  k-squire
 * Added copy constructor.
 * Added saveXFile, loadXFile.
 *
 * Revision 1.1  2004/07/23 20:49:49  k-squire
 * Added IMatInt, IVecInt files, classes, and tests.
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 * 
 */
static char *IMATINT_CC_Id = "$Id: IMatInt.cc,v 1.1 2005/05/09 20:53:19 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <torch/general.h>
#include <torch/Object.h>
#include <torch/Random.h>
#include <torch/Allocator.h>
#include <torch/XFile.h>

#include <cblas.h>

#include "IMat.hh"
#include "IMatInt.hh"

////////////////////
// Constructors/Destructor
//////////////////////

/** 
 * Empty Constructor
 * 
 */
IMatInt::IMatInt():
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

IMatInt::IMatInt(int m_, int n_):
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
IMatInt::IMatInt(int *base_, int m_, int n_, int ld_, bool do_copy, bool retain):
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

//       memcpy(base, base_, m*n*sizeof(int));
//    }
//    else
//    {
//       base = base_;
//       real_size = m*n;

//       fillPtr();
   
//       if (retain)
//          allocator->retain(base);
//    }
   
}

/** 
 * Copy constructor
 * 
 * @param A IMatInt matrix used to initialize this matrix.
 */

IMatInt::IMatInt(IMatInt &A):
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
      memcpy(ptr[i],A.ptr[i],n*sizeof(int));

}


/** 
 * Destructor
 * 
 */

IMatInt::~IMatInt()
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

int IMatInt::init(int m_, int n_)
{
   m = m_;
   n = n_;
   ld = n_;

   is_alias = false;

   base = (int *)allocator->alloc(sizeof(int) * m*n);
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

int IMatInt::fillPtr()
{
   int *b;
   int **p;
   
   if (m > real_ptr_len)
   {
      int **new_ptr = (int **)allocator->realloc(ptr, sizeof(int *)*m);
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

   for (int i = 0; i < m; i++, b+=n)
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

int IMatInt::set(int *base_, int m_, int n_, int ld_, bool do_copy, bool retain)
{
   if (do_copy)
   {
      if (reshape(m_,n_) != 0)
         return -1;

      if (ld_ <= n_)
         memcpy(base, base_, m*n*sizeof(int));
      else
      {
         // need to use ld_ as a stride in base_

         int *p = base_;
         
         for (int i = 0; i < m; i++, p+=ld_)
            memcpy(ptr[i], p, n*sizeof(int));
      }
    }
   else
   {
      if (!is_alias)
         allocator->free(base);

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

int &IMatInt::operator()(int r, int c)
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

IMatInt &IMatInt::operator=(const IMatInt &A)
{
   if (&A == this)
      return *this;

   if (m != A.m || n != A.n)
      reshape(A.m, A.n);
   
//   MatCopy(&A, this);
   memcpy(base, A.base, m*n);

   return *this;
}

IMatInt &IMatInt::operator=(IMat &A)
{
   int i,j;
   
   if (m != A.m || n != A.n)
      reshape(A.m, A.n);
   
   for (i = 0; i < m; i++)
      for (j = 0; j < n; j++)
         ptr[i][j] = (int)A(i,j);

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

void IMatInt::fill(int val)
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

void IMatInt::zero()
{
   if (base == NULL)
      return;

   if (ld == n)
      memset(base, 0, m*n*sizeof(int));
   else
      for (int i = 0; i < m; i++)
         memset(ptr[i],0, n*sizeof(int));
}

/** 
 * Fill the matrix with ones
 * 
 */

void IMatInt::ones()
{
   fill(1);
}

/** 
 * Fill the matrix with uniform int random values on [a,b]
 * 
 * @param a minimum random value
 * @param b maximum random value
 */

void IMatInt::rand(int a, int b)
{
   int i,j;

   for (i = 0; i < m; i++)
      for (j = 0; j < n; j++)
         ptr[i][j] = Random::random() % (b-a+1) + a;
}

/** 
 * Fill ones on the major diagonal, zeros on the off diagonals.
 * 
 */

void IMatInt::eye()
{
   zero();

   IVecInt d;
   
   getDiag(&d);
   d.ones();
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

void IMatInt::vecFillCols(IVecInt *vec)
{
   IVecInt row;
   
   for (int i = 0; i < m; i++)
   {
      getRow(&row,i);
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

void IMatInt::vecFillRows(IVecInt *vec)
{
   IVecInt row;
   
   for (int i = 0; i < m; i++)
   {
      getRow(&row,i);
      row = *vec;
   }
   
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

void IMatInt::transpose()
{
   int tmp;
      
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

int IMatInt::reshape(int m_, int n_, bool clear)
{
   int *new_base;

   if (m == m_ && n == n_)
      return 0;

   // Does base need to be bigger?
      
   if (m*n < m_*n_)
   {
      if (!is_alias)
      {
         new_base = (int *)allocator->realloc(base, sizeof(int) * m_*n_);
   
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
         memset(base+m*n, 0, (m_*n_-m*n)*sizeof(int));
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

int IMatInt::resize(int m_, int n_)
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

int IMatInt::addRow(unsigned int rows)
{
   if ((m+rows)*n > real_size)
   {
      if (is_alias)
      {
         error("Tried adding a row to an aliased matrix which is too small!\n");
         return -1;
      }

      int *new_base;

      new_base = (int *)allocator->realloc(base, (m+rows)*n*sizeof(int));
      if (new_base != NULL)
      {
         base = new_base;
         real_size = (m+rows)*n;
      }
      else
         return -1;
   }

   memset(base+m*n, 0, (n*rows)*sizeof(int));
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

int IMatInt::addCol(unsigned int cols)
{
   if ((m*(n+cols)) > real_size)
   {
      if (is_alias)
      {
         error("Tried adding a column to an aliased matrix which is too small!\n");
         return -1;
      }

      int *new_base;

      new_base = (int *)allocator->realloc(base, m*(n+cols)*sizeof(int));
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
      memmove(base + i*(n+cols), ptr[i], n*sizeof(int));
      ptr[i] = base + i*(n+cols);
      memset(ptr[i]+n, 0, cols*sizeof(int));
   }
   if (base != NULL)
      memset(base+n, 0, cols*sizeof(int));

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

int IMatInt::addRowCol(unsigned int rows, unsigned int cols)
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

int IMatInt::delRow(unsigned int row, unsigned int count)
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

   int *target = ptr[row];
   int *source = ptr[row+count];
   int len = m-(row+count);
   
   memmove(target, source, n*len*sizeof(int));
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

int IMatInt::delCol(unsigned int col, unsigned int count)
{
   int i, len;
   
   if (col > n-count)
      return -1;

   if (is_alias)
   {
      warning("Deleting columns from aliases not yet supported.\n");
      return -1;
   }

   int *new_loc = base + col;
   int *old_loc = base + (col+count);
   len = n-count;
   
   for (i = 0; i < m-1; i++, new_loc+=len, old_loc+=n)
      memmove(new_loc, old_loc, len*sizeof(int));

   if (col+count < n)
      memmove(new_loc, old_loc, (n-(col+count))*sizeof(int));

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

int IMatInt::delRowCol(unsigned int row, unsigned int col)
{
   if (delRow(row) < 0)
      return -1;
   if (delCol(col) < 0)
      return -1;

   return 0;
}

////////////////////
// Copy/Alias Matrix Parts
//////////////////////

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

int IMatInt::getRow(IVecInt *vec, 
                 unsigned int row,
                 unsigned int col,
                 int len,
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

int IMatInt::getRow(IMatInt *A, 
                 unsigned int row,
                 unsigned int col,
                 int m_,
                 int n_,
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
 * @param vec vector to use as an alias for col
 * @param col column to alias
 * @param row row start at for alias
 * @param len length of vector
 * @param do_copy if true, the column is copied; if not, it's
 *                aliased 
 * 
 * @return 0 on success, -1 on failure
 */

int IMatInt::getCol(IVecInt *vec, 
                 unsigned int col,
                 unsigned int row,
                 int len,
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

int IMatInt::getDiag(IVecInt *vec, int offset, bool do_copy)
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
 * Get an alias for a row of the matrix
 * 
 * @param vec vector to use as an alias for row
 * @param do_copy if true, the row is copied; if not, it's
 *                aliased 
 * 
 * @return 0 on success, -1 on failure
 */

int IMatInt::getAll(IVecInt *vec, 
                    bool do_copy)
{
   if (!vec)
      return -1;

   return (vec->set(base, m*n, 1, do_copy));
}

int IMatInt::hist(IMatInt *out, int bins)
{
   int i,j;
   
   out->resize(n, bins);
   out->zero();
   
   for (i = 0; i < m; i++)
      for (j = 0; j < n; j++)
         (*out)(j, ptr[i][j])++;
   
   return 0;
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

int IMatInt::print(int scrn_width, int col_width, char *num_format)
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

   delete output;

   return 0;
}


void IMatInt::saveXFile(XFile *file)
{
   file->printf("IMatInt: %d %d\n", m, n);
   for (int i = 0; i < m; i++)
   {
      for (int j = 0; j < n; j++)
         file->printf("  %8d", ptr[i][j]);
      file->printf("\n");
   }

   return;
}

void IMatInt::loadXFile(XFile *file)
{
   int m_, n_;
   
   if (file->scanf("IMatInt: %d", &m_) != 1)
      error ("Error reading matrix size (m) from file!\n");

   if (file->scanf("%d", &n_) != 1)
      error ("Error reading matrix size (n) from file!\n");
 
   if (reshape(m_,n_) < 0)
      error("Tried reading data into an array that is not big enough and cannot be resized.\n");
  
   for (int i = 0; i < m; i++)
      for (int j = 0; j < n; j++)
      {
         if (file->scanf("%d", &ptr[i][j]) != 1)
            error("Error reading matrix from file!\n");
         file->scanf("\n",NULL);
      }

   return;

}



