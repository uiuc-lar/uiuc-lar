/**
 * @file   IMat.hh
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Fri Jul  9 13:21:11 2004
 * 
 * @brief  Matrix class
 * 
 * $Log: IMat.hh,v $
 * Revision 1.2  2006/08/04 19:27:32  mrmcclai
 * minor comment added
 *
 * Revision 1.1  2005/05/09 20:53:20  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.5.2.2  2005/02/09 00:33:30  k-squire
 * Added copy constructor from IMatInt.
 * Added normalize(): normalize row sums 1.
 * Added quantization code.
 * Bug fixes.
 *
 * Revision 1.5.2.1  2004/10/26 10:47:06  k-squire
 * Added appendRows()
 * Changed variable names in mean and cov to be more understandable.
 * Added std(): calcs std. dev. for each column/row in a matrix.
 * Added mean1(): scales mean by (n-1); used by std.
 * Added scatter(): calcs scatter of a matrix.
 * Added standardize(): subtracts mean, scales by std. dev.
 * Changed saveXFile to no longer print "IMat:"
 *
 * Revision 1.5  2004/08/03 04:18:33  k-squire
 * Now uses ld as a stride from the beginning of one row to the next.
 *    (* WARNING: this is not yet tested thoroughly! *)
 * Added getSubMat() (using ld)
 * Fixed loadXFile() so that "IMat:" is not required when loading a matrix.
 * Added appendXFile() to append a file to this matrix.  Matrix sizes should
 *    be the same (although the matrix is resized appropriately if not).
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
 * Revision 1.2  2004/07/23 20:48:26  k-squire
 * Minor formatting changes.
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 * 
 */

#ifndef IMAT_HH
#define IMAT_HH

static char *IMAT_HH_Id = "$Id: IMat.hh,v 1.2 2006/08/04 19:27:32 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <torch/general.h>
#include <torch/Object.h>

#include "IVec.hh"
#include "IVecInt.hh"
#include "IMatInt.hh"

using namespace Torch;

class IMatInt;


/**
 * @class IMat
 * @brief matrix class, for use with IVec and IMatVecOps
 *
 */

class IMat : public Object
{
public:
   int m;               ///< number of rows
   int n;               ///< number of columns
   int ld;              ///< length from start of 1 row to start of
                        ///< next (ld >= n, bigger if this is a sub-matrix)

   real *base;          ///< base pointer of matrix (stored contiguously)
   real **ptr;          ///< 2D ptr for easy access to contents

   bool is_alias;       ///< if true, this matrix is an alias

   int real_size;       ///< the real size of the allocated/alias space
   int real_ptr_len;    ///< the real length allocated for ptr

public:

   ////////////////////
   // Constructors/Destructor
   //////////////////////

   IMat();                      ///< Empty constructor

   IMat(int m_, int n_);        ///< Create an m_ by n_ matrix

   IMat(real *base_, 
        int m_, 
        int n_, 
        int ld_ = -1,
        bool do_copy = false, 
        bool retain = false);   ///< Create an m_ by n_ matrix from
                                ///< base_, aliasing or copying as
                                ///< requested 

   IMat(IMat &A);               ///< Copy constructor

   ~IMat();                     ///< Destructor

   ////////////////////
   // Matrix Initialization
   //////////////////////

private:

   int init(int m_, int n_);                 ///< initialize with m_
                                             ///< rows, n_ columns

   int fillPtr();                           ///< fill ptr with values
                                             ///< to access array
public:

   int set(real *base_, 
           int m_, 
           int n_,
           int ld_ = -1,
           bool do_copy = false, 
           bool retain = false);     ///< copy/alias data from base_

   ////////////////////
   // Operators
   //////////////////////

   real &operator()(int r, int c);   ///< Access operator: returns a
                                     ///< reference to ptr[r][c]

   IMat &operator=(const IMat &A);   ///< = operator: copies the
                                     ///< matrix from A to this object

   IMat &operator=(IMatInt &A);   ///< = operator: copies the
                                     ///< integer matrix from A to this object
   
   ////////////////////
   // Matrix Fill Ops
   //////////////////////

   void fill(real val);          ///< Fill the matrix with val
   
   void zero();                  ///< zero the matrix
   void ones();                  ///< fill the matrix with ones
   void rand(real a = 0.0,
             real b = 1.0);      ///< fill the matrix with random values

   void eye();        ///< fill ones along the major diagonal, zeros
                      ///< on the off diagonals

   void normalize();  ///< normalize columns to sum to 1 (make row option?)

   void vecFillCols(IVec *vec);      ///< Fill the matrix columns with
                                     ///< vector vec
   void vecFillRows(IVec *vec);      ///< Fill the matrix rows with
                                     ///< vector vec

   ////////////////////
   // Matrix Shape Manipulation
   //////////////////////

   void transpose();                         ///< transpose the matrix
                                             ///< This is expensive!!!

   int reshape(int m_, int n_, 
               bool clear = false);          ///< reshape this
                                             ///< matrix

   int resize(int m_, int n_);               ///< resize this matrix, adding or
                                             ///< removing rows/cols as necessary
   
   int addRow(unsigned int rows = 1);        ///< add rows to the matrix
   int addCol(unsigned int cols = 1);        ///< add cols to the matrix
   int addRowCol(unsigned int rows = 1, 
                 unsigned int cols = 1);     ///< add rows and cols to
                                             ///< the matrix 

   int delRow(unsigned int row, 
              unsigned int count = 1);       ///< delete a row from the matrix
   int delCol(unsigned int col, 
              unsigned int count = 1);       ///< delete a column from the
                                             ///< matrix
   int delRowCol(unsigned int row, 
                 unsigned int col);          ///< delete a row and column
                                             ///< from the matrix

   int appendRows(IMat *A,
                  int start_row = 0,
                  int stop_row = -1);        ///< append rows from
                                             ///< matrix A to this matrix

   ////////////////////
   // Copy/Alias Matrix Rows/Cols/Diags
   //////////////////////

   int getRow(unsigned int row,
              IVec *vec, 
              bool do_copy = false);         ///< copy/alias a matrix row

   int getRow(unsigned int row,
              unsigned int col,
              int len,
              IVec *vec, 
              bool do_copy = false);         ///< copy/alias a matrix row

   int getRow(unsigned int row,
              unsigned int col,
              int m_,
              int n_,
              IMat *A, 
              bool do_copy = false);         ///< copy/alias a matrix row

   int getCol(unsigned int col,
              IVec *vec, 
              bool do_copy = false);         ///< copy/alias a matrix column

   int getCol(unsigned int col,
              unsigned int row,
              int len,
              IVec *vec, 
              bool do_copy = false);         ///< copy/alias a matrix column

   int getDiag(IVec *vec, 
               bool do_copy = false);        ///< copy/alias a matrix diagonal
   
   int getDiag(int offset,
               IVec *vec, 
               bool do_copy = false);        ///< copy/alias a matrix diagonal
   
   int getAll(IVec *vec,
              bool do_copy = false);         ///< copy/alias the whole matrix

   int getSubMat(unsigned int row1,
                 unsigned int col1,
                 unsigned int row2,
                 unsigned int col2,
                 IMat *A,
                 bool do_copy = false);      ///< copy/alias a submatrix 
                                             ///< (not fully implemented)

   ////////////////////
   // Set Matrix Rows/Cols/Diags
   //////////////////////

   int setRow(unsigned int row,
              IVec *vec);                    ///< set row to vec

   int setCol(unsigned int col,
              IVec *vec);                    ///< set col to vec

   int setDiag(IVec *vec);                   ///< set main diag to vec

   int setDiag(int offset,
               IVec *vec);                   ///< set diag to vec


   ////////////////////
   // Misc
   //////////////////////

   int print(int scrn_width=80, 
             int col_width=12, 
             char *num_format=" %11.8f");     ///< print out the matrix

   int prod(IVec *out,
            int dim = 1);                    ///< out = product along
                                             ///< dimension dim (1 =
                                             ///< columns, 2 = rows)

   int sum(IVec *out, 
           int dim = 1,
           IVecInt *ind = NULL);             ///< out = mean along
                                             ///< dimension dim (1 =
                                             ///< columns, 2 = rows)
   int mean(IVec *mu, 
            int dim = 1,
            IVecInt *ind = NULL);            ///< mu = mean along
                                             ///< dimension dim (1 =
                                             ///< columns, 2 = rows)

   int mean1(IVec *mu, 
             int dim = 1,
             IVecInt *ind = NULL);           ///< mu = mean along
                                             ///< dimension dim (1 =
                                             ///< columns, 2 = rows)
                                             ///< (divide by n-1)
   int std(IVec *mu,
           IVec *sigma,
           int dim = 1,
           IVecInt *ind = NULL);             ///< sigma = standard
                                             ///< deviation along dim (1
                                             ///< = columns, 2 = rows)

   int cov(IVec *mu,
           IMat *Sigma,
           int dim = 1,
           IVecInt *ind = NULL);             ///< Sigma = covariance along
                                             ///< dimension dim (1 =
                                             ///< columns, 2 = rows)

   int scatter(IVec *mu,
               IMat *scat,
               int dim = 1,
               IVecInt *ind = NULL);         ///< scat = scatter along
                                             ///< dimension dim (1 =
                                             ///< columns, 2 = rows)

   real mmin(int *pos = NULL);               ///< min value in matrix

   int mmin(IVec *out,
            IVecInt *pos,
            int dim = 1);                    ///< out = min along
                                             ///< dimension dim (1 =
                                             ///< columns, 2 = rows)

   real mmax(int *pos = NULL);               ///< max value in matrix

   int mmax(IVec *out,
            IVecInt *pos,
            int dim = 1);                    ///< out = max along
                                             ///< dimension dim (1 =
                                             ///< columns, 2 = rows)

   void abs();                               ///< take abs of elements

   void standardize(IVec *mu = NULL,
                    IVec *sigma = NULL,
                    int dim = 1);            ///< standardize
                                             ///< rows/cols of matrix
                                             ///< (subtract mean, divide
                                             ///< by std. dev.)

   void getQuantLevels(IMat *q_levels,
                       int bins=5);          ///< get quantization levels

   int quantize(IMatInt *out,
                IMat *q_levels);             ///< quantize rows of
                                             ///< this matrix

   ////////////////////
   // File stuff
   //////////////////////

   virtual void saveXFile(XFile *file);   ///< save the matrix to a file
   virtual void loadXFile(XFile *file);   ///< load the matrix from a file
   virtual void appendXFile(XFile *file); ///< load the matrix from a file
};



#endif // IMAT_HH
