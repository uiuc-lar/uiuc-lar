/**
 * @file   IMatInt.hh
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Fri Jul  9 13:21:11 2004
 * 
 * @brief  Matrix class
 * 
 * $Log: IMatInt.hh,v $
 * Revision 1.1  2005/05/09 20:53:19  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.2.2.1  2005/02/09 00:38:12  k-squire
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
 * Revision 1.2  2004/07/29 07:08:10  k-squire
 * Added copy constructor.
 * Added saveXFile, loadXFile.
 *
 * Revision 1.1  2004/07/23 20:49:50  k-squire
 * Added IMatInt, IVecInt files, classes, and tests.
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 * 
 */

#ifndef IMATINT_HH
#define IMATINT_HH

static char *IMATINT_HH_Id = "$Id: IMatInt.hh,v 1.1 2005/05/09 20:53:19 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <torch/general.h>
#include <torch/Object.h>

#include "IMat.hh"
#include "IVecInt.hh"

using namespace Torch;


/**
 * @class IMatInt
 * @brief matrix class, for use with IVecInt and IMatIntVecOps
 *
 */

class IMatInt : public Object
{
public:
   int m;               ///< number of rows
   int n;               ///< number of columns
   int ld;              ///< length from start of 1 row to start of
                        ///< next (ld >= n, bigger if this is a sub-matrix)

   int *base;          ///< base pointer of matrix (stored contiguously)
   int **ptr;          ///< 2D ptr for easy access to contents

   bool is_alias;       ///< if true, this matrix is an alias

   int real_size;       ///< the int size of the allocated/alias space
   int real_ptr_len;    ///< the int length allocated for ptr

public:

   ////////////////////
   // Constructors/Destructor
   //////////////////////

   IMatInt();                      ///< Empty constructor

   IMatInt(int m_, int n_);        ///< Create an m_ by n_ matrix

   IMatInt(int *base_, 
           int m_, 
           int n_, 
           int ld_,
           bool do_copy = false, 
           bool retain = false);   ///< Create an m_ by n_ matrix from
                                   ///< base_, aliasing or copying as
                                   ///< requested 

   IMatInt(IMatInt &A);            ///< Copy constructor

   ~IMatInt();                     ///< Destructor

   ////////////////////
   // Matrix Initialization
   //////////////////////

private:

   int init(int m_, int n_);                 ///< initialize with m_
                                             ///< rows, n_ columns

   int fillPtr();                           ///< fill ptr with values
                                             ///< to access array
public:

   int set(int *base_, 
           int m_, 
           int n_, 
           int ld_,
           bool do_copy = false, 
           bool retain = false);     ///< copy/alias data from base_

   ////////////////////
   // Operators
   //////////////////////

   int &operator()(int r, int c);   ///< Access operator: returns a
                                     ///< reference to ptr[r][c]

   IMatInt &operator=(const IMatInt &A);   ///< = operator: copies the
                                     ///< matrix from A to this object
   
   IMatInt &operator=(IMat &A);   ///< = operator: copies the
                                     ///< matrix from A to this object
   
   ////////////////////
   // Matrix Fill Ops
   //////////////////////

   void fill(int val);          ///< Fill the matrix with val
   
   void zero();                  ///< zero the matrix
   void ones();                  ///< fill the matrix with ones
   void rand(int a = 0.0,
             int b = 1.0);      ///< fill the matrix with random values

   void eye();        ///< fill ones along the major diagonal, zeros
                      ///< on the off diagonals

   void vecFillCols(IVecInt *vec);      ///< Fill the matrix columns with
                                     ///< vector vec
   void vecFillRows(IVecInt *vec);      ///< Fill the matrix rows with
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
   ////////////////////
   // Copy/Alias Matrix Parts
   //////////////////////

   int getRow(IVecInt *vec, 
              unsigned int row,
              unsigned int col = 0,
              int len = -1,
              bool do_copy = false);         ///< copy/alias a matrix row

   int getRow(IMatInt *A, 
              unsigned int row,
              unsigned int col,
              int m_,
              int n_,
              bool do_copy = false);         ///< copy/alias a matrix row

   int getCol(IVecInt *vec, 
              unsigned int col,
              unsigned int row = 0,
              int len = -1,
              bool do_copy = false);         ///< copy/alias a matrix column

   int getDiag(IVecInt *vec, 
               int offset = 0,
               bool do_copy = false);        ///< copy/alias a matrix diagonal
   
   int getAll(IVecInt *vec,
              bool do_copy = false);         ///< copy/alias the whole matrix


   int hist(IMatInt *out,
            int bins);                       ///< get a histogram for
                                             ///< each column in the matrix

//    int subMat(IMatInt *A,
//               unsigned int row1,
//               unsigned int col1,
//               unsigned int row2,
//               unsigned int col2,
//               bool do_copy = false);

   ////////////////////
   // Misc
   //////////////////////

   int print(int scrn_width=80, 
             int col_width=8, 
             char *num_format=" %6d");     ///< print out the matrix

   ////////////////////
   // File stuff
   //////////////////////

   virtual void saveXFile(XFile *file);   ///< save the matrix to a file
   virtual void loadXFile(XFile *file);   ///< load the matrix from a file

};



#endif // IMATINT_HH
