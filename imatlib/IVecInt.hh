/**
 * @file   IVecInt.hh
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Fri Jul  9 23:35:50 2004
 * 
 * @brief  Vector class
 * 
 * $Log: IVecInt.hh,v $
 * Revision 1.1  2005/05/09 20:53:20  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.3.2.1  2005/02/09 00:42:55  k-squire
 * Added fuzzy_find()
 * Added hist() -- histogram.
 * Fixed delete[].
 *
 * Revision 1.3  2004/07/29 07:13:00  k-squire
 * Changed constructor to always initialize parms to 0 first.
 * Added copy constructor.
 * Renamed max_elem-->vmax.
 * Added vmin.
 * Added find. (Find all elements equal to a given value)
 * Added sort (using quicksort).
 * Added saveXFile, loadXFile.
 *
 * Revision 1.2  2004/07/26 17:01:41  k-squire
 * Added constructor from real.
 * (Re-)Added operator[]
 *
 * Revision 1.1  2004/07/23 20:49:50  k-squire
 * Added IMatInt, IVecInt files, classes, and tests.
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 * 
 */

#ifndef IVECINT_HH
#define IVECINT_HH

static char *IVECINT_HH_Id = "$Id: IVecInt.hh,v 1.1 2005/05/09 20:53:20 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <torch/general.h>
#include <torch/Object.h>

using namespace Torch;

/**
 * @class IVecInt
 * @brief vector class, for use with IMat and IMatVecOps
 *
 * Please see the long description in IMat.hh for most details.
 *
 * There are two types of vectors that can be created with this
 * class: normal vectors, and aliases.  In normal vectors, memory
 * for the vector data is allocated by this class.  Aliases, instead,
 * point the vector to data managed outside of this class.  For
 * example, a part of a matrix or an externally allocated array can be
 * treated as an IVecInt object.
 * 
 *
 */

class IVecInt : public Object
{
public:
   int n;          ///< vector length
   int real_n;     ///< allocated length

   int stride;     ///< length between successive elements of the vector

   int *ptr;      ///< vector data

   bool is_alias;  ///< if true, this vector is an alias

public:

   ////////////////////
   // Constructors/Destructor
   //////////////////////

   IVecInt();                    ///< empty constructor

   IVecInt(int n_);              ///< create an n_ length vector
   
   IVecInt(int *ptr_, 
        int n_, 
        int stride_ = 1, 
        bool do_copy = false, 
        bool retain = false);    ///< create an n_ length vector from
                                 ///< ptr_, copying as necessary

   IVecInt(IVecInt &v);          ///< copy constructor
   
   ~IVecInt();                   ///< destructor


   ////////////////////
   // Vector Initialization
   //////////////////////

private:
   int init(int n_);             ///< initialize an n_ length vector

public:
   
   int set(int *ptr_, 
           int n_, 
           int stride_=1, 
           bool do_copy = false, 
           bool retain = false);           ///< copy/alias data from ptr_

   int set(real *ptr_, 
           int n_, 
           int stride_=1);                 ///< copy real data from ptr_

   ////////////////////
   // Operators
   //////////////////////

   int &operator()(int pos);               ///< reference a vector element
   int &operator[](int pos);               ///< reference a vector element
   IVecInt &operator=(const IVecInt &vec); ///< copy vec to this vector


   ////////////////////
   // Vector Fill Ops
   //////////////////////

   void fill(int val);          ///< fill the vector with val
   void zero();                 ///< zero the vector
   void ones();                 ///< fill the vector with ones
   void rand(int a = 1, 
             int b = 10);       ///< fill the vector with random values

   ////////////////////
   // Vector Manipulation
   //////////////////////

   int ins(unsigned int pos, 
           unsigned int len=1, 
           int val = 0.0);               ///< insert len values at pos

   int del(unsigned int pos, 
           unsigned int len=1);           ///< delete len elements
                                          ///< starting at pos

   int resize(unsigned int n_, 
              bool clear = false);        ///< resize the vector

   ////////////////////
   // Copy/Alias a Sub-vector
   //////////////////////

//   int subVec(unsigned int start,
//              unsigned int end = -1,
//              bool do_copy = false);

   ////////////////////
   // Misc
   //////////////////////

   int print(int scrn_width=80, 
             int col_width=8, 
             char *num_format=" %6d");    ///< print the vector

   int vmax(int *pos = NULL);  ///< find the maximum element
   int vmin(int *pos = NULL);  ///< find the minimum element

   int find(int num, IVecInt *pos);        ///< find the locations of num
   int fuzzy_find(int num, IVecInt *pos);  ///< find the closest location to num

   int sort(IVecInt *ind);      ///< get the indices of the sorted
                                ///< vector

   int hist(IVecInt *h, int bins); ///< histogram

private:
   void quicksort(IVecInt *ind, int p, int r);
   int partition(IVecInt *ind, int p, int r);

   ////////////////////
   // File stuff
   //////////////////////

public:
   virtual void saveXFile(XFile *file);   ///< save the vector to a file
   virtual void loadXFile(XFile *file);   ///< load the vector from a file

};


#endif // IVECINT_HH
