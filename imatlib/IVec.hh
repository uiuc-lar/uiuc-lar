/**
 * @file   IVec.hh
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Fri Jul  9 23:35:50 2004
 * 
 * @brief  Vector class
 * 
 * $Log: IVec.hh,v $
 * Revision 1.1  2005/05/09 20:53:19  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.5.2.2  2005/02/09 00:41:26  k-squire
 * Added copy constructor from IVecInt
 * Added normalize().
 * Added getInd() to return a vector of values with a set of given indices.
 * Added norm2()
 * Added sort()
 * Added standardize()
 * Added quantize()
 * Fixed delete[].
 *
 * Revision 1.5.2.1  2004/10/26 10:50:36  k-squire
 * Fixed loadXFile to load even if "IVec" is not present.
 * Added sqrt() of vector elements.
 *
 * Revision 1.5  2004/08/10 23:03:06  k-squire
 * Backported changes from illy/matrix
 *
 * Revision 1.1.2.1  2004/08/08 23:07:35  k-squire
 * Moved REAL_FORMAT_* definitions to gen_defines.h.
 * Added sum() function
 *
 * Revision 1.1  2004/08/03 06:39:08  k-squire
 * Import and initial checkin.
 *
 * Revision 1.4  2004/07/29 07:10:39  k-squire
 * Fixed constructor to initialize variables to 0 first.
 * Added copy constructor.
 * Init always allocates vector now, instead of comparing size first.
 * Renamed max_elem-->vmax.
 * Added vmin.
 * Added saveXFile, loadXFile.
 *
 * Revision 1.3  2004/07/26 17:01:06  k-squire
 * (Re-)Added operator[]
 *
 * Revision 1.2  2004/07/23 20:48:02  k-squire
 * Minor formatting changes.
 * rand() now takes range arguments.
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 * 
 */

#ifndef IVEC_HH
#define IVEC_HH

static char *IVEC_HH_Id = "$Id: IVec.hh,v 1.1 2005/05/09 20:53:19 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <torch/general.h>
#include <torch/Object.h>
#include <torch/XFile.h>

#include "IVecInt.hh"

class IMat;

using namespace Torch;

/**
 * @class IVec
 * @brief vector class, for use with IMat and IMatVecOps
 *
 * Please see the long description in IMat.hh for most details.
 *
 * There are two types of vectors that can be created with this
 * class: normal vectors, and aliases.  In normal vectors, memory
 * for the vector data is allocated by this class.  Aliases, instead
 * point the vector to data managed outside of this class.  For
 * example, a part of a matrix or an externally allocated array can be
 * treated as an IVec object.
 * 
 *
 */

class IVec : public Object
{
public:
   int n;          ///< vector length
   int real_n;     ///< allocated length

   int stride;     ///< length between successive elements of the vector

   real *ptr;      ///< vector data

   bool is_alias;  ///< if true, this vector is an alias

public:

   ////////////////////
   // Constructors/Destructor
   //////////////////////

   IVec();                       ///< empty constructor

   IVec(int n_);                 ///< create an n_ length vector
   
   IVec(real *ptr_, 
        int n_, 
        int stride_ = 1, 
        bool do_copy = false, 
        bool retain = false);    ///< create an n_ length vector from
                                 ///< ptr_, copying as necessary

   IVec(IVec &v);                ///< copy constructor
   
   ~IVec();                      ///< destructor


   ////////////////////
   // Vector Initialization
   //////////////////////

private:
   int init(int n_);             ///< initialize an n_ length vector

public:
   
   int set(real *ptr_, 
           int n_, 
           int stride_=1, 
           bool do_copy = false, 
           bool retain = false);          ///< copy/alias data from ptr_

   ////////////////////
   // Operators
   //////////////////////

   real &operator()(int pos);             ///< reference a vector element
   real &operator[](int pos);             ///< reference a vector element
   IVec &operator=(const IVec &vec);      ///< copy vec to this vector
   IVec &operator=(IVecInt &vecint);      ///< copy vecint to this vector


   ////////////////////
   // Vector Ops
   //////////////////////

   void fill(real val);          ///< fill the vector with val
   void zero();                  ///< zero the vector
   void ones();                  ///< fill the vector with ones
   void rand(real a = 0.0, 
             real b = 1.0);      ///< fill the vector with random values

   void sqrt();                  ///< take the square root of vector
                                 ///<  elements

   int normalize();             ///< normalize a vector sum to 1

   ////////////////////
   // Vector Manipulation
   //////////////////////

   int ins(unsigned int pos, 
           unsigned int len=1, 
           real val = 0.0);               ///< insert len values at pos

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

   int getInd(IVec *out, IVecInt *ind);

   ////////////////////
   // Misc
   //////////////////////

   int print(int scrn_width=80, 
             int col_width=12, 
             char *num_format=" %11.8f");    ///< print the vector

   real vmax(int *pos = NULL);  ///< find the maximum element
   real vmin(int *pos = NULL);  ///< find the minimum element

   real sum();

   real norm2();
   int sort(IVecInt *ind);      ///< get the indices of the sorted vector

   void standardize(IVec *mu,
                    IVec *sigma);            ///< standardize
                                             ///< (subtract mean, divide
                                             ///< by std. dev.)

   int quantize(IVecInt *out, IMat *q_levels);  ///< quantize this
                                                ///< vector according to
                                                ///< quantization
                                                ///< levels q_levels
   
   
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


#endif // IVEC_HH
