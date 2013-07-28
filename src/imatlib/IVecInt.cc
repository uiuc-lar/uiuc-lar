/**
 * @file   IVecInt.cc
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Sat Jul 10 00:07:12 2004
 * 
 * @brief  Vector class
 * 
 * $Log: IVecInt.cc,v $
 * Revision 1.1  2005/05/09 20:53:20  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.4.2.1  2005/02/09 00:42:55  k-squire
 * Added fuzzy_find()
 * Added hist() -- histogram.
 * Fixed delete[].
 *
 * Revision 1.4  2004/07/30 03:55:59  k-squire
 * Changed file format slightly for ease of editing.
 *
 * Revision 1.3  2004/07/29 07:12:50  k-squire
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

static char *IVECINT_CC_Id = "$Id: IVecInt.cc,v 1.1 2005/05/09 20:53:20 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <limits.h>          // INT_MIN

#include <torch/general.h>
#include <torch/Object.h>
#include <torch/Random.h>
#include <torch/XFile.h>

#include "IVecInt.hh"
#include "IMat.hh"
#include "IMatVecOps.hh"
#include "gen_defines.h"

using namespace Torch;

/** 
 * Empty constructor
 * 
 */

IVecInt::IVecInt():
      n(0),
      real_n(0),
      stride(1),
      ptr(NULL),
      is_alias(false)
{
}

/** 
 * Constuctor
 * 
 * @param n_ vector length
 */

IVecInt::IVecInt(int n_):
      n(n_),
      real_n(n_),
      stride(1),
      is_alias(false)
{
   ptr = (int *)allocator->alloc(sizeof(int)*n);
}

/** 
 * Constructor
 * 
 * @param ptr_ data source
 * @param n_ vector length
 * @param stride_ length between successive elements
 * @param do_copy if true, copy the data from ptr_
 * @param retain if true, and do_copy is false, retain and manage memory at ptr_
 */

IVecInt::IVecInt(int *ptr_, int n_, int stride_, bool do_copy, bool retain):
      n(0),
      real_n(0),
      stride(1),
      ptr(NULL),
      is_alias(false)
{
   if (do_copy)
   {
      init(n_);

      if (stride_ == 1)
         memcpy(ptr, ptr_, n*sizeof(int));
      else
      {
         int *p_to = ptr;
         int *p_from = ptr_;

         for (int i = 0; i < n; i++, p_to++, p_from+=stride_)
            *p_to = *p_from;
      }

   }
   else
   {
      n = n_;
      real_n = n_;
      stride = stride_;
      ptr = ptr_;
      is_alias = true;

      if (retain)
      {
         allocator->retain(ptr);
         is_alias = false;
      }
      
   }
}

/** 
 * Copy constructor
 * 
 * @param v vector used to initialize this vector
 */

IVecInt::IVecInt(IVecInt &v):
      n(0),
      real_n(0),
      stride(1),
      ptr(NULL),
      is_alias(true)
{
   init(v.n);

   for (int i = 0; i < n; i++)
      ptr[i] = v[i];
}

/** 
 * Destructor
 * 
 */

IVecInt::~IVecInt()
{
}

/** 
 * Vector initialization
 * 
 * @param n_ vector length
 * 
 * @return 0 on success, -1 on failure
 */

int IVecInt::init(int n_)
{
   if (n != n_)
   {
      int *new_ptr = (int *)allocator->realloc(ptr, sizeof(int)*n_);
      if (new_ptr)
         ptr = new_ptr;
      else
         return -1;
   }
   
   n = n_;
   real_n = n_;
   stride = 1;
   is_alias = false;

   return 0;
}

/** 
 * Fill the vector with a particular value
 * 
 * @param val value used to fill the vector
 */

void IVecInt::fill(int val)
{
   int *p = ptr;
   
   for (int i = 0; i < n; i++, p+=stride)
      *p = val;

}

/** 
 * Zero the vector
 * 
 */

void IVecInt::zero()
{
   fill(0.0);
}

/** 
 * Fill the vector with ones.
 * 
 */

void IVecInt::ones()
{
   fill(1.0);
}

/** 
 * Fill the vector with random values.
 *
 * @param a lower bound, inclusive
 * @param b upper bound, inclusive
 * 
 */

void IVecInt::rand(int a, int b)
{
   int *p = ptr;
   
   for (int i = 0; i < n; i++, p+=stride)
      *p = Random::random() % (b-a+1) + a;
}

/** 
 * Find the maximum element in the vector
 * 
 * @param pos if not NULL, contains the position of the maximum
 * element upon return
 * 
 * @return value of maximum element
 */

int IVecInt::vmax(int *pos)
{
   int *p = ptr;

   int mv = INT_MIN;
   int mp = -1;
   
   for (int i = 0; i < n; i++, p+=stride)
   {
      if (*p > mv)
      {
         mv = *p;
         mp = i;
      }
   }
   
   if (pos)
      *pos = mp;

   return mv;
}

/** 
 * Find the minimum element in the vector
 * 
 * @param pos if not NULL, contains the position of the minimum
 * element upon return
 * 
 * @return value of minimum element
 */

int IVecInt::vmin(int *pos)
{
   int *p = ptr;

   int mv = INT_MAX;
   int mp = -1;
   
   for (int i = 0; i < n; i++, p+=stride)
   {
      if (*p < mv)
      {
         mv = *p;
         mp = i;
      }
   }
   
   if (pos)
      *pos = mp;

   return mv;
}

/** 
 * Find an element in the vector
 * 
 * @param num number to find
 * @param pos if not NULL, contains the positions of the element
 * 
 * @return positions of minimum element
 */

int IVecInt::find(int num, IVecInt *pos)
{
   int *p = ptr;
   int *p2;

   int count = 0;
   
   pos->resize(n);

   p2 = pos->ptr;

   for (int i = 0; i < n; i++, p+=stride)
   {
      if (*p == num)
      {
         *p2++ = i;
         count++;
      }
   }

   pos->resize(count);

   return 0;
}

/** 
 * Find the position of the closest element to a number in the vector
 * 
 * @param num number to find
 * 
 * @return position of closest element to num
 */

int IVecInt::fuzzy_find(int num, IVecInt *pos)
{
   int min_dist = INT_MAX;

   int *p = ptr;
   int *p2;

   int count = 0;

   pos->resize(n);

   p2 = pos->ptr;

   for (int i = 0; i < n; i++, p+=stride)
   {
      if (abs(*p - num) < min_dist)
      {
         min_dist = abs(*p - num);
         p2 = pos->ptr;

         *p2++ = i;
         count = 1;
      }
      else if (abs(*p - num) == min_dist)
      {
         *p2++ = i;
         count++;
      }
   }

   pos->resize(count);

   return 0;
}

int IVecInt::hist(IVecInt *h, int bins)
{
   int i, v;
   
   h->resize(bins);
   h->zero();

   for (i = 0; i < n; i++)
      if ((v = (*this)[i]) < bins)
         (*h)[v]++;

   return 0;
}

/** 
 * Get the indices of the sorted vector
 * 
 * @param pos contains the order of the sorted vector upon return
 *  (note that the vector itself is not modified)
 * 
 * @return value of minimum element
 */

int IVecInt::sort(IVecInt *pos)
{
   int i;
   
   pos->resize(n);
   
   for (i = 0; i < n; i++)
      (*pos)[i] = i;

   quicksort(pos, 0, n-1);

   return 0;
}


void IVecInt::quicksort(IVecInt *ind, int p, int r)
{
   int q;

   if (p < r)
   {
      q = partition(ind, p, r);
      quicksort(ind, p, q);
      quicksort(ind, q+1,r);
   }
}

int IVecInt::partition(IVecInt *ind, int p, int r)
{
   int x = ptr[(*ind)[p]];
   int i = p-1;
   int j = r+1;

   int tmp;

   while (1)
   {
      do
         j--;
      while (ptr[(*ind)[j]] > x);
      
      do 
         i++;
      while (ptr[(*ind)[i]] < x);
      
      if (i < j)
         swap((*ind)[i],(*ind)[j],tmp);
      else
         return j;
   }

}


/** 
 * Insert elements (and increase array length)
 * 
 * @param pos position at which to start insert
 * @param len number of elements to insert
 * @param val value to insert
 * 
 * @return 0 on success, -1 on failure
 */

int IVecInt::ins(unsigned int pos, unsigned int len, int val)
{
   int *new_ptr;
   int *p;
   int i;

   if (pos > n)
      return -1;
   
   if (n+len > real_n)
   {
      // Can't increase size of a matrix row/column alias!

      if (is_alias)
         return -1;

      if ((new_ptr=(int *)allocator->realloc(ptr,(n+len)*sizeof(int)))==NULL)
         return -1;

      ptr = new_ptr;
      real_n=n+len;
   }
   
   if (pos < n)
      memmove(ptr+pos+len, ptr+pos, (n-pos)*sizeof(int));
   
   for (i = 0, p = ptr+pos; i < len; i++, p++)
      *p = val;

   n+=len;
   
   return 0;
}

/** 
 * Delete elements
 * 
 * @param pos position at which to start deletion
 * @param len number of elements to delete
 * 
 * @return 0 on success, -1 on failure
 */

int IVecInt::del(unsigned int pos, unsigned int len)
{
   if (pos+len > n)
      return -1;
   
   if (pos+len < n)
      memmove(ptr+pos, ptr+pos+len, (n-(pos+len))*sizeof(int));
   
   n-=len;
   
   return 0;
}

/** 
 * Resize the vector
 * 
 * @param n_ new vector length
 * @param clear if true, zero any added length
 * 
 * @return 0 on success, -1 on failure
 */

int IVecInt::resize(unsigned int n_, bool clear)
{
   int *new_ptr;

   if (n_ > real_n)
   {
      // Can't increase size of an alias!

      if (is_alias)
      {
         error ("Tried to increase the size of an alias!\n");
         return -1;
      }

      // Otherwise... resize the matrix to n_

      if ((new_ptr = (int *)allocator->realloc(ptr,(n_)*sizeof(int)))==NULL)
         return -1;
      
      ptr = new_ptr;
      real_n = n_;
   }

   if (clear && n_>n)
      memset(ptr+n, 0, sizeof(int)*(n_-n));

   n=n_;

   return 0;
}

/** 
 * Make this vector an alias to ptr
 * 
 * @param ptr_ data
 * @param n_ number of elements
 * @param stride_ length between successive elements
 * 
 * @return 0 on success, -1 on failure
 */

int IVecInt::set(int *ptr_, int n_, int stride_, bool do_copy, bool retain)
{
   if (do_copy)
   {
      if (resize(n_) != 0)
         return -1;
      
      if (stride == 1 && stride_ == 1)
         memcpy(ptr, ptr_, n*sizeof(int));
      else
      {
         int *p_to = ptr;
         int *p_from = ptr_;
         
         for (int i = 0; i < n; i++, p_to+=stride, p_from+=stride_)
            *p_to = *p_from;
      }
      
   }
   else
   {
      if (!is_alias)
         allocator->free(ptr);

      n = n_;
      real_n = n_;
      stride = stride_;
      ptr = ptr_;
      is_alias = true;

      if (retain)
         allocator->retain(ptr);
   }
   
   return 0;
}


/** 
 * Copy real data from ptr_ to int data here
 * 
 * @param ptr_ real data
 * @param n_ number of elements
 * @param stride_ length between successive elements
 * 
 * @return 0 on success, -1 on failure
 */

int IVecInt::set(real *ptr_, int n_, int stride_)
{
   if (resize(n_) != 0)
      return -1;
      
   int *p_to = ptr;
   real *p_from = ptr_;
         
   for (int i = 0; i < n; i++, p_to+=stride, p_from+=stride_)
      *p_to = (int)*p_from;
   
   return 0;
}


/** 
 * Print the vector
 * 
 * @param scrn_width screen width
 * @param col_width width of vector columns
 * @param num_format format to use for printing numbers
 * 
 * @return 0 on success, -1 on failure
 */

int IVecInt::print(int scrn_width, int col_width, char *num_format)
{
   int cols = (scrn_width-6)/col_width;

   static char *format = new char[10];
   static char *formatd = new char[10];

   char *output = new char[col_width+1];
   
   snprintf(format, 10, "%%%ds", col_width);
   snprintf(formatd, 10, "%%%dd", col_width);

   if (is_alias && stride == 1)
   {
      for (int i = 0; i < n; i+=cols)
      {
         int j;
         printf ("  Columns %d through %d\n\n", i, min(i+cols, n)-1);

         for (j = i; j < min(i+cols, n); j++)
            printf (formatd, j);

         printf("\n");

         for (j = i; j < min(i+cols, n); j++)
         {
            snprintf(output, col_width+1, num_format, ptr[j]);
            printf(format, output);
         }
         printf("\n\n");
      }
   }
   else
   {
      for (int i = 0, j = 0; i < n; i++, j+=stride)
      {
         printf (formatd, i);
         snprintf(output, col_width+1, num_format, ptr[j]);
         printf(format, output);
         printf("\n");
      }
   }

   printf("\n\n");

   delete[] output;

   return 0;
}

/** 
 * Reference a vector element
 * 
 * 
 * @return reference to vector element
 */

int &IVecInt::operator()(int pos)
{
   return ptr[pos*stride];
}

/** 
 * Reference a vector element
 * 
 * 
 * @return reference to vector element
 */

int &IVecInt::operator[](int pos)
{
   return ptr[pos*stride];
}

/** 
 * copy a vector using =
 * 
 * @param vec vector to copy
 * 
 * @return reference to this vector
 */

IVecInt &IVecInt::operator=(const IVecInt &vec)
{
   if (&vec == this)
      return *this;
   
   if (n != vec.n)
      resize(vec.n);
   
//   VecCopy(&vec, this);
   int *p_to = ptr;
   int *p_from = vec.ptr;

   for (int i = 0; i < n; i++, p_to+=stride, p_from+=vec.stride)
      *p_to = *p_from;
   
   return *this;
}

void IVecInt::saveXFile(XFile *file)
{
   file->printf("IVecInt: %d\n", n);
   for (int i = 0; i < n; i++)
      file->printf("  %8d\n", ptr[i*stride]);

   return;
}

void IVecInt::loadXFile(XFile *file)
{
   int n_;

   if (file->scanf("IVecInt: %d", &n_) != 1)
      error ("Error reading vector from file!\n");
 
   if (resize(n_) < 0)
      error("Tried reading data into an array that is not big enough and cannot be resized.\n");
  
   for (int i = 0; i < n; i++)
      if (file->scanf("%8d\n", &ptr[i*stride]) != 1)
         error ("Error reading vector from file!\n");
         
   return;

}

