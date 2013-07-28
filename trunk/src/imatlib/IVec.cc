/**
 * @file   IVec.cc
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Sat Jul 10 00:07:12 2004
 * 
 * @brief  Vector class
 * 
 * $Log: IVec.cc,v $
 * Revision 1.1  2005/05/09 20:53:19  mrmcclai
 * *** empty log message ***
 *
 * Revision 1.7.2.2  2005/02/09 00:41:26  k-squire
 * Added copy constructor from IVecInt
 * Added normalize().
 * Added getInd() to return a vector of values with a set of given indices.
 * Added norm2()
 * Added sort()
 * Added standardize()
 * Added quantize()
 * Fixed delete[].
 *
 * Revision 1.7.2.1  2004/10/26 10:50:36  k-squire
 * Fixed loadXFile to load even if "IVec" is not present.
 * Added sqrt() of vector elements.
 *
 * Revision 1.7  2004/08/10 23:03:06  k-squire
 * Backported changes from illy/matrix
 *
 * Revision 1.1.2.1  2004/08/08 23:07:35  k-squire
 * Moved REAL_FORMAT_* definitions to gen_defines.h.
 * Added sum() function
 *
 * Revision 1.1  2004/08/03 06:39:08  k-squire
 * Import and initial checkin.
 *
 * Revision 1.6  2004/07/30 03:55:50  k-squire
 * Changed file format slightly for ease of editing.
 *
 * Revision 1.5  2004/07/29 07:10:28  k-squire
 * Fixed constructor to initialize variables to 0 first.
 * Added copy constructor.
 * Init always allocates vector now, instead of comparing size first.
 * Renamed max_elem-->vmax.
 * Added vmin.
 * Added saveXFile, loadXFile.
 *
 * Revision 1.4  2004/07/26 17:01:06  k-squire
 * (Re-)Added operator[]
 *
 * Revision 1.3  2004/07/23 20:48:02  k-squire
 * Minor formatting changes.
 * rand() now takes range arguments.
 *
 * Revision 1.2  2004/07/23 17:16:30  k-squire
 * Fixed stride problem in IVec(ptr) constructor.
 *
 * Revision 1.1.1.1  2004/07/23 08:54:20  k-squire
 * Imported using TkCVS
 *
 * 
 */

static char *IVEC_CC_Id = "$Id: IVec.cc,v 1.1 2005/05/09 20:53:19 mrmcclai Exp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <torch/general.h>
#include <torch/Object.h>
#include <torch/Random.h>
#include <torch/XFile.h>

#include "IVec.hh"
#include "IMat.hh"
#include "IMatVecOps.hh"
#include "gen_defines.h"

using namespace Torch;

/** 
 * Empty constructor
 * 
 */

IVec::IVec():
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

IVec::IVec(int n_):
      n(n_),
      real_n(n_),
      stride(1),
      is_alias(false)
{
   ptr = (real *)allocator->alloc(sizeof(real)*n);
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

IVec::IVec(real *ptr_, int n_, int stride_, bool do_copy, bool retain):
      n(0),
      real_n(0),
      stride(1),
      ptr(NULL),
      is_alias(false)
{
   if (do_copy)
   {
      init(n);

      if (stride_ == 1)
         memcpy(ptr, ptr_, n*sizeof(real));
      else
      {
         real *p_to = ptr;
         real *p_from = ptr_;

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

IVec::IVec(IVec &v)
{
   init(v.n);
   VecCopy(&v, this);
}



/** 
 * Destructor
 * 
 */

IVec::~IVec()
{
}

/** 
 * Vector initialization
 * 
 * @param n_ vector length
 * 
 * @return 0 on success, -1 on failure
 */

int IVec::init(int n_)
{
   ptr = (real *)allocator->alloc(sizeof(real)*n_);
   
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

void IVec::fill(real val)
{
   real *p = ptr;
   
   for (int i = 0; i < n; i++, p+=stride)
      *p = val;

}

/** 
 * Zero the vector
 * 
 */

void IVec::zero()
{
   fill(0.0);
}

/** 
 * Fill the vector with ones.
 * 
 */

void IVec::ones()
{
   fill(1.0);
}

/** 
 * Fill the vector with random values.
 * 
 */

void IVec::rand(real a, real b)
{
   real *p = ptr;
   
   for (int i = 0; i < n; i++, p+=stride)
      *p = Random::boundedUniform(a,b);
}

/** 
 * Take the square root of vector elements
 * 
 */

void IVec::sqrt()
{
   real *p = ptr;
   
   for (int i = 0; i < n; i++, p+=stride)
      *p = ::sqrt(*p);
}

int IVec::normalize()
{
   real s = sum();
   
   if (s == 0)
      return -1;

   real *p = ptr;
   
   for (int i = 0; i < n; i++, p+=stride)
      *p = *p / s;

   return 0;
}


/** 
 * Find the maximum element in the vector
 * 
 * @param pos if not NULL, contains the position of the maximum
 * element upon return
 * 
 * @return value of maximum element
 */

real IVec::vmax(int *pos)
{
   real *p = ptr;

   real mv = -INF; // defined in torch/general.h
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

real IVec::vmin(int *pos)
{
   real *p = ptr;

   real mv = INF; // defined in torch/general.h
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
 * Sum the elements in the vector
 * 
 * @return sum the vector
 */

real IVec::sum()
{
   real s;
   real *p = ptr;

   for (int i = 0; i < n; i++, p+=stride)
      s += *p;
   
   return s;
}

/** 
 * Get the indices of the sorted vector
 * 
 * @param pos contains the order of the sorted vector upon return
 *  (note that the vector itself is not modified)
 * 
 * @return value of minimum element
 */

int IVec::sort(IVecInt *pos)
{
   int i;
   
   pos->resize(n);
   
   for (i = 0; i < n; i++)
      (*pos)[i] = i;

   quicksort(pos, 0, n-1);

   return 0;
}

void IVec::quicksort(IVecInt *ind, int p, int r)
{
   int q;

   if (p < r)
   {
      q = partition(ind, p, r);
      quicksort(ind, p, q);
      quicksort(ind, q+1,r);
   }
}

int IVec::partition(IVecInt *ind, int p, int r)
{
   real x = (*this)[(*ind)[p]];
   int i = p-1;
   int j = r+1;

   int tmp;

   while (1)
   {
      do
         j--;
      while ((*this)[(*ind)[j]] > x);
      
      do 
         i++;
      while ((*this)[(*ind)[i]] < x);
      
      if (i < j)
         swap((*ind)[i],(*ind)[j],tmp);
      else
         return j;
   }

}

/**
 * Compute the square of the length of the vector
 *
 * @return sum the vector
 */

real IVec::norm2()
{
   real s;
   real *p = ptr;

   for (int i = 0; i < n; i++, p+=stride)
      s += (*p)*(*p);

   return s;
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

int IVec::ins(unsigned int pos, unsigned int len, real val)
{
   real *new_ptr;
   real *p;
   int i;

   if (pos > n)
      return -1;
   
   if (n+len > real_n)
   {
      // Can't increase size of a matrix row/column alias!

      if (is_alias)
         return -1;

      if ((new_ptr=(real *)allocator->realloc(ptr,(n+len)*sizeof(real)))==NULL)
         return -1;

      ptr = new_ptr;
      real_n=n+len;
   }
   
   if (pos < n)
      memmove(ptr+pos+len, ptr+pos, (n-pos)*sizeof(real));
   
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

int IVec::del(unsigned int pos, unsigned int len)
{
   if (pos+len > n)
      return -1;
   
   if (pos+len < n)
      memmove(ptr+pos, ptr+pos+len, (n-(pos+len))*sizeof(real));
   
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

int IVec::resize(unsigned int n_, bool clear)
{
   real *new_ptr;

   if (n_ > real_n)
   {
      // Can't increase size of an alias!

      if (is_alias)
      {
         error ("Tried to increase the size of an alias!\n");
         return -1;
      }

      // Otherwise... resize the vector to n_

      if ((new_ptr = (real *)allocator->realloc(ptr,(n_)*sizeof(real)))==NULL)
         return -1;
      
      ptr = new_ptr;
      real_n = n_;
   }

   if (clear && n_>n)
      memset(ptr+n, 0, sizeof(real)*(n_-n));

   n=n_;

   return 0;
}

int IVec::getInd(IVec *out, IVecInt *ind)
{
   int i;
   
   out->resize(ind->n);
   
   for (i = 0; i < ind->n; i++)
      (*out)[i] = (*this)[(*ind)[i]];

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

int IVec::set(real *ptr_, int n_, int stride_, bool do_copy, bool retain)
{
   if (do_copy)
   {
      if (resize(n_) != 0)
         return -1;
      
      if (stride == 1 && stride_ == 1)
         memcpy(ptr, ptr_, n*sizeof(real));
      else
      {
         real *p_to = ptr;
         real *p_from = ptr_;
         
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
 * Print the vector
 * 
 * @param scrn_width screen width
 * @param col_width width of vector columns
 * @param num_format format to use for printing numbers
 * 
 * @return 0 on success, -1 on failure
 */

int IVec::print(int scrn_width, int col_width, char *num_format)
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

real &IVec::operator()(int pos)
{
   return ptr[pos*stride];
}

/** 
 * Reference a vector element
 * 
 * 
 * @return reference to vector element
 */

real &IVec::operator[](int pos)
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

IVec &IVec::operator=(const IVec &vec)
{
   if (&vec == this)
      return *this;
   
   if (n != vec.n)
      resize(vec.n);
   
   VecCopy(&vec, this);
   
   return *this;
}


/** 
 * copy an integer vector using =
 * 
 * @param vec vector to copy
 * 
 * @return reference to this vector
 */

IVec &IVec::operator=(IVecInt &vecint)
{
   int i;

   if (n != vecint.n)
      resize(vecint.n);
   
   for (i = 0; i < n; i++)
      (*this)[i] = vecint[i];
   
   return *this;
}

void IVec::standardize(IVec *mu,
                       IVec *sigma)
{
   int j;

   VecSub(mu,this);
   for (j = 0; j < n; j++)
      ptr[j] = ptr[j]/(*sigma)[j];

}

int IVec::quantize(IVecInt *out, IMat *q_levels)
{
   int i, j, bins;
   IVec q_row;

   out->resize(q_levels->m);

   bins = q_levels->n + 1;

   for (i = 0; i < n; i++)
   {
      q_levels->getRow(i, &q_row);
      
      for (j = bins-2; j >= 0; j--)
      {
         if ((*this)[i] >= q_row[j])
         {
            (*out)[i] = j+1;
            break;
         }
      }

      if ((*this)[i] < q_row[0])
         (*out)[i] = 0;
   }
   
   return 0;
}


void IVec::saveXFile(XFile *file)
{
//   file->printf("IVec: %d\n", n);
   file->printf("%d\n", n);
   for (int i = 0; i < n; i++)
      file->printf(REAL_FORMAT_OUT "\n", ptr[i*stride]);

   return;
}

void IVec::loadXFile(XFile *file)
{
   int n_;
   char inp[81];

   if ((file->gets(inp,80)) == NULL)
      error ("Error reading from file!");
   
   if (sscanf(inp, "IVec: %d", &n_) != 1)
      if (sscanf(inp, "%d", &n_) != 1)
         error ("Error: input string \"%s\" does not have vector length!\n",inp);
 
   if (resize(n_) < 0)
      error("Tried reading data into an array that is not big enough and cannot be resized.\n");
  
   for (int i = 0; i < n; i++)
      if (file->scanf(REAL_FORMAT_IN "\n", &ptr[i*stride]) != 1)
         error ("Error reading vector from file!\n");
         
   return;

}


