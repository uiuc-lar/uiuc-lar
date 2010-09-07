/**
 * @file   label_list.hh
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Tue Sep 21 12:32:21 2004
 * 
 * @brief  class definition for a list of strings
 * 
 * 
 */

#ifndef LABEL_LIST_HH
#define LABEL_LIST_HH

#include <torch/Allocator.h>
#include <torch/Object.h>
#include <torch/DiskXFile.h>
#include <torch/general.h>

#include "str_list.hh"

using namespace Torch;

class label_list: public str_list
{
public:
//   int size;
//   int      count;
//   char **strings;

   int *beg;
   int *end;

public:

   label_list(int _size = 10);
   ~label_list();

   virtual void reset();
   virtual void resize(int _size);

   virtual void append(char *string, bool retain = true);

   int set_str(unsigned int pos, char *str, bool retain=true);

   virtual void loadXFile(XFile *file, real sample_ratio=0.0);
   
   virtual void print();

//   label_list &operator=(const label_list &lst);

};

#endif // LABEL_LIST_HH
