/**
 * @file   str_list.hh
 * @author Kevin Squire <k-squire@sal.ifp.uiuc.edu>
 * @date   Tue Sep 21 12:32:21 2004
 * 
 * @brief  class definition for a list of strings
 * 
 * 
 */

#ifndef STR_LIST_HH
#define STR_LIST_HH

#include <torch/Allocator.h>
#include <torch/Object.h>
#include <torch/DiskXFile.h>
#include <torch/general.h>

using namespace Torch;

class str_list: public Object
{
protected:
   int size;

public:

   int      count;
   char **strings;

public:

   str_list(int _size = 10);
   ~str_list();

   virtual void reset();
   virtual void resize(int _size);

   virtual void append(char *string, bool retain = true);
   
   virtual int find (char *string);

   int get_num_fields(char *delimiters = " ");

   int split(int after_field, char *delimiters, str_list *new_list);

   char * &operator[](int n) 
   {
      return strings[n];
   }

   virtual void loadXFile(XFile *file);
   
   virtual void print();

   str_list &operator=(const str_list &lst);

};

#endif // STR_LIST_HH
