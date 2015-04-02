
#include <torch/Allocator.h>
#include "label_list.hh"

#ifndef MAX_STRLEN
#define MAX_STRLEN 256
#endif

label_list::label_list(int _size) :
      str_list(_size)
{
   beg = (int *)allocator->alloc(sizeof(int) * size);
   end = (int *)allocator->alloc(sizeof(int) * size);
}

label_list::~label_list()
{
}

void label_list::reset()
{
   str_list::reset();
   
   beg = (int *)allocator->alloc(sizeof(int) * size);
   end = (int *)allocator->alloc(sizeof(int) * size);
}

void label_list::resize(int _size)
{
   str_list::resize(_size);
   
   beg = (int *)allocator->realloc(beg, sizeof(int) * size);
   end = (int *)allocator->realloc(end, sizeof(int) * size);
   
   if (beg == NULL || end == NULL)
      error ("No space to allocate new beg/end lists of size %d", size);
}

void label_list::append(char *string, bool retain)
{
   str_list::append(string, retain);
   
   beg[count-1] = end[count-1] = 0;
}

int label_list::set_str(unsigned int pos, char *str, bool retain)
{
   if (pos < 0 || pos >= count)
      return -1;

   allocator->free(strings[pos]);
   if (retain)
      strings[pos] = str;
   else
      strings[pos] = strdup(str);
      
   allocator->retain(str);

   return 0;
}

void label_list::loadXFile(XFile *file, real frame_rate)
{
   int i, scan_count;
   char str[MAX_STRLEN];

   str_list::loadXFile(file);

   for (i = 0; i < count; i++)
   {
      scan_count = sscanf(strings[i], "%d %d %s", &beg[i], &end[i], str);
      
      if (scan_count < 3)
         error("Error determining begin and end values from \"%s\"\n");
      
      strcpy (strings[i], str);
   }

   // fix up begin/end pointers
   
   for (i = 0; i < count-1; i++)
   {
      end[i] = (int)(((real)end[i]/10000000.0*frame_rate)+0.5);
      beg[i+1] = end[i] + 1;
   }
   end[count-1] = (int)((real)end[count-1]/10000000.0*frame_rate);
}


void label_list::print()
{
   int i;
   
   for (i = 0; i < count; i++)
      printf("%10s: %4d to %4d\n", strings[i], beg[i], end[i]);

}

