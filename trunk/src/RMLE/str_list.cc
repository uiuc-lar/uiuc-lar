
#include <torch/Allocator.h>
#include "str_list.hh"

#ifndef MAX_STRLEN
#define MAX_STRLEN 256
#endif

str_list::str_list(int _size) :
      size(_size),
      count(0),
      strings(NULL)
{
   strings = (char **)allocator->alloc(sizeof(char *) * size);

   for (int i = 0; i < size; i++)
      strings[i] = NULL;
}

str_list::~str_list()
{
}

void str_list::reset()
{
   allocator->freeAll();
   
   size = 10;
   count = 0;
   
   strings = (char **)allocator->alloc(sizeof(char *) * size);

   for (int i = 0; i < size; i++)
      strings[i] = NULL;
}

void str_list::resize(int _size)
{
   size = _size;
   
   strings = (char **)allocator->realloc(strings, sizeof(char *) * size);
   
   if (strings == NULL)
      error ("No space to allocate new string list of size %d", size);
}


void str_list::append(char *string, bool retain)
{
   if (retain)
      allocator->retain(string);

   if (count == size)
      resize(size*2);

   strings[count++] = string;
}
   
int str_list::find(char *string)
{
   int i;

   for (i = 0; i < count; i++)
      if (strcmp(strings[i], string) == 0)
         return i;

   return -1;
}

int str_list::get_num_fields(char *delimiters)
{
   if (count == 0)
      return 0;

   char *s = strings[0];
   int num_fields = 1;
   
   while ((s = strpbrk(s, delimiters)) != NULL)
   {
      num_fields++;
      s++;
      while(*s != '\0' && strchr(delimiters, *s) != NULL)
         s++;
      if (*s == '\0')
         num_fields--;
   }

   return num_fields;
}

int str_list::split(int after_field, char *delimiters, str_list *new_list)
{
   char *s, *t;
   int n,i;

   if (new_list == NULL)
      return -1;
   
   if (get_num_fields(delimiters) <= after_field)
      return -1;
   
   new_list->reset();

   for (n = 0; n < count; n++)
   {
      s = strings[n];
      t = s;

      // skip past initial delimiters

      while(*t != '\0' && strchr(delimiters, *t) != NULL)
         t++;

      for (i = 0; i < after_field; i++)
      {
         t = strpbrk(t, delimiters);
         if (t == NULL)
            break;
         s = t;
         t++;

         // skip past multiple delimiters

         while(*t != '\0' && strchr(delimiters, *t) != NULL)
            t++;
      }

      new_list->append(strdup(t));

      // cut off the split string

      if (s != NULL)
         *s = '\0';
   }
   
   return 0;
}

void str_list::loadXFile(XFile *file)
{
   int len;

   char line[MAX_STRLEN];

   reset();

   while (1)
   {
     tryagain:
      if (file->gets(line,MAX_STRLEN) == NULL)
         break;

      len = strlen(line);

      if (len <= 1)
         goto tryagain;
      else
         line[len-1] = '\0';  // remove new line
      
      append(strdup(line));
   }
   
   return;
}


str_list &str_list::operator=(const str_list &lst)
{
   allocator->freeAll();
   
   size = lst.size;
   count = lst.count;

   strings = (char **)allocator->alloc(sizeof(char *) * size);

   for (int i = 0; i < size; i++)
   {
      if (lst.strings[i])
      {
         strings[i] = strdup(lst.strings[i]);
         allocator->retain(strings[i]);
      }
      else
         strings[i] = NULL;
      
   }

}

void str_list::print()
{
   int i;
   
   for (i = 0; i < count; i++)
      printf("%s\n", strings[i]);

}

