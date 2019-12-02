/*
 * array_test.c
 *
 *  Created on: 2019-11-28
 *      Author: FS00000
 */
#include <stdlib.h>
#include "Lib/m-array.h"
#include "Lib/m-algo.h"

ARRAY_DEF(r_int2, signed int, M_DEFAULT_OPLIST)

ARRAY_DEF (array_charp, char *)
#define M_OPL_array_charp_t() ARRAY_OPLIST(array_charp)

ARRAY_DEF(array_int, int)
const int n = 6;

void  array_test(void)
{
//	int n = 6;

#if 0
	  r_int2_t array;

	  r_int2_init(array);
	  //依次写入
	  int a[6] = {12,34,45,56,67,78};
	  for(int i = 0; i < n; i++) {
	    r_int2_push_back(array, a[i]);
	  }
	  //依次相加
	  int s = 0;
	  int size = r_int2_size(array);
	  for(int i = 0; i < n; i++) {
	    s += *r_int2_get(array, i);
	  }
	  //前序
	  r_int2_it_t itr;
	  for ( r_int2_it(itr, array) ; ! r_int2_end_p (itr);  r_int2_next(itr)) {
	    // Let's get a reference to the data pointed by the iterator.
	    // You'll always get a pointer to this data, not the data itself.
	    // So you need an extra '*' to read the data.
	    int element = *r_int2_ref(itr);
	    printf ("%d ", element);
	  }
	  printf("\n");
	  //倒序
	  printf("Array Direct: ");
	  for (r_int2_previous (itr) ; !r_int2_end_p (itr); r_int2_previous(itr)) {
	    int ele = *r_int2_ref(itr);
	    printf ("%d ", ele);
	  }
	  printf("\n");

	  r_int2_clear(array);
#endif

#if 1
	  M_LET (al, array_charp_t)
	  {
	    // add elements to the array list
	    array_charp_push_back (al,"C");
	    array_charp_push_back (al,"A");
	    array_charp_push_back (al,"E");
	    array_charp_push_back (al,"B");
	    array_charp_push_back (al,"D");
	    array_charp_push_back (al,"F");

	    // Display contents of al
	    printf("Original contents of al: ");
	    // M_EACH is a macro which iterates over a container.
	    // It first argument is the name of the created variable
	    // which will be a pointer to one element of the container,
	    // al is the container to iterate,
	    // the last argument is either the type of the container
	    // (if a global oplist has been registered) or is the oplist
	    // of the container (if none).
	    // Indeed the macro needs to access the methods of the container
	    // to iterate over the container.
	    // So for each element of the array al
	    for M_EACH(element, al, array_charp_t) {
	        printf ("%s ", *element);
	    }// After this point al is automatically cleared
	    printf("\n");
	  }
#endif

#if 0
	  // Declare and initilize the array
	  array_int_t x;
	  array_int_init(x);

	  int a[6] = {12,34,45,56,67,78};
	  // Push some integers in the array
	  for (int i = 0; i < n; i++) {
	    array_int_push_back(x, a[i]);
	  }
	  int s[12] = {0};
	  for(int i = 0; i < n; i++) {
	    s[i] = *array_int_get(x, i);
	    printf ("%d ", s[i]);
	  }

	  // Insert some integers in the array.
	  //直接在数组后面添加元素
	  for (int i = 0; i < n; i++) {
	    array_int_push_at (x, i, a[i]);
	  }
	  for(int i = 6; i < n+6; i++) {
	    s[i] = *array_int_get(x, i);
	    printf ("%d ", s[i]);
	  }

	  // Pop some integers from integer into NULL,
	  // i.e. erase them.
	  for (int i = 0; i < n; i++) {
	    array_int_pop_at (NULL, x, i);
	  }
	  int ss[6] = {0};
	  for(int i = 0; i < n; i++) {
	    ss[i] = *array_int_get(x, i);
	    printf ("%d ", ss[i]);
	  }

	  // Clear the array
	  array_int_clear(x);
#endif
}

