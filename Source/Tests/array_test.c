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

void  test_array(void)
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

#if 0
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

#if 0
	  int a[10] = {0};
	  int b[10] = {1,2,3,4,5,6,7,8};
//	  m_array_strcpy(a, b, 0, 7);

	  m_array_intcpy(a, b, 2, 6);

	  m_array_floatcpy(a, b, 3, 8);

#endif

#if 0
	  unsigned char a[6] = {'a', 'b', 'c', 'd', 'e'};
	  unsigned char b[6] = {'a', 'b', 'e', 'd', 'e'};
	  unsigned char c[6] = {0};

//	  int a[6] = {12,23,34,45,56,67};
//	  int b[6] = {12,23,34,45,34,78};
//	  int a[6] = {12,23,34,45,56,67};
//	  int b[6] = {12,23,34,45,34,78};
//	  int c[6] = {0};
	  int tmp = 2;
//	  tmp = m_array_intcmp(a, b, 2, 5);
	  tmp = m_array_ustrcmp(a, b, 4, 5);
//	  tmp = m_array_floatcmp(a, b, 1, 3);
	  tmp = m_array_ustrcmp(a, b, 1, 2);
	  tmp = m_array_ustrcmp(a, b, 3, 5);
//	  tmp = m_array_intcmp(a, b, 2, 6);

//	  tmp = m_array_uintcmp(a, b, 2, 5);
//	  tmp = m_array_ustrcmp(a, b, 4, 5);

	  m_array_ustrcpy(c, b, 1, 7);
	  m_array_ustrcpy(c, b, 3, 4);
	  m_array_ustrcpy(c, b, 2, 5);
//	  m_array_intcpy(c, b, 2, 6);
#endif

#if 1
//	  char b = 0;
//	  char a[20] = {12,23,34,45,56,67,78,89,90,1,2,3,4,5,6,7,8,9,10,11};
//	  b = m_array_strquery(a, 56, 0);
//	  b = m_array_strquery(a, 1, 0);
//	  b = m_array_strquery(a, 11, 2);
//	  b = m_array_strquery(a, 23, 0);
//	  b = m_array_strquery(a, 56, 1);
//	  b = m_array_strquery(a, 0, 12);
//	  b = m_array_strquery(a, 0, 7);
//	  b = m_array_strquery(a, 0, 3);
//	  b = m_array_strquery(a, 1, 3);

//	  int b = 0;
	  char b = '0';
//	  float b = 0.0;
//	  int a[20] = {12,23,34,45,56,67,78,89,90,1,2,3,4,5,6,7,8,9,10,11};
//	  char a[20] = {'a', 'b', 'c', 'd', 'e','a', 'b', 'c', 'd', 'e','a', 'b', 'c', 'd', 'e','a', 'b', 'c', 'd', 'e'};
	  float a[20] = {12,23,34,45,56,67,78,89,90,1,2,3,4,5,6,7,8,9,10,11};
//	  int c[30] = {0};
	  int size = sizeof(a)/sizeof(a[0]);
//	  b = m_array_intquery(a, size, 34, 0);
//	  b = m_array_intquery(a, 67, 0);
//	  b = m_array_intquery(a, 90, 0);
//	  b = m_array_intquery(a, 0, 6);
//	  b = m_array_intquery(a, 0, 10);
//	  b = m_array_intquery(a, size, 0, 5);
//	  b = m_array_intquery(a, size, 3, 3);
//	  b = m_array_intquery(a, size, 0, 90);
//	  b = m_array_intquery(a, size, 0, 9);
//	  b = m_array_intquery(a, size, 0, 21);
//	  b = m_array_intquery(a, size, 0, 22);
//	  b = m_array_intquery(a, size, 0, 23);
	  b = m_array_strquery(a, size, 0, 'd');
	  b = m_array_strquery(a, size, 'd', 0);
	  b = m_array_strquery(a, size, 0, 6);
	  b = m_array_strquery(a, size, 0, 7);
	  b = m_array_strquery(a, size, 'c', 2);
#endif
}

