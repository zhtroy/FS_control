/*
 * m-array.c
 *
 *  Created on:   2019-12-2
 *      Author:   FS00000
 *      function: 实现数组的拷贝、查询、比较
 */

#include <string.h>
#include "m-array.h"
#include "m-core.h"

//数组的拷贝
char  *m_array_strcpy(char 		*__restrict dest, const char *__restrict 	src, int size, int start, int end)
{
	if((start < 0) || (end < 0) || (start == end))
	{
		return 0;
	}

	int tmp = (end-start);
	for(int i = start; i < end; i++)
	{
		dest[i-start] = src[i];
		if(tmp-- == 0)
			break;
	}

	return dest;
}

int   *m_array_intcpy(int 		*__restrict dest, const int *__restrict 	src, int size, int start, int end)
{
	if((start < 0) || (end < 0) || (start == end))
	{
		return 0;
	}

	int tmp = (end-start);
	for(int i = start; i < end; i++)
	{
		dest[i-start] = src[i];
		if(tmp-- == 0)
			break;
	}

	return dest;
}

float *m_array_floatcpy(float 	*__restrict dest, const float *__restrict 	src, int size, int start, int end)
{
	if((start < 0) || (end < 0) || (start == end))
	{
		return 0;
	}

	int tmp = (end-start);
	for(int i = start; i < end; i++)
	{
		dest[i-start] = src[i];
		if(tmp-- == 0)
			break;
	}

	return dest;
}

unsigned char  *m_array_ustrcpy(unsigned char 		*__restrict dest, const unsigned char *__restrict 	src, int size, int start, int end)
{
	if((end == 0) || (start == end))
	{
		return 0;
	}

	int tmp = (end-start);
	for(int i = start; i < end; i++)
	{
		dest[i-start] = src[i];
		if(tmp-- == 0)
			break;
	}

	return dest;
}

unsigned int   *m_array_uintcpy(unsigned int 		*__restrict dest, const unsigned int *__restrict 	src, int size, int start, int end)
{
	if((end == 0) || (start == end))
	{
		return 0;
	}

	int tmp = (end-start);
	for(int i = start; i < end; i++)
	{
		dest[i-start] = src[i];
		if(tmp-- == 0)
			break;
	}

	return dest;
}

//数组的比较
int   m_array_strcmp(const char 	*string1, const char 	*string2, int size, int start, int end)
{
	if((start < 0) || (end < 0) || (start == end))
	{
		return 0;
	}

	int temp = 2;
	int tmp = (end-start);

	for(int i = start; i < end; i++)
	{
		if(string1[i] != string2[i])
		{
			temp = (string1[i] > string2[i]) ? 1 : -1 ;
			break;
		}
		else
		{
			int count = 0;
			if(string1[i] == string2[i])
			{
				if(count++ == tmp--)
				{
					temp = 0;
					break;
				}
			}
		}
	}

	return temp;
}

int   m_array_intcmp(const int 		*int1, const int 	*int2, int size, int start, int end)
{
	if((start < 0) || (end < 0) || (start == end))
	{
		return 0;
	}

	int temp = 2;
	int tmp = (end-start);

	for(int i = start; i < end; i++)
	{
		if(int1[i] != int2[i])
		{
			temp = (int1[i] > int2[i]) ? 1 : -1 ;
			break;
		}
		else
		{
			int count = 0;
			if(int1[i] == int2[i])
			{
				if(count++ == tmp--)
				{
					temp = 0;
					break;
				}
			}
		}
	}

	return temp;
}

int   m_array_floatcmp(const float 	*float1, const float 	*float2, int size, int start, int end)
{
	if((start < 0) || (end < 0) || (start == end))
	{
		return 0;
	}

	int temp = 2;
	int tmp = (end-start);

	for(int i = start; i < end; i++)
	{
		if(float1[i] != float2[i])
		{
			temp = (float1[i] > float2[i]) ? 1 : -1 ;
			break;
		}
		else
		{
			int count = 0;
			if(float1[i] == float2[i])
			{
				if(count++ == tmp--)
				{
					temp = 0;
					break;
				}
			}
		}
	}

	return temp;
}

int   m_array_ustrcmp(const unsigned char 	*string1, const unsigned char 	*string2, int size, int start, int end)
{
	if((start < 0) || (end < 0) || (start == end))
	{
		return 0;
	}

	int temp = 2;
	int tmp = (end-start);

	for(int i = start; i < end; i++)
	{
		if(string1[i] != string2[i])
		{
			temp = (string1[i] > string2[i]) ? 1 : -1 ;
			break;
		}
		else
		{
			int count = 0;
			if(string1[i] == string2[i])
			{
				if(count++ == tmp--)
				{
					temp = 0;
					break;
				}
			}
		}
	}

	return temp;
}

int   m_array_uintcmp(const unsigned int 		*int1, const unsigned int 	*int2, int size, int start, int end)
{
	if((start < 0) || (end < 0) || (start == end))
	{
		return 0;
	}

	int temp = 2;
	int tmp = (end-start);

	for(int i = start; i < end; i++)
	{
		if(int1[i] != int2[i])
		{
			temp = (int1[i] > int2[i]) ? 1 : -1 ;
			break;
		}
		else
		{
			int count = 0;
			if(int1[i] == int2[i])
			{
				if(count++ == tmp--)
				{
					temp = 0;
					break;
				}
			}
		}
	}

	return temp;
}


//数组的查询
/******如果数据有重复性，该如何处理呢？*******/
char   m_array_strquery(char *array, int size, char value, int index)
{
	char temp = -2;
	if(size > 0)
	{
		if((NULL == array) || (size < 0))
		{
			return -1;
		}

		int i = 0;
		if(value > 0)
		{
			while( (array[i++] != value) && (i < size) );
			if(i < size)
			{
				temp = i;
			}
			else
			{
				return -1;
			}
		}
		else if(value == 0)
		{
			;
		}
		else
		{
			return -1;
		}

		if((index > 0) && (index < size) && (temp == -2))
		{
			temp = array[index-1];
		}
		else if(0 == index)
		{
			;
		}
		else
		{
			return -1;
		}
	}
	else
	{
		return -1;
	}

	return temp;
}

int   m_array_intquery(int *array, int size, int  value, int index)
{
	int temp = -2;
	if(size > 0)
	{
		if(NULL == array)
		{
			return -1;
		}

		int i = 0;
		if(value > 0)
		{
			while( (array[i++] != value) || (i > size) );
			if(i < size)
			{
				temp = i;
			}
			else
			{
				return -1;
			}
		}
		else if(value == 0)
		{
			;
		}
		else
		{
			return -1;
		}

		if((index > 0) && (index < size) && (temp == -2))
		{
			temp = array[index-1];
		}
		else if(0 == index)
		{
			;
		}
		else
		{
			return -1;
		}
	}
	else
	{
		return -1;
	}

	return temp;
}

float   m_array_floatquery(float *array, int size, float value, int index)
{
	int temp = -2;
	if(size > 0)
	{
		if(NULL == array)
		{
			return -1;
		}

		int i = 0;
		if(value > 0)
		{
			while( (array[i++] != value) || (i > size) );
			if(i < size)
			{
				temp = i;
			}
			else
			{
				return -1;
			}
		}
		else if(value == 0)
		{
			;
		}
		else
		{
			return -1;
		}

		if((index > 0) && (index < size) && (temp == -2))
		{
			temp = array[index-1];
		}
		else if(0 == index)
		{
			;
		}
		else
		{
			return -1;
		}
	}
	else
	{
		return -1;
	}

	return temp;
}

char   m_array_ustrquery(unsigned char *array, int size, unsigned char value, int index)
{
	int temp = -2;
	if(size > 0)
	{
		if(NULL == array)
		{
			return -1;
		}

		int i = 0;
		if(value > 0)
		{
			while( (array[i++] != value) || (i > size) );
			if(i < size)
			{
				temp = i;
			}
			else
			{
				return -1;
			}
		}
		else if(value == 0)
		{
			;
		}
		else
		{
			return -1;
		}

		if((index > 0) && (index < size) && (temp == -2))
		{
			temp = array[index-1];
		}
		else if(0 == index)
		{
			;
		}
		else
		{
			return -1;
		}
	}
	else
	{
		return -1;
	}

	return temp;
}

int   m_array_uintquery(unsigned int *array, int size, unsigned int value, int index)
{
	int temp = -2;
	if(size > 0)
	{
		if(NULL == array)
		{
			return -1;
		}

		int i = 0;
		if(value > 0)
		{
			while( (array[i++] != value) || (i > size) );
			if(i < size)
			{
				temp = i;
			}
			else
			{
				return -1;
			}
		}
		else if(value == 0)
		{
			;
		}
		else
		{
			return -1;
		}

		if((index > 0) && (index < size) && (temp == -2))
		{
			temp = array[index-1];
		}
		else if(0 == index)
		{
			;
		}
		else
		{
			return -1;
		}
	}
	else
	{
		return -1;
	}

	return temp;
}

