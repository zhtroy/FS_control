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
char  *m_array_strcpy(char 		*__restrict dest, const char *__restrict 	src, int start, int end)
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

int   *m_array_intcpy(int 		*__restrict dest, const int *__restrict 	src, int start, int end)
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

float *m_array_floatcpy(float 	*__restrict dest, const float *__restrict 	src, int start, int end)
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

unsigned char  *m_array_ustrcpy(unsigned char 		*__restrict dest, const unsigned char *__restrict 	src, int start, int end)
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

unsigned int   *m_array_uintcpy(unsigned int 		*__restrict dest, const unsigned int *__restrict 	src, int start, int end)
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
int   m_array_strcmp(const char 	*string1, const char 	*string2, int start, int end)
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

int   m_array_intcmp(const int 		*int1, const int 	*int2, int start, int end)
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

int   m_array_floatcmp(const float 	*float1, const float 	*float2, int start, int end)
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

int   m_array_ustrcmp(const unsigned char 	*string1, const unsigned char 	*string2, int start, int end)
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

int   m_array_uintcmp(const unsigned int 		*int1, const unsigned int 	*int2, int start, int end)
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

