/*
 * bitop.h
 *
 *  Created on: 2019-5-13
 *      Author: zhtro
 */

#ifndef BITOP_H_
#define BITOP_H_

#define BIT(n)                  ( 1<<(n) )

#define BIT_SET(y, mask)        ( y |=  (mask) )
#define BIT_CLEAR(y, mask)      ( y &= ~(mask) )
#define BIT_FLIP(y, mask)       ( y ^=  (mask) )

//! Create a bitmask of length len.
#define BIT_MASK(len)           ( BIT(len)-1 )

//! Create a bitfield mask of length len starting at bit  start.
#define BF_MASK(start, len)     ( BIT_MASK(len)<<(start) )

//! Prepare a bitmask for insertion or combining.
#define BF_PREP(x, start, len)  ( ((x)&BIT_MASK(len)) << (start) )


//! Extract a bitfield of length  len starting at bit  start from  y.
#define BF_GET(y, start, len)   ( ((y)>>(start)) & BIT_MASK(len) )

//! Insert a new bitfield value  x into  y.
#define BF_SET(y, x, start, len)    \
    ( y= ((y) &~ BF_MASK(start, len)) | BF_PREP(x, start, len) )


/* Network Support Macros for "Unix-like" functions */
#ifdef BIGENDIAN
#define htons(a) (a)
#define htonl(a) (a)
#define htonll(a) (a)
#define ntohl(a) (a)
#define ntohs(a) (a)
#define ntohll(a) (a)
#else
#define htons(a) ( (((a)>>8)&0xff) + (((a)<<8)&0xff00) )
/*
 * Fix warning when compiling for IAR (SDOCM00103001):
 *
 *     Warning[Pe061]: integer operation result is out of range
 *
 * This macro has been updated to perform masking operations before shifting.
 * In its previous form (which shifts THEN masks), the IAR compiler generated
 * a warning because it did not like shaving off bits, as it is a (potential)
 * accidental loss of data.  Changing the code to mask first (purposefully
 * losing the data) then shifting afterward fixes the warning.
 *
 * Note that the TI and GCC compilers never cared about this ...
 *
 */
#define htonl(a) ((((a) & 0xff000000) >> 24) | (((a) & 0x00ff0000) >> 8) | \
                  (((a) & 0x0000ff00) << 8)  | (((a) & 0x000000ff) << 24) )

#define htonll(a) ((((uint64_t)htonl(a)) << 32) + htonl((a) >> 32))
inline double htond(double a)
{
	uint64_t temp;

	temp = htonll(*((uint64_t*) (&a)));

	return *((double*)(&temp));
}

inline float htonf(float a)
{
	uint32_t temp;

	temp = htonl(*((uint32_t*) (&a)));

	return *((float*)(&temp));
}

#define ntohl(a) htonl(a)
#define ntohs(a) htons(a)
#define ntohll(a) htonll(a)
#define ntohd(a) htond(a)

#endif

#endif /* BITOP_H_ */
