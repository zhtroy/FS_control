#ifndef _DEBUG_LOG_H
#define _DEBUG_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "logLib.h"

#define DEBUG_GENERAL	(0x00000001)   
#define DEBUG_INFO	    (0x00000002)    
#define DEBUG_WARNING   (0x00000004)
#define DEBUG_ERROR     (0x00000008)

#define FSZX_DEBUG_ERROR

#if defined (FSZX_DEBUG_GENERAL)
#define FSZX_DBG_CURRENT_TYPES ((DEBUG_INFO) | (DEBUG_GENERAL) | (DEBUG_WARNING) | (DEBUG_ERROR))
#elif defined (FSZX_DEBUG_INFO)
#define FSZX_DBG_CURRENT_TYPES ((DEBUG_INFO) | (DEBUG_GENERAL) | (DEBUG_WARNING))
#elif defined (FSZX_DEBUG_WARNING)
#define FSZX_DBG_CURRENT_TYPES ((DEBUG_INFO) | (DEBUG_GENERAL))
#elif defined (FSZX_DEBUG_ERROR)
#define FSZX_DBG_CURRENT_TYPES (DEBUG_ERROR)
#else
#define FSZX_DBG_CURRENT_TYPES 0
#endif

#define FSZX_DEBUG_LOG(type,fmt,...) \
		if (((type) & FSZX_DBG_CURRENT_TYPES))  {LogMsg (fmt,##__VA_ARGS__); }


#ifdef __cplusplus
}
#endif

#endif /* _DEBUG_LOG_H */
