//========================================================================================
//  Copyright Max Fierke (TeenDev)
//  http://www.maxfierke.com
//
//  License: GPLv2
//========================================================================================

#ifndef __BOOTLIB_H
#define __BOOTLIB_H

#include <nds.h>


#ifdef __cplusplus
extern "C" {
#endif

#define LOAD_DEFAULT_NDS 0
#define BOOTLIB_VERSION "1.3b"

//====================================
#ifdef ARM9
//====================================
	void bootnds(const char *filename);
#endif


//====================================
#ifdef ARM7
//====================================
	void bootndsCheck(void);

#endif


#ifdef __cplusplus
}
#endif

#endif
