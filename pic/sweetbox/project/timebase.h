/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol) 
 * 	http://www.vscp.org
 *
 *  File: main.c
 *
 *  The SweetBox
 * 	Version information in version.h
 * 	akhe@eurosource.se
 *
 * Copyright (C) 2008-2011 Ake Hedman, eurosource, <akhe@eurosource.se>
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 
 *	This file is part of VSCP - Very Simple Control Protocol 	
 *	http://www.vscp.org
 *
 * ******************************************************************************
*/

// Implemntion is in main.c

// Why no clear of the 32 bit value?
// This value will roll for about 136 years so instead read the value
// and wait for this value + your delay to delay for a specific amout of time  

#ifndef __TIMEBASE_H__
#define __TIMEBASE_H__

#include <inttypes.h>

/*!
    Clear the 16-bit timebase
*/
void timeBase16Clear( void ) ;


/*!
    Get the 16-bit timebase
    \return 16-bit timebase value.
*/
uint16_t timeBaseGet16( void );


/*!
    Get the 32-bit timebase
    \return 32-bit timebase value.
*/
uint32_t timeBaseGet32( void );



#endif  // __TIMEBASE_H__

