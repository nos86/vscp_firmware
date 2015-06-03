/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol) 
 * 	http://www.vscp.org
 *
 *  Copyright (C) 1995-2006 Ake Hedman, eurosource, <akhe@eurosource.se>
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

#include <xc.h>

#define _XTAL_FREQ 32000000

#define APP_VARIANT       1
#define APP_VERSION       1
#define APP_SUB_VERSION   0

#define DEVICE_CODE 0x00000001
#define DEVICE_TYPE 0x00000001
#define APP_EEPROM_SIZE 0x100

#define APP_BOOTLOADER VSCP_BOOTLOADER_NONE
#define APP_REG_PAGES 1

#define PIN_OUT_SIZE 8
#define PIN_IN_SIZE  8

#define VSCP_DM_COUNT       16  //Number of Decision Matrix row
#define VSCP_REG_DM_OFFSET  0x00
#define VSCP_REG_DM_PAGE    0x0001
