// File:  main.h

/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol) 
 * 	http://www.vscp.org
 *
 * 	Version: See project header
 * 	akhe@eurosource.se
 *
 *  Copyright (C) 1995-2005 Ake Hedman, eurosource
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

#ifndef MAIN_H
#define MAIN_H

#include "inttypes.h"
//Defines
#define	TRUE			1
#define	FALSE			0

//
// Size for array used for temp mean calculations
//
#define TEMPERATURE_ARRAY_SIZE 	0x0A

/* EEPROM Layout
 * FROM -  TO  SIZE(dec)   BLOCKNAME
 * 0000 - 0000     01   Bootloader check
 * 0001 - 0001     01   VSCP Address ID
 * 0002 - 0002     01   Eeprom checksum
 * 0003 - 0038     40   Augusto Driver
 *
 */
#define VSCP_ACTION_NOP            0x00
#define VSCP_ACTION_COMMAND_OUTPUT 0x01

// Function Prototypes
void init_app_ram( void );
void init_app_eeprom( void ); 
void read_app_register( unsigned char reg );
void write_app_register( unsigned char reg, unsigned char val );

#endif
