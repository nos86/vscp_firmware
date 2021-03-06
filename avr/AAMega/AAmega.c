/* This is an modification of the demo_vscp_node_can128, credits to Akhe
 * see original header below this file
 * goal is to implement a module for home automation including:
 * - 16 output (on/off)	
 * - bootloader support
 *
 * hardware supported:
 * custom board, AAmega 0.0
 *  
 * 
 * version 0.0.1
 * 29/08/2011
 *
 * Sven Zwiers
 *
 * more information about VSCP: http://www.vscp.org
 *---------------------------------------------------------------------------
*/


#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "methods.h"
#include <stdio.h>
#include <string.h>
#include "AAmega.h"
#include "hardware.h"
#include "vscp_projdefs.h"
#include "vscp_compiler.h"
#include "version.h"
#include "can_at90can128.h"
#include "uart.h"
#include "vscp_firmware.h"
#include "vscp_class.h"
#include "vscp_type.h"
#include "vscp_registers.h"
#include "vscp_actions.c"
#include "bootsupport.c"

#ifndef GUID_IN_EEPROM
// GUID is stored in ROM for this module
//		IMPORTANT!!!
//		if using this GUID in a real environment
//		please set the four MSB to a unique id.
//
// IMPORTANT!!!!!
// The GUID is stored MSB byte first
const uint8_t GUID[ 16 ] = {
	0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x03, 0x00, 0x04, 0x00, 0x00, 0x13
};
#endif
#include "methods.c"


// Device string is stored in ROM for this module (max 32 bytes)
const uint8_t vscp_deviceURL[]  = "127.0.0.1/mdf/aamega00.xml";


// Manufacturer id's is stored in ROM for this module
// offset 0 - Manufacturer device id 0
// offset 1 - Manufacturer device id 1
// offset 2 - Manufacturer device id 2
// offset 3 - Manufacturer device id 3
// offset 4 - Manufacturer subdevice id 0
// offset 5 - Manufacturer subdevice id 1
// offset 6 - Manufacturer subdevice id 2
// offset 7 - Manufacturer subdevice id 3
const uint8_t vscp_manufacturer_id[8] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x11
};


// Variables
volatile uint16_t measurement_clock;	// 1 ms timer counter

//int16_t btncnt[ 1 ];    // Switch counters not used now

// Prototypes
static void initTimer();
static void init_app_eeprom( void );
static void doDM( void );
static void doWork();


///////////////////////////////////////////////////////////////////////////////
// Timer 0 Compare interupt
//
// We should come to this point once every millisecond
//

SIGNAL( SIG_OUTPUT_COMPARE0 )
{
	// handle millisecond counter
	vscp_timer++;
	measurement_clock++;

	// Check for init button
	if ( BTN_INIT_PRESSED ) {
		// Active
		vscp_initbtncnt++;
	}
	else {
		vscp_initbtncnt = 0;
	}
   /* // Check for Switch 1 not used now
	if ( BTN_SW1_PRESSED ) {
		// Active
		btncnt[1]++;
	}*/
	// Status LED
	vscp_statuscnt++;
	if ( ( VSCP_LED_BLINK1 == vscp_initledfunc ) && ( vscp_statuscnt > 100 ) ) {
		LED_STATUS_TOGGLE;
		vscp_statuscnt = 0;
	}
	else if ( VSCP_LED_ON == vscp_initledfunc ) {
		LED_STATUS_ON;
		vscp_statuscnt = 0;
	}
	else if ( VSCP_LED_OFF == vscp_initledfunc ) {
		LED_STATUS_OFF;
		vscp_statuscnt = 0;
	}

}

///////////////////////////////////////////////////////////////////////////////
//  initTimer
//

static void initTimer()
{
    // Sets the compare value
	// OCR0A = fclk/(2nf) - 1 = fclk/512000 - 1
	#ifndef FOSC
	#error You must define FOSC in yor makefile
	#elif FOSC == 8000
	OCR0A = 32;
	#elif FOSC == 16000
	OCR0A = 61;
	#endif
	// Set Clear on Timer Compare (CTC) mode, CLK/256 prescaler
	TCCR0A = ( 1 << WGM01 ) | ( 0 << WGM00 ) | ( 4 << CS00 );
	// Enable timer0 compare interrupt
	TIMSK0 = ( 1 << OCIE0A );
}

///////////////////////////////////////////////////////////////////////////////
// main
//
int main( void )

{
	unsigned int lastoutput, currentoutput; //detection change on output

	ini_hardware();
    
    // Initialize UART
    UCSRA = 0;
    UCSRC = MSK_UART_8BIT;	// 8N1
    Uart_set_baudrate( 9600 );
     
    UCSRB = MSK_UART_ENABLE_TX | MSK_UART_ENABLE_RX;

	//check button S1 --> selftest
	if BTN_SW1_PRESSED
	{
		#ifdef PRINT_CAN_EVENTS
			uart_puts( "AAMEGA selftest\n" );
		#endif
		outputport1 = 254;
		unsigned long t =0;
		LED_IND_ON;
		LED_STATUS_OFF;
		while (outputport1 != 0)
		{
			t=0;
			LED_IND_TOGGLE;
			LED_STATUS_TOGGLE;
			while (t < 650000) 
			{
				t++;
				asm ("NOP");
			}
			outputport1 = (outputport1 <<1);
		}
		outputport2 = 254;
		while (outputport2 != 0)
		{
			t=0;
			LED_IND_TOGGLE;
			while (t < 650000) 
			{
				t++;
				asm ("NOP");
			}
			outputport2 = (outputport2 >>1);
		}
		ini_hardware();
	}	
    // Initialize the 1ms timer
    initTimer();

    sei(); // Enable interrupts


    // Init can
    if ( ERROR_OK != can_Open( CAN_BITRATE_125K, 0, 0, 0 ) ) {
		#ifdef PRINT_CAN_EVENTS
        	uart_puts("Failed to open channel!!\n");
		#endif
    }

	#ifdef PRINT_CAN_EVENTS
		uart_puts( "AAMEGA 0.1\n" );
	#endif


	// Check VSCP persistent storage and
	// restore if needed
	if ( !vscp_check_pstorage() ) 
	{
		// Spoiled or not initialized - reinitialize
		init_app_eeprom();
	}


	// Initialize the VSCP functionality
	vscp_init();

	//initialize output change detection
	lastoutput = (((bitflip(read_output2))<<8)&0xFF00) | read_output1 ;

	//read default values for output ports
	outputport1 = ~readEEPROM(VSCP_EEPROM_REGISTER + REG_OUTPUT_DEFAULT_STATE1);
	outputport2 = ~bitflip(readEEPROM(VSCP_EEPROM_REGISTER + REG_OUTPUT_DEFAULT_STATE2));

    while ( 1 ) {  // Loop Forever


        if ( ( vscp_initbtncnt > 200 ) && ( VSCP_STATE_INIT != vscp_node_state ) ) 
		{
	        // State 0 button pressed
            vscp_nickname = VSCP_ADDRESS_FREE;
            writeEEPROM( VSCP_EEPROM_NICKNAME, VSCP_ADDRESS_FREE );
            vscp_init();
			#ifdef PRINT_CAN_EVENTS
            uart_puts("Node initialization");
			#endif
        }

        // Check for any valid CAN message
        vscp_imsg.flags = 0;
        vscp_getEvent();

        // do a work if it's time for it
        if ( measurement_clock > 1000 ) 
		{
            measurement_clock = 0;
            // Do VSCP one second jobs 
            vscp_doOneSecondWork();
			LED_IND_TOGGLE; // toggle indicator LED every second (hartbeat signal)
        }

        switch ( vscp_node_state ) 
		{
		    case VSCP_STATE_STARTUP:        // Cold/warm reset

                // Get nickname from EEPROM
                if ( VSCP_ADDRESS_FREE == vscp_nickname ) 
				{
                    // new on segment need a nickname
                    vscp_node_state = VSCP_STATE_INIT; 	
                }
                else 
				{
                    // been here before - go on
                    vscp_node_state = VSCP_STATE_ACTIVE;
                    vscp_goActiveState();
                }
                break;


            case VSCP_STATE_INIT:           // Assigning nickname
                vscp_handleProbeState();
                break;

            case VSCP_STATE_PREACTIVE:      // Waiting for host initialisation
                vscp_goActiveState();					
                break;


            case VSCP_STATE_ACTIVE:         // The normal state
                if ( vscp_imsg.flags & VSCP_VALID_MSG ) 
				{	// incoming message?
					#ifdef PRINT_CAN_EVENTS
	                    char buf[30];
	                    uint8_t i;
	                    sprintf(buf, "rx: %03x/%02x/%02x/",
	                    vscp_imsg.vscp_class, vscp_imsg.vscp_type, vscp_imsg.oaddr);
	                    for (i=0; i<(vscp_imsg.flags&0xf); i++) 
						{
	                        char dbuf[5];
	                        sprintf(dbuf, "/%02x", vscp_imsg.data[i]);
	                        strcat(buf, dbuf);
	                    }
	                    uart_puts(buf);
					#endif
                    vscp_handleProtocolEvent();
					
					doDM();					// run message through DM
                }
                break;


            case VSCP_STATE_ERROR:          // Everything is *very* *very* bad.
                vscp_error();
                break;

            default:                        // Should not be here...
                vscp_node_state = VSCP_STATE_STARTUP;
                break;

        } // switch 



		//check outputs and if change detected > send out event
        currentoutput = (((bitflip(read_output2))<<8)&0xFF00) | read_output1 ;
		if (currentoutput != lastoutput) vscp_outputevent(currentoutput,lastoutput);
		lastoutput = currentoutput;	    


        doWork();

    } // while
}


///////////////////////////////////////////////////////////////////////////////
// init_app_eeprom
// should  only be performed after first startup!
// all registers cleared!

static void init_app_eeprom( void )
{
    uint8_t pos;
    for (pos = VSCP_EEPROM_REGISTER ; pos <= REG_END ; pos++)
	{
	    writeEEPROM(pos, 0x00 );
	}
}



///////////////////////////////////////////////////////////////////////////////
//   						VSCP Functions
//							--------------
//
// All methods below must be implemented to handle VSCP functionality
//
///////////////////////////////////////////////////////////////////////////////
// common routines moved to avr\common\methods.c
// only routines that are hardware dependant are left here
///////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////
//                        VSCP Required Methods
//////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// vscp_readAppReg
//

uint8_t vscp_readAppReg( uint8_t reg )
{
    uint8_t rv;


	if ((reg >= REG_ZONE) & (reg <= REG_ZONE_END))
	{
		rv =  readEEPROM( reg + VSCP_EEPROM_REGISTER);
	}



    // DM register space    
    else if ( ( reg >= REG_DM_START ) & ( reg <  REG_DM_START + (DESCION_MATRIX_ELEMENTS * 8) ) )
	{
        rv =  readEEPROM( VSCP_EEPROM_REGISTER +  reg  );
    }


    else {
        rv = 0x00;
    }

   // Read actual output state
   if ( REG_OUTPUT_STATE1 == reg ) rv =  ~read_output1;
   if ( REG_OUTPUT_STATE2 == reg ) rv =  ~bitflip(read_output2);

    return rv;

}

///////////////////////////////////////////////////////////////////////////////
// vscp_writeAppReg
//

uint8_t vscp_writeAppReg( uint8_t reg, uint8_t val )
{
    uint8_t rv;

    rv = ~val; // error return

	if ((reg >= REG_ZONE) & (reg <= REG_ZONE_END))
	{
		writeEEPROM(VSCP_EEPROM_REGISTER + reg, val); 
		rv =  readEEPROM( reg + VSCP_EEPROM_REGISTER);
	}

    // DM register space    
    else if ( ( reg >= REG_DM_START ) & ( reg <= REG_DM_START + (DESCION_MATRIX_ELEMENTS * 8) ) )
	{
        writeEEPROM(VSCP_EEPROM_REGISTER + reg, val); 
		rv =  readEEPROM( VSCP_EEPROM_REGISTER +  reg  );
    }

   	// write actual output state
   	if ( REG_OUTPUT_STATE1 == reg ) 
   	{
   		outputport1 = ~val;
		rv =  ~read_output1;
	}
   
   	if ( REG_OUTPUT_STATE2 == reg )
	{
		outputport2 = ~bitflip(val);
		rv =  ~bitflip(read_output2);
	}



    else {
        rv = ~val; // error return	
    }


    return rv;	

}







///////////////////////////////////////////////////////////////////////////////
//                       Implemention of Decision Matrix
//////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////
// Do decision Matrix handling
//
// The routine expects vscp_imsg to contain a valid incoming event
//

static void doDM( void )
{
	uint8_t i;
    uint8_t dmflags;
    uint16_t class_filter;
    uint16_t class_mask;
    uint8_t type_filter;
    uint8_t type_mask;
	#ifdef PRINT_CAN_EVENTS
   	char buf[30];
	#endif

  
    // Don't deal with the control functionality
    if ( VSCP_CLASS1_PROTOCOL == vscp_imsg.vscp_class ) return;
	#ifdef PRINT_CAN_EVENTS
	uart_puts( "debug  doDM\n" );
    #endif
    for ( i=0; i<DESCION_MATRIX_ELEMENTS; i++ ) {
        // Get DM flags for this row
        dmflags = readEEPROM( VSCP_EEPROM_REGISTER + REG_DM_START + ( VSCP_SIZE_STD_DM_ROW * i ) +VSCP_DM_POS_FLAGS);
		#ifdef PRINT_CAN_EVENTS
		sprintf(buf, "check row:%i", i);
		uart_puts(buf);
		sprintf(buf, "DM: %02x", dmflags);
		uart_puts(buf);

		#endif
        // Is the DM row enabled?
        if ( dmflags & VSCP_DM_FLAG_ENABLED ) {
			#ifdef PRINT_CAN_EVENTS
			uart_puts( "debug  doDM row enabled\n" );
    		#endif
            // Should the originating id be checked and if so is it the same?
            if ( !( dmflags & VSCP_DM_FLAG_CHECK_OADDR ) &&
                    !(  vscp_imsg.oaddr == 
                        readEEPROM( VSCP_EEPROM_REGISTER + REG_DM_START + 
                                        ( VSCP_SIZE_STD_DM_ROW * i ) + VSCP_DM_POS_OADDR ) ) ) {
                continue;					
            	}	

			// zone /subzone verification is done in actionroutine

	            class_filter = readEEPROM( VSCP_EEPROM_REGISTER + REG_DM_START + ( VSCP_SIZE_STD_DM_ROW * i ) + 
	                                                    VSCP_DM_POS_CLASSFILTER  );


	             class_mask = ( ( dmflags & VSCP_DM_FLAG_CLASS_MASK ) << 8 ) + readEEPROM( VSCP_EEPROM_REGISTER + REG_DM_START + 
	                                                    ( VSCP_SIZE_STD_DM_ROW * i ) + VSCP_DM_POS_CLASSMASK  );
	            type_filter = readEEPROM( VSCP_EEPROM_REGISTER + REG_DM_START + ( VSCP_SIZE_STD_DM_ROW * i ) + VSCP_DM_POS_TYPEFILTER );
	            type_mask = readEEPROM( VSCP_EEPROM_REGISTER + REG_DM_START + ( VSCP_SIZE_STD_DM_ROW * i ) + VSCP_DM_POS_TYPEMASK  );


	            if ( !( ( class_filter ^ vscp_imsg.vscp_class ) & class_mask ) &&
	                    !( ( type_filter ^ vscp_imsg.vscp_type ) & type_mask )) {
			
				#ifdef PRINT_CAN_EVENTS
				uart_puts( "debug  doDMtrigger\n" );
				#endif

	            // OK Trigger this action
	            switch ( readEEPROM( VSCP_EEPROM_REGISTER + REG_DM_START + ( 8 * i ) +
					 VSCP_DM_POS_ACTION  ) ) {
					// actions can be found in vscp_actions
	                case ACTION_OUTP_TOGGLE1:			// Toggle relays
	                        doActionToggleOut(1, dmflags, readEEPROM( VSCP_EEPROM_REGISTER + REG_DM_START + ( 8 * i ) + VSCP_DM_POS_ACTIONPARAM  ) );
	                        break;
	                case ACTION_OUTP_TOGGLE2:			// Toggle relays
	                        doActionToggleOut(2, dmflags, readEEPROM( VSCP_EEPROM_REGISTER + REG_DM_START + ( 8 * i ) + VSCP_DM_POS_ACTIONPARAM  ) );
	                        break;
					case ACTION_OUTP_ON1:			// Turn on relays
	                        doActionOnOut( 1, dmflags, readEEPROM( VSCP_EEPROM_REGISTER + REG_DM_START + ( 8 * i ) + VSCP_DM_POS_ACTIONPARAM  ) );
	                        break;
					case ACTION_OUTP_ON2:			// Turn on relays
	                        doActionOnOut( 2, dmflags, readEEPROM( VSCP_EEPROM_REGISTER + REG_DM_START + ( 8 * i ) + VSCP_DM_POS_ACTIONPARAM  ) );
	                        break;
					case ACTION_OUTP_OFF1:			// Turn off relays
	                        doActionOffOut( 1, dmflags, readEEPROM( VSCP_EEPROM_REGISTER + REG_DM_START + ( 8 * i ) + VSCP_DM_POS_ACTIONPARAM  ) );
	                        break;
					case ACTION_OUTP_OFF2:			// Turn off relays
	                        doActionOffOut( 2, dmflags, readEEPROM( VSCP_EEPROM_REGISTER + REG_DM_START + ( 8 * i ) + VSCP_DM_POS_ACTIONPARAM  ) );
	                        break;
					} // case	

	            } 
				#ifdef PRINT_CAN_EVENTS
				else uart_puts( "// Filter/mask");
				#endif
//			}//selection
        } // Row enabled
    } // for each row	
}
///////////////////////////////////////////////////////////////////////////////
// SendInformationEvent
//

void SendInformationEvent( uint8_t idx, uint8_t eventClass, uint8_t eventTypeId )
{
    vscp_omsg.priority = VSCP_PRIORITY_MEDIUM;
    vscp_omsg.flags = VSCP_VALID_MSG + 3;
    vscp_omsg.vscp_class = eventClass;
    vscp_omsg.vscp_type = eventTypeId;

    vscp_omsg.data[ 0 ] = idx;	// Register
    vscp_omsg.data[ 1 ] = readEEPROM( VSCP_EEPROM_REGISTER + REG_OUTPUT1_ZONE + idx );
    vscp_omsg.data[ 2 ] = readEEPROM( VSCP_EEPROM_REGISTER + REG_OUTPUT1_SUBZONE + idx );

    vscp_sendEvent();	// Send data
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//                       end of VSCP Required Methods
//////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// SendInformationEventExtended same function as SendInformationEvent but 
// with more options to have more control over the events sent
// SendInformationEvent is not altered to remain compatible with common routines

void SendInformationEventExtended(uint8_t priority, uint8_t zone, uint8_t subzone, uint8_t idx, uint8_t eventClass, uint8_t eventTypeId )
{
    vscp_omsg.priority = priority;
    vscp_omsg.flags = VSCP_VALID_MSG + 3;
    vscp_omsg.vscp_class = eventClass;
    vscp_omsg.vscp_type = eventTypeId;

    vscp_omsg.data[ 0 ] = idx;	// Register
    vscp_omsg.data[ 1 ] = zone;
    vscp_omsg.data[ 2 ] = subzone;

    vscp_sendEvent();	// Send data
}

///////////////////////////////////////////////////////////////////////////////

// doWork
//
// Do work here
//

void doWork( void )
{

}






//debug routines

void sendCharHex (char data)
{
	// sends out a byte in its HEX-notation
	unsigned char dataC = 0;
	dataC = data; //backup
	
	data = (data >> 4);
	data = (data & 0x0F);
	if ((data >= 0) && (data <= 9)) sendChar(data+0x30);
	else sendChar(data+0x37);

	data = (dataC & 0x0F); //start from the backup
	if ((data >= 0) && (data <= 9)) sendChar(data+0x30);
	else sendChar(data+0x37);
	
	sendChar (' ');

	
}

void sendChar (char data)
{	
	int i = 0;
	// to send data with the usart put the data in the usart data register
	UDR0 = data;
	
	// check to see if the global interrupts are enabled
	if (SREG & 0x80) 
	{
		// wait until the byte is sent or we count out
			while ( !(UCSR0A&0x40) && (i<10000) )
			{
				asm ("nop");
				i++;
			}
	}
	else
		// wait until the byte is sent
		while ( !(UCSR0A&0x40) );
		
		// clear the TXCflag
		UCSR0A=(UCSR0A|0x40);
}

// original header
/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol)
 * 	http://www.vscp.org
 *
 * 	2006-04-21
 * 	akhe@eurosource.se
 *
 *  Copyright (C) 2006-2009 Ake Hedman, eurosource
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

