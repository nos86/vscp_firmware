/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol) 
 * 	http://www.vscp.org
 *
 *  Window Motors and thermal radiator control
 * 	Version: See project header
 * 	akhe@eurosource.se
 *
 *  Copyright (C) 1995-2006 Ake Hedman, eurosource
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
 *
 * This software is part of the VSCP node Chicago
 * 
 * ******************************************************************************
*/

#include <xc.h>
#include "eeprom_map.h" //Once a project

#include "main.h"

#include <eeprom.h>
#include <vscp_firmware.h>
#include <vscp_class.h>	
#include <vscp_type.h>
#include "vscp_appLayer.h"

#include <driver.h>

const uint8_t firmwareVersion[3] = {0,0,1};
const uint8_t GuID[16] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const uint8_t deviceFamilyCode=0x00;
const uint8_t deviceFamilyType=0x00;

void _startup (void);
extern timeBasedEventStruct timeEvent, timeOverride;

const char mdfLink[] = "vscp.salvomusumeci.com/mdf/augusto.xml";


void interrupt low_priority isr_low( void ){
	
}

void interrupt isr_high(){
    di();
    TMR0_interrupt();
    ei();
}


void init_app_eeprom(void){

}

void init_app_ram(void){
    init_augusto_ram();
}

//***************************************************************************
// Main() - Main Routine
//***************************************************************************
void main(){
    OSCCON = 0xF0;      //Initialitation of Internal Oscillator
    OSCTUNEbits.PLLEN = 1;
    init_app_ram();     // Initialize data
    hardware_setup();   //Hardware initialization

    if ( !vscp_check_pstorage() )   // Check VSCP persistent storage and
        init_app_eeprom();          // restore if needed
    vscp_init();                    // Initialize the VSCP functionality
    
    while(1){ //Handler scheduler
        ClrWdt();			// Feed the dog
        if(timeEvent._10mS){    //10mS Event
            timeEvent._10mS = 0;
            vscp_10mS_Running();
            hardware_10mS();
        }
        if(timeEvent._100mS){   //100mS Event
            timeEvent._100mS = 0;
            vscp_100mS_Running();
            vscp_ledActivity();

            for (uint8_t i=0; i<8; i++)
                setOutput(i,hardware_input[i].currentStatus);
        }
        if(timeEvent._1s){      //1second Event
            timeEvent._1s = 0;
            vscp_doOneSecondWork(); // Do VSCP one second jobs
            greenLed_pin = !greenLed_pin;
        }
        vscp_freeRunning();
        
        //TODO: Handling the override event
    }

}

void doApplicationDM(int DecisionMatrixIndex){

}
