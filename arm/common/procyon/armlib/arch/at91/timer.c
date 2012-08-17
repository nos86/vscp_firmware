/*! \file timer.c \brief Timer Support Library for AT91SAM7S. */
//*****************************************************************************
//
// File Name	: 'timer.c'
// Title		: Timer Support Library for AT91SAM7S
// Author		: Pascal Stang - Copyright (C) 2005-2006
// Created		: 2005.05.26
// Revised		: 2006.07.27
// Version		: 0.1
// Target MCU	: Atmel ARM AT91SAM7S Series
// Editor Tabs	: 4
//
// NOTE: This code is currently below version 1.0, and therefore is considered
// to be lacking in some functionality or documentation, or may not be fully
// tested.  Nonetheless, you can expect most functions to work.
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

// system includes
#include "at91sam7s64.h"
// local includes
#include "processor.h"
#include "timer.h"
// project includes
#include "global.h"

typedef void (*voidFuncPtr)(void);
volatile static voidFuncPtr TimerIntFunc[TIMER_NUM_INTERRUPTS];

// counter/timer registers
volatile DWORD TimerPauseReg;
volatile DWORD PitCounter;


// functions
void delay_us(unsigned long t)
{
	// this function is not yet calibrated
	while(t--)
	{
		asm volatile ("nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n");
		asm volatile ("nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n");
		asm volatile ("nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n");
		asm volatile ("nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n");
		asm volatile ("nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n");
		asm volatile ("nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n nop\r\n");
	}
}

void timerInit(void)
{
	int i;
	// initialize interrupt handlers
	for(i=0; i<TIMER_NUM_INTERRUPTS; i++)
		timerAttach(i,0);

	// initialize various timers

	// default PIT setup is 1000Hz or 1ms interval
	// This default is for code compatibility,
	// simply call timerInitPit() yourself if you need a different rate.
	timerInitPit(1000);
}

void timerInitPit(int rate)
{
    // PIT is the Periodic Interval Timer
	
	// PIT is part of system peripherals so no need to enable clock

	// enable and setup the PIT  for specified interval
	AT91C_BASE_PITC->PITC_PIMR = AT91C_PITC_PITEN|AT91C_PITC_PITIEN|(F_CPU/(16*rate)-1);
	// reset the div counter by reading PIVR
	AT91C_BASE_PITC->PITC_PIVR;

	// init global variables
	TimerPauseReg = 0;
	PitCounter = 0;

    // attach interrupt handlers
	processorAicAttachSys(SYSPID_PITC, timerServicePit);
}

void timerAttach(u08 interruptNum, void (*userFunc)(void) )
{
	// make sure the interrupt number is within bounds
	if(interruptNum < TIMER_NUM_INTERRUPTS)
	{
		// set the interrupt function to run
		// the supplied user's function
		TimerIntFunc[interruptNum] = userFunc;
	}
}

void timerPause(int pause)
{
	// wait until [pause] number of pit overflows goes by
	TimerPauseReg = pause;
	while(TimerPauseReg);
}


// interrupt handlers

// This function is the handler for interrupts generated by the PIT
void timerServicePit(void)
{
	// PITC interval interrupt has fired
	// read PIVR (this also clears int flag)
	PitCounter+=AT91C_BASE_PITC->PITC_PIVR>>20;
	
	// decrement the pause register if non-zero
	if(TimerPauseReg)
		TimerPauseReg--;

	// execute user handler if pointer to routine is NOT null.
	if(TimerIntFunc[TIMER_PITOVERFLOW_INT])
		TimerIntFunc[TIMER_PITOVERFLOW_INT]();
}

