/********************************************************************
* FileName:		Console.c
* Dependencies: Console.h
* Processor:	PIC18, PIC24F, PIC24H, dsPIC30, dsPIC33
*               tested with 18F4620, dsPIC33FJ256GP710	
* Hardware:		PICDEM Z, Explorer 16
* Complier:     Microchip C18 v3.04 or higher
*				Microchip C30 v2.03 or higher	
* Company:		Microchip Technology, Inc.
*
* Copyright and Disclaimer Notice for MiWi Software:
*
* Copyright � 2006-2008 Microchip Technology Inc.  All rights reserved.
*
* Microchip licenses to you the right to use, modify, copy and 
* distribute Software only when embedded on a Microchip
* microcontroller or digital signal controller and used with a 
* Microchip radio frequency transceiver, which are integrated into
* your product or third party product (pursuant to the sublicense 
* terms in the accompanying license agreement).  
*
* You should refer to the license agreement accompanying this 
* Software for additional information regarding your rights 
* and obligations.
*
* SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF
* ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION,
* ANY WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND
* FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR
* ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE,
* STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL
* EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
* INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT,
* PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST
* OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
* CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
* THEREOF), OR OTHER SIMILAR COSTS.
*
*********************************************************************
* File Description:
*
*   This file configures and provides the function for using the
*   UART to transmit data over RS232 to the computer.
*
* Change History:
*  Rev   Date         Description
*  0.1   11/09/2006   Initial revision
*  1.0   01/09/2007   Initial release
********************************************************************/

/************************ HEADERS **********************************/
#include ".\Common\Console.h"
#include "SystemProfile.h"
#include "Compiler.h"
#include "GenericTypeDefs.h"

#if defined(ENABLE_CONSOLE)
#if defined(__dsPIC30F__) || defined(__dsPIC33F__) || defined(__PIC24F__) || defined(__PIC24H__)

/************************ VARIABLES ********************************/
ROM unsigned char CharacterArray[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

/************************ FUNCTIONS ********************************/

/*********************************************************************
* Function:         void ConsoleInit(void)
*
* PreCondition:     none
*
* Input:		    none
*
* Output:		    none
*
* Side Effects:	    UART2 is configured
*
* Overview:		    This function will configure the UART for use at 
*                   in 8 bits, 1 stop, no flowcontrol mode
*
* Note:			    None
********************************************************************/
void ConsoleInit(void)
{
    U2BRG   = (CLOCK_FREQ/2/16)/BAUD_RATE-1;
    IFS1bits.U2RXIF = 0;
    U2STA  = 0;
    U2MODE = 0b0000000010000000;
    U2MODEbits.UARTEN = 1;
    U2STAbits.UTXEN = 1;
}

/*********************************************************************
* Function:         void ConsolePutROMString(ROM char* str)
*
* PreCondition:     none
*
* Input:		    str - ROM string that needs to be printed
*
* Output:		    none
*
* Side Effects:	    str is printed to the console
*
* Overview:		    This function will print the inputed ROM string
*
* Note:			    Do not power down the microcontroller until 
*                   the transmission is complete or the last 
*                   transmission of the string can be corrupted.  
********************************************************************/
void ConsolePutROMString(ROM char* str)
{
    BYTE c;

    while( c = *str++ )
        ConsolePut(c);
}

/*********************************************************************
* Function:         void ConsolePut(BYTE c)
*
* PreCondition:     none
*
* Input:		    c - character to be printed
*
* Output:		    none
*
* Side Effects:	    c is printed to the console
*
* Overview:		    This function will print the inputed character
*
* Note:			    Do not power down the microcontroller until 
*                   the transmission is complete or the last 
*                   transmission of the string can be corrupted.  
********************************************************************/
void ConsolePut(BYTE c)
{
    while(U2STAbits.TRMT == 0);
    U2TXREG = c;
}

/*********************************************************************
* Function:         void PrintChar(BYTE toPrint)
*
* PreCondition:     none
*
* Input:		    toPrint - character to be printed
*
* Output:		    none
*
* Side Effects:	    toPrint is printed to the console
*
* Overview:		    This function will print the inputed BYTE to 
*                   the console in hexidecimal form
*
* Note:			    Do not power down the microcontroller until 
*                   the transmission is complete or the last 
*                   transmission of the string can be corrupted.  
********************************************************************/
void PrintChar(BYTE toPrint)
{
    BYTE PRINT_VAR;
    PRINT_VAR = toPrint;
    toPrint = (toPrint>>4)&0x0F;
    ConsolePut(CharacterArray[toPrint]);
    toPrint = (PRINT_VAR)&0x0F;
    ConsolePut(CharacterArray[toPrint]);
    return;
}

#elif defined(__18CXX) || defined(__PICC__)

/************************ VARIABLES ********************************/
ROM unsigned char CharacterArray[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    
/************************ DEFINITIONS ******************************/
#define USART_USE_BRGH_HIGH
#if defined(USART_USE_BRGH_LOW)
    #define SPBRG_VAL   ( ((CLOCK_FREQ/BAUD_RATE)/64) - 1)
#else
    #define SPBRG_VAL   ( ((CLOCK_FREQ/BAUD_RATE)/16) - 1)
#endif

#if SPBRG_VAL > 255
    #error "Calculated SPBRG value is out of range for currnet CLOCK_FREQ."
#endif

/************************ FUNCTIONS ********************************/
    
/*********************************************************************
* Function:         void ConsoleInit(void)
*
* PreCondition:     none
*
* Input:		    none
*
* Output:		    none
*
* Side Effects:	    UART2 is configured
*
* Overview:		    This function will configure the UART for use at 
*                   in 8 bits, 1 stop, no flowcontrol mode
*
* Note:			    None
********************************************************************/
void ConsoleInit(void)
{
#if defined(USART_USE_BRGH_HIGH)
    TXSTA = 0x24;
#else
    TXSTA = 0x20;
#endif

    RCSTA = 0x90; // 0b10010000;
    SPBRG = SPBRG_VAL;
}

/*********************************************************************
* Function:         void ConsolePutROMString(ROM char* str)
*
* PreCondition:     none
*
* Input:		    str - ROM string that needs to be printed
*
* Output:		    none
*
* Side Effects:	    str is printed to the console
*
* Overview:		    This function will print the inputed ROM string
*
* Note:			    Do not power down the microcontroller until 
*                   the transmission is complete or the last 
*                   transmission of the string can be corrupted.  
********************************************************************/
#if defined(HI_TECH_C) && !defined(__PICC__)
#pragma interrupt_level 0
#endif
void ConsolePutROMString(ROM char* str)
{
    BYTE c;

    while( c = *str++ )
        ConsolePut(c);
}

/*********************************************************************
* Function:         void ConsolePut(BYTE c)
*
* PreCondition:     none
*
* Input:		    c - character to be printed
*
* Output:		    none
*
* Side Effects:	    c is printed to the console
*
* Overview:		    This function will print the inputed character
*
* Note:			    Do not power down the microcontroller until 
*                   the transmission is complete or the last 
*                   transmission of the string can be corrupted.  
********************************************************************/
void ConsolePut(BYTE c)
{
    while( !ConsoleIsPutReady() );
    TXREG = c;
}

/*********************************************************************
* Function:         void PrintChar(BYTE toPrint)
*
* PreCondition:     none
*
* Input:		    toPrint - character to be printed
*
* Output:		    none
*
* Side Effects:	    toPrint is printed to the console
*
* Overview:		    This function will print the inputed BYTE to 
*                   the console in hexidecimal form
*
* Note:			    Do not power down the microcontroller until 
*                   the transmission is complete or the last 
*                   transmission of the string can be corrupted.  
********************************************************************/
void PrintChar(BYTE toPrint)
{
    BYTE PRINT_VAR;
    PRINT_VAR = toPrint;
    toPrint = (toPrint>>4)&0x0F;
    ConsolePut(CharacterArray[toPrint]);
    toPrint = (PRINT_VAR)&0x0F;
    ConsolePut(CharacterArray[toPrint]);
    return;
}
#else
    #error Unknown processor.  See Compiler.h
#endif

#endif  //ENABLE_CONSOLE
