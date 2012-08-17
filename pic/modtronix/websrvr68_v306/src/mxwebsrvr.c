/*********************************************************************
*
*       Modtronix Example Web Server Application using Microchip TCP/IP Stack
*
*********************************************************************
* FileName:        WebSrvr.c
* Dependencies:    string.H
*                  usart.h
*                  stacktsk.h
*                  Tick.h
*                  http.h
*                  MPFS.h
* Processor:       PIC18
* Complier:        MCC18 v1.00.50 or higher
*                  HITECH PICC-18 V8.35PL3 or higher
* Company:         Microchip Technology, Inc.
*
* Software License Agreement
*
* The software supplied herewith by Microchip Technology Incorporated
* (the Company) for its PICmicro� Microcontroller is intended and
* supplied to you, the Company�s customer, for use solely and
* exclusively on Microchip PICmicro Microcontroller products. The
* software is owned by the Company and/or its supplier, and is
* protected under applicable copyright laws. All rights are reserved.
* Any use in violation of the foregoing restrictions may subject the
* user to criminal sanctions under applicable laws, as well as to
* civil liability for the breach of the terms and conditions of this
* license.
*
* THIS SOFTWARE IS PROVIDED IN AN AS IS CONDITION. NO WARRANTIES,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
* IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
* CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*
*
* HiTech PICC18 Compiler Options excluding device selection:
*                  -FAKELOCAL -G -O -Zg -E -C
*
*
*
*
* Author               Date         Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* David Hosken         15/02/04     Original (Rev. 1.0)
 ********************************************************************/

/*
 * Following define uniquely deines this file as main
 * entry/application In whole project, there should only be one such
 * definition and application file must define AppConfig variable as
 * described below.
 */
#define THIS_IS_STACK_APPLICATION

#include <string.h>

#include "projdefs.h"
#include "debug.h"
#include "serint.h"
#include "sercfg.h"
//#include "appcfg.h"
#include "httpexec.h"
#include "cmd.h"

#include "net\cpuconf.h"
#include "net\stacktsk.h"
#include "net\fsee.h"
#include "net\tick.h"
#include "net\helpers.h"

#if defined(STACK_USE_DHCP)
#include "net\dhcp.h"
#endif

#if defined(STACK_USE_HTTP_SERVER)
#include "net\http.h"
#endif

#if defined(STACK_USE_FTP_SERVER)
#include "net\ftp.h"
#endif

#if defined(STACK_USE_ANNOUNCE)
#include "net\announce.h"
#endif

#if defined(STACK_USE_NBNS)
#include "net\nbns.h"
#endif


/////////////////////////////////////////////////
//Debug defines
#define debugPutMsg(msgCode) debugPut2Bytes(0xD6, msgCode)
#define debugPutMsgRomStr(msgCode, strStr) debugMsgRomStr(0xD6, msgCode, msgStr)


/////////////////////////////////////////////////
//Version number
// - n = major part, and can be 1 or 2 digets long
// - mm is minor part, and must be 2 digets long!
ROM char APP_VER_STR[] = "V3.06";


BYTE myDHCPBindCount = 0;
#if defined(STACK_USE_DHCP)
    extern BYTE DHCPBindCount;
#else
     //If DHCP is not enabled, force DHCP update.
    #define DHCPBindCount 1
#endif

//TODO Remove this later
#if defined(APP_USE_ADC8)
extern BYTE AdcValues[ADC_CHANNELS];
#elif defined(APP_USE_ADC10)
extern WORD AdcValues[ADC_CHANNELS];
#endif

/*
static union
{
    struct
    {
        unsigned int bFreezeADCBuf : 1;     //Stop updating AdcValues[] ADC buffer
    } bits;
    BYTE Val;
} mainFlags;
*/

static void InitializeBoard(void);
static void ProcessIO(void);


/////////////////////////////////////////////////
//High Interrupt ISR
#if defined(MCHP_C18)
    #pragma interrupt HighISR save=section(".tmpdata")
    void HighISR(void)
#elif defined(HITECH_C18)
    #if defined(STACK_USE_SLIP)
        extern void MACISR(void);
    #endif
    void interrupt HighISR(void)
#endif
{
    //TMR0 is used for the ticks
    if (INTCON_TMR0IF)
    {
        TickUpdate();

        #if defined(STACK_USE_SLIP)
        MACISR();
        #endif

        INTCON_TMR0IF = 0;
    }


    #ifdef SER_USE_INTERRUPT
    //USART Receive
    if(PIR1_RCIF && PIE1_RCIE)
    {
        serRxIsr();
    }

    //USART Transmit
    if(PIR1_TXIF && PIE1_TXIE)
    {
        serTxIsr();
    }
    #endif
}

#if defined(MCHP_C18)
#pragma code highVector=HIVECTOR_ADR
void HighVector (void)
{
    _asm goto HighISR _endasm
}
#pragma code /* return to default code section */
#endif


/*
 * Fast User process. Place all high priority code that has to be called often
 * in this function.
 *
 * !!!!! IMPORTANT !!!!!
 * This function is called often, and should be very fast!
 */
void fastUserProcess(void)
{
    CLRWDT();
}


/*
 * Main entry point.
 */
void main(void)
{
    static TICK8 t = 0;
    static TICK8 tmr10ms = 0;
    
    //Initialize any application specific hardware.
    InitializeBoard();

    //Initialize all stack related components. Following steps must
    //be performed for all applications using PICmicro TCP/IP Stack.
    TickInit();

    //Initialize file system.
    fsysInit();

    //Intialize HTTP Execution unit
    htpexecInit();

    //Initialze serial port
    serInit();
    
    //Initialize Stack and application related NV variables.
    appcfgInit();
    appcfgUSART();              //Configure the USART
    #ifdef SER_USE_INTERRUPT    //Interrupt enabled serial ports have to be enabled
    serEnable();
    #endif
    appcfgCpuIO();          //Configure the CPU's I/O port pin directions - input or output
    appcfgCpuIOValues();    //Configure the CPU's I/O port pin default values
    appcfgADC();            //Configure ADC unit
    appcfgPWM();            //Configure PWM Channels

    //Serial configuration menu - display it for configured time and allow user to enter configuration menu
    scfInit(appcfgGetc(APPCFG_STARTUP_SER_DLY));

    StackInit();

#if defined(STACK_USE_HTTP_SERVER)
    HTTPInit();
#endif

#if defined(STACK_USE_FTP_SERVER)
    FTPInit();
#endif

    //Initializes "UDP Command Port" and "UDP Command Responce Port".
    cmdUdpInit();

#if defined(STACK_USE_DHCP) || defined(STACK_USE_IP_GLEANING)
    DHCPReset();    //Initialize DHCP module
    
    //If DHCP is NOT enabled
    if ((appcfgGetc(APPCFG_NETFLAGS) & APPCFG_NETFLAGS_DHCP) == 0)
    {
        //Force IP address display update.
        myDHCPBindCount = 1;
        
        #if defined(STACK_USE_DHCP)
        DHCPDisable();
        #endif
    }
#endif

    #if (DEBUG_MAIN >= LOG_DEBUG)
        debugPutMsg(1); //@mxd:1:Starting main loop
    #endif

    /*
     * Once all items are initialized, go into infinite loop and let
     * stack items execute their tasks.
     * If application needs to perform its own task, it should be
     * done at the end of while loop.
     * Note that this is a "co-operative mult-tasking" mechanism
     * where every task performs its tasks (whether all in one shot
     * or part of it) and returns so that other tasks can do their
     * job.
     * If a task needs very long time to do its job, it must broken
     * down into smaller pieces so that other tasks can have CPU time.
     */
    while(1)
    {
        //Clear timer 1 every cycle, can be used to measure events. Has a overflow of 65ms.
        //Get delay in this function with:
        // TMR1L | (TMR1H<<8)
        TMR1H = 0;  //First write to TMR1H buffer!
        TMR1L = 0;  //This write will also update TMR1H with value written above to buffer

        //Blink SYSTEM LED every second.
        if (appcfgGetc(APPCFG_SYSFLAGS) & APPCFG_SYSFLAGS_BLINKB6) {
            if ( TickGetDiff8bit(t) >= ((TICK8)TICKS_PER_SECOND / (TICK8)2) )
            {
                t = TickGet8bit();
                TRISB_RB6 = 0;
                LATB6 ^= 1;
            }
        }
        
        //Enter each 10ms
        if ( TickGetDiff8bit(tmr10ms) >= ((TICK8)TICKS_PER_SECOND / (TICK8)100) )
        {
            tmr10ms = TickGet8bit();
        }
        
        //This task performs normal stack task including checking for incoming packet,
        //type of packet and calling appropriate stack entity to process it.
        StackTask();
        
        //Process "UDP Command Port" and "UDP Command Responce Port"
        cmdProcess();

#if defined(STACK_USE_HTTP_SERVER)
        //This is a TCP application.  It listens to TCP port 80
        //with one or more sockets and responds to remote requests.
        HTTPServer();
#endif

#if defined(STACK_USE_FTP_SERVER)
        FTPServer();
#endif

#if defined(STACK_USE_ANNOUNCE)
        DiscoveryTask();
#endif

#if defined(STACK_USE_NBNS)
        NBNSTask();
#endif

        //Add your application speicifc tasks here.
        ProcessIO();

        //For DHCP information, display how many times we have renewed the IP
        //configuration since last reset.
        if ( DHCPBindCount != myDHCPBindCount )
        {
            #if (DEBUG_MAIN >= LOG_INFO)
                debugPutMsg(2); //@mxd:2:DHCP Bind Count = %D
                debugPutByteHex(DHCPBindCount);
            #endif
            
            //Display new IP address
            #if (DEBUG_MAIN >= LOG_INFO)
                debugPutMsg(3); //@mxd:3:DHCP complete, IP = %D.%D.%D.%D
                debugPutByteHex(AppConfig.MyIPAddr.v[0]);
                debugPutByteHex(AppConfig.MyIPAddr.v[1]);
                debugPutByteHex(AppConfig.MyIPAddr.v[2]);
                debugPutByteHex(AppConfig.MyIPAddr.v[3]);
            #endif
            myDHCPBindCount = DHCPBindCount;
            
            #if defined(STACK_USE_ANNOUNCE)
                AnnounceIP();
            #endif             
        }
    }
}


static void ProcessIO(void)
{
//Convert next ADC channel, and store result in adcChannel array
#if (defined(APP_USE_ADC8) || defined(APP_USE_ADC10)) && (ADC_CHANNELS > 0)
    static BYTE adcChannel; //Current ADC channel. Value from 0 - n

    //We are allowed to update AdcValues[] buffer
    //if (!mainFlags.bits.bFreezeADCBuf)
    {
        //Increment to next ADC channel    
        if ((++adcChannel) >= ADC_CHANNELS)
        {
            adcChannel = 0;
        }

        //Check if current ADC channel (adcChannel) is configured to be ADC channel
        if (adcChannel < ((~ADCON1) & 0x0F))
        {
            //Convert next ADC Channel
            ADCON0 &= ~0x3C;
            ADCON0 |= (adcChannel << 2);

            ADCON0_ADON = 1;    //Switch on ADC
            ADCON0_GO = 1;      //Go
    
            while (ADCON0_GO) FAST_USER_PROCESS(); //Wait for conversion to finish

            #if defined(APP_USE_ADC8)
            AdcValues[adcChannel] = ADRESH;
            #elif defined(APP_USE_ADC10)
            AdcValues[adcChannel] = ((WORD)ADRESH << 8) || ADRESL;
            #endif
        }
        //Not ADC channel, set to 0
        else {
            AdcValues[adcChannel] = 0;
        }
    }
#endif

}

static ROM char HTTPHDR_AUTHORIZATION[] = "AUTHORIZATION: BASIC ";

/**
 * This function is a "callback" from HTTPServer task. For each HTTP Header found in the HTTP Request
 * message from the client, this function is called. The application has the chance to parse the
 * received headers.
 *
 * @param httpInfo  HTTP_INFO structure of current HTTP connection
 * @param hdr       Buffer containing NULL terminated header to handle
 * @param rqstRes   Name of the Requested resource - GET command's action. All characters are in uppercase!
 */
void HTTPProcessHdr(HTTP_INFO* httpInfo, BYTE* hdr, BYTE* rqstRes) {
    BYTE i;
    char unpw[20];      //Buffer for storing username and password, seperated by ':'
    char base64[25];    //Buffer for storing base64 result

    //Check if buffer begins with ROM string, ignore case
    if (strBeginsWithIC((char*)hdr, HTTPHDR_AUTHORIZATION)) {
        i = strcpyee2ram(unpw, APPCFG_USERNAME0, 8);    //Returns number of bytes written, excluding terminating null
        unpw[i++] = ':';                                //Overwrite terminating null with ':'
        strcpyee2ram(&unpw[i], APPCFG_PASSWORD0, 8);
        
        base64Encode((char*)base64, (char*)unpw, strlen(unpw));

        if (strcmp( (char*)(&hdr[sizeof(HTTPHDR_AUTHORIZATION)-1]), base64) == 0) {
            httpInfo->flags.bits.bUserLoggedIn = TRUE;

            #if (DEBUG_MAIN >= LOG_DEBUG)
                debugPutMsg(6); //@mxd:6:HTTP User Authenticated
            #endif
        }
    }
}


/////////////////////////////////////////////////
//Implement callback for FTPVerify() function
#if defined(STACK_USE_FTP_SERVER)
    #if (FTP_USER_NAME_LEN > APPCFG_USERNAME_LEN)
    #error "The FTP Username length can not be shorter then the APPCFG Username length!"
    #endif

BOOL FTPVerify(char *login, char *password)
{
    #if (DEBUG_MAIN >= LOG_INFO)
        debugPutMsg(4); //@mxd:4:Received FTP Login (%s) and Password (%s)
        debugPutString(login);
        debugPutString(password);
    #endif

    if (strcmpee2ram(login, APPCFG_USERNAME0) == 0) {
        if (strcmpee2ram(password, APPCFG_PASSWORD0) == 0) {
            return TRUE;
        }
    }
    return FALSE;
}
#endif


/**
 * Initialize the boards hardware
 */
static void InitializeBoard(void)
{
    #if (defined(MCHP_C18) && (defined(__18F458) || defined(__18F448) || defined(__18F6680))) \
        || (defined(HITECH_C18) && (defined(_18F458) || defined(_18F448) || defined(_18F6680)))
        CMCON  = 0x07; /* Comparators off CM2:CM0 = 111 for PIC 18F448, 18F458, 18F6680 */
    #endif


    /////////////////////////////////////////////////
    //Enable 4 x PLL if configuration bits are set to do so
    #if ( defined(MCHP_C18) && defined(__18F6621))
    OSCCON = 0x04;              //Enable PLL (PLLEN = 1)
    while (OSCCON_LOCK == 0);   //Wait until PLL is stable (LOCK = 1)
    
    //Seeing that this code does currently not work with Hi-Tech compiler, disable this feature
    OSCCON_SCS1 = 1;
    //Switch on software 4 x PLL if flag is set in configuration data
    //if (appcfgGetc(APPCFG_SYSFLAGS) & APPCFG_SYSFLAGS_PLLON) {
    //    OSCCON_SCS1 = 1;
    //}
    #endif
    

    /////////////////////////////////////////////////
    //Setup timer1 as a free running 16 bit timer. Is incremented every 100ns.
    //Is used to measure events in code
    //1xxx xxxx = Enable read/write as a 16bit times. TMR1H is a buffer that is loaded when TMR1L is accessed.
    //xx00 xxxx = No prescaler
    //xxxx 0xxx = Timer1 oscillator disabled (RC0 and RC1)
    //xxxx xx0x = Internal clock source = Fosc/4 = 40/4 = 10MHz for a 40MHz clock
    //xxxx xxx1 = Timer 1 on
    T1CON = 0x81;


    //Disable external pull-ups
    INTCON2_RBPU = 1;

    //Enable interrupts
    T0CON = 0;
    INTCON_GIEH = 1;
    INTCON_GIEL = 1;
}
