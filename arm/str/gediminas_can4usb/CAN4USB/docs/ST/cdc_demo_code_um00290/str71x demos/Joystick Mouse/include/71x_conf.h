/******************** (C) COPYRIGHT 2006 STMicroelectronics ********************
* File Name          : 71x_conf.h
* Author             : MCD Application Team
* Date First Issued  : 10/01/2006 : V1.0
* Description        : Joystick Demo configuration  header
********************************************************************************
* History:
* 10/01/2006 : V1.0
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __71x_CONF_H
#define __71x_CONF_H


/* Comment the line below to put the library in release mode */
//#define DEBUG

#ifdef __IAR_SYSTEMS_ICC__     /* IAR Compiler */
#define inline inline
#endif


#ifdef  __CC_ARM               /* ARM Compiler */
#define inline __inline
#endif

//#define USE_SERIAL_PORT
//#define USE_UART0
//#define USE_UART1
//#define USE_UART2
//#define USE_UART3


/* Comment the lines below corresponding to unwanted peripherals */
//#define _ADC12

#define _APB
#define _APB1
#define _APB2

//#define _BSPI
//#define _BSPI0
//#define _BSPI1

//#define _CAN

#define _EIC

//#define _EMI

#define _FLASH

#define _GPIO
#define _GPIO0
#define _GPIO1
#define _GPIO2

//#define _I2C
//#define _I2C0
//#define _I2C1

#define _PCU

#define _RCCU
// Main Oscillator Frequency value = 16 Mhz
#define RCCU_Main_Osc 16000000

//#define _RTC

//#define _TIM
//define _TIM0
//#define _TIM1
//#define _TIM2
//#define _TIM3

//#define _UART
//#define _UART0
//#define _UART1
//#define _UART2
//#define _UART3

#define _USB

//#define _WDG

#define _XTI


//#define _IRQVectors


#endif /* __71x_CONF_H */

/******************* (C) COPYRIGHT 2006 STMicroelectronics *****END OF FILE****/
