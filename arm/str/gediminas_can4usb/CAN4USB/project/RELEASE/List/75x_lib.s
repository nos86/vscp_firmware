///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ARM ANSI C/C++ Compiler V5.12.0.50667/W32        18/Apr/2008  14:00:22 /
// Copyright 1999-2007 IAR Systems. All rights reserved.                      /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  D:\Program\IAR Systems\Embedded Workbench               /
//                    5.0\ARM\examples\ST\STR75x\STR75xLibrary\library\src\75 /
//                    x_lib.c                                                 /
//    Command line =  "D:\Program\IAR Systems\Embedded Workbench              /
//                    5.0\ARM\examples\ST\STR75x\STR75xLibrary\library\src\75 /
//                    x_lib.c" -lC D:\development\gediminas\CAN4USB\project\R /
//                    ELEASE\List\ -lA D:\development\gediminas\CAN4USB\proje /
//                    ct\RELEASE\List\ -o D:\development\gediminas\CAN4USB\pr /
//                    oject\RELEASE\Obj\ --no_cse --no_unroll --no_inline     /
//                    --no_code_motion --no_tbaa --no_clustering              /
//                    --no_scheduling --debug --endian little --cpu           /
//                    ARM7TDMI-S -e --fpu None --dlib_config "D:\Program\IAR  /
//                    Systems\Embedded Workbench                              /
//                    5.0\ARM\INC\DLib_Config_Normal.h" -I                    /
//                    D:\development\gediminas\CAN4USB\project\ -I            /
//                    D:\development\gediminas\CAN4USB\project\app\ -I        /
//                    D:\development\gediminas\CAN4USB\project\board\ -I      /
//                    D:\development\gediminas\CAN4USB\project\module\ -I     /
//                    D:\development\gediminas\CAN4USB\project\STR75xLibrary\ /
//                    library\inc\ -I j:\common\ -I "D:\Program\IAR           /
//                    Systems\Embedded Workbench 5.0\ARM\INC\" --interwork    /
//                    --cpu_mode thumb -On                                    /
//    List file    =  D:\development\gediminas\CAN4USB\project\RELEASE\List\7 /
//                    5x_lib.s                                                /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME `75x_lib`


        END
// D:\Program\IAR Systems\Embedded Workbench 5.0\ARM\examples\ST\STR75x\STR75xLibrary\library\src\75x_lib.c
//    1 /******************** (C) COPYRIGHT 2006 STMicroelectronics ********************
//    2 * File Name          : 75x_lib.c
//    3 * Author             : MCD Application Team
//    4 * Date First Issued  : 03/10/2006
//    5 * Description        : This file provides all peripherals pointers initialization.
//    6 ********************************************************************************
//    7 * History:
//    8 * 07/17/2006 : V1.0
//    9 * 03/10/2006 : V0.1
//   10 ********************************************************************************
//   11 * THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
//   12 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
//   13 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
//   14 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
//   15 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
//   16 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//   17 *******************************************************************************/
//   18 
//   19 #define EXT
//   20 
//   21 /* Includes ------------------------------------------------------------------*/
//   22 #include "75x_lib.h"
//   23 
//   24 /* Private typedef -----------------------------------------------------------*/
//   25 /* Private define ------------------------------------------------------------*/
//   26 /* Private macro -------------------------------------------------------------*/
//   27 /* Private variables ---------------------------------------------------------*/
//   28 /* Private function prototypes -----------------------------------------------*/
//   29 /* Private functions ---------------------------------------------------------*/
//   30 #ifdef DEBUG
//   31 
//   32 /*******************************************************************************
//   33 * Function Name  : debug
//   34 * Description    : This function initialize peripherals pointers.
//   35 * Input          : None
//   36 * Output         : None
//   37 * Return         : None
//   38 *******************************************************************************/
//   39 void debug(void)
//   40 {
//   41 /************************************* SMI ************************************/
//   42 #ifdef _SMI
//   43   SMI = (SMI_TypeDef *)  SMIR_BASE;
//   44 #endif /*_SMI */
//   45 
//   46 /************************************* CFG ************************************/
//   47 #ifdef _CFG
//   48   CFG = (CFG_TypeDef *)  CFG_BASE;
//   49 #endif /*_CFG */
//   50 
//   51 /************************************* MRCC ***********************************/
//   52 #ifdef _MRCC
//   53   MRCC = (MRCC_TypeDef *)  MRCC_BASE;
//   54 #endif /*_MRCC */
//   55 
//   56 /************************************* ADC ************************************/	
//   57 #ifdef _ADC
//   58   ADC = (ADC_TypeDef *)  ADC_BASE;
//   59 #endif /*_ADC */
//   60 
//   61 /************************************* TB *************************************/
//   62 #ifdef _TB
//   63   TB = (TB_TypeDef *)  TB_BASE;
//   64 #endif /*_TB */
//   65 
//   66 /************************************* TIM ************************************/
//   67 #ifdef _TIM0
//   68   TIM0 = (TIM_TypeDef *)  TIM0_BASE;
//   69 #endif /*_TIM0 */
//   70 
//   71 #ifdef _TIM1
//   72   TIM1 = (TIM_TypeDef *)  TIM1_BASE;
//   73 #endif /*_TIM1 */
//   74 
//   75 #ifdef _TIM2
//   76   TIM2 = (TIM_TypeDef *)  TIM2_BASE;
//   77 #endif /*_TIM2 */
//   78 
//   79 /************************************* PWM ************************************/
//   80 #ifdef _PWM
//   81   PWM = (PWM_TypeDef *)  PWM_BASE;
//   82 #endif /*_PWM */
//   83 
//   84 /************************************* WDG ************************************/
//   85 #ifdef _WDG
//   86   WDG = (WDG_TypeDef *)  WDG_BASE;
//   87 #endif /*_WDG */
//   88 
//   89 /************************************* SSP ************************************/
//   90 #ifdef _SSP0
//   91   SSP0 = (SSP_TypeDef *)  SSP0_BASE;
//   92 #endif /*_SSP0 */
//   93 
//   94 #ifdef _SSP1
//   95   SSP1 = (SSP_TypeDef *)  SSP1_BASE;
//   96 #endif /*_SSP1 */
//   97 
//   98 /************************************* CAN ************************************/
//   99 #ifdef _CAN
//  100   CAN = (CAN_TypeDef *)  CAN_BASE;
//  101 #endif /*_CAN */
//  102 
//  103 /************************************* I2C ************************************/
//  104 #ifdef _I2C
//  105   I2C = (I2C_TypeDef *)  I2C_BASE;
//  106 #endif /*_I2C */
//  107 
//  108 /************************************* UART ***********************************/
//  109 #ifdef _UART0
//  110   UART0 = (UART_TypeDef *) UART0_BASE;
//  111 #endif /*_UART0 */
//  112 
//  113 #ifdef _UART1
//  114   UART1 = (UART_TypeDef *) UART1_BASE;
//  115 #endif /*_UART1 */
//  116 
//  117 #ifdef _UART2
//  118   UART2 = (UART_TypeDef *) UART2_BASE;
//  119 #endif /*_UART2 */
//  120 
//  121 /************************************* GPIO ***********************************/
//  122 #ifdef _GPIO0
//  123   GPIO0 = (GPIO_TypeDef *) GPIO0_BASE;
//  124 #endif /*_GPIO0 */
//  125 
//  126 #ifdef _GPIO1
//  127   GPIO1 = (GPIO_TypeDef *) GPIO1_BASE;
//  128 #endif /*_GPIO1 */
//  129 
//  130 #ifdef _GPIO2
//  131   GPIO2 = (GPIO_TypeDef *) GPIO2_BASE;
//  132 #endif /*_GPIO2 */
//  133 
//  134 #ifdef _GPIOREMAP
//  135   GPIOREMAP = (GPIOREMAP_TypeDef *) GPIOREMAP_BASE;
//  136 #endif /*_GPIOREMAP */
//  137 
//  138 /************************************* DMA ************************************/
//  139 #ifdef _DMA
//  140   DMA = (DMA_TypeDef *)  DMA_BASE;
//  141 #endif /*_DMA */
//  142 
//  143 #ifdef _DMA_Stream0
//  144   DMA_Stream0 = (DMA_Stream_TypeDef *)  DMA_Stream0_BASE;
//  145 #endif /*_DMA_Stream0 */
//  146 
//  147 #ifdef _DMA_Stream1  
//  148   DMA_Stream1 = (DMA_Stream_TypeDef *)  DMA_Stream1_BASE;
//  149 #endif /*_DMA_Stream1 */  
//  150   
//  151 #ifdef _DMA_Stream2
//  152   DMA_Stream2 = (DMA_Stream_TypeDef *)  DMA_Stream2_BASE;
//  153 #endif /*_DMA_Stream2 */  
//  154 
//  155 #ifdef _DMA_Stream3
//  156   DMA_Stream3 = (DMA_Stream_TypeDef *)  DMA_Stream3_BASE;
//  157 #endif /*_DMA_Stream3 */
//  158 
//  159 /************************************* RTC ************************************/
//  160 #ifdef _RTC
//  161   RTC = (RTC_TypeDef *)  RTC_BASE;
//  162 #endif /*_RTC */
//  163 
//  164 /************************************* EXTIT **********************************/
//  165 #ifdef _EXTIT
//  166   EXTIT = (EXTIT_TypeDef *)  EXTIT_BASE;
//  167 #endif /*_EXTIT */
//  168 
//  169 /************************************* EIC ************************************/
//  170 #ifdef _EIC
//  171   EIC = (EIC_TypeDef *)  EIC_BASE;
//  172 #endif /*_EIC */
//  173 
//  174 }
//  175 
//  176 #endif
//  177 
//  178 /******************* (C) COPYRIGHT 2006 STMicroelectronics *****END OF FILE****/
// 
// 
// 0 bytes of memory
//
//Errors: none
//Warnings: none
