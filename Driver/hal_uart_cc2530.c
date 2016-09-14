/**************************************************************************************************
  Filename:       hal_uart_cc2530.h
  Revised:        $Date: 2016-05-19 13:44:16 +0800 (Thus, 19 May 2016) $
  Revision:       $Revision: 1 $

  Description:    This file contains the interface to uart Service.


  Copyright 2016 Bupt. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact kylinnevercry@gami.com. 
  UART 驱动
**************************************************************************************************/

/***************************************************************************************************
 *                                             INCLUDES
 ***************************************************************************************************/
#include "hal_uart_cc2530.h"
/***************************************************************************************************
 *                                             CONSTANTS
 ***************************************************************************************************/


/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/

  
/***************************************************************************************************
 *                                              MACROS
 ***************************************************************************************************/


/**************************************************************************************************
 *                                        INNER GLOBAL VARIABLES
 **************************************************************************************************/

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      UART1_Config_Init
 *
 * @brief   Initialize UART1
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void UART1_Config_Init(void)
{
  P3SEL |= BIT4 + BIT5; // P3.3 P3.5 设置成复用功能
  P3DIR |= BIT4;        // P3.4 设置为输出 UTXD0
  P3DIR &= ~BIT5;       // P3.5 设置成输入 URXD0
  
  ME1 |= UTXE0 + URXE0; //允许USART0接受和发送
  U0CTL |= CHAR;	                        // [b0]   0 -  使能可以改变状态
                                                // [b1]   0 - Idle-line
                                                // [b2]   0 - UART mode
                                                // [b3]   0 - One stop mode
                                                // [b4]   1 - 8-bit data
                                                // [b5]   0 - One stop mode
                                                // [b6]   0 - b7disable 该位无效
                                                // [b7]   0 - 无奇偶校验位.
  
  UTCTL0 |= SSEL1;                        // UCLK = SMCLK 8Mhz
  UBR00 = 0x45;                         // 115200 
  UBR10 = 0x00;                           
  UMCTL0 = 0x4A;                          // Modulation 
  UCTL0 &= ~SWRST;                 // Clear SW reset, resume operation  
  
  IFG1 &= ~ URXIFG0;               // 清空接收中断标志
  IE1 |= URXIE0;                   //使能USART0接收中断
 
}


/**************************************************************************************************
 * @fn      UART1_Send_Buffer
 *
 * @brief   Send buffer
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void UART1_Send_Buffer(uint8 *buffer,uint16 len)
{
  uint16 i;
  for(i = 0; i < len ; ++i)
  {
    TXBUF0=buffer[i];//数据被写入发送缓冲寄存器，发送中断标志清0
    while (!(IFG1 & UTXIFG0));//等待中断标志置1，即等待发送完成 
  }
}


/**************************************************************************************************
 * @fn      UUART1_Send_Byte
 *
 * @brief   Send Byte
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void UART1_Send_Byte(uint8 data)
{
  TXBUF0=data;//数据被写入发送缓冲寄存器，发送中断标志清0
  while (!(IFG1 & UTXIFG0));//等待中断标志置1，即等待发送完成 
}





