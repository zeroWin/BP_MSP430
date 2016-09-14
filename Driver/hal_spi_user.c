/**************************************************************************************************
  Filename:       hal_spi_user.c
  Revised:        $Date: 2016-04-14 17:21:16 +0800 (Thus, 14 Apr 2016) $
  Revision:       $Revision: 1 $

  Description:    This file contains the interface to Spi Service.


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
  SPI 使用UCA0
**************************************************************************************************/

/***************************************************************************************************
 *                                             INCLUDES
 ***************************************************************************************************/
#include "hal_spi_user.h"
#include  <msp430f1611.h>
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
 * @fn      SPI1_Config_Init
 *
 * @brief   Initialize SPI
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void SPI1_Config_Init(void)
{
  P5SEL |= BIT1 + BIT2 + BIT3; // P5.1 P5.2 P5.3 设置成复用功能
  P5DIR |= BIT1 + BIT3;        // P5.1 设置为输出 SIMO P5.3 设置成输出CLK
  P5DIR &= ~BIT2;              // P5.2 设置成输入 SOMI
   
  ME2 |= USPIE1;                // 允许USART1的SPI模式
  U1CTL |= CHAR + SYNC + MM;	                // [b0]   0 -  使能可以改变状态
                                                // [b1]   1 - USART is master
                                                // [b2]   1 - SPI mode
                                                // [b3]   0 - One stop mode
                                                // [b4]   1 - 8-bit data
                                                // [b5]   0 - SPImode
                                                // [b6]   Unused
                                                // [b7]   Unused
  U1TCTL = SSEL1 + STC;                         // smclk 8Mhz 3-pin SPI
  U1CTL &= ~SWRST;
}


/**************************************************************************************************
 * @fn      SD_SPI_ReadWriteByte
 *
 * @brief   Write a Byte
 *
 * @param   Write Byte
 *
 * @return  Read Byte
 **************************************************************************************************/
uint8 SPI1_ReadWriteByte(uint8 TxData)
{
  uint8 RxData;
  
  TXBUF1 = TxData;
  while(!(IFG2 & URXIFG1));//等待接收完成
  RxData = RXBUF1;
  
  return RxData;
}


/**************************************************************************************************
 * @fn      SPI1_SetSpeed_Low
 *
 * @brief   set spi baud 115200
 *
 * @param   
 *
 * @return  
 **************************************************************************************************/
void SPI1_SetSpeed_Low(void)
{
    U1CTL |= SWRST;               		// Enable SW reset
    UBR01 = 0x45;                               // 8MHz/69(45) = 115200
    UBR11 = 0;     
    U1CTL &= ~SWRST;                            // Clear SW reset, resume operation
}


/**************************************************************************************************
 * @fn      SPI1_SetSpeed_High
 *
 * @brief   set spi baud 4M
 *
 * @param   
 *
 * @return  
 **************************************************************************************************/
void SPI1_SetSpeed_High(void)
{
    U1CTL |= SWRST;               		// Enable SW reset
    UBR01 = 0x02;                             // 8 MHz /2 = 4Mhz
    UBR11 = 0;     
    U1CTL &= ~SWRST;                         // Clear SW reset, resume operation
}
