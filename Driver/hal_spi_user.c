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
#include "msp430f5528.h"
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
  P3SEL |= BIT3 + BIT4; // P3.3 P3.4 设置成复用功能
  P3DIR |= BIT3;        // P3.3 设置为输出 SIMO
  P3DIR &= ~BIT4;       // P3.4 设置成输入 SOMI
  
  P2SEL |= BIT7;        // P2.7 设置成复用功能
  P2DIR |= BIT7;        // P2.7 设置成输出
  
    
  UCA0CTL1 |= UCSWRST;               		// Enable SW reset
  UCA0CTL0 |= UCMSB+UCMST+UCSYNC;	        // [b0]   1 -  Synchronous mode 这一位就是用来选择是UART(set 0) 还是 SPI模式(set 1)的
                                                // [b2-1] 00-  3-pin SPI
                                                // [b3]   1 -  Master mode
                                                // [b4]   0 - 8-bit data
                                                // [b5]   1 - MSB first
                                                // [b6]   0 - Clock polarity low
                                                // [b7]   0 - Clock phase - Data is changed on the first UCLK edge and captured on the following edge.
  
  UCA0CTL1 |= UCSSEL_2;               	// select SMCLK as clock source 16Mhz
  UCA0CTL1 &= ~UCSWRST;                 // Clear SW reset, resume operation
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
  
  UCA0TXBUF = TxData;
  while(!(UCA0IFG & UCRXIFG));//等待接收完成
  RxData = UCA0RXBUF;
  
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
    UCA0CTL1 |= UCSWRST;               		// Enable SW reset
    UCA0BR0 = 0x8B;                                // 16MHz/138(8b) = 115200
    UCA0BR1 = 0;     
    UCA0CTL1 &= ~UCSWRST;                         // Clear SW reset, resume operation
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
    UCA0CTL1 |= UCSWRST;               		// Enable SW reset
    UCA0BR0 = 0x01;                                // 16 MHz
    UCA0BR1 = 0;     
    UCA0CTL1 &= ~UCSWRST;                         // Clear SW reset, resume operation
}
