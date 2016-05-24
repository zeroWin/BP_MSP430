/**************************************************************************************************
  Filename:       hal_uart_cc2530.h
  Revised:        $Date: 2016-04-14 17:21:16 +0800 (Thus, 14 Apr 2016) $
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

#ifndef HAL_UART_CC2530_H
#define HAL_UART_CC2530_H

#ifdef __cplusplus
extern "C"
{
#endif
  
/**************************************************************************************************
 *                                             INCLUDES
 **************************************************************************************************/
#include "hal_type.h"
#include  <msp430x14x.h>
/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/


/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

  
/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
#define START_MEASURE   0x01
#define STOP_MEASURE    0x02
#define SYNC_MEASURE    0x03
#define FIND_NWK        0x04
#define END_DEVICE      0x05
#define CLOSEING        0x06
#define CLOSE_NWK       0x07
#define DATA_START      0x33    // 数据开始校验位
#define DATA_END        0x55    // 数据结束校验位
  
/**************************************************************************************************
 *                                             FUNCTIONS - API
 **************************************************************************************************/

/*
 * Initialize UCA1 UART.
 */
extern void UART1_Config_Init(void);

/*
 * Send buffer.
 */
extern void UART1_Send_Buffer(uint8 *buffer,uint16 len);

/*
 * Send Byte
 */
extern void UART1_Send_Byte(uint8 data);

#ifdef __cplusplus
}
#endif  
#endif
