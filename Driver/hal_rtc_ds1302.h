/**************************************************************************************************
  Filename:       hal_rtc_ds1302.h
  Revised:        $Date: 2016-04-07 15:20:16 +0800 (Tues, 7 Apr 2016) $
  Revision:       $Revision: 1 $

  Description:    This file contains the interface to the RTC Service.


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
  使用DS1302 芯片
**************************************************************************************************/

#ifndef HAL_RTC_DS1302_H
#define HAL_RTC_DS1302_H

#ifdef __cplusplus
extern "C"
{
#endif
  
/**************************************************************************************************
 *                                             INCLUDES
 **************************************************************************************************/
#include "hal_board.h"

/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/
typedef struct
{
  uint8 sec;      //00-59
  uint8 min;      //00-59
  uint8 hour;     //使用24小时模式 00-23
  uint8 date;     //01-28/29 01-30 01-31
  
  uint8 month;    //01-12
  uint8 week;     //01-07
  uint8 year;     //00-99
  uint8 WP;       //don't use
} RTCStruct_t;

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
#define RTC_DS1302_GET        0x00
#define RTC_DS1302_SET        0x01
 
#define RTC_REGISTER_SEC      0x00
#define RTC_REGISTER_MIN      0x01
#define RTC_REGISTER_HOUR     0x02
#define RTC_REGISTER_DATE     0x03
#define RTC_REGISTER_MONTH    0x04
#define RTC_REGISTER_WEEK     0x05
#define RTC_REGISTER_YEAR     0x06
  
/**************************************************************************************************
 *                                             FUNCTIONS - API
 **************************************************************************************************/

/*
 * Initialize RTC Service.
 */
extern void HalRTCInit(void);

/*
 * Set or Get DS1302 one register
 */
extern void HalRTCGetOrSet(uint8 getOrSetFlag,uint8 registerName,uint8 *value);


/*
 * Set or Get DS1302 all register
 */
extern void HalRTCGetOrSetFull(uint8 getOrSetFlag, RTCStruct_t *RTCStruct);

/*
 * RTCstruct init
 * 输入参数为十进制
 */
extern void HalRTCStructInit(RTCStruct_t *RTCStruct,uint8 sec,uint8 min,uint8 hour,uint8 date,
                             uint8 month,uint8 week,uint8 year);
#ifdef __cplusplus
}
#endif  
#endif