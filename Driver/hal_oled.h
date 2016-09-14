/**************************************************************************************************
  Filename:       hal_oled.h
  Revised:        $Date: 2016-03-12 19:37:16 +0800 (Sat, 12 Mar 2016) $
  Revision:       $Revision: 1 $
  Description:    This file contains the interface to the OLED Service.
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
  PROVIDED ¡°AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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
**************************************************************************************************/

#ifndef HAL_OLED_H
#define HAL_OLED_H

#ifdef __cplusplus
extern "C"
{
#endif
  
/**************************************************************************************************
 *                                             INCLUDES
 **************************************************************************************************/
#include "hal_type.h"
#include "msp430f1611.h"
  
/**************************************************************************************************
 * MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
#define HAL_OLED_MODE_OFF 0x00 
#define HAL_OLED_MODE_ON  0x01  
  
/**************************************************************************************************
 *                                             FUNCTIONS - API
 **************************************************************************************************/
/*
 * Initialize OLED Service.
 */
extern void HalOledInit(void);

/*
 * Clear OLED.
 */
extern void HalOledClear(void);

/*
 * Show a char on OLED.
 */
extern void HalOledShowChar(uint8 x,uint8 y,uint8 chr,uint8 size,uint8 mode);

/*
 * Show a num on OLED.
 */
extern void HalOledShowNum(uint8 x,uint8 y,uint32 num,uint8 len,uint8 size);

/*
 * Show string on OLED.
 */
extern void HalOledShowString(uint8 x,uint8 y,uint8 size,const uint8 *p);  

/*
 * Set the OLED ON/OFF.
 */
extern void HalOledOnOff(uint8 mode);
  
/*
 * Del function.
 */
extern void halMcuWaitUs(uint16 microSecs); 

/*
 * ?¨²OLED¨¦???¨º?¦Ì?¨¢?¡¤?o?
 */
extern void HalOledShowPowerSymbol(uint8 x,uint8 y,uint8 mode,uint8 power_num);

/*
 * Show wait symbol
 */
extern void HalOledShowWaitSymbol(uint8 x,uint8 y,uint8 mode);

#ifdef __cplusplus
}
#endif  
#endif
