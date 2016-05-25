/**************************************************************************************************
  Filename:       hal_BP_measure.h
  Revised:        $Date: 2016-04-05 15:41:16 +0800 (Tues, 5 Apr 2016) $
  Revision:       $Revision: 1 $

  Description:    This file contains the interface to the BP measure.


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
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

#ifndef HAL_BP_MEASURE_H
#define HAL_BP_MEASURE_H

#ifdef __cplusplus
extern "C"
{
#endif
  
/**************************************************************************************************
 *                                             INCLUDES
 **************************************************************************************************/
#include "hal_type.h"
#include "pingPongBuf.h"

  /**************************************************************************************************
 * MACROS
 **************************************************************************************************/

  
/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
/* pingPong Buffer */
/* for Send to Network size --- 32(DC+AC)+2(BP_HºÍBP_L) uint16 =  68 byte */
/* for Send to SD      size --- 256 uint16 = 512 byte */
#define BP_WAVEFORM_SAMPLER_NUM_PER_PACKET     34
#define BP_WAVEFORM_SAMPLER_NUM_FOR_SD         256
  
#define BP_WAVEFORM_READ_ONE_TIME              480
#define BP_WAVEFORM_SEND_ONE_TIME              60

/* pingPong Buffer Choose */
#define BP_BUFFER_FOR_ZIGBEE   0x00
#define BP_BUFFER_FOR_SD       0x01
  
/***************************************************************************************************
 *                                             TYPEDEFS
 ***************************************************************************************************/

/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/


/**************************************************************************************************
 *                                             FUNCTIONS - API
 **************************************************************************************************/

/*
 * Start BP measure. 
 */
extern void HalBPMeasStart(uint8 deviceStatus);

/*
 * Stop BP measure.
 */
extern void HalBPMeasStop(void);

/*
 * Write BP value into PingPong buff
 */
extern BufOpStatus_t HalBPMeasWriteToBuf(uint16 writeData);

/*
 * Read BP value from PingPong buff
 */
extern void HalBPMeasReadFromBuf(uint16 **dataBuf);

/*
 * Reset BP ping-pong buffer
 */
extern void HalBPMeasBuffReset(void);

#ifdef __cplusplus
}
#endif  
#endif