/**************************************************************************************************
  Filename:       hal_BP_measure.c
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

  使用Timer1 作为BP采集的定时器
**************************************************************************************************/

/***************************************************************************************************
 *                                             INCLUDES
 ***************************************************************************************************/
#include "hal_BP_measure.h"

PingPongBuf_t *pingPongBuf_BP = NULL;

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/


/**************************************************************************************************
 * @fn      HalBPMeasWriteToBuf
 *
 * @brief   Write AD sample data to buffer
 *
 * @param   writeData
 *          deviceStatus -- online or offline
 *
 * @return  Buf status
 **************************************************************************************************/
BufOpStatus_t HalBPMeasWriteToBuf(uint16 writeData)
{
  BufOpStatus_t OpStatus;
  
  OpStatus = PingPongBufWrite(pingPongBuf_BP,writeData);
    
  return OpStatus;
}


/**************************************************************************************************
 * @fn      HalBPMeasReadFromBuf
 *
 * @brief   Read AD sample data from buffer
 *
 * @param   dataBuf -- store the data
 *          deviceStatus -- online or offline
 *
 * @return  void
 **************************************************************************************************/
void HalBPMeasReadFromBuf(uint16 **dataBuf)
{
  PingPongBufRead(pingPongBuf_BP,dataBuf);
}


/***************************************************************************************************
* @fn      HalBPMeasBuffReset
*
* @brief   Reset BP ping-pong buffer
*
* @param   void
*
* @return  void
***************************************************************************************************/
void HalBPMeasBuffReset(void)
{
  PingPongBufReset(pingPongBuf_BP);
}

/***************************************************************************************************
* @fn      HalBPMeasStart
*
* @brief   Start the BP Meas Service
*
* @param   timerPerTick - number of micro sec per tick, (ticks x prescale) / clock = usec/tick
* 
*
* @return  
***************************************************************************************************/
void HalBPMeasStart(uint8 deviceStatus)
{
  //开辟ping-pong buffer
  if ( deviceStatus == BP_BUFFER_FOR_ZIGBEE )
    pingPongBuf_BP = PingPongBufInit(BP_WAVEFORM_SAMPLER_NUM_PER_PACKET); //for send
  else if ( deviceStatus == BP_BUFFER_FOR_SD )
    pingPongBuf_BP = PingPongBufInit(BP_WAVEFORM_SAMPLER_NUM_FOR_SD);     //for write to SD
}


/***************************************************************************************************
* @fn      HalBPMeasStop
*
* @brief   Stop the BP Meas Service
*
* @param   
*
* @return  
***************************************************************************************************/
void HalBPMeasStop(void)
{
  // Free ping-pong buffer
  PingPongBufFree(pingPongBuf_BP);
  pingPongBuf_BP = NULL;
}
