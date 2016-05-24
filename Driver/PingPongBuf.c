/**************************************************************************************************
  Filename:       pingPongBuf.c
  Revised:        $Date: 2016-04-06 15:41:16 +0800 (Wen, 6 Apr 2016) $
  Revision:       $Revision: 1 $

  Description:    This file contains the interface to the pingPong buff.


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
**************************************************************************************************/

/***************************************************************************************************
 *                                             INCLUDES
 ***************************************************************************************************/
#include "PingPongBuf.h"
#include "stdlib.h"
/***************************************************************************************************
 *                                             CONSTANTS
 ***************************************************************************************************/
#define PING_BUFFER_ACTIVE    0x00
#define PONG_BUFFER_ACTIVE    0x01

/***************************************************************************************************
 *                                              MACROS
 ***************************************************************************************************/


/***************************************************************************************************
 *                                              TYPEDEFS
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


/******************************************************************************
 * @fn          PingPongBufInit
 *
 * @brief       to construct uint16 ping-pong buffer.
 *
 * @param       haflBufSize - ping buffer size or pong buffer size, which is 
 *                            equal to half size of total buffer.
 *
 * @return      none
 */
PingPongBuf_t *PingPongBufInit(uint16 pingPongBufSize)
{
  PingPongBuf_t *pingPongBuf = NULL;
  
  pingPongBuf = (PingPongBuf_t *)malloc(sizeof(PingPongBuf_t));
  
  if(pingPongBuf == NULL) // don't have enough memory
    return NULL;
  
  pingPongBuf->active_buf_flag = PING_BUFFER_ACTIVE;
  pingPongBuf->write_count = 0;
  pingPongBuf->buf_size = pingPongBufSize;
  
  pingPongBuf->pBuf_ping = (uint32 *)malloc(pingPongBufSize * sizeof(uint32));
  pingPongBuf->pBuf_pong = (uint32 *)malloc(pingPongBufSize * sizeof(uint32));
  
  if((pingPongBuf->pBuf_ping == NULL) || (pingPongBuf->pBuf_pong) == NULL)// don't have enough memory
  {
    // free memory 这里需要注意
    // 由于osal_mem_free不做指针为空的判断
    // 所以用户应该先进行判断之后，再调用该函数
    if(pingPongBuf->pBuf_ping != NULL)
      free(pingPongBuf->pBuf_ping);
    if(pingPongBuf->pBuf_pong != NULL)
      free(pingPongBuf->pBuf_pong);
    
    free(pingPongBuf);
    pingPongBuf = NULL;
  }
  
  return pingPongBuf;
    
}


/******************************************************************************
 * @fn          PingPongBufReset
 *
 * @brief       to reset parameters of uint16 ping-pong buffer.
 *
 * @param       .
 *
 * @return      none
 */
void PingPongBufReset(PingPongBuf_t *pingPongBuf)
{
  pingPongBuf->active_buf_flag = PING_BUFFER_ACTIVE;
  pingPongBuf->write_count = 0;  
}


/******************************************************************************
 * @fn          PingPongFree
 *
 * @brief       to reset parameters of uint16 ping-pong buffer.
 *
 * @param       .
 *
 * @return      none
 */
void PingPongBufFree(PingPongBuf_t *pingPongBuf)
{
    // free memory 这里需要注意
    // 由于osal_mem_free不做指针为空的判断
    // 所以用户应该先进行判断之后，再调用该函数
  if(pingPongBuf != NULL)
  {
    if(pingPongBuf->pBuf_ping != NULL)
      free(pingPongBuf->pBuf_ping);
    if(pingPongBuf->pBuf_pong != NULL)
      free(pingPongBuf->pBuf_pong);
    
    free(pingPongBuf);
  }
}


/******************************************************************************
 * @fn          PingPongBufWrite
 *
 * @brief       Write one data into the active buffer
 *
 * @param       
 *
 * @return      BufOpStatus_t
 */
BufOpStatus_t PingPongBufWrite(PingPongBuf_t *pingPongBuf,uint32 writeData)
{
   BufOpStatus_t OpStatus = PingPongBuf_WRITE_SUCCESS;
  
   uint8 active_buff_flag_temp = pingPongBuf->active_buf_flag;
   uint16 write_count_temp = pingPongBuf->write_count;
   
   uint32 *pingBufTemp = pingPongBuf->pBuf_ping;
   uint32 *pongBufTemp = pingPongBuf->pBuf_pong;
   
   if( write_count_temp >=  pingPongBuf->buf_size)
   {
     OpStatus = PingPongBuf_WRITE_FAIL;
   }
   else
   {
     if(active_buff_flag_temp == PING_BUFFER_ACTIVE) // ping buff active
       pingBufTemp[write_count_temp] = writeData;
     else // pong buff active
       pongBufTemp[write_count_temp] = writeData;
     
     OpStatus = PingPongBuf_WRITE_SUCCESS;
     
     pingPongBuf->write_count++;
     if(pingPongBuf->write_count >= pingPongBuf->buf_size)
     {
       pingPongBuf->write_count = 0;
       
       if(active_buff_flag_temp == PING_BUFFER_ACTIVE)
         active_buff_flag_temp = PONG_BUFFER_ACTIVE;
       else
         active_buff_flag_temp = PING_BUFFER_ACTIVE;
       
       pingPongBuf->active_buf_flag = active_buff_flag_temp;
       OpStatus = PingPongBuf_WRITE_SWITCH;
     }
   }
   return OpStatus;
}


/******************************************************************************
 * @fn          PingPongBufRead
 *
 * @brief       Read all data from inactive buffer.
 *
 * @param       *pingPongBuf - point to the buffer which to read
 *              *dataBuf     - point to the buffer which store the read data
 * @return      Buf status
 */
BufOpStatus_t PingPongBufRead(PingPongBuf_t *pingPongBuf,
                                     uint32 **dataBuf)
{
  BufOpStatus_t OpStatus = PingPongBuf_READ_SUCCESS;
  uint8 active_buff_flag_temp = pingPongBuf->active_buf_flag;
  
  
  if(active_buff_flag_temp == PING_BUFFER_ACTIVE)
    *dataBuf = pingPongBuf->pBuf_pong;
  else
    *dataBuf = pingPongBuf->pBuf_ping;
  
  return OpStatus;
}

