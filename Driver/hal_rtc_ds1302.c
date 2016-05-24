/**************************************************************************************************
  Filename:       hal_rtc_ds1302.c
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

/***************************************************************************************************
 *                                             INCLUDES
 ***************************************************************************************************/
#include "hal_rtc_ds1302.h"
#include  <msp430x14x.h>
/***************************************************************************************************
 *                                             CONSTANTS
 ***************************************************************************************************/
/* RTC SCLK is at P2.7 */
/* RTC I/O is at P2.6 */
/* RTC CE is at P2.5 */
/* Data read IO P2.6*/
#define DS1302_IO_READ_HIGH 0x40

/* RTC register address 
 * 1byte:address/command + 1byte:data
 * address/command bit as follow:
 *  7   6          5   4   3   2   1    0
 *  1  RAM/CK#    A4  A3  A2  A1  A0   R/W#
 */
#define RTC_R_SEC       0x81
#define RTC_R_MIN       0x83
#define RTC_R_HOUR      0x85
#define RTC_R_DATE      0x87
#define RTC_R_MONTH     0x89
#define RTC_R_WEEK      0x8b
#define RTC_R_YEAR      0x8d
#define RTC_R_CONTROL   0x8f
#define RTC_R_TCS       0x91
#define RTC_R_CLK_BURST 0xbf

#define RTC_W_SEC       0x80
#define RTC_W_MIN       0x82
#define RTC_W_HOUR      0x84
#define RTC_W_DATE      0x86
#define RTC_W_MONTH     0x88
#define RTC_W_WEEK      0x8a
#define RTC_W_YEAR      0x8c
#define RTC_W_CONTROL   0x8e
#define RTC_W_TCS       0x90
#define RTC_W_CLK_BURST 0xbe

#define RTC_DS1302_CLK_ENABLE   0x00
#define RTC_DS1302_CLK_DISABLE  0x01
/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/

/**************************************************************************************************
 *                                        INNER GLOBAL VARIABLES
 **************************************************************************************************/

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
void HalRTCWriteByte(uint8 data);
uint8 HalRTCReadByte(void);

void HalRTCSingleWrite(uint8 addr,uint8 data);
uint8 HalRTCSingleRead(uint8 addr);

void HalRTCCalBurstRead(uint8 *calendar);
void HalRTCCalBurstWrite(uint8 *calendar);

uint8 HalBCD2Decimal(uint8 BCD);
uint8 HalDecimal2BCD(uint8 decimal);

void HalRTCDS1302Work(uint8 DS1302Enable);

bool HalRTCDS1302OK(void);

void halMcuWaitUs(uint16 microSecs);

  
/***************************************************************************************************
 *                                              MACROS
 ***************************************************************************************************/
/* ----------- SCLK's P2.7---------- */
#define DS1302_SCLK_OUT()   P2DIR |= BIT7
#define DS1302_SCLK_HIGH()  P2OUT |= BIT7
#define DS1302_SCLK_LOW()   P2OUT &= ~BIT7

/* ----------- IO's P2.6---------- */
#define DS1302_IO_OUT()  P2DIR |= BIT6
#define DS1302_IO_HIGH() P2OUT |= BIT6
#define DS1302_IO_LOW()  P2OUT &= ~BIT6

#define DS1302_IO_IN()   P2DIR &= ~BIT6
#define GET_DS1302_IO()  P2IN

/* ----------- CE's P2.5---------- */
#define DS1302_CE_OUT()     P2DIR |= BIT5
#define DS1302_CE_HIGH()    P2OUT |= BIT5
#define DS1302_CE_LOW()     P2OUT &= ~BIT5    

/* ----------- delay ---------- */
#define DS1302_WRITE_DELAY()  halMcuWaitUs(5);
#define DS1302_READ_DELAY()   halMcuWaitUs(5);
#define DS1302_CE_DELAY()     halMcuWaitUs(5);  
  
/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      HalRTCInit
 *
 * @brief   Initialize RTC Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalRTCInit(void)
{
  // 设置端口功能
  P2SEL &= ~(BIT5 + BIT6 + BIT7);
  //set port dir and power low
  DS1302_CE_OUT(); 
  DS1302_SCLK_OUT();
  DS1302_IO_OUT();  
  
  if(HalRTCDS1302OK() == FALSE)
  {
    /* Setting DS1302 default time */
    /* 2016-4-26 00:00:00 周二 */
    RTCStruct_t RTCStruct;
    HalRTCStructInit(&RTCStruct,0,0,0,1,5,2,16);
    HalRTCGetOrSetFull(RTC_DS1302_SET,&RTCStruct);
    
    /* Start DS1302 work */
    HalRTCDS1302Work(RTC_DS1302_CLK_ENABLE);
  }
}
  

/**************************************************************************************************
 * @fn      HalRTCGetOrSetFull
 *
 * @brief   Set or Get RTC ALL register data.
 *
 * @param   GetOrSetFlag -- RTC_DS1302_GET、RTC_DS1302_SET
 *          RTCStructTemp -- store the calendar to set or store the calendar get from DS1302.
 * @return  
 **************************************************************************************************/
void HalRTCGetOrSetFull(uint8 getOrSetFlag, RTCStruct_t *RTCStruct)
{
  uint8 *RTCStructTemp = (uint8 *)RTCStruct;
  uint8 i;
  
  if(getOrSetFlag == RTC_DS1302_SET) // Write data
  {
    // from decimal to BCD
    for( i = 0; i< 7; i++ )
    {
      *RTCStructTemp = HalDecimal2BCD(*RTCStructTemp);
      RTCStructTemp++;
    }
    *RTCStructTemp = 0; //Set WP to 0
    
     //Point to head and write data
    RTCStructTemp = (uint8 *)RTCStruct;
    HalRTCCalBurstWrite(RTCStructTemp);    
  
  }
  else if(getOrSetFlag == RTC_DS1302_GET) // Read data
  {
    //Read data
    HalRTCCalBurstRead(RTCStructTemp);
    
    // Get useful value and change data from BCD to decimal
    RTCStruct->sec = HalBCD2Decimal(  RTCStruct->sec&0x7F );
    RTCStruct->min = HalBCD2Decimal(  RTCStruct->min&0x7F );
    RTCStruct->hour = HalBCD2Decimal( RTCStruct->hour&0x3F );
    RTCStruct->date = HalBCD2Decimal( RTCStruct->date&0x3F );
    RTCStruct->month = HalBCD2Decimal(RTCStruct->month&0x1F );
    RTCStruct->week = HalBCD2Decimal( RTCStruct->week&0x07 );
    RTCStruct->year = HalBCD2Decimal( RTCStruct->year&0xFF );
  }
  else
  {}
  
  return;
  
}

/**************************************************************************************************
 * @fn      HalRTCGetOrSet
 *
 * @brief   Set or Get RTC one register data.
 *
 * @param   GetOrSetFlag -- RTC_DS1302_GET、RTC_DS1302_SET
 *          RegisterName -- which register want to operation
 *          value -- get return value . set return 0xFF.All use decimal.
 * @return  
 **************************************************************************************************/
void HalRTCGetOrSet(uint8 getOrSetFlag,uint8 registerName,uint8 *value)
{
  uint8 RTCRegisterAddr;
  uint8 valueTemp;
  switch(registerName)
  {
    case RTC_REGISTER_SEC   : RTCRegisterAddr = RTC_R_SEC   ; break;
    case RTC_REGISTER_MIN   : RTCRegisterAddr = RTC_R_MIN   ; break;
    case RTC_REGISTER_HOUR  : RTCRegisterAddr = RTC_R_HOUR  ; break;
    case RTC_REGISTER_DATE  : RTCRegisterAddr = RTC_R_DATE  ; break;
    case RTC_REGISTER_MONTH : RTCRegisterAddr = RTC_R_MONTH ; break;
    case RTC_REGISTER_WEEK  : RTCRegisterAddr = RTC_R_WEEK  ; break;
    case RTC_REGISTER_YEAR  : RTCRegisterAddr = RTC_R_YEAR  ; break;
    default:*value = 0xFF; return;
  }
  
  if(getOrSetFlag == RTC_DS1302_SET)  //写地址，最后一位置0，其余不变
  {
    RTCRegisterAddr &= ~0x01;
    valueTemp = HalDecimal2BCD(*value);
    
    // Disable write protect
    HalRTCSingleWrite(RTC_W_CONTROL,0x00);

    // Write data
    HalRTCSingleWrite(RTCRegisterAddr,valueTemp);
    
    // Enable write protect
    HalRTCSingleWrite(RTC_W_CONTROL,0x80);
    
    *value = 0xFF;
  }
  else if(getOrSetFlag == RTC_DS1302_GET)
  {
    valueTemp = HalRTCSingleRead(RTCRegisterAddr);  //Get value
    switch(registerName)  //Get useful value
    {
      case RTC_REGISTER_SEC   : valueTemp &= 0x7F ; break;  //bit:0-6
      case RTC_REGISTER_MIN   : valueTemp &= 0x7F ; break;  //bit:0-6
      case RTC_REGISTER_HOUR  : valueTemp &= 0x3F ; break;  //bit:0-5
      case RTC_REGISTER_DATE  : valueTemp &= 0x3F ; break;  //bit:0-5
      case RTC_REGISTER_MONTH : valueTemp &= 0x1F ; break;  //bit:0-4
      case RTC_REGISTER_WEEK  : valueTemp &= 0x07 ; break;  //bit:0-2
      case RTC_REGISTER_YEAR  : valueTemp &= 0xFF ; break;  //bit:0-7
    }    
    *value = HalBCD2Decimal(valueTemp);
  }
  else
  {
    *value = 0xFF;
  }
  return;
}

/**************************************************************************************************
 * @fn      HalRTCDS1302Work
 *
 * @brief   Control DS1302 clk enable or disable by set CH on register sec bit7
 *
 * @param   Enable or disable DS1302 work
 *
 * @return  
 **************************************************************************************************/
void HalRTCDS1302Work(uint8 DS1302Enable)
{
   //Disable write protect
  HalRTCSingleWrite(RTC_W_CONTROL,0x00);

  if( DS1302Enable == RTC_DS1302_CLK_ENABLE )
    HalRTCSingleWrite(RTC_W_SEC,0x00);
  else
    HalRTCSingleWrite(RTC_W_SEC,0x80);
    
    
  //Enable write protect
  HalRTCSingleWrite(RTC_W_CONTROL,0x80);
}


/**************************************************************************************************
 * @fn      HalRTCDS1302Work
 *
 * @brief   Control DS1302 clk enable or disable by set CH on register sec bit7
 *
 * @param   True is OK,flase is lose power,need configuration.
 *
 * @return  
 **************************************************************************************************/
bool HalRTCDS1302OK(void)
{
  uint8 CH = HalRTCSingleRead(RTC_R_SEC)&0x80;
  return (CH == 0x80)?FALSE:TRUE;
}


/**************************************************************************************************
 * @fn      HalRTCWriteByte
 *
 * @brief   Wrie a byte to DS1302.Used by burst mode
 *
 * @param   write data
 *
 * @return  
 **************************************************************************************************/
void HalRTCWriteByte(uint8 data)
{
  uint8 i;
  
  for(i = 0; i < 8; i++)
  {
    if( data & 0x01 )
      DS1302_IO_HIGH();
    else
      DS1302_IO_LOW();
    
    //Tdc = 200ns + Tf = 2us + Tcl = 1us All time 5us enough
    DS1302_WRITE_DELAY();
    DS1302_SCLK_HIGH(); // rising edge write data
    

    //Tr = 2us + Tcdh = 280ns + Tch = 1us All time 5us enough
    DS1302_WRITE_DELAY();
    DS1302_SCLK_LOW();
    
    data >>= 1;
  }
}


/**************************************************************************************************
 * @fn      HalRTCReadByte
 *
 * @brief   Read a byte from DS1302.Used by burst mode
 *
 * @param   
 *
 * @return  read data
 **************************************************************************************************/
uint8 HalRTCReadByte(void)
{
  uint8 i;
  uint8 data = 0;
  
  //set IO in
  DS1302_IO_IN();
  
  // Read data
  for( i = 0; i < 8; i++ )
  {
    //Tr = 2us + Tccz = 280ns All time 5 us enough
    DS1302_READ_DELAY();
    DS1302_SCLK_LOW();   //SCLK falling rising read data
    //Tf = 2us + Tcdd = 800ns All time 5 us enough
    DS1302_READ_DELAY(); 
    //当SCLK拉高时，DS1302会释放IO，如果原来输出0，IO电压会不断
    //升高，可能造成数据读取错误，所以要先读取数据再拉高SCLK
    
    //IO是P2.6 所以返回0或0100 0000
    if( GET_DS1302_IO() & DS1302_IO_READ_HIGH )
      data = data + (1<<i);
    
    DS1302_SCLK_HIGH();
  }
  
  DS1302_READ_DELAY();
  DS1302_IO_OUT();  // IO default is out, just read change to in
  
  return data;
}


/**************************************************************************************************
 * @fn      HalRTCCalBurstRead
 *
 * @brief   read all 8 register about calendar
 *
 * @param   RTC struct to store data
 *
 * @return  
 **************************************************************************************************/
void HalRTCCalBurstRead(uint8 *calendar)
{
  uint8 i;
  
  //Set CE high Tcc = 4us
  DS1302_CE_HIGH();
  DS1302_CE_DELAY();  
  
  //Enable Burst Read。从这个函数中出来时已经是一个下降沿
  HalRTCWriteByte(RTC_R_CLK_BURST); 
  
  //Read 8 bit data
  for( i = 0; i < 8; i++ )
  {
    *calendar = HalRTCReadByte();
    calendar++;
  }
  
  //Set CE low Tcwh = 4us
  DS1302_CE_LOW();
  DS1302_CE_DELAY(); 
  
  DS1302_SCLK_LOW();
  DS1302_IO_LOW(); 
}


/**************************************************************************************************
 * @fn      HalRTCCalBurstWrite
 *
 * @brief   write all 8 register about calendar
 *
 * @param   RTC struct to be writed
 *
 * @return  
 **************************************************************************************************/
void HalRTCCalBurstWrite(uint8 *calendar)
{
  uint8 i;
  
  //Disable write protect
  HalRTCSingleWrite(RTC_W_CONTROL,0x00);  
    
  //Set CE high Tcc = 4us
  DS1302_CE_HIGH();
  DS1302_CE_DELAY(); 
  
  //Enable Burst Write
  HalRTCWriteByte(RTC_W_CLK_BURST);
  
  //Write 8 bit data
  for( i = 0; i < 8; i++ )
  {
    HalRTCWriteByte(*calendar);
    calendar++;
  }
  
  //Set CE low Tcwh = 4us
  DS1302_CE_LOW();
  DS1302_CE_DELAY(); 
  
  DS1302_SCLK_LOW();
  DS1302_IO_LOW();
  
  //Enable write protect
  HalRTCSingleWrite(RTC_W_CONTROL,0x80);
}


/**************************************************************************************************
 * @fn      HalRTCSingleWrite
 *
 * @brief   write a data to a register
 *
 * @param   addr -- write register address
 *          data -- write data
 *
 * @return  
 **************************************************************************************************/
void HalRTCSingleWrite(uint8 addr,uint8 data)
{
  uint8 i;

  //Set CE high Tcc = 4us
  DS1302_CE_HIGH();
  DS1302_CE_DELAY();

 // Write addr 
  for( i = 0; i < 8 ; i++)
  {
    if( addr & 0x01 )
      DS1302_IO_HIGH();
    else
      DS1302_IO_LOW();
    
    //Tdc = 200ns + Tf = 2us + Tcl = 1us All time 5us enough
    DS1302_WRITE_DELAY();
    DS1302_SCLK_HIGH(); // rising edge write data
    

    //Tr = 2us + Tcdh = 280ns + Tch = 1us All time 5us enough
    DS1302_WRITE_DELAY();
    DS1302_SCLK_LOW();
    
    addr >>= 1;
  }
  
  // Write data
  for( i = 0; i < 8; i++)
  {
    if( data & 0x01 )
      DS1302_IO_HIGH();
    else
      DS1302_IO_LOW();
    
    //Tdc = 200ns + Tf = 2us + Tcl = 1us All time 5us enough
    DS1302_WRITE_DELAY();
    DS1302_SCLK_HIGH(); // rising edge write data
    

    //Tr = 2us + Tcdh = 280ns + Tch = 1us All time 5us enough
    DS1302_WRITE_DELAY();
    DS1302_SCLK_LOW();
    
    data >>= 1;
  }  
  
  //Set CE low Tcwh = 4us
  DS1302_CE_LOW();
  DS1302_CE_DELAY(); 
  
  DS1302_SCLK_LOW();
  DS1302_IO_LOW();
}


/**************************************************************************************************
 * @fn      HalRTCSingleRead
 *
 * @brief   read data from a register
 *
 * @param   read register address
 *
 * @return  data on register
 **************************************************************************************************/
uint8 HalRTCSingleRead(uint8 addr)
{
  uint8 data = 0;
  uint8 i;
  
  //Set CE high Tcc = 4us
  DS1302_CE_HIGH();
  DS1302_CE_DELAY();
  
  // Write addr 
  for( i = 0; i < 8 ; i++)
  {
    if( addr & 0x01 )
      DS1302_IO_HIGH();
    else
      DS1302_IO_LOW();
    
    //Tdc = 200ns + Tf = 2us + Tcl = 1us All time 5us enough
    DS1302_WRITE_DELAY();
    DS1302_SCLK_HIGH(); // rising edge write data
    
    if( i < 7 )
    {
      //Tr = 2us + Tcdh = 280ns + Tch = 1us All time 5us enough
      DS1302_WRITE_DELAY();
      DS1302_SCLK_LOW();
    }
    addr >>= 1;
  }
  
  // read data
  DS1302_IO_IN();
  
  for( i = 0; i < 8; i++ )
  {
    //Tr = 2us + Tccz = 280ns All time 5 us enough
    DS1302_READ_DELAY();
    DS1302_SCLK_LOW();   //SCLK falling rising read data
    //Tf = 2us + Tcdd = 800ns All time 5 us enough
    DS1302_READ_DELAY(); 
    //当SCLK拉高时，DS1302会释放IO，如果原来输出0，IO电压会不断
    //升高，可能造成数据读取错误，所以要先读取数据再拉高SCLK
    
    //IO是P2.6 所以返回0或0100 0000
    if( GET_DS1302_IO() & DS1302_IO_READ_HIGH )
      data = data + (1<<i);
    
    DS1302_SCLK_HIGH();
  }
  
  DS1302_READ_DELAY();
  DS1302_IO_OUT();  // IO default is out, just read change to in
  
  //Set CE low Tcwh = 4us
  DS1302_CE_LOW();
  DS1302_CE_DELAY(); 
  
  DS1302_SCLK_LOW();
  DS1302_IO_LOW();
  
  return data;
}


/**************************************************************************************************
 * @fn      HalDecimal2BCD
 *
 * @brief   decimal to BCD code
 *
 * @param   decimal--must less than 100
 *
 * @return  BCD
 **************************************************************************************************/
uint8 HalDecimal2BCD(uint8 decimal)
{
  uint8 BCD;
  uint8 BCD_high,BCD_low;
  
  BCD_high = decimal / 10;
  BCD_low  = decimal - BCD_high*10;
  
  BCD = ((BCD_high << 4) & 0xF0) | (BCD_low & 0x0F);
  
  return BCD;
}


/**************************************************************************************************
 * @fn      HalBCD2Decimal
 *
 * @brief   BCD code to decimal
 *
 * @param   BCD code
 *
 * @return  deciaml
 **************************************************************************************************/
uint8 HalBCD2Decimal(uint8 BCD)
{
  uint8 decimal;

  decimal = ((BCD>>4) & 0x0F) * 10 + (BCD & 0x0F);

  return decimal;
}


/**************************************************************************************************
 * @fn      HalRTCStructInit
 *
 * @brief   Init RTC struct
 *
 * @param   
 *
 * @return  
 **************************************************************************************************/
void HalRTCStructInit(RTCStruct_t *RTCStruct,uint8 sec,uint8 min,uint8 hour,uint8 date,
                             uint8 month,uint8 week,uint8 year)
{
  RTCStruct->sec = sec;
  RTCStruct->min = min;
  RTCStruct->hour = hour;
  RTCStruct->date = date;
  RTCStruct->month = month;
  RTCStruct->week = week;
  RTCStruct->year = year;
  RTCStruct->WP = 0;
}



/**************************************************************************************************
 * @fn      halMcuWaitUs
 *
 * @brief   wait for x us. @ 8MHz MCU clock it takes 8 "nop"s for 1 us delay.
 *
 * @param   x us. range[0-65536]
 *
 * @return  None
 **************************************************************************************************/
void halMcuWaitUs(uint16 microSecs)
{
  while(microSecs--)
  {
    /* 8 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop");
  }
}

