/**************************************************************************************************
  Filename:       hal_battery_monitor.c
  Revised:        $Date: 2016-03-12 19:37:16 +0800 (Sat, 12 Mar 2016) $
  Revision:       $Revision: 1 $

  Description:    This file contains the interface to the battery monitor.


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
#include "hal_battery_monitor.h"
#include "msp430f1611.h"
#include "hal_oled.h"
#include "math.h"

/***************************************************************************************************
 *                                             CONSTANTS
 ***************************************************************************************************/
/* Batter Monitor enable/disable at P2.2
*/
#define BATTER_MONITOR_EN_PORT_SEL   P2SEL
#define BATTER_MONITOR_EN_PORT_DIR   P2DIR
#define BATTER_MONITOR_EN_PORT       P2OUT
#define BATTER_MONIROT_EN_PIN        2

/* Set ADC channel and resolution P6.2  Analog input A2 – ADC
*/
#define BATTER_MONITOR_PORT_SEL   P6SEL
#define BATTER_MONITOR_PORT_DIR   P6DIR
#define BATTER_MONIROT_PIN        2

   
/***************************************************************************************************
 *                                              MACROS
 ***************************************************************************************************/
#define BATTER_MINITOR_ENABLE       P2OUT |= (1 << BATTER_MONIROT_EN_PIN)
#define BATTER_MINITOR_DISABLE      P2OUT &= ~(1 << BATTER_MONIROT_EN_PIN)


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


/**************************************************************************************************
 * @fn      HalBattMonInit
 *
 * @brief   Initilize Battery Monitor
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalBattMonInit(void)
{
  // 初始化使能端口 
  BATTER_MONITOR_EN_PORT_SEL &= ~(1 << BATTER_MONIROT_EN_PIN);
  BATTER_MONITOR_EN_PORT_DIR |=  (1 << BATTER_MONIROT_EN_PIN);
  BATTER_MINITOR_DISABLE;
  
  // 初始化ADC端口
  BATTER_MONITOR_PORT_SEL |= (1 << BATTER_MONIROT_PIN);  // 设置端口为ADC功能
  BATTER_MONITOR_PORT_DIR &= ~(1 << BATTER_MONIROT_PIN); // 设置端口为输入

  ADC12MCTL2 = SREF_0 + INCH_2; // 参考电压AVCC 3.3V，通道2
  
 // HalGetBattVol();
}


/**************************************************************************************************
 * @fn      HalGetBattVol
 *
 * @brief   Get Battery Volage
 *
 * @param   none
 *
 * @return  Battery Volage
 **************************************************************************************************/
float HalGetBattVol(void)
{
  float tempVol;
  
//  // ADC12使能
//  ADC12CTL0 |= ADC12ON;
//  ADC12CTL0 |= ENC;
  
  BATTER_MINITOR_ENABLE;    // Enable BATT_MON_EN, P0.1 high
  
  ADC12CTL0 |= ADC12SC; // 启动转换
  while(!(ADC12IFG & BIT0)) // 等待转换结束
  //max value = 0xfff/2, battery voltage = input voltage x 2
  //ref volage=3.3V
  tempVol = ADC12MEM2;
  tempVol = (tempVol/4095)*3.3*2;
  ADC12IFG = 0; // 清空中断标志位
  BATTER_MINITOR_DISABLE;   // Disable BATT_MON_EN, P0.1 low
  
//  // 关闭AD节能
//  ADC12CTL0 &= ~ENC;
//  ADC12CTL0 &= ~ADC12ON;

  
  return tempVol;
}

/**************************************************************************************************
 * @fn      HalGetBattVol
 *
 * @brief   Show Battery Volage on OLED
 *
 * @param   monitor and show or show last monitor
 *
 * @return  0 Battery volage enough
            1 warring--Battery volage 20%
            2 warring--Battery gone   0%-10%
 **************************************************************************************************/
uint8 HalShowBattVol(uint8 fThreshold)
{
  static float fThreshold_temp;
  float fBattV;
  
  if(fThreshold == BATTERY_MEASURE_SHOW)//测量
  {
     fBattV = HalGetBattVol();
     fThreshold_temp = fBattV;
  }
  
  if(fThreshold_temp >= 4.000)
  {
    HalOledShowString(BATTER_CYCLE_X,BATTER_CYCLE_Y,12,"100%");
    HalOledShowPowerSymbol(BATTER_CYCLE_X+28,BATTER_CYCLE_Y,1,10);  //100%
  }
  else if(fThreshold_temp >= 3.900)
  {
    HalOledShowString(BATTER_CYCLE_X,BATTER_CYCLE_Y,12," 90%");
    HalOledShowPowerSymbol(BATTER_CYCLE_X+28,BATTER_CYCLE_Y,1,9);  //90%
  }
  else if(fThreshold_temp >= 3.800)
  {
    HalOledShowString(BATTER_CYCLE_X,BATTER_CYCLE_Y,12," 80%");
    HalOledShowPowerSymbol(BATTER_CYCLE_X+28,BATTER_CYCLE_Y,1,8);   //80%
  }
  else if(fThreshold_temp >= 3.700)
  {
    HalOledShowString(BATTER_CYCLE_X,BATTER_CYCLE_Y,12," 70%");
    HalOledShowPowerSymbol(BATTER_CYCLE_X+28,BATTER_CYCLE_Y,1,7);   //70%
  }
  else if(fThreshold_temp >= 3.600)
  {
    HalOledShowString(BATTER_CYCLE_X,BATTER_CYCLE_Y,12," 60%");
    HalOledShowPowerSymbol(BATTER_CYCLE_X+28,BATTER_CYCLE_Y,1,6);   //60%
  }
  else if(fThreshold_temp >= 3.500)
  {
    HalOledShowString(BATTER_CYCLE_X,BATTER_CYCLE_Y,12," 50%");
    HalOledShowPowerSymbol(BATTER_CYCLE_X+28,BATTER_CYCLE_Y,1,5);   //50%
  }
  else if(fThreshold_temp >= 3.400)
  {
    HalOledShowString(BATTER_CYCLE_X,BATTER_CYCLE_Y,12," 40%");
    HalOledShowPowerSymbol(BATTER_CYCLE_X+28,BATTER_CYCLE_Y,1,4);   //40%
  }
  else if(fThreshold_temp >= 3.300)
  {
    HalOledShowString(BATTER_CYCLE_X,BATTER_CYCLE_Y,12," 30%");
    HalOledShowPowerSymbol(BATTER_CYCLE_X+28,BATTER_CYCLE_Y,1,3);   //30%
  }
  else if(fThreshold_temp >= 3.200)
  {
    HalOledShowString(BATTER_CYCLE_X,BATTER_CYCLE_Y,12," 20%");
    HalOledShowPowerSymbol(BATTER_CYCLE_X+28,BATTER_CYCLE_Y,1,2);   //20%
  }
  else if(fThreshold_temp >= 3.100)
  {
    HalOledShowString(BATTER_CYCLE_X,BATTER_CYCLE_Y,12," 10%");
    HalOledShowPowerSymbol(BATTER_CYCLE_X+28,BATTER_CYCLE_Y,1,1);  //10%---警告电量，屏幕只显示LowPower
    return 1;
  }
  else
  {
    HalOledShowString(BATTER_CYCLE_X,BATTER_CYCLE_Y,12,"  0%");
    HalOledShowPowerSymbol(BATTER_CYCLE_X+28,BATTER_CYCLE_Y,1,0);  //0%---屏幕黑屏
    return 2;
  }
  return 0;
}

