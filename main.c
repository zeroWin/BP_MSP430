//******************************************************************************
//  MSP430F149-Noninvasive Blood Pressure Monitor
//  Nov 2013
//  Built with IAR Embedded Workbench Version: 5.3
//******************************************************************************
#include  <msp430x14x.h>
#include  <math.h>
#include  "typedefs.h"
#include "oled_main.h"
#include "hal_type.h"
#include "hal_uart_cc2530.h"
#include "hal_rtc_ds1302.h"
#include "hal_delay.h"
#include "main.h"
#include "hal_BP_measure.h"
/*****************************************************************************
变量定义
******************************************************************************/
u32 BP_DC;
u32 BP_AC;
u16 BP_MAX;
u16 BP_MIN;
u16 AC_MAX;
int BP_MID;
int BP_P2P;
int BP_Low;
int BP_High;
float k1=0;
float k2=0;
float abp=0;
float bp;
int SYS;
int DIA;
u32 pressure;
int display_time=0;
u8  show=0;
struct Pointss
{
  int bp;
  int p2p;
} Points[100];                                     //DC、AC矩阵
unsigned int n_P=0;                                //number of points
unsigned int last_is_low=0;

unsigned int delay_times=0;
unsigned char step=101;

int Prepressure=3500;
//int Prepressure=4000;
int Device_waite_time1;
/*****************************************************************************
函数声明
*****************************************************************************/
void Init_Clock();
void Init_Port1();
void Init_Port2();
void Init_Port4();
void Init_Port5();
void Init_Port6();
void Init_TimerA();
void Stop_BPMeasure(void);
void Start_BPMeasure(void);
extern void usDelay(int microeconds);
extern void Write_8402(u16 data_8402);
extern void delay_ms(u16 nms);
extern void delay_s(u16 ns);
extern void PUMP_ON(u16 motorspeed);
extern void PUMP_OFF(void);
extern void VALVE_ON(void);
extern void VALVE_OFF(void);
extern void write_oled_command(unsigned char ucCmd);
extern void write_oled_data(unsigned char ucData);
unsigned int adc2mmhg(unsigned int adc);
void RBUF_PROCESS(void);
int abs(int a);
void Find_Hign_Low_BP();
void SendDCAndACToCC2530(uint16 DC_temp,uint16 AC_temp,uint16 BP_HIGH_temp,uint16 BP_LOW_temp);
void SendBP_HighAndLowoCC2530(uint16 BP_HIGH_temp,uint16 BP_LOW_temp);

BPSystemStatus_t BPSystemStatus;
uint8 writeNum;
uint8 bufferFullFlag;
/*****************************************************************************
主程序
*****************************************************************************/
void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                //关闭看门狗电路
  uint16 *dataTemp;
  Init_Clock();
  Init_Port1();
  Init_Port2();
  P2OUT &= ~BIT4;
  UART1_Config_Init();                     // UART-CC2530初始化
  HalRTCInit();                            // DS1302 初始化
  Init_Port4();
  Init_Port6();                             
  //VALVE_OFF();                             //通道0 控制电磁阀
  VALVE_ON();
  PUMP_OFF();                              //通道1 控制气泵
  Init_TimerA();
  step=101;
  oledinit();
  OLED_ShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
  OLED_ShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");    
  Show_Wait_Symbol("Off_IDLE");    
  OLED_Refresh_Gram();
  delay_ms(10);
  BPSystemStatus =  BP_OFFLINE_IDLE;
  _EINT();
  while(1)
  {
    if(BPSystemStatus == BP_OFFLINE_IDLE || BPSystemStatus == BP_ONLINE_IDLE)       //离线等待状态
    {                                              
      //step=101;
      OLED_SLEEP(0);                           //打开显示器 
      if(Device_waite_time1 <15000)
      {
        delay_ms(2);
        Device_waite_time1++;
      }
      else                                     //等待15s，如果无操作则进入休眠状态
      {
        if(BPSystemStatus == BP_OFFLINE_IDLE) // 离线空闲状态
          BPSystemStatus = BP_OFF_SLEEP;
        if(BPSystemStatus == BP_ONLINE_IDLE) // 在线空闲状态
          BPSystemStatus = BP_ON_SLEEP;
      }
    }
    else if(BPSystemStatus == BP_OFF_SLEEP || BPSystemStatus == BP_ON_SLEEP)       //进入睡眠状态
    {
      OLED_SLEEP(1);                             //关闭显示器        
      _BIS_SR(LPM0_bits + GIE);                  //进入低功耗
    }
    else if(BPSystemStatus == BP_OFFLINE_MEASURE || BPSystemStatus == BP_ONLINE_MEASURE)     //测量状态
    {
      if(bufferFullFlag == 1) // buffer满了
      {
        HalBPMeasReadFromBuf( &dataTemp );
        if(BPSystemStatus == BP_OFFLINE_MEASURE) // 离线测量状态
        { // 写入SD卡
          
        }
        if(BPSystemStatus == BP_ONLINE_MEASURE) // 在线测量状态
        {
          //发送给CC2530
          UART1_Send_Buffer((uint8 *)dataTemp,68);
        }
        bufferFullFlag = 0;
      }
      if(step==30) // step=30说明测量结果
      { 
        Find_Hign_Low_BP();                    //计算高低血压值
        // BP_High 和 BP_Low
        step =101;
        if(BPSystemStatus == BP_OFFLINE_MEASURE) // 离线测量状态
        {
          Show_Wait_Symbol("Off_IDLE");
          BPSystemStatus = BP_OFFLINE_IDLE;
        }
        if(BPSystemStatus == BP_ONLINE_MEASURE) // 在线测量状态
        {
          // 发送最后结果
          SendBP_HighAndLowoCC2530(BP_High,BP_Low);
          Show_Wait_Symbol("On_IDLE ");
          BPSystemStatus = BP_ONLINE_IDLE;
        }
        OLED_ShowString(0,30,32,"        ");  //8个空格，完全清空
        
        if(BP_High<100)
          OLED_ShowNum(SPO2_Show2Num_Start_X,SPO2_Show2Num_Start_Y,BP_High,2,32);              
        else
          OLED_ShowNum(SPO2_Show3Num_Start_X,SPO2_Show3Num_Start_Y,BP_High,3,32);
        if(BP_Low < 100)
          OLED_ShowNum(HR_Show2Num_Start_X,HR_Show2Num_Start_Y,BP_Low,2,32);              
        else
          OLED_ShowNum(HR_Show3Num_Start_X ,HR_Show3Num_Start_Y,BP_Low,3,32);
        OLED_Refresh_Gram();                  
        BPSystemStatus = BP_OFFLINE_IDLE;
      }
    } 
  }              
}

/*****************************************************************************
定时器A中断服务程序
*****************************************************************************/
uint16 aa = 0;
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
     static long BP_AC_AVE=0;  
     static long BP_AC_AVE_times=0;
     P2OUT ^= 0x10;     
     ADC12CTL0 |= ADC12SC;                     // Start conversion
     BP_DC = ADC12MEM0;                        // Move A0 results
     BP_AC = ADC12MEM1;                        // Move A1 results
     if(delay_times<30000)
       delay_times++;                            //延时次数累加
     if(display_time==255)
     {
       display_time=0;
       show=1;
     }
     else
     {
       display_time++;
     }       
     //pressure=((BP_DC*3.3/4095)/233.56)*1000*15.52;
     bp=((BP_DC*3.3/4095)/233.56)*1000*15.52;
     pressure=(u32)(bp);
     if(show==1)
     {
       show=0;
       if(pressure<10)
       {
         OLED_ShowNum(SPO2_Show2Num_Start_X+27,SPO2_Show2Num_Start_Y,0,1,32);
         OLED_ShowNum(SPO2_Show2Num_Start_X+43,SPO2_Show2Num_Start_Y,0,1,32);
         OLED_ShowNum(SPO2_Show2Num_Start_X+59,SPO2_Show2Num_Start_Y,pressure,1,32);
       }
       else if((pressure<100)&&(pressure>9))
       {
         OLED_ShowNum(SPO2_Show2Num_Start_X+27,SPO2_Show2Num_Start_Y,0,1,32);
         OLED_ShowNum(SPO2_Show2Num_Start_X+43,SPO2_Show2Num_Start_Y,pressure,2,32);
       }
       else
         OLED_ShowNum(SPO2_Show3Num_Start_X+27,SPO2_Show3Num_Start_Y,pressure,3,32);
       OLED_Refresh_Gram();             
     }
     switch(step)
     {       
     case 0:                                    //变量初始化
       delay_times=0;
       BP_AC_AVE=0;
       BP_AC_AVE_times=0;
       BP_MIN=BP_AC_AVE;
       BP_MAX=0;
       AC_MAX=0;
       last_is_low=0;              
       step=5;
       break;
     
     case 5:                                   //BP_AC基准求取并控制气泵充气
       BP_AC_AVE+=BP_AC;
       BP_AC_AVE_times++;
       if(BP_AC_AVE_times>=128)
       { 
         BP_AC_AVE=BP_AC_AVE>>7;
         PUMP_ON(0x0100);             //开始充气
         step=10;
       }                
       break;
       
     case 10:                                 //判断是否到达预置压力
       if(BP_DC>=Prepressure)
       {
         PUMP_OFF();                  //停止充气，此时匀速泄气阀已经开始泄气
         //delay_s(2);
         delay_times=0;
         BP_MIN=BP_AC_AVE;
         BP_MAX=0;
         last_is_low=1;
         n_P=0;                       //number of points
         step=20;
       }
       break; 
       
     case 20:                                 //将BP_DC和BP_AC通过串口发送                 
       // 只传输此时的采样值
       if(BPSystemStatus == BP_OFFLINE_MEASURE) // 离线测量状态
       {
       }
       if(BPSystemStatus == BP_ONLINE_MEASURE) // 在线测量状态
       {
         //SendDCAndACToCC2530(BP_DC,BP_AC,100,100);
         SendDCAndACToCC2530(aa,aa,100,100);
         aa++;
         if(aa==600)
           aa = 0;
       }
       if(last_is_low==1)
       {
         if(BP_AC>BP_AC_AVE)
         {
           last_is_low=0;
           Points[n_P].bp=BP_DC;
           delay_times=0;
         }                
       }
       else
       {
         if(BP_MAX<BP_AC)BP_MAX=BP_AC;
         if(BP_AC<BP_AC_AVE && delay_times>=250)
         {
           last_is_low=1;
           BP_P2P=BP_MAX;
           Points[n_P].p2p=BP_MAX;
           if(n_P<99)n_P++;             //number of points
           BP_MAX=BP_AC_AVE;
         }
       }              
       if(BP_DC<930)                  //快速放气
       {
         PUMP_OFF();
         VALVE_ON();
         //delay_s(5);
       }      
       if(BP_DC<300)
       {
         PUMP_OFF();
         VALVE_ON();
         CCTL0 &= ~CCIE;                       //关闭定时器A中断
         step=30;
       }
       break;
     
     default:
       PUMP_OFF();
       VALVE_OFF();
       break;
     }
}


/*****************************************************************************
UART中断服务程序
*****************************************************************************/
#pragma vector=USART0RX_VECTOR
__interrupt void USART0_RX_ISR(void)
{
  static uint8 receiveFlag = 0;
  static uint8 controlMessage = 0;
  uint8 receiveMessage = RXBUF0;
  switch(receiveFlag)
  {
    case 0:
      if(receiveMessage == DATA_START)
        receiveFlag = 1;
      else
        receiveFlag = 0;
      break;
    case 1:
        controlMessage = receiveMessage;
        receiveFlag = 2;
      break;
    case 2:
      if(receiveMessage == DATA_END)
      {
        switch(controlMessage)
        {
          case START_MEASURE: //启动测量命令
            if(BPSystemStatus == BP_ON_SLEEP || BPSystemStatus == BP_ONLINE_IDLE) // 处于睡眠或在线空闲状态
            {
              if(BPSystemStatus == BP_ON_SLEEP)
              {
                Device_waite_time1 = 0;
                LPM0_EXIT;
                OLED_SLEEP(0);
              }
              BPSystemStatus = BP_ONLINE_MEASURE;

              OLED_Clear();
              OLED_ShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
              OLED_ShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");   
              Show_Wait_Symbol("On_Go   "); 
              OLED_ShowString(0,30,32,"        ");  //8个空格，完全清空
              OLED_Refresh_Gram();                
              Start_BPMeasure();
            }
            break;
            
          case STOP_MEASURE: // 停止测量
            if(BPSystemStatus == BP_ONLINE_MEASURE) //处于在线测量状态
            {
              Stop_BPMeasure();
              BPSystemStatus = BP_ONLINE_IDLE;
              OLED_Clear();
              OLED_ShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
              OLED_ShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");    
              Show_Wait_Symbol("On_IDLE ");    
              OLED_Refresh_Gram();
            }
            break;
            
          case SYNC_MEASURE: // 同步消息，暂不处理
            break;
            
          case FIND_NWK:  // 正在找网消息
            if(BPSystemStatus == BP_ONLINE_MEASURE || BPSystemStatus == BP_OFFLINE_MEASURE) // 如果是正在在线或离线测量
            {
              Stop_BPMeasure();
              OLED_Clear();
            }
            if(BPSystemStatus == BP_OFF_SLEEP || BPSystemStatus == BP_ON_SLEEP) // 在线或离线睡眠状态
            {
              Device_waite_time1 = 0;
              LPM0_EXIT;
              OLED_SLEEP(0);      
            }
            BPSystemStatus = BP_FIND_NETWORK;
            OLED_ShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
            OLED_ShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");    
            Show_Wait_Symbol("FIND_NWK");    
            OLED_Refresh_Gram();
            break;
            
          case END_DEVICE:  // 找到网络消息 只可能从FIND_NWK到这个状态
            BPSystemStatus = BP_ONLINE_IDLE;
            Show_Wait_Symbol("On_IDLE ");    
            OLED_Refresh_Gram();
            break;
            
          case CLOSEING:   // 正在关闭网络消息
            if(BPSystemStatus == BP_ONLINE_MEASURE) // 在线测量状态关闭网络
              Stop_BPMeasure();
            if(BPSystemStatus == BP_ON_SLEEP) // 睡眠状态下关闭网络
            {
              Device_waite_time1 = 0;
              LPM0_EXIT;
              OLED_SLEEP(0);                 
            }            
            // 在线空闲状态或寻找网络状态下下关闭网络
            // do nothing
            BPSystemStatus = BP_CLOSING;
            OLED_Clear();
            OLED_ShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
            OLED_ShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");    
            Show_Wait_Symbol("CLOSING ");    
            OLED_Refresh_Gram();           
            break;
            
          case CLOSE_NWK:
            BPSystemStatus = BP_OFFLINE_IDLE;
            Show_Wait_Symbol("Off_IDLE");
            OLED_Refresh_Gram();             
            break;
            
        }
      }
      receiveFlag = 0;
      break;
  }  
}


/************************************************************************
                        按键控制中断服务程序																                 
*************************************************************************/
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{    
  P1IE &= ~BIT3;                                                         //P1.3 interrupt DISABLE
  Device_waite_time1 = 0;                                                //等待状态计数值清0
  if(BPSystemStatus == BP_OFF_SLEEP || BPSystemStatus == BP_ON_SLEEP)    //处于睡眠态，长按短按都是进入等待态
  {
    LPM0_EXIT;
    OLED_Clear();
    OLED_ShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
    OLED_ShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");
    if(BPSystemStatus == BP_OFF_SLEEP) // 离线睡眠状态
    {
      BPSystemStatus = BP_OFFLINE_IDLE;    
      Show_Wait_Symbol("Off_IDLE");    
    }
    if(BPSystemStatus == BP_ONLINE_IDLE) // 在线睡眠状态
    {
      BPSystemStatus = BP_ONLINE_IDLE;    
      Show_Wait_Symbol("On_IDLE ");   
    }
    OLED_Refresh_Gram();
  }
  else                                                                   //处于等待状态或是测量状态有按键按下
  {
    int Press_type = 0;                                                  //0表示短按，1表示长按
    //判断是长按还是短按
    for(Press_type = 0;Press_type < 1000; ++Press_type)                  //按下约3s
    {
      if(P1IN&BIT3)                                                      //P1.3是高电平，表示松开按键
        break;
      delay_ms(2);
    }
    if(Press_type != 1000)                                               //短按
      Press_type = 0;
    else
      Press_type = 1;                                                    //长按
    if(BPSystemStatus == BP_OFFLINE_IDLE || BPSystemStatus == BP_ONLINE_IDLE) //处于在线/离线空闲状态
    {
      if(Press_type == 0)                                                //短按，开始测量
      {
        OLED_Clear();
        OLED_ShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
        OLED_ShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");
        if(BPSystemStatus == BP_OFFLINE_IDLE) // 离线空闲状态
        {
          Show_Wait_Symbol("Off_Go  ");
          BPSystemStatus = BP_OFFLINE_MEASURE;
        }
        if(BPSystemStatus == BP_ONLINE_IDLE) // 在线空闲状态
        {
          Show_Wait_Symbol("On_Go   ");
          BPSystemStatus = BP_ONLINE_MEASURE;
        }
        OLED_ShowString(0,30,32,"        ");  //8个空格，完全清空
        OLED_Refresh_Gram();                
        Start_BPMeasure();
      }
      else//长按，关屏
      {
        if(BPSystemStatus == BP_OFFLINE_IDLE) // 离线空闲状态
          BPSystemStatus = BP_OFF_SLEEP;
        if(BPSystemStatus == BP_ONLINE_IDLE) // 在线空闲状态
          BPSystemStatus = BP_ON_SLEEP;
        OLED_Clear();
      }
    }
    else if(BPSystemStatus == BP_OFFLINE_MEASURE || BPSystemStatus == BP_ONLINE_MEASURE)//处于在线或离线测量状态
    {
      if(Press_type == 0)//短按，停止测量
      {
        Stop_BPMeasure();
        OLED_Clear();
        OLED_ShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
        OLED_ShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");    
        if(BPSystemStatus == BP_OFFLINE_MEASURE) // 离线测量状态
        {
          Show_Wait_Symbol("Off_IDLE");
          BPSystemStatus = BP_OFFLINE_IDLE;
        }
        if(BPSystemStatus == BP_ONLINE_MEASURE) // 在线测量状态
        {
          Show_Wait_Symbol("On_IDLE ");
          BPSystemStatus = BP_ONLINE_IDLE;
        }
        OLED_Refresh_Gram();
      }
      else//长按，关屏
      {
        Stop_BPMeasure();
        if(BPSystemStatus == BP_OFFLINE_MEASURE) // 离线测量状态
          BPSystemStatus = BP_OFF_SLEEP;
        if(BPSystemStatus == BP_ONLINE_MEASURE) // 在线测量状态
          BPSystemStatus = BP_ON_SLEEP;
        OLED_Clear();
      }
    }
  }
  P1IFG &= ~BIT3;             // P1.3 IFG cleared
  P1IE |= BIT3;               // P1.3 interrupt enabled
}


/*********************************************************************
 * @fn      Start_BPMeasure()
 *
 * @brief   Start BP measure
 *
 * @param   none
 *
 * @return  none
 */
void Start_BPMeasure(void)
{
  // 申请空间
  if(BPSystemStatus == BP_OFFLINE_IDLE) // 离线空闲状态
  {
  }
  if(BPSystemStatus == BP_ONLINE_IDLE) // 在线空闲状态
    HalBPMeasStart(BP_BUFFER_FOR_ZIGBEE);
  
  writeNum = 0;
  bufferFullFlag = 0;
  VALVE_OFF();
  step=0;
  CCTL0 |= CCIE;                        //启动定时器A中断
}

/*********************************************************************
 * @fn      Stop_BPMeasure()
 *
 * @brief   Stop BP measure
 *
 * @param   none
 *
 * @return  none
 */
void Stop_BPMeasure(void)
{
  CCTL0 &= ~CCIE;
  // 释放申请的空间
  HalBPMeasStop();
  
  step =101;
  PUMP_OFF();
  VALVE_ON(); //放气
}


/*****************************************************************************
电压-压力转换
*****************************************************************************/
unsigned int adc2mmhg(unsigned int adc)
{
  abp=((adc*3.3/4095)/233.56)*1000*15.52;
  return (int)(abp);
  //return (int)((adc*3.3/4095)/233.56)*1000*15.52;
}
/*****************************************************************************
绝对值函数
*****************************************************************************/
int abs(int a)
{
if(a<0)a=-a;
return a;
}
/*****************************************************************************
收缩压舒张压计算
*****************************************************************************/
void Find_Hign_Low_BP()
{
  unsigned int p2p_max=0,n_max=0;
  unsigned int p2p_min=65535;  
  unsigned int i=0;  
  unsigned int high,low;  
  int tmp_v[100];  
  //--------滤波----------
  for(i=0;i<n_P;i++) 
        tmp_v[i]=Points[i].p2p;
  for(i=3;i<n_P-3;i++)
  {
    if(abs(tmp_v[i]-tmp_v[i-1])+abs(tmp_v[i]-tmp_v[i+1])>500)
        Points[i].p2p= (tmp_v[i-2]+tmp_v[i-1]+tmp_v[i+1]+tmp_v[i+2])/4;
    Points[i].p2p=(Points[i-1].p2p+Points[i].p2p+Points[i+1].p2p)/3;
  }  
  //--------三点滑移平均------------
  for(i=0;i<n_P;i++) 
        tmp_v[i]=Points[i].p2p;
  for(i=3;i<n_P-3;i++)
  {
    Points[i].p2p=(tmp_v[i-1]+tmp_v[i]+tmp_v[i+1])/3;
  }  
  //---去掉直流分量（整体向下平移）--------------------------
  for(i=3;i<n_P-3;i++)
  {
    if(p2p_min>Points[i].p2p)
    {
      p2p_min=Points[i].p2p;
      //n_min=i;
    }    
  }
  for(i=3;i<n_P-3;i++)
  {
    Points[i].p2p=Points[i].p2p-p2p_min;
  }
  //--寻找最大值及其位置---------------------------
  for(i=3;i<n_P-3;i++)
  {
    if(p2p_max<Points[i].p2p)
    {
      p2p_max=Points[i].p2p;
      n_max=i;
    }    
  }
  //--确定收缩压系数-------------------------------
  if (p2p_max>120 && p2p_max<=135)
    k1=0.52;
  else if (p2p_max>110 && p2p_max<=120)
    k1=0.57;
  else if (p2p_max>70 && p2p_max<=110)
    k1=0.58;
  else if (p2p_max<=70)
    k1=0.64;
  else
    k1=0.45;
   //--确定收缩压系数-------------------------------
  if (p2p_max>120 && p2p_max<=140)
    k2=0.85;
  else if (p2p_max>60 && p2p_max<=120)
    k2=0.78;
  else if (p2p_max>50 && p2p_max<=60)
    k2=0.60;
  else if (p2p_max<=50)
    k2=0.50;
  else
    k2=0.85;
  //-------用系数找舒张压-------
  for(i=n_max;i<n_P-3;i++)
  {
    low=Points[i].bp;
    if(Points[i].p2p<(int)(k1*p2p_max))//系数待计算
    {
      break;
    }
  }
  //-------用系数找收缩压-------
  for(i=n_max;i>2;i--)
  {
   high=Points[i].bp;
   if(Points[i].p2p<(int)(k2*p2p_max))//系数待计算
    {
      break;
    }
  }  
  DIA=adc2mmhg(low);
  SYS=adc2mmhg(high);  
  //---零点修正----------  
  //BP_High=BP_High-90;
  //BP_Low=BP_Low-70; 
  if (DIA<=40)
    DIA=DIA-29;
  else if (DIA>40 && DIA<=50)
    DIA=DIA-9;
  else if (DIA>50 && DIA<=60)
    DIA=DIA-4;
  else if (DIA>60 && DIA<=90)
    DIA=DIA+2;
  else if (DIA>90 && DIA<=120)
    DIA=DIA+4;
  else
    DIA=DIA+9;
  
  if (SYS<=70)
    SYS=SYS-2;
  else if (SYS>70 && SYS<=90)
    SYS=SYS+3;
  else if (SYS>90 && SYS<=110)
    SYS=SYS+6;
  else if (SYS>110 && SYS<=160)
    SYS=SYS+12;
  else
    SYS=SYS+18;  
  BP_High=SYS;
  BP_Low=DIA; 
}


/*****************************************************************************
MSP430单片机时钟初始化
*****************************************************************************/
void Init_Clock()
{ 
       unsigned int i;
       BCSCTL1 = 0X00;			//将寄存器的内容清零
					//XT2震荡器开启
					//LFTX1工作在低频模式
					//ACLK的分频因子为1	32768			
       do 
       {
	   IFG1 &= ~OFIFG;              // 清除OSCFault标志
	   for (i = 0xFF; i > 0; i--);                
       }
       while ((IFG1 & OFIFG) == OFIFG); // 如果OSCFault =1   
					
       BCSCTL2 = 0X00;			//将寄存器的内容清零
       BCSCTL2 = SELM_2 + SELS;		//MCLK的时钟源为TX2CLK，分频因子为1 8M
                         		//SMCLK的时钟源为TX2CLK，分频因子为1 8M
}
/*****************************************************************************
端口P1初始化 AD8402 前端控制相关端口
*****************************************************************************/
void Init_Port1()
{
       P1DIR |= BIT0; //P1.0输出 AD8402   CLK
       P1DIR |= BIT1; //P1.1输出 AD8402   SDI
       P1DIR |= BIT2; //P1.2输出 AD8402   CS
       P1DIR &= ~BIT3;//P1.3按键S2 
       P1IES |=  BIT3;//捕捉下降沿中断                          		
       P1IFG &= ~BIT3;//IFG清零                           	
       P1IE  |=  BIT3;//中断使能  
}
/*****************************************************************************
端口P2初始化 电机-蜂鸣器-LED相关端口
*****************************************************************************/
void Init_Port2()
{
       P2DIR |= BIT0;//P2.0输出 控制电磁阀放气
       P2DIR |= BIT1;//P2.1输出 控制电磁阀放气
       P2DIR |= BIT3;//P2.3输出 控制蜂鸣器
       P2OUT |= BIT3;
       P2DIR |= BIT4;//P2.4输出 LED D5
}

/*****************************************************************************
端口P4初始化 OLED相关端口
*****************************************************************************/
void Init_Port4()
{
       P4DIR |= BIT0;                           //OLED CSB
       P4OUT |= BIT0;
       P4DIR |= BIT1;                           //OLED RESB
       P4OUT |= BIT1;
       P4DIR |= BIT2;                           //OLED D/C#;
       P4OUT |= BIT2;
       P4DIR |= BIT3;                           //OLED D0---------->SCLK
       P4OUT |= BIT3;                           
       P4DIR |= BIT4;                           //OLED D1---------->SDIN
       P4OUT |= BIT4;       
}
/*****************************************************************************
端口P5初始化 SD卡相关端口
*****************************************************************************/
void Init_Port5()
{
       P5DIR |= BIT0;                            //microsd CSB
       P5OUT |= BIT0;
       P5SEL |= 0x0E;                            // P5.1,2,3 SPI option select
       P5OUT &= ~0x01;
       P5DIR |= 0x01;                            //Reset Slave
       P5DIR &= ~0x01;
       U1CTL = CHAR + SYNC + MM + SWRST;         // 8-bit, SPI, Master
       U1TCTL = CKPL + SSEL1 + STC;              // Polarity, SMCLK, 3-wire
       U1BR0 = 0x02;                             // SPICLK = SMCLK/2
       U1BR1 = 0x00;
       U1MCTL = 0x00;
       ME2 |= USPIE1;                            // Module enable
       U1CTL &= ~SWRST;                          // SPI enable
       IE2 |= URXIE1 + UTXIE1;                   // RX and TX interrupt enable
}
/*****************************************************************************
端口P6初始化 ADC
*****************************************************************************/
void Init_Port6()
{
       P6SEL |= 0x03;                    // Enable A/D channel inputs
       ADC12CTL0 = ADC12ON+MSC+SHT0_2;  //+REFON+ REF2_5V;//+SHT0_8;//+REFON+ REF2_5V; 
                                        // Turn on ADC12, extend sampling time 
                                        // to avoid overflow of results
       ADC12CTL1 = SHP+CONSEQ_1;        // Use sampling timer, repeated sequence
       ADC12MCTL0 = INCH_0+SREF_0;      // ref+=AVcc, channel = A0
       ADC12MCTL1 = INCH_1+EOS+SREF_0;  // ref+=AVcc, channel = A1
       //ADC12IE = 0x02;                // Enable ADC12IFG.3
       ADC12CTL0 |= ENC;                // Enable conversions
}
/*****************************************************************************
定时器A初始化
*****************************************************************************/
void Init_TimerA()
{
  CCTL0 &= ~CCIE;                           // CCR0 interrupt Disabled
  CCR0 = 64-1;                              // 256,定时器每秒产生中断256次,采样率256Hz
  TACTL = TASSEL_1 + MC_1;                  // ACLK, upmode
}
/*****************************************************************************
气泵开
*****************************************************************************/
void PUMP_ON(u16 motorspeed)
{
       Write_8402(motorspeed);    
}
/*****************************************************************************
气泵关
*****************************************************************************/
void PUMP_OFF(void)
{
       Write_8402(0x01ffu);    
}
/*****************************************************************************
电磁阀打开
*****************************************************************************/
void VALVE_ON(void)
{
       P2OUT |= BIT0;
       P2OUT |= BIT1;
}
/*****************************************************************************
电磁阀关闭
*****************************************************************************/
void VALVE_OFF(void)
{
       P2OUT &= ~BIT0;
       P2OUT &= ~BIT1;
}
/*****************************************************************************
数字电位器控制子程序
*****************************************************************************/
void Write_8402(u16 data_8402)	 
{
       int i;
       P1OUT &= ~BIT2;                     //CS=0
       data_8402 <<= 6;
       for(i=10;i>0;i--)
	{
	  P1OUT &= ~BIT0;                  //CLK=0
	  if(data_8402&0x8000)
             P1OUT |= BIT1;                //SDI赋值
          else
             P1OUT &= ~BIT1;
             data_8402 <<= 1; 
             P1OUT |= BIT0;
             usDelay(1);
         }
       P1OUT &= ~BIT0;
       usDelay(1); 	
       P1OUT |= BIT2;
}

void SendDCAndACToCC2530(uint16 DC_temp,uint16 AC_temp,uint16 BP_HIGH_temp,uint16 BP_LOW_temp)
{
  BufOpStatus_t BufOpStatus;
  HalBPMeasWriteToBuf(DC_temp);
  HalBPMeasWriteToBuf(AC_temp);
  writeNum++;
  if(writeNum == 16)
  {
    HalBPMeasWriteToBuf(BP_HIGH_temp);
    BufOpStatus = HalBPMeasWriteToBuf(BP_LOW_temp);
    bufferFullFlag = 1;
    writeNum = 0;
  }
}

void SendBP_HighAndLowoCC2530(uint16 BP_HIGH_temp,uint16 BP_LOW_temp)
{
  uint8 i;
  for(i = 0; i < 64; ++i)
    UART1_Send_Byte(0xFF);
  
  UART1_Send_Byte(BP_HIGH_temp & 0x00FF);
  UART1_Send_Byte((BP_HIGH_temp & 0xFF00) >> 8);
  UART1_Send_Byte(BP_LOW_temp & 0x00FF);
  UART1_Send_Byte((BP_LOW_temp & 0xFF00) >> 8);  
}