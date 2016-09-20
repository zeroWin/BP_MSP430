//******************************************************************************
//  MSP430F149-Noninvasive Blood Pressure Monitor
//  Nov 2013
//  Built with IAR Embedded Workbench Version: 5.3
//******************************************************************************
#include  <msp430f1611.h>
#include  <math.h>
#include <string.h>
#include <stdlib.h>
#include  "typedefs.h"
#include "hal_oled.h"
#include "hal_battery_monitor.h"
#include "hal_type.h"
#include "hal_uart_cc2530.h"
#include "hal_rtc_ds1302.h"
#include "hal_delay.h"
#include "main.h"
#include "hal_BP_measure.h"
#include "hal_SDcard.h"
#include "exfuns.h"
#include "ff.h"
/*****************************************************************************
��������
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
} Points[100];                                     //DC��AC����
unsigned int n_P=0;                                //number of points
unsigned int last_is_low=0;

unsigned int delay_times=0;
unsigned char step=101;

int Prepressure=3500;
//int Prepressure=4000;
int Device_waite_time1;
/*****************************************************************************
��������
*****************************************************************************/
void Init_Clock();
void Init_Port1();
void Init_Port2();
void Init_Port4();
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
void Find_Hign_Low_BP();
void SendDCAndACToCC2530(uint16 DC_temp,uint16 AC_temp,uint16 BP_HIGH_temp,uint16 BP_LOW_temp);
void SendBP_HighAndLowoCC2530(uint16 BP_HIGH_temp,uint16 BP_LOW_temp);
void SendDCAndACToSDcard(uint16 DC_temp,uint16 AC_temp);
void GenericApp_GetWriteName(void);
bool GenericApp_OpenDir(void);
void SyncData(void);

BPSystemStatus_t BPSystemStatus;
uint16 writeNum;
uint8 bufferFullFlag;
char fileName[30]; // store file name
char pathname[30]; // read file name
/*****************************************************************************
������
*****************************************************************************/
void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                //�رտ��Ź���·
  uint16 *dataTemp;
  Init_Clock();
  Init_Port1();
  Init_Port2();
  P2OUT &= ~BIT4;
  

  // Init UART to CC2530
  UART1_Config_Init(); 
  // RTC��ʼ��
  HalRTCInit(); 
  // Initialize OLED
  HalOledInit();    
  
  // SD����ʼ��
  while(SD_Initialize());
  exfuns_init();
  f_mount(0,fs);      // �����ļ�ϵͳ  
  f_mkdir("0:B");     // �����ļ���  
  
  Init_Port6();                             
  HalBattMonInit();   //Initialize Battery monitor
  
  VALVE_ON();
  PUMP_OFF();                              //ͨ��1 ��������
  Init_TimerA();
  step=101;
  HalOledShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
  HalOledShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");    
  Show_Wait_Symbol("Off_IDLE");    
  delay_ms(10);
  BPSystemStatus =  BP_OFFLINE_IDLE;
  _EINT();
  while(1)
  {
    if(BPSystemStatus == BP_OFFLINE_IDLE || BPSystemStatus == BP_ONLINE_IDLE)       //���ߵȴ�״̬
    {                                              
      //step=101;
      HalOledOnOff(HAL_OLED_MODE_ON);                           //����ʾ�� 
      if(Device_waite_time1 <15000)
      {
        delay_ms(2);
        Device_waite_time1++;
      }
      else                                     //�ȴ�15s������޲������������״̬
      {
        if(BPSystemStatus == BP_OFFLINE_IDLE) // ���߿���״̬
          BPSystemStatus = BP_OFFLINE_SLEEP;
        if(BPSystemStatus == BP_ONLINE_IDLE) // ���߿���״̬
          BPSystemStatus = BP_ONLINE_SLEEP;
      }
    }
    else if(BPSystemStatus == BP_OFFLINE_SLEEP || BPSystemStatus == BP_ONLINE_SLEEP)       //����˯��״̬
    {
      HalOledOnOff(HAL_OLED_MODE_OFF);           //�ر���ʾ��        
      _BIS_SR(LPM0_bits + GIE);                  //����͹���
    }
    else if(BPSystemStatus == BP_OFFLINE_MEASURE || BPSystemStatus == BP_ONLINE_MEASURE)     //����״̬
    {
      if(bufferFullFlag == 1) // buffer����
      {
        HalBPMeasReadFromBuf( &dataTemp );
        if(BPSystemStatus == BP_OFFLINE_MEASURE) // ���߲���״̬
        { // д��SD��
          f_write(file,dataTemp,512,&bw);
        }
        if(BPSystemStatus == BP_ONLINE_MEASURE) // ���߲���״̬
        {
          //���͸�CC2530
          UART1_Send_Buffer((uint8 *)dataTemp,68);
        }
        bufferFullFlag = 0;
      }
      if(step==30) // step=30˵���������
      { 
        Find_Hign_Low_BP();                    //����ߵ�Ѫѹֵ
        step = 101;
        Device_waite_time1 = 0;
        // �ͷ�����Ŀռ�
        HalBPMeasStop();
        if(BPSystemStatus == BP_OFFLINE_MEASURE) // ���߲���״̬
        {
          f_close(file);
          Show_Wait_Symbol("Off_IDLE");
          BPSystemStatus = BP_OFFLINE_IDLE;
        }
        if(BPSystemStatus == BP_ONLINE_MEASURE) // ���߲���״̬
        {
          // ���������
          SendBP_HighAndLowoCC2530(BP_High,BP_Low);
          Show_Wait_Symbol("On_IDLE ");
          BPSystemStatus = BP_ONLINE_IDLE;
        }
        HalOledShowString(0,34,32,"        ");  //8���ո���ȫ���
        
        if(BP_High<100)
          HalOledShowNum(SPO2_Show2Num_Start_X,SPO2_Show2Num_Start_Y,BP_High,2,32);              
        else
          HalOledShowNum(SPO2_Show3Num_Start_X,SPO2_Show3Num_Start_Y,BP_High,3,32);
        if(BP_Low < 100)
          HalOledShowNum(HR_Show2Num_Start_X,HR_Show2Num_Start_Y,BP_Low,2,32);              
        else
          HalOledShowNum(HR_Show3Num_Start_X ,HR_Show3Num_Start_Y,BP_Low,3,32);
      }
    }
    else if(BPSystemStatus == BP_SYNC_DATA)
    {
      SyncData();
    }
  }              
}

/*****************************************************************************
��ʱ��A�жϷ������
*****************************************************************************/
uint16 aa = 2000;
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
       delay_times++;                            //��ʱ�����ۼ�
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
         HalOledShowNum(SPO2_Show2Num_Start_X+27,SPO2_Show2Num_Start_Y,0,1,32);
         HalOledShowNum(SPO2_Show2Num_Start_X+43,SPO2_Show2Num_Start_Y,0,1,32);
         HalOledShowNum(SPO2_Show2Num_Start_X+59,SPO2_Show2Num_Start_Y,pressure,1,32);
       }
       else if((pressure<100)&&(pressure>9))
       {
         HalOledShowNum(SPO2_Show2Num_Start_X+27,SPO2_Show2Num_Start_Y,0,1,32);
         HalOledShowNum(SPO2_Show2Num_Start_X+43,SPO2_Show2Num_Start_Y,pressure,2,32);
       }
       else
         HalOledShowNum(SPO2_Show3Num_Start_X+27,SPO2_Show3Num_Start_Y,pressure,3,32);         
     }
     switch(step)
     {       
     case 0:                                    //������ʼ��
       delay_times=0;
       BP_AC_AVE=0;
       BP_AC_AVE_times=0;
       BP_MIN=BP_AC_AVE;
       BP_MAX=0;
       AC_MAX=0;
       last_is_low=0;              
       step=5;
       break;
     
     case 5:                                   //BP_AC��׼��ȡ���������ó���
       BP_AC_AVE+=BP_AC;
       BP_AC_AVE_times++;
       if(BP_AC_AVE_times>=128)
       { 
         BP_AC_AVE=BP_AC_AVE>>7;
         PUMP_ON(0x0100);             //��ʼ����
         step=10;
       }                
       break;
       
     case 10:                                 //�ж��Ƿ񵽴�Ԥ��ѹ��
       if(BP_DC>=Prepressure)
       {
         PUMP_OFF();                  //ֹͣ��������ʱ����й�����Ѿ���ʼй��
         //delay_s(2);
         delay_times=0;
         BP_MIN=BP_AC_AVE;
         BP_MAX=0;
         last_is_low=1;
         n_P=0;                       //number of points
         step=20;
       }
       break; 
       
     case 20:                                 //��BP_DC��BP_ACͨ�����ڷ���                 
       // ֻ�����ʱ�Ĳ���ֵ
       if(BPSystemStatus == BP_OFFLINE_MEASURE) // ���߲���״̬
       {
         SendDCAndACToSDcard(BP_DC,BP_AC);
         //SendDCAndACToSDcard(0x4141,0x4242);
       }
       if(BPSystemStatus == BP_ONLINE_MEASURE) // ���߲���״̬
       {
         SendDCAndACToCC2530(BP_DC,BP_AC,100,100);
//         SendDCAndACToCC2530(aa,aa,100,100);
//         aa++;
//         if(aa==3000)
//           aa = 0;
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
       if(BP_DC<930)                  //���ٷ���
       {
         PUMP_OFF();
         VALVE_ON();
         //delay_s(5);
       }      
       if(BP_DC<300)
       {
         PUMP_OFF();
         VALVE_ON();
         CCTL0 &= ~CCIE;                       //�رն�ʱ��A�ж�
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
UART�жϷ������
*****************************************************************************/
#pragma vector=USART0RX_VECTOR
__interrupt void USART0_RX_ISR(void)
{
  static uint8 receiveFlag = 0;
  static uint8 controlMessage = 0;
  Device_waite_time1 = 0;
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
          case START_MEASURE: //������������
            if(BPSystemStatus == BP_ONLINE_SLEEP || BPSystemStatus == BP_ONLINE_IDLE) // ����˯�߻����߿���״̬
            {
              if(BPSystemStatus == BP_ONLINE_SLEEP)
              {
                LPM0_EXIT;
                HalOledOnOff(HAL_OLED_MODE_ON);
              }
              BPSystemStatus = BP_ONLINE_MEASURE;

              
              HalOledShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
              HalOledShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");   
              Show_Wait_Symbol("On_Go   "); 
              HalOledShowString(0,34,32,"        ");  //8���ո���ȫ���               
              Start_BPMeasure();
            }
            break;
            
          case STOP_MEASURE: // ֹͣ����
            if(BPSystemStatus == BP_ONLINE_MEASURE) //�������߲���״̬
            {
              Stop_BPMeasure();
              BPSystemStatus = BP_ONLINE_IDLE;
              
              HalOledShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
              HalOledShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");    
              HalOledShowString(0,34,32,"        ");  //8���ո���ȫ���    
              Show_Wait_Symbol("On_IDLE ");    
            }
            break;
            
          case SYNC_MEASURE: // ͬ����Ϣ���ݲ�����
            if(BPSystemStatus == BP_ONLINE_SLEEP || BPSystemStatus == BP_ONLINE_IDLE) // ����˯��״̬�����߿���״̬
            {
              if(BPSystemStatus == BP_ONLINE_SLEEP) // ����˯��״̬���˳��͹���
                LPM0_EXIT;
              
              BPSystemStatus = BP_SYNC_DATA;
              
              HalOledShowString(SPO2_Symbol_Start_X,0,12,"On_Sync ");
              //����ʾ��
              HalOledOnOff(HAL_OLED_MODE_ON);
            }
            break;
            
          case FIND_NWK:  // ����������Ϣ
            if(BPSystemStatus == BP_ONLINE_MEASURE || BPSystemStatus == BP_OFFLINE_MEASURE) // ������������߻����߲���
            {
              Stop_BPMeasure();
            }
            if(BPSystemStatus == BP_OFFLINE_SLEEP || BPSystemStatus == BP_ONLINE_SLEEP) // ���߻�����˯��״̬
            {
              LPM0_EXIT;
              HalOledOnOff(HAL_OLED_MODE_ON);      
            }
            BPSystemStatus = BP_FIND_NETWORK;
            HalOledShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
            HalOledShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");
            HalOledShowString(0,34,32,"        ");  //8���ո���ȫ��� 
            Show_Wait_Symbol("FIND_NWK");    
            break;
            
          case END_DEVICE:  // �ҵ�������Ϣ ֻ���ܴ�FIND_NWK�����״̬
            BPSystemStatus = BP_ONLINE_IDLE;
            Show_Wait_Symbol("On_IDLE ");    
            break;
            
          case CLOSEING:   // ���ڹر�������Ϣ
            if(BPSystemStatus == BP_ONLINE_MEASURE) // ���߲���״̬�ر�����
              Stop_BPMeasure();
            if(BPSystemStatus == BP_ONLINE_SLEEP) // ˯��״̬�¹ر�����
            {
              LPM0_EXIT;
              HalOledOnOff(HAL_OLED_MODE_ON);                 
            }            
            // ���߿���״̬��Ѱ������״̬���¹ر�����
            // do nothing
            BPSystemStatus = BP_CLOSING;
           
            HalOledShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
            HalOledShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");
            HalOledShowString(0,34,32,"        ");  //8���ո���ȫ��� 
            Show_Wait_Symbol("CLOSING ");              
            break;
            
          case CLOSE_NWK:
            BPSystemStatus = BP_OFFLINE_IDLE;
            Show_Wait_Symbol("Off_IDLE");          
            break;
            
        }
      }
      receiveFlag = 0;
      break;
  }  
}


/************************************************************************
                        ���������жϷ������																                 
*************************************************************************/
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{    
  P1IE &= ~BIT3;                                                         //P1.3 interrupt DISABLE
  Device_waite_time1 = 0;                                                //�ȴ�״̬����ֵ��0
  if(BPSystemStatus == BP_OFFLINE_SLEEP || BPSystemStatus == BP_ONLINE_SLEEP)    //����˯��̬�������̰����ǽ���ȴ�̬
  {
    LPM0_EXIT;
    HalOledShowString(0,34,32,"        ");  //8���ո���ȫ���  ����ӵ��������ڴ����߷��صȴ�״̬ʱ��ʾ�ո�
    if(BPSystemStatus == BP_OFFLINE_SLEEP) // ����˯��״̬
    {
      BPSystemStatus = BP_OFFLINE_IDLE;
      Show_Wait_Symbol("Off_IDLE");
    }
    if(BPSystemStatus == BP_ONLINE_SLEEP) // ����˯��״̬
    {
      BPSystemStatus = BP_ONLINE_IDLE;
      Show_Wait_Symbol("On_IDLE");
    }
  }
  else                                                                   //���ڵȴ�״̬���ǲ���״̬�а�������
  {
    int Press_type = 0;                                                  //0��ʾ�̰���1��ʾ����
    //�ж��ǳ������Ƕ̰�
    for(Press_type = 0;Press_type < 1000; ++Press_type)                  //����Լ3s
    {
      if(P1IN&BIT3)                                                      //P1.3�Ǹߵ�ƽ����ʾ�ɿ�����
        break;
      delay_ms(2);
    }
    if(Press_type != 1000)                                               //�̰�
      Press_type = 0;
    else
      Press_type = 1;                                                    //����
    if(BPSystemStatus == BP_OFFLINE_IDLE || BPSystemStatus == BP_ONLINE_IDLE) //��������/���߿���״̬
    {
      if(Press_type == 0)                                                //�̰�����ʼ����
      {
       
        HalOledShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
        HalOledShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");
        //HalOledShowString(0,34,32,"        ");  //8���ո���ȫ���
        if(BPSystemStatus == BP_OFFLINE_IDLE) // ���߿���״̬
        {
          Show_Wait_Symbol("Off_Go  ");
          BPSystemStatus = BP_OFFLINE_MEASURE;
        }
        if(BPSystemStatus == BP_ONLINE_IDLE) // ���߿���״̬
        {
          Show_Wait_Symbol("On_Go   ");
          BPSystemStatus = BP_ONLINE_MEASURE;
        }
        HalOledShowString(0,34,32,"        ");  //8���ո���ȫ���             
        Start_BPMeasure();
      }
      else//����������
      {
        if(BPSystemStatus == BP_OFFLINE_IDLE) // ���߿���״̬
          BPSystemStatus = BP_OFFLINE_SLEEP;
        if(BPSystemStatus == BP_ONLINE_IDLE) // ���߿���״̬
          BPSystemStatus = BP_ONLINE_SLEEP;
       
      }
    }
    else if(BPSystemStatus == BP_OFFLINE_MEASURE || BPSystemStatus == BP_ONLINE_MEASURE)//�������߻����߲���״̬
    {
      if(Press_type == 0)//�̰���ֹͣ����
      {
        Stop_BPMeasure();
       
        HalOledShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
        HalOledShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA"); 
        HalOledShowString(0,34,32,"        ");  //8���ո���ȫ���
        if(BPSystemStatus == BP_OFFLINE_MEASURE) // ���߲���״̬
        {
          Show_Wait_Symbol("Off_IDLE");
          BPSystemStatus = BP_OFFLINE_IDLE;
        }
        if(BPSystemStatus == BP_ONLINE_MEASURE) // ���߲���״̬
        {
          Show_Wait_Symbol("On_IDLE ");
          BPSystemStatus = BP_ONLINE_IDLE;
        }
      }
      else//����������
      {
        Stop_BPMeasure();
        if(BPSystemStatus == BP_OFFLINE_MEASURE) // ���߲���״̬
          BPSystemStatus = BP_OFFLINE_SLEEP;
        if(BPSystemStatus == BP_ONLINE_MEASURE) // ���߲���״̬
          BPSystemStatus = BP_ONLINE_SLEEP;
       
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
  // ����ռ�
  if(BPSystemStatus == BP_OFFLINE_MEASURE) // ���߿���״̬
  {
     HalBPMeasStart(BP_BUFFER_FOR_SD);
     GenericApp_GetWriteName();
     f_open(file,fileName,FA_CREATE_ALWAYS | FA_WRITE);     
  }
  if(BPSystemStatus == BP_ONLINE_MEASURE) // ���߿���״̬
    HalBPMeasStart(BP_BUFFER_FOR_ZIGBEE);
  
  writeNum = 0;
  bufferFullFlag = 0;
  VALVE_OFF();
  step=0;
  CCTL0 |= CCIE;                        //������ʱ��A�ж�
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
  // �ͷ�����Ŀռ�
  HalBPMeasStop();
  // �ر��ļ�
  if(BPSystemStatus == BP_OFFLINE_MEASURE) // ���߲���״̬
    f_close(file);
  step =101;
  PUMP_OFF();
  VALVE_ON(); //����
}


/*****************************************************************************
��ѹ-ѹ��ת��
*****************************************************************************/
unsigned int adc2mmhg(unsigned int adc)
{
  abp=((adc*3.3/4095)/233.56)*1000*15.52;
  return (int)(abp);
  //return (int)((adc*3.3/4095)/233.56)*1000*15.52;
}

/*****************************************************************************
����ѹ����ѹ����
*****************************************************************************/
void Find_Hign_Low_BP()
{
  unsigned int p2p_max=0,n_max=0;
  unsigned int p2p_min=65535;  
  unsigned int i=0;  
  unsigned int high,low;  
  int tmp_v[100];  
  //--------�˲�----------
  for(i=0;i<n_P;i++) 
        tmp_v[i]=Points[i].p2p;
  for(i=3;i<n_P-3;i++)
  {
    if(abs(tmp_v[i]-tmp_v[i-1])+abs(tmp_v[i]-tmp_v[i+1])>500)
        Points[i].p2p= (tmp_v[i-2]+tmp_v[i-1]+tmp_v[i+1]+tmp_v[i+2])/4;
    Points[i].p2p=(Points[i-1].p2p+Points[i].p2p+Points[i+1].p2p)/3;
  }  
  //--------���㻬��ƽ��------------
  for(i=0;i<n_P;i++) 
        tmp_v[i]=Points[i].p2p;
  for(i=3;i<n_P-3;i++)
  {
    Points[i].p2p=(tmp_v[i-1]+tmp_v[i]+tmp_v[i+1])/3;
  }  
  //---ȥ��ֱ����������������ƽ�ƣ�--------------------------
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
  //--Ѱ�����ֵ����λ��---------------------------
  for(i=3;i<n_P-3;i++)
  {
    if(p2p_max<Points[i].p2p)
    {
      p2p_max=Points[i].p2p;
      n_max=i;
    }    
  }
  //--ȷ������ѹϵ��-------------------------------
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
   //--ȷ������ѹϵ��-------------------------------
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
  //-------��ϵ��������ѹ-------
  for(i=n_max;i<n_P-3;i++)
  {
    low=Points[i].bp;
    if(Points[i].p2p<(int)(k1*p2p_max))//ϵ��������
    {
      break;
    }
  }
  //-------��ϵ��������ѹ-------
  for(i=n_max;i>2;i--)
  {
   high=Points[i].bp;
   if(Points[i].p2p<(int)(k2*p2p_max))//ϵ��������
    {
      break;
    }
  }  
  DIA=adc2mmhg(low);
  SYS=adc2mmhg(high);  
  //---�������----------  
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
MSP430��Ƭ��ʱ�ӳ�ʼ��
*****************************************************************************/
void Init_Clock()
{ 
       unsigned int i;
       BCSCTL1 = 0X00;			//���Ĵ�������������
					//XT2��������
					//LFTX1�����ڵ�Ƶģʽ
					//ACLK�ķ�Ƶ����Ϊ1	32768			
       do 
       {
	   IFG1 &= ~OFIFG;              // ���OSCFault��־
	   for (i = 0xFF; i > 0; i--);                
       }
       while ((IFG1 & OFIFG) == OFIFG); // ���OSCFault =1   
					
       BCSCTL2 = 0X00;			//���Ĵ�������������
       BCSCTL2 = SELM_2 + SELS;		//MCLK��ʱ��ԴΪTX2CLK����Ƶ����Ϊ1 8M
                         		//SMCLK��ʱ��ԴΪTX2CLK����Ƶ����Ϊ1 8M
}
/*****************************************************************************
�˿�P1��ʼ�� AD8402 ǰ�˿�����ض˿�
*****************************************************************************/
void Init_Port1()
{
       P1DIR |= BIT0; //P1.0��� AD8402   CLK
       P1DIR |= BIT1; //P1.1��� AD8402   SDI
       P1DIR |= BIT2; //P1.2��� AD8402   CS
       P1DIR &= ~BIT3;//P1.3����S2 
       P1IES |=  BIT3;//��׽�½����ж�                          		
       P1IFG &= ~BIT3;//IFG����                           	
       P1IE  |=  BIT3;//�ж�ʹ��  
}
/*****************************************************************************
�˿�P2��ʼ�� ���-������-LED��ض˿�
*****************************************************************************/
void Init_Port2()
{
       P2DIR |= BIT0;//P2.0��� ���Ƶ�ŷ�����
       P2DIR |= BIT1;//P2.1��� ���Ƶ�ŷ�����
       P2DIR |= BIT3;//P2.3��� ���Ʒ�����
       P2OUT |= BIT3;
       P2DIR |= BIT4;//P2.4��� LED D5
}

/*****************************************************************************
�˿�P4��ʼ�� OLED��ض˿�
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
�˿�P6��ʼ�� ADC
*****************************************************************************/
void Init_Port6()
{
  P6SEL |= 0x07;                    // Enable A/D channel inputs
  ADC12CTL0 = ADC12ON+MSC+SHT0_2;  //+REFON+ REF2_5V;//+SHT0_8;//+REFON+ REF2_5V; 
                                        // Turn on ADC12, extend sampling time 
                                        // to avoid overflow of results
  ADC12CTL1 = SHP+CONSEQ_1;        // Use sampling timer, repeated sequence
  ADC12MCTL0 = INCH_0+SREF_0;      // ref+=AVcc, channel = A0
  ADC12MCTL1 = INCH_1+SREF_0;  // ref+=AVcc, channel = A1
  ADC12MCTL2 = INCH_2+EOS+SREF_0 ;   // ref+=AVcc, channel = A2
  //ADC12IE = 0x02;                // Enable ADC12IFG.3
  ADC12CTL0 |= ENC;                // Enable conversions
}
/*****************************************************************************
��ʱ��A��ʼ��
*****************************************************************************/
void Init_TimerA()
{
  CCTL0 &= ~CCIE;                           // CCR0 interrupt Disabled
  CCR0 = 128-1;                              // 256,��ʱ��ÿ������ж�256��,������256Hz
  TACTL = TASSEL_1 + MC_1;                  // ACLK, upmode
}
/*****************************************************************************
���ÿ�
*****************************************************************************/
void PUMP_ON(u16 motorspeed)
{
       Write_8402(motorspeed);    
}
/*****************************************************************************
���ù�
*****************************************************************************/
void PUMP_OFF(void)
{
       Write_8402(0x01ffu);    
}
/*****************************************************************************
��ŷ���
*****************************************************************************/
void VALVE_ON(void)
{
       P2OUT |= BIT0;
       P2OUT |= BIT1;
}
/*****************************************************************************
��ŷ��ر�
*****************************************************************************/
void VALVE_OFF(void)
{
       P2OUT &= ~BIT0;
       P2OUT &= ~BIT1;
}
/*****************************************************************************
���ֵ�λ�������ӳ���
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
             P1OUT |= BIT1;                //SDI��ֵ
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
  HalBPMeasWriteToBuf(DC_temp);
  HalBPMeasWriteToBuf(AC_temp);
  writeNum++;
  if(writeNum == 16)
  {
    HalBPMeasWriteToBuf(BP_HIGH_temp);
    HalBPMeasWriteToBuf(BP_LOW_temp);
    bufferFullFlag = 1;
    writeNum = 0;
  }
}


void SendDCAndACToSDcard(uint16 DC_temp,uint16 AC_temp)
{
  HalBPMeasWriteToBuf(DC_temp);
  HalBPMeasWriteToBuf(AC_temp);
  writeNum++;
  if(writeNum == 128)
  {
    bufferFullFlag = 1;
    writeNum = 0;
  }
}
void SendBP_HighAndLowoCC2530(uint16 BP_HIGH_temp,uint16 BP_LOW_temp)
{
  uint8 i,bufferSend[68];
  for(i = 0; i < 64; ++i)
    bufferSend[i] = 0xFF;
  
  bufferSend[i++] = (BP_HIGH_temp & 0x00FF);
  bufferSend[i++] = ((BP_HIGH_temp & 0xFF00) >> 8);
  bufferSend[i++] = (BP_LOW_temp & 0x00FF);
  bufferSend[i++] = ((BP_LOW_temp & 0xFF00) >> 8); 
  delay_ms(15);
  UART1_Send_Buffer(bufferSend,68);
  delay_ms(15);
  UART1_Send_Buffer(bufferSend,68);
  delay_ms(15);
  UART1_Send_Buffer(bufferSend,68);
}

void Show_Wait_Symbol(const char *p)
{
    HalOledShowString(SPO2_Symbol_Start_X,0,12,p);
//    HalOledShowString(SPO2_Symbol_Start_X+88,0,12,"mmHg");
    HalOledShowWaitSymbol(SPO2_Wait_Symbol_Start_X,SPO2_Wait_Symbol_Start_Y,1);
    HalOledShowWaitSymbol(SPO2_Wait_Symbol_Start_X+16,SPO2_Wait_Symbol_Start_Y,1);
    HalOledShowWaitSymbol(SPO2_Wait_Symbol_Start_X+32,SPO2_Wait_Symbol_Start_Y,1);
    HalOledShowWaitSymbol(HR_Wait_Symbol_Start_X,HR_Wait_Symbol_Start_Y,1);
    HalOledShowWaitSymbol(HR_Wait_Symbol_Start_X+16,HR_Wait_Symbol_Start_Y,1);
    HalOledShowWaitSymbol(HR_Wait_Symbol_Start_X+32,HR_Wait_Symbol_Start_Y,1);
    
    // ��ʾ����
    HalShowBattVol(BATTERY_MEASURE_SHOW);
}


/*********************************************************************
 * @fn      GenericApp_GetWriteName
 *
 * @brief   Get the RTC time and make file name.
 *
 * @param   char *
 *          0:D/20xx-xx-xx xx-xx-xx x.txt + \0 = 30Byte
 *
 * @return  
 *
 */
void GenericApp_GetWriteName(void)
{
  RTCStruct_t RTCStruct;
  HalRTCGetOrSetFull(RTC_DS1302_GET,&RTCStruct);

  // Make file name
  fileName[0] = '0';
  fileName[1] = ':';
  fileName[2] = 'B';
  fileName[3] = '/';
  fileName[4] = '2';
  fileName[5] = '0';
  fileName[6] = RTCStruct.year/10 + '0';
  fileName[7] = RTCStruct.year%10 + '0';
  fileName[8] = '-';
  fileName[9] = RTCStruct.month/10 + '0';
  fileName[10] = RTCStruct.month%10 + '0';
  fileName[11] = '-';
  fileName[12] = RTCStruct.date/10 + '0';
  fileName[13] = RTCStruct.date%10 + '0';
  fileName[14] = ' ';
  fileName[15] = RTCStruct.hour/10 + '0';
  fileName[16] = RTCStruct.hour%10 + '0';
  fileName[17] = '-';
  fileName[18] = RTCStruct.min/10 + '0';
  fileName[19] = RTCStruct.min%10 + '0';
  fileName[20] = '-';
  fileName[21] = RTCStruct.sec/10 + '0';
  fileName[22] = RTCStruct.sec%10 + '0';
  fileName[23] = ' ';  
  fileName[24] = RTCStruct.week + '0';
  fileName[25] = '.';
  fileName[26] = 't';
  fileName[27] = 'x';
  fileName[28] = 't';
  fileName[29] = '\0';
}

/*********************************************************************
 * @fn      GenericApp_OpenDir
 *
 * @brief   �ҵ�ָ��Ŀ¼���ļ�
 *
 * @param  
 *
 * @return  true is over,flase is not over
 *
 */
bool GenericApp_OpenDir(void)
{
  uint8 res = 0;
  DIR *fddir = 0;         // Ŀ¼
  FILINFO *finfo = 0;     // �ļ���Ϣ
  uint8 *fn = 0;          // ���ļ���
  
  // initilize pathname
  strcpy(pathname,"0:B/");
  
  // �����ڴ�
  fddir = (DIR *)malloc(sizeof(DIR));
  finfo = (FILINFO *)malloc(sizeof(FILINFO));
  if(fddir == NULL || finfo == NULL)
  {
    if(fddir != NULL)
      free(fddir);
    if(finfo != NULL)
      free(finfo);
    return FALSE;
  }
  
  finfo->lfsize = 28 + 1;
  finfo->lfname = malloc(finfo->lfsize);
  if(finfo->lfname == NULL)
  {
    free(finfo->lfname);
    free(fddir);
    free(finfo);
    return FALSE;
  }
  
  // ��ԴĿ¼
  res = f_opendir(fddir,"0:B");

  if(res == 0)  // ��Ŀ¼�ɹ�
  {
    while(res == 0)
    {
      res = f_readdir(fddir,finfo);   //��ȡĿ¼�µ�һ���ļ�
     
      if(res != FR_OK || finfo->fname[0] == 0) // ������߶����˽�β
      {
        free(finfo->lfname);
        free(fddir);
        free(finfo);
        return FALSE;
      }
      
      if(finfo->fname[0] == '.') // �����ϲ��ļ�
        continue;
      
      /* �����ļ�,�����ַ��� */
      fn = (uint8 *)(*finfo->lfname?finfo->lfname:finfo->fname);
      strcat((char *)pathname,(char *)fn);
      
      break;
    }
  }
  
  free(finfo->lfname);
  free(fddir);
  free(finfo);
  return TRUE;  
}


/*********************************************************************
 * @fn      SyncData
 *
 * @brief   Sync data һ��ͬ��һ���ļ�
 *
 * @param  
 *
 * @return  
 *
 */
void SyncData(void)
{
  uint8 *dataSendBuffer;
  uint8 *dataSendBufferTemp;
  uint8 i,j;
  uint16 flagNum;
  if(GenericApp_OpenDir() == TRUE) // Ŀ¼�����ļ��������ļ�
  {
    dataSendBuffer = malloc(sizeof(uint8)*512);
    dataSendBufferTemp = malloc(sizeof(uint8)*68);
    if(dataSendBuffer == NULL)
    {
      BPSystemStatus = BP_ONLINE_IDLE; // ͬ������
      return;
    }
    if(dataSendBufferTemp == NULL)
    {
      free(dataSendBuffer);
      BPSystemStatus = BP_ONLINE_IDLE; // ͬ������
      return;
    }
    f_open(file,(char *)pathname,FA_OPEN_EXISTING | FA_READ); //  ���ļ�
    f_read(file,dataSendBuffer, BP_WAVEFORM_READ_ONE_TIME,&br);
    br = br/BP_WAVEFORM_SEND_ONE_TIME;
    j = 0;
    while(br != 0)
    {
      while(br != 0)
      {
        flagNum = j*64;
        for(i = 0; i < 64; ++i)
        {
          dataSendBufferTemp[i] = dataSendBuffer[flagNum+i];
        }
        dataSendBufferTemp[i++] = 0x64;
        dataSendBufferTemp[i++] = 0x00;
        dataSendBufferTemp[i++] = 0x64;
        dataSendBufferTemp[i++] = 0x00;
        UART1_Send_Buffer(dataSendBufferTemp,68);
        br--;
        ++j;
        halMcuWaitUs(50000);
      }
      f_read(file,dataSendBuffer, BP_WAVEFORM_READ_ONE_TIME,&br);
      br = br/BP_WAVEFORM_SEND_ONE_TIME;
      j = 0;
    }
    
    // ������� �ر�ɾ���ļ�
    f_close(file);
    f_unlink((char *)pathname);
    free(dataSendBuffer);
    free(dataSendBufferTemp);
  }
  else // û���ļ��ˣ�����ͬ����������
  {
    UART1_Send_Buffer(fileName,10);   
    BPSystemStatus = BP_ONLINE_IDLE; // ͬ������
    Show_Wait_Symbol("On_IDLE ");
  }  
}