//******************************************************************************
//  MSP430F149-Noninvasive Blood Pressure Monitor
//  Nov 2013
//  Built with IAR Embedded Workbench Version: 5.3
//******************************************************************************
#include  <msp430x14x.h>
#include  <math.h>
#include  "typedefs.h"
#include "oled_main.h"
/*****************************************************************************
�궨��
******************************************************************************/
#define DEVICE_SLEEP    0x00                       //˯��״̬
#define DEVICE_WAIT     0x01                       //�ȴ�״̬
#define DEVICE_MEASURE  0x02                       //����״̬
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
unsigned char is_BP_messure_finished=0;            //Ѫѹ������������
unsigned int delay_times=0;
unsigned char step=101;
char command;
int value1;
int ch1;
int value2;
int ch2;
int value3;
int ch3;
int value4;
int ch4;
unsigned char const hello[4]={0x4B,0x3A,0x1F,0x2E};//��������Э��
unsigned char RBUF[8];
unsigned char R_SP=0;
unsigned char RBUF_NEED_PROCESS=0;
int Prepressure=3500;
//int Prepressure=4000;
int Device_status;                                 //Ĭ�ϵȴ�״̬
int Device_waite_time1;
/*****************************************************************************
��������
*****************************************************************************/
void Init_Clock();
void Init_Port1();
void Init_Port2();
void Init_Port3();
void Init_Port4();
void Init_Port5();
void Init_Port6();
void Init_TimerA();
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
/*****************************************************************************
������
*****************************************************************************/
void main(void)
{
       int i;
       WDTCTL = WDTPW + WDTHOLD;                //�رտ��Ź���·
       Device_status = DEVICE_WAIT;             //Ĭ����ͨ��ʾ״̬
       Init_Clock();
       Init_Port1();
       Init_Port2();
       P2OUT &= ~BIT4;
       Init_Port3();
       Init_Port4();
       Init_Port6();                             
       //VALVE_OFF();                             //ͨ��0 ���Ƶ�ŷ�
       VALVE_ON();
       PUMP_OFF();                              //ͨ��1 ��������
       Init_TimerA();
       step=101;
       oledinit();
       OLED_ShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
       OLED_ShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");    
       Show_Wait_Symbol();    
       OLED_Refresh_Gram();
       delay_ms(10);        
       _EINT();      
       while(1)
            {
              if(Device_status == DEVICE_WAIT)             //�ȴ�״̬
              {                                              
                  //step=101;
                  OLED_SLEEP(0);                           //����ʾ�� 
                  if(Device_waite_time1 <15000)
                    {
                      delay_ms(2);
                      Device_waite_time1++;
                    }
                  else                                     //�ȴ�15s������޲������������״̬
                    Device_status = DEVICE_SLEEP;
               }
              else if(Device_status == DEVICE_SLEEP)       //����˯��״̬
              {
                //step=101;
                OLED_SLEEP(1);                             //�ر���ʾ��        
                CCTL0 &= ~CCIE;                            //�رն�ʱ���ж�
                _BIS_SR(LPM0_bits + GIE);                  //����͹���
              }
              else if(Device_status == DEVICE_MEASURE)     //����״̬
              {
                if(RBUF_NEED_PROCESS)
                {
                  RBUF_PROCESS();
                }
                if(step==30)
                {
                  Find_Hign_Low_BP();                    //����ߵ�Ѫѹֵ
                  value3=BP_High;                        //����ѹ
                  ch3=(value3&0x00FF);
                  value4=BP_Low;
                  ch4=(value4&0x00FF);
                  TXBUF0 = 0xEF;
                  while(!(U0TCTL & TXEPT));
                  TXBUF0 = 0x12;
                  while(!(U0TCTL & TXEPT));
                  TXBUF0 = ch3;
                  while(!(U0TCTL & TXEPT));
                  TXBUF0 = ch4;
                  while(!(U0TCTL & TXEPT));
                  TXBUF0 = 0x12;
                  while(!(U0TCTL & TXEPT));
                  TXBUF0 = 0xEF;
                  while(!(U0TCTL & TXEPT));
                                                       //��������
                  for(i=0;i<120;i++)
                  {
                    while(!(U0TCTL & TXEPT));
                    TXBUF0 = 0x00;
                    while(!(U0TCTL & TXEPT));
                  }  
                  is_BP_messure_finished=1;
                  step=40;                  
                  //delay_s(2);
                  CCTL0 &= ~CCIE;                       //�رն�ʱ��A�ж�
                  OLED_ShowString(SPO2_Symbol_Start_X,0,12,"Finished");
                  OLED_ShowString(0,30,32,"        ");  //8���ո���ȫ���
                  if(BP_High<100)
                    OLED_ShowNum(SPO2_Show2Num_Start_X,SPO2_Show2Num_Start_Y,BP_High,2,32);              
                  else
                    OLED_ShowNum(SPO2_Show3Num_Start_X,SPO2_Show3Num_Start_Y,BP_High,3,32);
                  if(BP_Low < 100)
                    OLED_ShowNum(HR_Show2Num_Start_X,HR_Show2Num_Start_Y,BP_Low,2,32);              
                  else
                    OLED_ShowNum(HR_Show3Num_Start_X ,HR_Show3Num_Start_Y,BP_Low,3,32);
                  OLED_Refresh_Gram();                  
                  //delay_s(50);
                  //OLED_Clear();
                  //Device_status = DEVICE_WAIT;
                  
                 }
               } 
            }              
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
					//ACLK�ķ�Ƶ����Ϊ1				
       do 
       {
	   IFG1 &= ~OFIFG;              // ���OSCFault��־
	   for (i = 0xFF; i > 0; i--);                
       }
       while ((IFG1 & OFIFG) == OFIFG); // ���OSCFault =1   
					
       BCSCTL2 = 0X00;			//���Ĵ�������������
       BCSCTL2 = SELM_2 + SELS;		//MCLK��ʱ��ԴΪTX2CLK����Ƶ����Ϊ1
                         		//SMCLK��ʱ��ԴΪTX2CLK����Ƶ����Ϊ1
}
/*****************************************************************************
�˿�P1��ʼ��
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
�˿�P2��ʼ��
*****************************************************************************/
void Init_Port2()
{
       P2DIR |= BIT0;//P2.0��� ���Ƶ�ŷ�����
       P2DIR |= BIT1;//P2.1��� ���Ƶ�ŷ�����
       P2DIR |= BIT3;//P2.3��� ���Ʒ�����
       P2OUT |= BIT3;
       P2DIR |= BIT4;//P2.4��� LED D5
       P2DIR |= BIT5;//P2.5��� DS1302 RST
       P2DIR |= BIT6;//P2.6��� DS1302 I/O
       P2DIR |= BIT7;//P2.7��� DS1302 SCLK
}
/*****************************************************************************
�˿�P3��ʼ�� UART
*****************************************************************************/
void Init_Port3()
{
       P3SEL |= 0x30;                          //P3.4 P3.5����ΪUART
       UCTL0 &= ~SWRST;
       ME1 |= UTXE0 + URXE0;                   // Enable USART0 TXD/RXD
       UCTL0 |= CHAR;                          // 8-bit character
       UTCTL0 |= SSEL1;                        // UCLK = ACLK
       UBR00 = 0xD0;                           // 32k/9600 - 3.41
       UBR10 = 0x00;                           
       UMCTL0 = 0x00;                          // Modulation 
       UCTL0 &= ~SWRST;                        // Initialize USART state machine
       IE1 |= URXIE0;                          // Enable USART0 RX/TX interrupt 
}
/*****************************************************************************
�˿�P4��ʼ��
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
�˿�P5��ʼ��
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
�˿�P6��ʼ�� ADC
*****************************************************************************/
void Init_Port6()
{
       P6SEL = 0x03;                    // Enable A/D channel inputs
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
��ʱ��A��ʼ��
*****************************************************************************/
void Init_TimerA()
{
       CCTL0 &= ~CCIE;                           // CCR0 interrupt Disabled
       CCR0 = 64-1;                              //256,��ʱ��ÿ������ж�256��
       TACTL = TASSEL_1 + MC_1;                  // SMCLK, upmode
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
/*****************************************************************************
��ʱ�ӳ���
*****************************************************************************/
void usDelay(int microeconds)
{
       do
       {
           _NOP(); 
           _NOP();
       }while (--microeconds > 0);
}
void delay_ms(u16 nms)
{
       u16 i;
       u16 j;
       _NOP();
       for (j=0;j<nms;j++)
       {
          for (i=0;i<1335;i++)       
             {
                _NOP();
             }
       }  
}
void delay_s(u16 ns)
{
       u16 i;
       u16 j;
       _NOP();
       for (j=0;j<ns;j++)
       {
          for (i=0;i<5;i++) 
             {
                delay_ms(200);
             }
       }   
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
����ֵ����
*****************************************************************************/
int abs(int a)
{
if(a<0)a=-a;
return a;
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
UART�������ݴ���
*****************************************************************************/
void RBUF_PROCESS(void)  //���������е���
{
    
  if(RBUF[2]==0x03)      //����һ��Ѫѹ����
  {
   step=0;
   CCTL0 |= CCIE;   
  } 

  else if(RBUF[2]==0x06) //ֹͣѪѹ����
  {
   step =101;
   CCTL0 &= ~CCIE;
   PUMP_OFF();
   VALVE_ON();
   delay_s(5);
   VALVE_OFF();
   
  } 
 
  else if(RBUF[2]==0x30)//��ز�������
  {
    if(RBUF[3]==0x01)   //���ò�����Ϊ128Hz
    {
    CCR0 = 128-1;
    }
    if(RBUF[3]==0x02)   //���ò�����Ϊ256Hz
    {
    CCR0 = 64-1;
    }
    if(RBUF[3]==0x03)   //���ò�����Ϊ512Hz
    {
    CCR0 = 32-1;
    }
    if(RBUF[3]==0x04)   //����Ԥ��ѹ��ֵΪ2900
    {
    Prepressure=2900;
    }
    if(RBUF[3]==0x05)   //����Ԥ��ѹ��ֵΪ2800
    {
    Prepressure=2800;
    }
    if(RBUF[3]==0x06)   //����Ԥ��ѹ��ֵΪ2700
    {
    Prepressure=2700;
    }
    if(RBUF[3]==0x07)   //���ò�����Ϊ115200
    {
    UBR00 = 0x45;                             
    UBR10 = 0x00;                             
    UMCTL0 =0x2C; 
    }
    if(RBUF[3]==0x08)   //���ò�����Ϊ38400
    {
    UBR00 = 0xD0;
    UBR10 = 0x00;                             
    UMCTL0 = 0x00;
    }
    if(RBUF[3]==0x09)   //���ò�����Ϊ19200
    {
    UBR00 = 0xA0;
    UBR10 = 0x01;                             
    UMCTL0 = 0x00;
    }    
  }  
  RBUF_NEED_PROCESS=0;  
}
/*****************************************************************************
��ʱ��A�жϷ������
*****************************************************************************/
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
       case 0:                                    //������ʼ��
                delay_times=0;
                BP_AC_AVE=0;
                BP_AC_AVE_times=0;
                BP_MIN=BP_AC_AVE;
                BP_MAX=0;
                AC_MAX=0;
                last_is_low=0;
                is_BP_messure_finished=0;                  
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
                value1=BP_DC;                   //���������̬ѹ��
                //value1=0x0123;
                ch1=(value1&0x000F)|0x10;
                TXBUF0 = ch1;
                while(!(U0TCTL & TXEPT));
                ch1=(value1&0x00F0)>>4|0x20;
                TXBUF0 = ch1;
                while(!(U0TCTL & TXEPT));
                ch1=(value1&0x0F00)>>8|0x30;
                TXBUF0 = ch1;
                while(!(U0TCTL & TXEPT));       //��������������
                value2=BP_AC;
                ch2=(value2&0x000F)|0x40;
                TXBUF0 = ch2;
                while(!(U0TCTL & TXEPT));
                ch2=(value2&0x00F0)>>4|0x50;
                TXBUF0 = ch2;
                while(!(U0TCTL & TXEPT));
                ch2=(value2&0x0F00)>>8|0x60;
                TXBUF0 = ch2;
                while(!(U0TCTL & TXEPT));       //���Ϳ�ʼ                
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
                 VALVE_OFF();
                 step=30;
                }
                break;
       case 40:  
               step=101; 
               PUMP_OFF();
               VALVE_ON();
               break;
       default:
               PUMP_OFF();
               VALVE_ON();
               break;
       }
}
/*****************************************************************************
UART�жϷ������
*****************************************************************************/
#pragma vector=USART0RX_VECTOR
__interrupt void USART0_RX_ISR(void)
{
 unsigned char ch;
 ch=RXBUF0;   
  if(R_SP<2)                          //֡ͷ�б�
    {
	if(ch==hello[R_SP])R_SP++;
	else R_SP=0;
	}
  else if(R_SP<4)                     //֡����
    {
	     RBUF[R_SP]=ch;
	     R_SP++;
    }
  else if(ch==hello[3])               //֡β�б�
         {
		 R_SP=0;
		 if(RBUF_NEED_PROCESS==0)
		      {
			  RBUF_NEED_PROCESS=1;
		       }
		 }  
  else 
  {
    RBUF_NEED_PROCESS=1;
    R_SP=0;
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
    if(Device_status == DEVICE_SLEEP)                                      //����˯��̬�������̰����ǽ���ȴ�̬
    {
      Device_status = DEVICE_WAIT;
      LPM4_EXIT;
      OLED_ShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
      OLED_ShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");    
      Show_Wait_Symbol();    
      OLED_Refresh_Gram();
      delay_ms(10);  
    }
    else                                                                   //���ڵȴ�״̬���ǲ���״̬�а�������
    {
      int Press_type = 0;                                                  //0��ʾ�̰���1��ʾ����
      //�ж��ǳ������Ƕ̰�
      for(Press_type = 0;Press_type < 1000; Press_type++)                  //����Լ3s
      {
        if(P1IN&BIT3)                                                      //P1.3�Ǹߵ�ƽ����ʾ�ɿ�����
          break;
        delay_ms(2);
      }
      if(Press_type != 1000)                                               //�̰�
        Press_type = 0;
      else
        Press_type = 1;                                                    //����
      if(Device_status == DEVICE_WAIT)                                     //���ڵȴ�״̬
      {
        if(Press_type == 0)                                                //�̰�����ʼ����
        {
          Device_status = DEVICE_MEASURE;
          VALVE_OFF();
          if(step==101) 
                {
                step=0;
                CCTL0 |= CCIE;                        //������ʱ��A�ж�
                OLED_Clear();
                OLED_ShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
                OLED_ShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");    
                Show_Wait_Symbol(); 
                OLED_ShowString(SPO2_Symbol_Start_X,0,12,"Go       ");
                OLED_ShowString(0,30,32,"        ");  //8���ո���ȫ���
                OLED_Refresh_Gram();                
                delay_ms(10);
                }
                else 
                {
                step =101;
                OLED_Clear();
                OLED_ShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
                OLED_ShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");    
                Show_Wait_Symbol();    
                OLED_Refresh_Gram();              
                delay_ms(10);
                }
        }
        else//����������
        {
          Device_status = DEVICE_SLEEP;
          step=101;
        }
      }
      else if(Device_status == DEVICE_MEASURE)//���ڲ���״̬
      {
        if(Press_type == 0)//�̰���ֹͣ����
        {
          Device_status = DEVICE_WAIT;
          PUMP_OFF();
          VALVE_ON();
          CCTL0 &= ~CCIE;
          OLED_Clear();
          OLED_ShowString(SPO2_Symbol_Start_X+16,SPO2_Symbol_Start_Y,16,"SYS");
          OLED_ShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"DIA");    
          Show_Wait_Symbol();    
          OLED_Refresh_Gram();
          step=101;  
        }
        else//����������
        {
          PUMP_OFF();
          VALVE_OFF();
          Device_status = DEVICE_SLEEP;
          OLED_Clear();
          step=101;
        }
      }
    }
    P1IFG &= ~BIT3;             // P1.6 IFG cleared
    P1IE |= BIT3;               // P1.7 interrupt enabled
  
  }

