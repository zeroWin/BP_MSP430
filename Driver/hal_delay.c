/*****************************************************************************
延时子程序
*****************************************************************************/
#include "hal_delay.h"

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