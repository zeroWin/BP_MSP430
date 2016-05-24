//******************************************************************************
//  Praveen Aroul
//  HealthTech, MHR
//  (C) Texas Instruments Inc., 2013
//  All Rights Reserved.
//  Built with IAR Workbench 5.50.2
//
//-------------------------------------------------------------------------------
// THIS PROGRAM IS PROVIDED "AS IS". TI MAKES NO WARRANTIES OR
// REPRESENTATIONS, EITHER EXPRESS, IMPLIED OR STATUTORY,
// INCLUDING ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR
// COMPLETENESS OF RESPONSES, RESULTS AND LACK OF NEGLIGENCE.
// TI DISCLAIMS ANY WARRANTY OF TITLE, QUIET ENJOYMENT, QUIET
// POSSESSION, AND NON-INFRINGEMENT OF ANY THIRD PARTY
// INTELLECTUAL PROPERTY RIGHTS WITH REGARD TO THE PROGRAM OR
// YOUR USE OF THE PROGRAM.
//
// IN NO EVENT SHALL TI BE LIABLE FOR ANY SPECIAL, INCIDENTAL,
// CONSEQUENTIAL OR INDIRECT DAMAGES, HOWEVER CAUSED, ON ANY
// THEORY OF LIABILITY AND WHETHER OR NOT TI HAS BEEN ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGES, ARISING IN ANY WAY OUT
// OF THIS AGREEMENT, THE PROGRAM, OR YOUR USE OF THE PROGRAM.
// EXCLUDED DAMAGES INCLUDE, BUT ARE NOT LIMITED TO, COST OF
// REMOVAL OR REINSTALLATION, COMPUTER TIME, LABOR COSTS, LOSS
// OF GOODWILL, LOSS OF PROFITS, LOSS OF SAVINGS, OR LOSS OF
// USE OR INTERRUPTION OF BUSINESS. IN NO EVENT WILL TI'S
// AGGREGATE LIABILITY UNDER THIS AGREEMENT OR ARISING OUT OF
// YOUR USE OF THE PROGRAM EXCEED FIVE HUNDRED DOLLARS
// (U.S.$500).
//
// Unless otherwise stated, the Program written and copyrighted
// by Texas Instruments is distributed as "freeware".  You may,
// only under TI's copyright in the Program, use and modify the
// Program without any charge or restriction.  You may
// distribute to third parties, provided that you transfer a
// copy of this license to the third party and the third party
// agrees to these terms by its first use of the Program. You
// must reproduce the copyright notice and any other legend of
// ownership on each copy or partial copy, of the Program.
//
// You acknowledge and agree that the Program contains
// copyrighted material, trade secrets and other TI proprietary
// information and is protected by copyright laws,
// international copyright treaties, and trade secret laws, as
// well as other intellectual property laws.  To protect TI's
// rights in the Program, you agree not to decompile, reverse
// engineer, disassemble or otherwise translate any object code
// versions of the Program to a human-readable form.  You agree
// that in no event will you alter, remove or destroy any
// copyright notice included in the Program.  TI reserves all
// rights not specifically granted under this license. Except
// as specifically provided herein, nothing in this agreement
// shall be construed as conferring by implication, estoppel,
// or otherwise, upon you, any license or other right under any
// TI patents, copyrights or trade secrets.
//
// You may not use the Program in non-TI devices.
//--------------------------------------------------------------------------------

#ifndef AFE44x0_MAIN_H_
#define AFE44x0_MAIN_H_

#define SOT     0x02
#define EOT     0x03
#define CR      0x0D

#define WRITE_REG_CMD                   0x02
#define READ_REG_CMD                    0x03
#define START_READ_ADC_REG_CMD          0x01
#define STOP_READ_ADC_REG_CMD           0x06
#define DEV_ID_CMD                      0x04
#define FW_UPGRADE_CMD                  0x05
#define FW_VERSION_CMD                  0x07

#define __AFE4400__
//#define __AFE4490__

#define DEVICE_SLEEP    0x00        //Ë¯Ãß×´Ì¬
#define DEVICE_WAIT     0x01        //µÈ´ý×´Ì¬
#define DEVICE_MEASURE  0x02        //²âÁ¿×´Ì¬

#define SPO2_Wait_Symbol_Start_X 11
#define SPO2_Wait_Symbol_Start_Y 40

#define HR_Wait_Symbol_Start_X   76
#define HR_Wait_Symbol_Start_Y   40

#define SPO2_Show3Num_Start_X   15
#define SPO2_Show3Num_Start_Y   30

#define HR_Show3Num_Start_X     76   
#define HR_Show3Num_Start_Y     30

#define SPO2_Show2Num_Start_X   15
#define SPO2_Show2Num_Start_Y   30

#define HR_Show2Num_Start_X     83   
#define HR_Show2Num_Start_Y     30

#define SPO2_Symbol_Start_X     8
#define SPO2_Symbol_Start_Y     12

#define PR_Symbol_Start_X     88
#define PR_Symbol_Start_Y     12

#define Heart_Sympol_Start_X    65
#define Heart_Sympol_Start_Y    0
//Function declarations
void Init_Ports (void);
void Init_Clock (void);
void Show_Wait_Symbol(void);
void Init_TimerA1 (void);
void Cal_spo2_and_HR(void);
void Init_KEY_Interrupt (void);
//VOID Init_TimerA1 (VOID);
void Init_ADG1608();
void ADG1608_select(unsigned char S_number);

void UART_send(unsigned char* byt_string, int length);
void Init_UART();
void UART_send(unsigned char* byt_string, int length);

//volatile unsigned char RxBuffer[6]; 
//unsigned char MMA8451_senddata[8];

void delay(unsigned long num);

void Init_MPY(void);




extern void oledinit(void);
extern void delay(unsigned long num);
extern void write_oled_data(unsigned char ucData);
extern void OLED_Refresh_Gram(void);

extern void OLED_Clear(void);
extern void OLED_ShowSymbol(u8 x,u8 y,u8 sym,u8 mode);
extern void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
extern void OLED_ShowString(u8 x,u8 y,u8 size,const u8 *p);

extern void OLED_ShowNum(u8 x,u8 y,int num,u8 len,u8 size);
extern void OLED_SLEEP(u8 st);
extern void OLED_ShowWaitSymbol(u8 x,u8 y,u8 mode);
extern void OLED_ShowHeartSymbol(u8 x,u8 y,u8 mode,u8 HeartType);

#endif /*AFE44x0_MAIN_H_*/




























