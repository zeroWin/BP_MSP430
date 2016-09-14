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

#ifndef __MAIN_H_
#define __MAIN_H_
#include "hal_type.h"               //Basic Type declarations

typedef enum
{
  BP_ONLINE_IDLE,             // 在线等待状态
  BP_ONLINE_MEASURE,          // 在线测量状态
  BP_OFFLINE_IDLE,            // 离线等待状态
  BP_OFFLINE_MEASURE,         // 离线测量状态
  BP_FIND_NETWORK,            // 找网状态
  BP_SYNC_DATA,               // 同步数据状态
  BP_CLOSING,                 // 关闭网络状态
  BP_ON_SLEEP,                // 在线睡眠状态
  BP_OFF_SLEEP,               // 离线睡眠状态 
} BPSystemStatus_t;


#define SPO2_Wait_Symbol_Start_X 11
#define SPO2_Wait_Symbol_Start_Y 44

#define HR_Wait_Symbol_Start_X   76
#define HR_Wait_Symbol_Start_Y   44

#define SPO2_Show3Num_Start_X   15
#define SPO2_Show3Num_Start_Y   36

#define HR_Show3Num_Start_X     76   
#define HR_Show3Num_Start_Y     36

#define SPO2_Show2Num_Start_X   15
#define SPO2_Show2Num_Start_Y   36

#define HR_Show2Num_Start_X     83   
#define HR_Show2Num_Start_Y     36

#define SPO2_Symbol_Start_X     8
#define SPO2_Symbol_Start_Y     16

#define PR_Symbol_Start_X     88
#define PR_Symbol_Start_Y     16

#define Heart_Sympol_Start_X    65
#define Heart_Sympol_Start_Y    0



void Show_Wait_Symbol(const char *p);

#endif /*AFE44x0_MAIN_H_*/




























