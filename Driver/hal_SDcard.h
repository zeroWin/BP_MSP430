#ifndef	__SDcard_H
#define	__SDcard_H

//////////////////////////////////////////////////////////////////////////////////

//待填说明

//////////////////////////////////////////////////////////////////////////////////	
//---------------------------头文件--------------------------
#include "hal_type.h"
#include "msp430f5528.h"
//---------------------------SD卡类型定义 -------------------------- 
#define SD_TYPE_ERR     0X00		//无卡或卡不能识别
#define SD_TYPE_MMC     0X01		//MMC卡
#define SD_TYPE_V1      0X02		//使用V1.0标准的SD卡
#define SD_TYPE_V2      0X04		//使用V2.0标准的SD卡
#define SD_TYPE_V2HC    0X06	 	//V2.0HC高速卡

//--------------------------- SD卡指令表-------------------------- 
#define CMD0    0       //卡复位
#define CMD1    1		//命令1 ，读OCR寄存器
#define CMD8    8       //命令8 ，SEND_IF_COND 只有V2.0卡才有该命令，可用于判断SD卡类型，返回0x01是V2.0，不是0x01不是V2.0
#define CMD9    9       //命令9 ，读CSD数据
#define CMD10   10      //命令10，读CID数据
#define CMD12   12      //命令12，停止数据传输
#define CMD16   16      //命令16，设置SectorSize 应返回0x00
#define CMD17   17      //命令17，读sector
#define CMD18   18      //命令18，读Multi sector
#define CMD23   23      //命令23，设置多sector写入前预先擦除N个block
#define CMD24   24      //命令24，写sector
#define CMD25   25      //命令25，写Multi sector
#define CMD41   41      //命令41，应返回0x00	及ACOM41
#define CMD55   55      //命令55，应返回0x01 CMD55表示下一条是应用指令即ACMD CMD55+CMD41组合用于判断是V1.0还是MMC卡 
#define CMD58   58      //命令58，读OCR信息	第31位可用于判断V2.0卡是否为SDHC类型
#define CMD59   59      //命令59，使能/禁止CRC，应返回0x00

//--------------------------- 数据写入回应字意义 -------------------------- 
#define MSD_DATA_OK                0x05
#define MSD_DATA_CRC_ERROR         0x0B
#define MSD_DATA_WRITE_ERROR       0x0D
#define MSD_DATA_OTHER_ERROR       0xFF

//--------------------------- SD卡回应标记字 -------------------------- 
#define MSD_RESPONSE_NO_ERROR      0x00
#define MSD_IN_IDLE_STATE          0x01
#define MSD_ERASE_RESET            0x02
#define MSD_ILLEGAL_COMMAND        0x04
#define MSD_COM_CRC_ERROR          0x08
#define MSD_ERASE_SEQUENCE_ERROR   0x10
#define MSD_ADDRESS_ERROR          0x20
#define MSD_PARAMETER_ERROR        0x40
#define MSD_RESPONSE_FAILURE       0xFF

//--------------------------- SD卡CS端口及操作宏定义 -------------------------- 
#define	SD_GPIO_CS_PORT							1
#define SD_CS_GPIO_PIN							2		//P1.2:CS

#define SD_CS_H P3OUT |= BIT2	     // P3.2 CS = 1 
#define SD_CS_L P3OUT &= ~BIT2      // p3.2 CS = 0

//--------------------------- 相关定值宏定义 -------------------------- 
#define DUMMY_DATA 0xFF			//不被SD卡识别的命令值，用于产生时钟信号
#define UNKNOW_ERROR 0xAA		//未知错误

//---------------------------相关变量声明-------------------------- 
extern uint8  SD_Type;//SD卡的类型


//SDcard控制函数
uint8 SD_Initialize(void);//SD相关初始化
uint8 SD_ReadDisk(uint8*buf,uint32 sector,uint8 cnt);//读SD卡扇区操作
uint8 SD_WriteDisk(uint8*buf,uint32 sector,uint8 cnt);//写SD卡扇区操作

uint32 SD_GetSectorCount(void);//得到SD卡的扇区（或总容量）
uint8 SD_GetCSD(uint8 *csd_data);//得到SD卡的CSD信息
uint8 SD_GetCID(uint8 *cid_data);//获取SD卡的CID信息
uint8 SD_WaitReady(void);//等待SD卡准备好

void SD_SPI_SpeedLow(void);//设置低速模式	
void SD_SPI_SpeedHigh(void);//设置高速模式
uint8 SD_SPI_ReadWriteByte(uint8 data);//向SD卡发送接收单字节数据

#endif
