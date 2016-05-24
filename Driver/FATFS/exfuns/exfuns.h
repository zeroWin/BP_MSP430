#ifndef __EXFUNS_H
#define __EXFUNS_H 		   
#include "ff.h"
#include "hal_type.h"
//////////////////////////////////////////////////////////////////////////////////

//V1.0 主要定义一些全局变量，放量FATFS的使用，同时实现一些如磁盘容量获取的功能函数

//////////////////////////////////////////////////////////////////////////////////	
extern FATFS *fs;  
extern FIL *file;	  
extern UINT br,bw;
extern FILINFO fileinfo;
extern DIR dir;
//extern uint8 *fatbuf;//SD卡数据缓存区


//fypetell返回的类型定义
//根据表FILEYPEBL获得.在exfuns.c里面定义
#define T_BIN		0X00	//bin文件
#define T_LRC		0X10	//lrc文件
#define T_NES		0X20	//nes文件
#define TEXT		0X30	//.txt文件
#define T_C			0X31	//.c文件
#define T_H			0X32    //.h文件
#define T_FLAC		0X4C	//flac文件
#define T_BMP		0X50	//bmp文件
#define T_JPG		0X51	//jpg文件
#define T_JPEG		0X52	//jpeg文件		 
#define T_GIF		0X53	//gif文件  

 
uint8 exfuns_init(void);		//申请内存
uint8 f_typetell(uint8 *fname);	//识别文件类型
uint8 exf_getfree(uint8 *drv,uint32 *total,uint32 *free);//得到磁盘总容量和剩余容量
uint8 char_upper(uint8 c);      //转换大小写
//uint32 exf_fdsize(uint8 *fdname);																				   //得到文件夹大小
//uint8* exf_get_src_dname(uint8* dpfn);																		   
//uint8 exf_copy(uint8(*fcpymsg)(uint8*pname,uint8 pct,uint8 mode),uint8 *psrc,uint8 *pdst,uint32 totsize,uint32 cpdsize,uint8 fwmode);	   //文件复制
//uint8 exf_fdcopy(uint8(*fcpymsg)(uint8*pname,uint8 pct,uint8 mode),uint8 *psrc,uint8 *pdst,uint32 *totsize,uint32 *cpdsize,uint8 fwmode);//文件夹复制

#endif


