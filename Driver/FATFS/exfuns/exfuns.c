#include "exfuns.h"
#include "string.h"
#include "stdlib.h"
//////////////////////////////////////////////////////////////////////////////////

//待填说明

//////////////////////////////////////////////////////////////////////////////////	


///////////////////////////////公共文件区,使用malloc的时候////////////////////////////////////////////
FATFS *fs;  		//逻辑磁盘工作区.	 
FIL *file;	        //文件1
UINT br,bw;	        //读写变量

///////////////////////////////////////////////////////////////////////////////////////
//为exfuns申请内存
//返回值:0,成功
//1,失败
uint8 exfuns_init(void)
{
	fs=(FATFS*)malloc(sizeof(FATFS));	//为磁盘0工作区申请内存	
	file=(FIL*)malloc(sizeof(FIL));	//为file申请内存
	if(fs&&file)return 0;  //申请有一个失败,即失败.
	else return 1;	
}




















