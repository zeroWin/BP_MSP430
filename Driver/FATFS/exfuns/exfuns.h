#ifndef __EXFUNS_H
#define __EXFUNS_H 		   
#include "ff.h"
#include "hal_type.h"
//////////////////////////////////////////////////////////////////////////////////

//V1.0 ��Ҫ����һЩȫ�ֱ���������FATFS��ʹ�ã�ͬʱʵ��һЩ�����������ȡ�Ĺ��ܺ���

//////////////////////////////////////////////////////////////////////////////////	
extern FATFS *fs;  
extern FIL *file;	  
extern UINT br,bw;
 
uint8 exfuns_init(void);		//�����ڴ�


#endif


