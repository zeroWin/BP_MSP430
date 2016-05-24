#include	"hal_SDcard.h"
#include	"hal_spi_user.h"

//////////////////////////////////////////////////////////////////////////////////

//待填说明

//////////////////////////////////////////////////////////////////////////////////	

//---------------------------相关变量声明-------------------------- 
uint8  SD_Type; 	//SD卡的类型

//---------------------------内部调用函数声明--------------------------
uint8 SD_SendCmd(uint8 cmd, uint32 arg, uint8 crc);
uint8 SD_Select(void);
uint8 SD_TypeJudge(void);
uint8 SD_RecvData(uint8*buf,uint16 len);
uint8 SD_SendBlock(uint8*buf,uint8 cmd);
uint8 SD_GetResponse(uint8 Response);


void SD_DisSelect(void);
void SD_SPI_Init(void);

//=============== 函数实现 =====================
/*
 * 函数名：SD_Initialize()
 * 输入：void
 * 输出：uint8 返回0：正确；返回R1：根据R1判断错误；返回UNKNOW_ERROR:未知错误
 * 功能：SPI硬件层初始化，内部调用，供SD_Initialize()函数调用
 */
uint8 SD_Initialize(void)							
{
	uint8 r1;      // 存放SD卡的返回值
	uint16 i;		 // 各类循环计数
	
	SD_SPI_Init();
	SD_SPI_SpeedLow();	//设置到低速模式 
	
	for(i=0;i<10;i++)SD_SPI_ReadWriteByte(DUMMY_DATA);		//SD操作前至少发哦是那个74个脉冲信号，发80个
	
	r1 = SD_TypeJudge();	//SD卡类型判断
	
	SD_DisSelect();//取消片选
	SD_SPI_SpeedHigh();//高速

	if(SD_Type)return 0;	//SD_TYPE不为0，表示判断出SD卡类型，返回0正确
	else if(r1)return r1; 	//R1不为0，表示对SD操作过程中有问题，返回R1根据R1进行判断
	return UNKNOW_ERROR;	//0xAA为未知错误	
}

/*
 * 函数名：SD_ReadDisk()
 * 输入：uint8*buf,uint32 sector,uint8 cnt
 * 		buf：数据缓存区
 *		sector：要读的扇区开始编号
 * 		cnt:要连续读取的扇区数
 * 输出：uint8 返回值:0,ok;其他,失败.
 * 功能：读SD卡
 */
uint8 SD_ReadDisk(uint8*buf,uint32 sector,uint8 cnt)
{
	uint8 r1;
	if(SD_Type!=SD_TYPE_V2HC)sector <<= 9;//转换为字节地址
	if(cnt==1)
	{
		r1=SD_SendCmd(CMD17,sector,0X01);//读命令
		if(r1==0)//指令发送成功
		{
			r1=SD_RecvData(buf,512);//接收512个字节	   
		}
	}else
	{
		r1=SD_SendCmd(CMD18,sector,0X01);//连续读命令
		do
		{
			r1=SD_RecvData(buf,512);//接收512个字节	 
			buf+=512;  
		}while(--cnt && r1==0); 	
		SD_SendCmd(CMD12,0,0X01);	//发送停止命令
	}   
	SD_DisSelect();//取消片选
	return r1;
}

/*
 * 函数名：SD_WriteDisk()
 * 输入：uint8*buf,uint32 sector,uint8 cnt
 * 		buf：数据缓存区
 *		sector：要写的扇区开始编号
 * 		cnt:要连续写入的扇区数
 * 输出：uint8 返回值:0,ok;其他,失败.
 * 功能：写SD卡
 */
uint8 SD_WriteDisk(uint8*buf,uint32 sector,uint8 cnt)
{
	uint8 r1;
	if(SD_Type!=SD_TYPE_V2HC)sector *= 512;//转换为字节地址
	if(cnt==1)
	{
		r1=SD_SendCmd(CMD24,sector,0X01);//写命令
		if(r1==0)//指令发送成功
		{
			r1=SD_SendBlock(buf,0xFE);//写512个字节	   
		}
	}else
	{
		if(SD_Type!=SD_TYPE_MMC)
		{
			SD_SendCmd(CMD55,0,0X01);	
			SD_SendCmd(CMD23,cnt,0X01);//发送指令	
		}
 		r1=SD_SendCmd(CMD25,sector,0X01);//连续写命令
		if(r1==0)
		{
			do
			{
				r1=SD_SendBlock(buf,0xFC);//写入512个字节	 
				buf+=512;  
			}while(--cnt && r1==0);
			r1=SD_SendBlock(0,0xFD);//发送停止命令
		}
	}   
	SD_DisSelect();//取消片选
	return r1;//
}	   

/*
 * 函数名：SD_GetSectorCount()
 * 输入：void
 * 输出：uint32
 * 		0： 取容量出错
 *		其他:SD卡的容量(扇区数*512字节)
 * 功能：获取SD卡的总容量（或扇区数）
 * PS：//每扇区的字节数必为512，因为如果不是512，则初始化不能通过.	
 */  										  
uint32 SD_GetSectorCount(void)
{
    uint8 csd[16];
    uint32 Capacity;  
    uint8 n;
	uint16 csize;  					    
	//取CSD信息，如果期间出错，返回0
    if(SD_GetCSD(csd)!=0) return 0;	    
    //如果为SDHC卡，按照下面方式计算
    if((csd[0]&0xC0)==0x40)	 //V2.00HC卡
    {	
		csize = csd[9] + ((uint16)csd[8] << 8) + 1; //得到扇区数
		Capacity = (uint32)csize << 10;//扇区数*512KB，得到有多少个KB字节,之所以移动10位，是为了和下面的通用
    }else//V2.0或V1.XX或MMC卡
    {	
		//READ_BL_LEN为CSD[83:80]   csd[5]低4位
		//C_SIZE_MULT为CSD[59:47]	csd[9]的低两位和csd[10]最高位组成
		//C_SIZE为	   CSD[73:62]	csd[6]的低两位、csd[7]所有位、csd[8]的最高两位组成
		//容量 = 2^(READ_BL_LEN + C_SIZE_MULT + 2) *  （C_SIZE + 1）
		n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
		csize = (csd[8] >> 6) + ((uint16)csd[7] << 2) + ((uint16)(csd[6] & 3) << 10) + 1;
		Capacity= (uint32)csize << (n - 9);//得到扇区数   
    }
    return Capacity;
}

/*
 * 函数名：SD_GetCSD()
 * 输入：uint8 *:存放CID的内存，至少16Byte
 * 输出：uint8 返回值:0：NO_ERR 1：错误	
 * 功能：获取SD卡的CSD信息，包括容量和速度信息
 */  			   
uint8 SD_GetCSD(uint8 *csd_data)
{
    uint8 r1;	 
    r1=SD_SendCmd(CMD9,0,0x01);//发CMD9命令，读CSD
    if(r1==0)
	{
    	r1=SD_RecvData(csd_data, 16);//接收16个字节的数据 
    }
	SD_DisSelect();//取消片选
	if(r1)return 1;
	else return 0;
}  

/*
 * 函数名：SD_GetCID()
 * 输入：uint8 *:存放CID的内存，至少16Byte
 * 输出：uint8 返回值:0：NO_ERR 1：错误	
 * 功能：获取SD卡的CID信息，包括制造商信息
 */ 												   
uint8 SD_GetCID(uint8 *cid_data)
{
    uint8 r1;	   
    //发CMD10命令，读CID
    r1=SD_SendCmd(CMD10,0,0x01);
    if(r1==0x00)
	{
		r1=SD_RecvData(cid_data,16);//接收16个字节的数据	 
    }
	SD_DisSelect();//取消片选
	if(r1)return 1;
	else return 0;
}		

/*
 * 函数名：SD_WaitReady()
 * 输入：void
 * 输出：uint8:0,成功;1,失败;
 * 功能：等待卡准备好，超时表示错误
 */
uint8 SD_WaitReady(void)
{
  uint32 t=0;
  do
  {
    if(SD_SPI_ReadWriteByte(DUMMY_DATA)==0XFF)return 0;//OK,SD卡准备好会返回0XFF
    t++;		  	
  }while(t<0XFFFFFF);//等待 
  return 1;
}

/*
 * 函数名：SD_SPI_ReadWriteByte()
 * 输入：uint8 data：要写入的数据
 * 输出：uint8 读到的数据
 * 功能：对SD写入并读出数据
 */
uint8 SD_SPI_ReadWriteByte(uint8 data)
{
	return SPI1_ReadWriteByte(data);
}	  

/*
 * 函数名：SD_SPI_SpeedLow()
 * 输入：void
 * 输出：void
 * 功能：SD卡初始化的时候,需要SPI为低速
 */
void SD_SPI_SpeedLow(void)
{
 	SPI1_SetSpeed_Low();//设置到低速模式	
}

/*
 * 函数名：SD_SPI_SpeedHigh()
 * 输入：void
 * 输出：void
 * 功能：SD卡正常工作的时候,设置为高速
 */
void SD_SPI_SpeedHigh(void)
{
  SPI1_SetSpeed_High();//设置到高速模式	
}
////////////////////////////////////内部调用函数区///////////////////////////////////
/*
 * 函数名：SD_SendBlock()
 * 输入：uint8*buf,uint16 lens
 * 		buf:数据缓存区
 * 		cmd:指令
 * 输出：uint8:0,成功;其他,失败;
 * 功能：向sd卡写入一个数据包的内容 512字节
 */
uint8 SD_SendBlock(uint8*buf,uint8 cmd)
{	
	uint16 t;		  	  
	if(SD_WaitReady())return 1;//等待准备失效
	SD_SPI_ReadWriteByte(cmd);
	if(cmd!=0XFD)//不是结束指令
	{
          for(t=0;t<512;t++)SPI1_ReadWriteByte(buf[t]);//提高速度,减少函数传参时间
	  SD_SPI_ReadWriteByte(DUMMY_DATA);//忽略crc
	  SD_SPI_ReadWriteByte(DUMMY_DATA);
          t=SD_SPI_ReadWriteByte(DUMMY_DATA);//接收响应
          if((t&0x1F)!=0x05)return 2;//响应错误									  					    
	}						 									  					    
    return 0;//写入成功
}


/*
 * 函数名：SD_RecvData()
 * 输入：uint8*buf,uint16 lens
 * 		buf:数据缓存区
 * 		len:要读取的数据长度.
 * 输出：uint8:0,成功;其他,失败;	
 * 功能：从sd卡读取一个数据包的内容
 */
uint8 SD_RecvData(uint8*buf,uint16 len)
{			  	  
	if(SD_GetResponse(0xFE))return 1;//等待SD卡发回数据起始令牌0xFE
    while(len--)//开始接收数据
    {
        *buf=SPI1_ReadWriteByte(DUMMY_DATA);
        buf++;
    }
    //下面是2个伪CRC（dummy CRC）
    SD_SPI_ReadWriteByte(DUMMY_DATA);
    SD_SPI_ReadWriteByte(DUMMY_DATA);									  					    
    return 0;//读取成功
}
/*
 * 函数名：SD_TypeJudge()
 * 输入：void
 * 输出：uint8:返回R1的值
 * 功能：SD卡类型判断函数，用来设置SD卡的类型
 * 判断方法：yuleiSD卡学习感悟第四条
 */
uint8 SD_TypeJudge(void)
{
	uint16 retry;	// 用来进行超时计数
	uint16 i;		// 各类循环计数
	uint8 r1;		// SD命令响应R1 
	uint8 buf[4]; // 存储R7的除R1以外的32bit
	retry = 20;
	do
	{
		r1 = SD_SendCmd(CMD0,0,0x95);	//返回R1，进入IDLE(空闲)状态
	}while(r1!=0x01 && retry--);		//响应的8bit，最后一位是空闲标志位，可参见http://www.openedv.com/posts/list/21392.htm中某楼的图或V2.0的白皮书
	
	SD_Type = SD_TYPE_ERR;		//默认没有SD卡或SD卡有问题
	
	if(r1 == 0x01)			//发送的CMD0得到响应，表示有卡
	{
		if(SD_SendCmd(CMD8,0x1AA,0x87) == 1) //返回R7：R1+32bit，0x01表示空闲，没有错误 是SDV2.0
		{
			for(i=0;i<4;i++)buf[i]=SD_SPI_ReadWriteByte(DUMMY_DATA);	//接收R7剩下的32bit
			if(buf[2] == 0x01 && buf[3] == 0xAA)	//SD对CMD8命令响应正确
			{
				retry=0XFFFE;
				do
				{
					SD_SendCmd(CMD55,0,0X01);	//发送CMD55,CMD55表示下一条是应用指令即ACMD
					r1=SD_SendCmd(CMD41,0x40000000,0X01);//发送CMD41，返回R1
				}while(r1 && retry--);	//SD卡初始化，r1=0证明初始化成功
				
				//如果retry=0表示初始化失败，retry!=0表示初始化成功，鉴别SD2.0的版本
				//初始化之前，只有这几个命令有效，初始化成功后其他命令才有效
				if(retry && SD_SendCmd(CMD58,0,0X01) == 0x00)		//返回R3：R1+OCR(32bit) 正确响应0x00表示处于忙状态
				{
					for(i=0;i<4;i++)buf[i]=SD_SPI_ReadWriteByte(DUMMY_DATA);//得到OCR值 OCR第31位（30bit）用于判断V2.0的卡是否为SDHC类型
					if(buf[0] & 0x40) SD_Type = SD_TYPE_V2HC;	//为1：是告诉SD
					else SD_Type = SD_TYPE_V2;	//不为1：是普通SD
				}
			}
		}
		else //不是SDV2.0 判断是否为SD V1.x或者是MMC V3
		{
			SD_SendCmd(CMD55,0,0X01);		//发送CMD55
			r1=SD_SendCmd(CMD41,0,0X01);	//发送CMD41
			if(r1<=1)
			{		
				SD_Type=SD_TYPE_V1;
				retry=0XFFFE;
				do //等待退出IDLE模式
				{
					SD_SendCmd(CMD55,0,0X01);	//发送CMD55
					r1=SD_SendCmd(CMD41,0,0X01);//发送CMD41
				}while(r1&&retry--);
			}else
			{
				SD_Type=SD_TYPE_MMC;//MMC V3
				retry=0XFFFE;
				do //等待退出IDLE模式
				{											    
					r1=SD_SendCmd(CMD1,0,0X01);//发送CMD1
				}while(r1&&retry--);  
			}
			if(retry==0||SD_SendCmd(CMD16,512,0X01)!=0)SD_Type=SD_TYPE_ERR;//错误的卡，CMD16为选择一个扇区
		}
	}
	return r1;
}
/*
 * 函数名：SD_SPI_Init()
 * 输入：void
 * 输出：void
 * 功能：SPI硬件层初始化，内部调用，供SD_Initialize()函数调用
 */
void SD_SPI_Init(void)
{
  SPI1_Config_Init();	//SPI模块初始化
  
  // 初始化CS端口
  P3SEL &= ~BIT2;    // 普通端口功能
  P3DIR |= BIT2;     // 输出
  
  SD_CS_H;		//拉高CS释放SPI总线
}

/*
 * 函数名：SD_DisSelect()
 * 输入：void
 * 输出：void
 * 功能：取消选择,释放SPI总线
 */
void SD_DisSelect(void)
{
  SD_CS_H;
  SD_SPI_ReadWriteByte(DUMMY_DATA);//提供额外的8个时钟
}

/*
 * 函数名：SD_DisSelect()
 * 输入：void
 * 输出：uint8:0,成功;1,失败;
 * 功能：选择sd卡,并且等待卡准备OK
 */
uint8 SD_Select(void)
{
  SD_CS_L;
  if(SD_WaitReady()==0)return 0;//等待成功
  SD_DisSelect();
  return 1;//等待失败
}

/*
 * 函数名：SD_SendCmd()
 * 输入：
 * 		uint8 cmd   命令 
 * 		uint8 arg   命令参数
 * 		uint8 crc   crc校验值
 * 输出：uint8:SD卡返回的响应
 * 功能：向SD卡发送一个命令
 */   
//返回值:														  
uint8 SD_SendCmd(uint8 cmd, uint32 arg, uint8 crc)
{
  uint8 r1;	
  uint8 Retry=0; 
  SD_DisSelect();//取消上次片选
  if(SD_Select())return 0XFF;//片选失效 
  //发送
  SD_SPI_ReadWriteByte(cmd | 0x40);//分别写入命令，首两位固定为01
  SD_SPI_ReadWriteByte(arg >> 24);
  SD_SPI_ReadWriteByte(arg >> 16);
  SD_SPI_ReadWriteByte(arg >> 8);
  SD_SPI_ReadWriteByte(arg);	  
  SD_SPI_ReadWriteByte(crc); 
  if(cmd==CMD12)SD_SPI_ReadWriteByte(DUMMY_DATA);//Skip a stuff byte when stop reading
  //等待响应，或超时退出
  Retry=0X1F;//31
  do
  {
    r1=SD_SPI_ReadWriteByte(DUMMY_DATA);
  }while((r1&0X80) && Retry--);	 //只有收到命令响应，首位才为0，为什么r1&0X80可以参见http://www.openedv.com/posts/list/21392.htm中某楼的图或V2.0的白皮书
  //返回状态值
  return r1;
}	

/*
 * 函数名：SD_GetResponse()
 * 输入：uint8 Response:要得到的回应值
 * 输出：uint8:0,成功得到了该回应值；其他,得到回应值失败
 * 功能：等待SD卡回应
 */
uint8 SD_GetResponse(uint8 Response)
{
  uint16 Count=0xFFFF;//等待次数	   						  
  while ((SD_SPI_ReadWriteByte(0XFF)!=Response)&&Count)Count--;//等待得到准确的回应  	  
  if (Count==0)return MSD_RESPONSE_FAILURE;//得到回应失败   
  else return MSD_RESPONSE_NO_ERROR;//正确回应
}
////////////////////////////////////内部调用函数区结束///////////////////////////////////

