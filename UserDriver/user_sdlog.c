/**
******************************************************************************
* @文件    user_sdlog.C
* @作者    
* @版本    V0.0.1
******************************************************************************
*日志记录相关函数
*基于FATFS
*所有文件存放SD卡根目录。包含一个index.txt目录文件和若干（最多2000）个日志文件
*建立新日志文件时，会在index.txt目录文件中进行记录，便于之后读取
*日志文件命名格式：yyyymmdd_hhmmss.txt，时间为日志文件建立的时间
*单文件最多保存100000条日志记录，超过时自动重新建立新的日志文件
*超过2000个文件时，会覆盖最早的日志文件（根据index.txt目录文件的记录）
******************************************************************************
*/
#include "user_sdlog.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"

__align(8) FATFS fs; //逻辑磁盘工作区
__align(8) FIL fil;	 //文件

extern TimeStructDef SysTime; //系统时间
uint32_t total_kbyte = 0;
uint32_t free_kbyte = 0;
FRESULT f_rst;
uint32_t f_idx = 0; //文件索引
uint32_t RecordStored = 0;
uint8_t namebuff[30] = {0};
uint8_t tempbuff[300] = {0};
uint8_t SDstatus = 0; //0-无卡，1-正常，2-错误
extern xSemaphoreHandle SDCARD_Mutex;
/**
  * @brief  得到磁盘剩余容量
  * @param  drv:磁盘编号("0:"/"1:") total:总容量（单位KB） free:剩余容量（单位KB）
  * @retval 0-成功 其他-失败
  */
uint8_t exf_getfree(uint8_t *drv, uint32_t *total, uint32_t *free)
{
	FATFS *fs1;
	uint8_t res;
	uint32_t fre_clust = 0, fre_sect = 0, tot_sect = 0;
	//得到磁盘信息及空闲簇数量
	res = (uint32_t)f_getfree((const TCHAR *)drv, (DWORD *)&fre_clust, &fs1);
	if (res == 0)
	{
		tot_sect = (fs1->n_fatent - 2) * fs1->csize; //得到总扇区数
		fre_sect = fre_clust * fs1->csize;			 //得到空闲扇区数
#if _MAX_SS != 512									 //扇区大小不是512字节,则转换为512字节
		tot_sect *= fs1->ssize / 512;
		fre_sect *= fs1->ssize / 512;
#endif
		*total = tot_sect >> 1; //单位为KB
		*free = fre_sect >> 1;	//单位为KB
	}
	return res;
}

/**
  * @brief  删除SD根目录所有文件
  * @param  none
  * @retval 0-成功 其他-失败
  */
FRESULT SDDeleteAllFiles(void)
{
	uint8_t FilePath[30] = {0};
	FRESULT res;
	DIR dir;
	//    UINT i;
	static FILINFO fno;
	uint8_t path[] = "0:";
	res = f_mount(&fs, "0:", 1);
	if (res != FR_OK)
	{
		printf("SDCard Fail!\n");
		SDstatus = 2;
		return res;
	}
	res = f_opendir(&dir, (char *)path); /* Open the directory */
	if (res == FR_OK)
	{
		for (;;)
		{
			res = f_readdir(&dir, &fno); /* Read a directory item */
			if (res != FR_OK || fno.fname[0] == 0)
				break; /* Break on error or end of dir */
					   // if (fno.fattrib & AM_DIR) {                    /* It is a directory */
					   //     i = strlen(path);
					   //     sprintf(&path[i], "/%s", fno.fname);
					   //     res = delete_files();                    /* Enter the directory */
					   //     if (res != FR_OK) break;
					   //     path[i] = 0;
					   // }
			// else
			{ /* It is a file. */
				sprintf((char *)FilePath, "%s%s", path, fno.fname);
				if (f_unlink((char *)FilePath) == FR_OK)
					printf("Delete File %s Success\n", FilePath); /*删除文件*/
			}
		}
		f_closedir(&dir);
	}
	if (res == FR_OK)
	{
		memset(namebuff, 0, sizeof(namebuff));
	}
	return res;
}

/**
  * @brief  将SD卡中的文件列表打印在串口上
  * @param  filename 文件名
  * @retval 0-成功 其他-失败
  */
FRESULT SDPrintFileList(uint8_t *filename)
{
	FRESULT res;
	DIR dir;
	static FILINFO fno;
	uint8_t path[] = "0:";
	res = f_mount(&fs, "0:", 1);
	if (res != FR_OK)
	{
		printf("SDCard Fail!\n");
		SDstatus = 2;
		return res;
	}
	res = f_opendir(&dir, (char *)path); /* Open the directory */
	if (res == FR_OK)
	{
		for (;;)
		{
			res = f_readdir(&dir, &fno); /* Read a directory item */
			if (res != FR_OK || fno.fname[0] == 0)
				break; /* Break on error or end of dir */
			if (!(fno.fattrib & AM_DIR))
			{
				if (memcmp(filename, fno.fname, 8) == 0)
				{
					printf("%s\n", fno.fname);
				}
			}
		}
		f_closedir(&dir);
	}
	return res;
}

/**
  * @brief  将SD卡中的文件打印在串口上
  * @param  filename 文件名
  * @retval 0-成功 其他-失败
  */
FRESULT SDPrintFile(uint8_t *filename)
{
	FRESULT res;
	uint8_t tempbuff[200];
	res = f_mount(&fs, "0:", 1);
	if (res != FR_OK)
	{
		printf("SDCard Fail!\n");
		SDstatus = 2;
		return res;
	}
	res = f_open(&fil, (char *)filename, FA_READ | FA_OPEN_EXISTING);
	while (1)
	{
		memset(tempbuff, 0, sizeof(tempbuff));
		if (f_gets((char *)tempbuff, 200, &fil) == NULL)
		{
			break;
		}
		printf("%s", tempbuff);
	}
	res = f_close(&fil);
	return res;
}

/**
  * @brief  将日志写入SD卡
  * @param  None
  * @retval 0-成功 其他-失败
  */
uint8_t SD_Log_Write(void)
{
	if (SDstatus == 0) //没有SD卡
	{
		return 1;
	}
	if (RecordStored == RecordCurr)
	{
		return 0;
	}
	if (xSemaphoreTake(SDCARD_Mutex, 0) != pdTRUE)
	{
		return 1;
	}
	if (f_mount(&fs, "0:", 1) != FR_OK)
	{
		if (SDstatus != 2)
		{
			SDstatus = 2;
			//WorkRecordReadWrite(0, 0, __LINE__, "user_sdlog: WARN LOG SDCard Failed");
		}
		if (xSemaphoreGive(SDCARD_Mutex) != pdTRUE)
		{
			return 1;
		}
		return 1;
	}
	else if (SDstatus == 2)
	{
		SDstatus = 1;
	}
	if (exf_getfree((uint8_t *)"0:", &total_kbyte, &free_kbyte))
	{
		if (xSemaphoreGive(SDCARD_Mutex) != pdTRUE)
		{
			return 1;
		}
		//WorkRecordReadWrite(0, 0, __LINE__, "user_sdlog: WARN LOG SDCard Failed");
		return 1;
	}
	if (free_kbyte < 10240) //剩余空间小于10MB
	{
		if (xSemaphoreGive(SDCARD_Mutex) != pdTRUE)
		{
			return 1;
		}
		//WorkRecordReadWrite(0, 0, __LINE__, "user_sdlog: WARN LOG SDCard Not Enough Space");
		return 1;
	}

	if (namebuff[0] == 0) //没建立文件
	{
		RecordStored = 0;
		sprintf((char *)namebuff, "0:%04ld%02d%02d_%02d%02d%02d.txt", (long)SysTime.year + 2000, SysTime.month, SysTime.day, SysTime.hour, SysTime.minute, SysTime.second);
		//打开index.txt文件
		f_rst = f_open(&fil, "0:index.txt", FA_READ | FA_OPEN_EXISTING);
		if (f_rst != FR_OK) //创建index.txt文件
		{
			f_idx = 0;
			f_rst = f_open(&fil, "0:index.txt", FA_WRITE | FA_OPEN_ALWAYS);
			memset(tempbuff, 0, sizeof(tempbuff));
			f_printf(&fil, "0000\n");					   //写入初始idx
			f_printf(&fil, "%04ld %s\n", f_idx, namebuff); //写入本次文件索引
			f_rst = f_close(&fil);
			//WorkRecordReadWrite(0, 0, __LINE__, "user_sdlog: INFO LOG Create index.txt");
		}
		else
		{
			f_gets((char *)tempbuff, 5, &fil);
			f_idx = atol((char *)tempbuff);
			f_idx = U32T_LimitAdd(f_idx, 1, 2000);
			f_rst = f_lseek(&fil, 6 + (f_idx * 28)); //Idx占用6个字节，一个索引占用28个字节
			memset(tempbuff, 0, sizeof(tempbuff));
			f_gets((char *)tempbuff, 30, &fil);
			f_rst = f_close(&fil);
			if (tempbuff[0] != 0) //有文件
			{
				f_rst = f_unlink((char *)&tempbuff[5]);
			}
			f_rst = f_open(&fil, "0:index.txt", FA_WRITE | FA_OPEN_ALWAYS);
			f_printf(&fil, "%04ld\n", f_idx);			   //写入idx
			f_rst = f_lseek(&fil, 6 + (f_idx * 28));	   //Idx占用6个字节，一个索引占用28个字节
			f_printf(&fil, "%04ld %s\n", f_idx, namebuff); //写入本次文件索引
			f_rst = f_close(&fil);
			//WorkRecordReadWrite(0, 0, __LINE__, "user_sdlog: INFO LOG Create New Logfile");
		}
	}
	if (RecordStored < RecordCurr) //正常情况
	{
		//创建、打开日志文件
		f_rst = f_open(&fil, (char *)namebuff, FA_WRITE | FA_OPEN_ALWAYS);
		f_rst = f_lseek(&fil, fil.obj.objsize);
		//写入日志文件

		for (; RecordStored < RecordCurr; RecordStored++)
		{
			memset(tempbuff, 0, sizeof(tempbuff));
			WorkRecordReadWrite(1, RecordStored, (char *)tempbuff);
			f_printf(&fil, "%05ld %s\n", RecordStored, tempbuff);
		}
		f_rst = f_close(&fil);
	}
	else //已经超过了最大sdram记录idx，记录在原有文件，超过的部分新建文件
	{
		//打开日志文件
		f_rst = f_open(&fil, (char *)namebuff, FA_WRITE | FA_OPEN_ALWAYS);
		f_rst = f_lseek(&fil, fil.obj.objsize);
		//写入日志文件

		for (; RecordStored < 100000; RecordStored++)
		{
			memset(tempbuff, 0, sizeof(tempbuff));
			WorkRecordReadWrite(1, RecordStored, (char *)tempbuff);
			f_printf(&fil, "%05ld %s\n", RecordStored, tempbuff);
		}
		f_rst = f_close(&fil);

		//创建新的日志文件
		sprintf((char *)namebuff, "0:%04ld%02d%02d_%02d%02d%02d.txt", (long)SysTime.year + 2000, SysTime.month, SysTime.day, SysTime.hour, SysTime.minute, SysTime.second);
		//打开index.txt文件
		f_rst = f_open(&fil, "0:index.txt", FA_READ | FA_OPEN_EXISTING);
		f_gets((char *)tempbuff, 5, &fil);
		f_idx = atol((char *)tempbuff);
		f_idx = U32T_LimitAdd(f_idx, 1, 2000);
		f_rst = f_lseek(&fil, 6 + (f_idx * 28)); //Idx占用6个字节，一个索引占用28个字节
		memset(tempbuff, 0, sizeof(tempbuff));
		f_gets((char *)tempbuff, 30, &fil);
		f_rst = f_close(&fil);
		if (tempbuff[0] != 0) //有文件
		{
			f_rst = f_unlink((char *)&tempbuff[5]);
		}
		f_rst = f_open(&fil, "0:index.txt", FA_WRITE | FA_OPEN_ALWAYS);
		f_printf(&fil, "%04ld\n", f_idx);			   //写入idx
		f_rst = f_lseek(&fil, 6 + (f_idx * 28));	   //Idx占用6个字节，一个索引占用28个字节
		f_printf(&fil, "%04ld %s\n", f_idx, namebuff); //写入本次文件索引
		f_rst = f_close(&fil);
		//WorkRecordReadWrite(0, 0, __LINE__, "user_sdlog: INFO LOG Create New Logfile");

		//创建、打开日志文件
		f_rst = f_open(&fil, (char *)namebuff, FA_WRITE | FA_OPEN_ALWAYS);
		f_rst = f_lseek(&fil, fil.obj.objsize);
		RecordStored = 0;
		//写入日志文件
		//if (Use_RightActRecord == 1)
		//WorkRecordReadWrite(0, 0, __LINE__, "user_sdlog: INFO LOG Write to SD");
		for (; RecordStored < RecordCurr; RecordStored++)
		{
			memset(tempbuff, 0, sizeof(tempbuff));
			WorkRecordReadWrite(1, RecordStored, (char *)tempbuff);
			f_printf(&fil, "%05ld %s\n", RecordStored, tempbuff);
		}
		f_rst = f_close(&fil);
	}
	if (xSemaphoreGive(SDCARD_Mutex) != pdTRUE)
	{
		return 1;
	}
	return 0;
}
/*****************************END OF FILE*****************************/
