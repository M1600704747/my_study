/**
******************************************************************************
* @文件    user_sdlog.H
* @作者    
* @版本    V0.0.1
******************************************************************************

******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_SDLOG_H
#define __USER_SDLOG_H 					   
#include "user_gpm.h"
#include "main.h"
#include "ff.h"
#include "ffconf.h"

extern FATFS fs ;
extern FIL fil ;
extern uint32_t f_idx;//文件索引
extern uint8_t SDstatus;

uint8_t exf_getfree(uint8_t *drv,uint32_t *total,uint32_t *free);
FRESULT SDDeleteAllFiles (void);
FRESULT SDPrintFileList (uint8_t* filename);
FRESULT SDPrintFile (uint8_t* filename);
uint8_t SD_Log_Write(void);
#endif

/*****************************END OF FILE*****************************/
