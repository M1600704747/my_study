/**
  ******************************************************************************
  * File Name          : RTC
  * Description        : 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_RTC_H
#define __USER_RTC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "user_gpm.h"

/*define-----------------------------------------------------------------------*/
#define RTC_SCL_Pin								GPIO_PIN_8
#define RTC_SCL_GPIO_Port					GPIOF
#define RTC_SDA_Pin								GPIO_PIN_9
#define RTC_SDA_GPIO_Port					GPIOF

#define RTC_SECONDS						    	0x00
#define RTC_MINUTES   							0x01
#define RTC_CENT_HOURS							0x02
#define RTC_DAY							        0x03
#define RTC_DATE					        	0x04
#define RTC_MONTH					        	0x05
#define RTC_YEAR					        	0x06
#define RTC_CAL_CFG1					    	0x07
#define RTC_CFG2						        0x09

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
//extern TimeStructDef RTC_Test_Time;
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

uint8_t RTC_Init(void);
uint8_t RTC_Register_Read(uint8_t addr);
void RTC_Register_Write(uint8_t addr,uint8_t data);
uint8_t RTC_Time_Read(void);
uint8_t RTC_Time_Write(void);
#endif 

/*****END OF FILE****/
