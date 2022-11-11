/**
******************************************************************************
* @文件    user_gpm.H
* @作者    耿旭
* @版本    V0.0.1
******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_GPM_H
#define __USER_GPM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
  
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* Private types ------------------------------------------------------------*/
/**
 * @brief Status Types Definition
 */
typedef enum
{
    sRST=0,
    sSET=1,
}Status_TypeDef;

typedef enum
{
    F_OK=0,
    F_ERROR=1,
}Fuction_StatusDef;


/* Exported types ------------------------------------------------------------*/
typedef enum
{
  UserLED1 = 0U,
  UserLED2 = 1U,
  UserLED3 = 2U,
  UserLED4 = 3U,
  UserLED5 = 4U,
  UserLEDn
}UserLed_TypeDef;

typedef struct
{
  uint8_t year;//年，取值0~200表示2000~2200年
  uint8_t month;//月，取值1~12
  uint8_t day;//日，取值1~31
  uint8_t hour;//小时，取值0~23，24小时制
  uint8_t minute;//分钟，取值0~59
  uint8_t second;//秒，取值0~59
  uint8_t decisecond;//十分之一秒，取值0~9
//  uint8_t millisecond;//千分之一秒，取值0~99
}TimeStructDef;
/* Exported constants --------------------------------------------------------*/
#define BYTECOUNTOF(__BUFFER__)        (sizeof(__BUFFER__) / sizeof(uint8_t))
#define UNITCOUNTOF(__BUFFER__)        (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported macro ------------------------------------------------------------*/
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t


#define   UserSetBit(x,y)     x|=(1<<y) //将X的第Y位置1
#define   UserClrBit(x,y)     x&=~(1<<y) //将X的第Y位清0
#define   UserReadBit(x,y)    (x&(1<<y))>>y//读取X的第Y位

#define  DWT_CR                 *(volatile uint32_t *)0xE0001000
#define  DWT_CYCCNT             *(volatile uint32_t *)0xE0001004
#define  DWT_LAR                *(volatile uint32_t *)0xE0001FB0
#define  DWT_LAR_UNLOCK          (uint32_t)0xC5ACCE55
#define  DEM_CR                 *(volatile uint32_t *)0xE000EDFC
#define  DEM_CR_TRCENA           (1 << 24)
#define  DWT_CR_CYCCNTENA        (1 <<  0)

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_14
#define USARTx_TX_GPIO_PORT              GPIOB
#define USARTx_TX_AF                     GPIO_AF4_USART1
#define USARTx_RX_PIN                    GPIO_PIN_15
#define USARTx_RX_GPIO_PORT              GPIOB
#define USARTx_RX_AF                     GPIO_AF4_USART1

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#define UserLED1_GPIO_PORT                   GPIOJ
#define UserLED1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOJ_CLK_ENABLE()
#define UserLED1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOJ_CLK_DISABLE()
#define UserLED1_PIN                         GPIO_PIN_0

#define UserLED2_GPIO_PORT                   GPIOJ
#define UserLED2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOJ_CLK_ENABLE()
#define UserLED2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOJ_CLK_DISABLE()
#define UserLED2_PIN                         GPIO_PIN_1

#define UserLED3_GPIO_PORT                   GPIOJ
#define UserLED3_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOJ_CLK_ENABLE()
#define UserLED3_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOJ_CLK_DISABLE()
#define UserLED3_PIN                         GPIO_PIN_2

#define UserLED4_GPIO_PORT                   GPIOJ
#define UserLED4_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOJ_CLK_ENABLE()
#define UserLED4_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOJ_CLK_DISABLE()
#define UserLED4_PIN                         GPIO_PIN_3

#define UserLED5_GPIO_PORT                   GPIOJ
#define UserLED5_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOJ_CLK_ENABLE()
#define UserLED5_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOJ_CLK_DISABLE()
#define UserLED5_PIN                         GPIO_PIN_4

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/* Exported variables ------------------------------------------------------- */
extern char LogBuffer[130];
/* Exported functions ------------------------------------------------------- */

void MCU_GPIO_CLK_ENABLE(void);

void MCU_EXIT_SET(void);

void DWT_Init(void);

void User_delay_us(uint32_t us);

//void User_Error_Handler(char *function ,int line);
void User_Error_Handler(char *function ,int line ,uint32_t erridx);

uint8_t IWDG_Init_Start(uint8_t prer,uint16_t rlr);

uint8_t IWDG_Feed(void);

void Bit_Set_U32t(uint32_t * data,uint8_t bit,uint8_t value);

uint8_t Bit_Get_U32t(uint32_t data,uint8_t bit);

void uint32_To_Array(const uint32_t data_32,uint8_t * data_8);

uint32_t Array_To_uint32(const uint8_t * data_8);

void float_to_uint8(const float data_f,uint8_t * data_8);

float uint8_to_float(const uint8_t * data_8);

uint8_t User_CRC_Init(void);

uint32_t CRC_Value_Computer(uint8_t * aDataBuffer,uint32_t BUFFER_SIZE);

uint8_t MPU_Set_Protection(uint32_t baseaddr,uint32_t size,uint32_t rnum,uint32_t ap);

void MPU_Memory_Protection(void);

uint8_t Uart1_Comm_Init(void);

void ULED_Init(UserLed_TypeDef Led);

void ULED_On(UserLed_TypeDef Led);

void ULED_Off(UserLed_TypeDef Led);

void ULED_Toggle(UserLed_TypeDef Led);

void ULED_INIT_ALL(void);

void UserButtonInit(void);

uint8_t UserButtonRead(void);

void UserSwitchInit(void);

uint8_t UserSwitchRead(void);

void UserSDRAM_Write_U8(uint32_t StartAddr,uint8_t * Buffer,uint32_t Size);

void UserSDRAM_Read_U8(uint32_t StartAddr,uint8_t * Buffer,uint32_t Size);

void TimeWrite(TimeStructDef *time,
                  uint8_t year,
                  uint8_t month,
                  uint8_t day,
                  uint8_t hour,
                  uint8_t minute,
                  uint8_t second,
                  uint8_t decisecond,
                  uint8_t millisecond);

void TimeAddDS(TimeStructDef *time);

uint32_t U32T_LimitAdd(const uint32_t x,const uint32_t y,uint32_t max);

void Debug_RX_Callback(void);

void Uart1DebugTask(void *pvParameters);

#endif /* __USER_GPM_H */
/*****************************END OF FILE*****************************/
