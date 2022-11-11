#ifndef _HW_CONFIG_H_
#define _HW_CONFIG_H_

///< include files

///< stdlib && 3th lib

/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"

/* 标准库头文件 */
#include <string.h>
#include <stdio.h>

///< stm32h743
#include "stm32h7xx.h"
#include "stm32h743i_eval_nor.h"
#include "stm32h743i_eval_sdram.h"

///< hard drivers && device
#include "user_gpm.h"
#include "user_can.h"
#include "user_ethernet.h"
#include "user_scanner_f.h"

///< app
#include "ethernetif.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "app_ethernet.h"
#include "main.h"


#define Uart5_USART                             UART5
#define Uart5_USART_CLK_ENABLE()                __UART5_CLK_ENABLE();

#define Uart5_USART_RX_GPIO_PORT                GPIOB
#define Uart5_USART_RX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define Uart5_USART_RX_PIN                      GPIO_PIN_12
#define Uart5_USART_RX_AF                       GPIO_AF14_UART5


#define Uart5_USART_TX_GPIO_PORT                GPIOB
#define Uart5_USART_TX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define Uart5_USART_TX_PIN                      GPIO_PIN_13
#define Uart5_USART_TX_AF                       GPIO_AF14_UART5

#define Uart5_USART_IRQHandler                  UART5_IRQHandler
#define Uart5_USART_IRQ                 		UART5_IRQn

#define Uart5_USART_DMA_CLK_ENABLE()      		__DMA1_CLK_ENABLE()	
#define Uart5_USART_DMA_REQUEST                 DMA_REQUEST_UART5_TX
#define Uart5_USART_DMA_STREAM            		DMA1_Stream0

#define Uart5_USART_BAUDRATE                    115200

#define Uart5_USART_RBUFF_SIZE                  1000

//#define  USART_RBUFF_SIZE            1000 

int16_t bsp_init(void);



#endif



