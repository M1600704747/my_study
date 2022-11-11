#ifndef __Uart5_USART_H
#define __Uart5_USART_H

#include "hwconfig.h"
#include "user_scanner.h"

//
/*******************************************************/
#define USART5 UART5
#define USART5_CLK_ENABLE() __UART5_CLK_ENABLE();

#define RCC_PERIPHCLK_UARTx RCC_PERIPHCLK_UART5
#define RCC_UARTxCLKSOURCE_SYSCLK RCC_USART234578CLKSOURCE_D2PCLK1

#define USART5_RX_GPIO_PORT GPIOB
#define USART5_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define USART5_RX_PIN GPIO_PIN_12
#define USART5_RX_AF GPIO_AF14_UART5

#define USART5_TX_GPIO_PORT GPIOB
#define USART5_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define USART5_TX_PIN GPIO_PIN_13
#define USART5_TX_AF GPIO_AF14_UART5

#define USART5_IRQHandler UART5_IRQHandler
#define USART5_IRQ UART5_IRQn
/************************************************************/
extern Scanner_Uart_typ ScannerF;

//
#define USART5_BAUDRATE 9600

void ScannerF_UART_RXS_Start(void);

void ScannerF_Init(void);

Fuction_StatusDef ScannerF_DEFALT(void);

Fuction_StatusDef ScannerF_232(void);
Fuction_StatusDef ScannerF_INIT(void);

Fuction_StatusDef ScannerF_Start_Scan(void);

void ScannerF_Stop_Scan(void);

Fuction_StatusDef ScannerF_Reset(void);

void ScannerF_RX_Callback(void);

#endif /* __UART5_H */
