/**
******************************************************************************
* @文件    scanner.H
* @作者    
* @版本    V0.0.1
******************************************************************************

******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USRE_SCANNER_H
#define __USRE_SCANNER_H

#ifdef __cplusplus
extern "C" {
#endif
    
/* Includes ------------------------------------------------------------------*/
#include "user_gpm.h"
/*define-----------------------------------------------------------------------*/

#define Buffer_Length             50//数据帧长度
#define USART_SCAN_TICK           10//串口接收扫描延时
#define USART_TIMEOUT             1500//超时延迟

/* Private types ------------------------------------------------------------*/
typedef struct
{
    __IO Status_TypeDef BusyFlag;//忙标志：发送前置位，发送结束自动复位；接收开始置位，重启接收时复位
    __IO uint8_t Buffer[Buffer_Length];//缓冲区
    __IO uint32_t Time;//占用时间
}Scanner_TXBuff_typ;

typedef struct
{
    __IO Status_TypeDef BusyFlag;//忙标志：发送前置位，发送结束自动复位；接收开始置位，重启接收时复位
    __IO uint8_t Buffer[Buffer_Length];//缓冲区
    __IO uint8_t BufferByte;
    __IO uint8_t BufferIdx;
    __IO uint8_t Received;
    __IO uint32_t Time;//占用时间
}Scanner_RXBuff_typ;

/* Exported types ------------------------------------------------------------*/
typedef struct 
{
    UART_HandleTypeDef UartHandle;
    Scanner_TXBuff_typ TXD;
    Scanner_RXBuff_typ RXD;
}Scanner_Uart_typ;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
extern Scanner_Uart_typ  ScannerF;
extern Scanner_Uart_typ  ScannerS;
extern Scanner_Uart_typ  ScannerCR100;
/* Exported functions ------------------------------------------------------- */

/* Private functions ---------------------------------------------------------*/
void ScannerF_UART_RXS_Start(void);

void ScannerF_Init(void);

Fuction_StatusDef ScannerF_DEFALT(void);

Fuction_StatusDef ScannerF_232(void);

Fuction_StatusDef ScannerF_Start_Scan(void);

void ScannerF_Stop_Scan(void);

Fuction_StatusDef ScannerF_Reset(void);

void ScannerF_RX_Callback(void);


void ScannerS_UART_RXS_Start(void);

void ScannerS_Init(void);

Fuction_StatusDef ScannerS_DEFALT(void);

Fuction_StatusDef ScannerS_232(void);

Fuction_StatusDef ScannerS_Start_Scan(void);

void ScannerS_Stop_Scan(void);

Fuction_StatusDef ScannerS_Reset(void);

void ScannerS_RX_Callback(void);


void ScannerCR100_UART_RXS_Start(void);

void ScannerCR100_Init(void);

Fuction_StatusDef ScannerCR100_RESET(void);

Fuction_StatusDef ScannerCR100_DEFALT(void);

Fuction_StatusDef ScannerCR100_Start_Scan(void);

Fuction_StatusDef ScannerCR100_INIT(void);

void ScannerCR100_RX_Callback(void);

#endif 
/*****************************END OF FILE*****************************/
