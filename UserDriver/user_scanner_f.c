/**
  ******************************************************************************
  * @file    bsp_Uart5_USART.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   使用串口1，重定向c库printf函数到usart端口，中断接收模式
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 H743 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */

#include "user_scanner_f.h"
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

Scanner_Uart_typ  ScannerF;


extern uint8_t ucTemp;
/**
  * @brief  USART5 GPIO ÅäÖÃ,¹¤×÷Ä£Ê½ÅäÖÃ¡£115200 8-N-1
  * @param  ÎÞ
  * @retval ÎÞ
  */
static void USART5_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

  USART5_RX_GPIO_CLK_ENABLE();
  USART5_TX_GPIO_CLK_ENABLE();

  RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UARTx;
  RCC_PeriphClkInit.Usart234578ClockSelection = RCC_UARTxCLKSOURCE_SYSCLK;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

  USART5_CLK_ENABLE();

  /**USART2 GPIO Configuration    
    PB13    ------> UART5_TX
    PB12    ------> UART5_RX 
    */

  GPIO_InitStruct.Pin = USART5_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = USART5_TX_AF;
  HAL_GPIO_Init(USART5_TX_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USART5_RX_PIN;
  GPIO_InitStruct.Alternate = USART5_RX_AF;
  HAL_GPIO_Init(USART5_RX_GPIO_PORT, &GPIO_InitStruct);

 


}


/**
  * @brief  缓冲区初始化
  * @param  
  * @retval NONE
  */
static void ScannerF_TxBuffer_CLR(void)
{
    //初始化串口状态
    ScannerF.TXD.BusyFlag = sRST;
    //配置缓冲区为空
    memset((void *)ScannerF.TXD.Buffer,0,Buffer_Length);
}

static void ScannerF_RxBuffer_CLR(void)
{
    //初始化串口状态
    ScannerF.RXD.BusyFlag = sRST;
    //配置缓冲区为空
    memset((void *)ScannerF.RXD.Buffer,0,Buffer_Length);
}

/**
  * @brief  启动发送
  * @param  
  * @retval NONE
  */
static void UART_TXS_Start(void)
{
  ScannerF.TXD.BusyFlag=sSET;
  HAL_UART_Transmit_IT(&ScannerF.UartHandle, (uint8_t *)ScannerF.TXD.Buffer, strlen((char *)ScannerF.TXD.Buffer));
}

/**
  * @brief  启动接收
  * @param  
  * @retval NONE
  */
void ScannerF_UART_RXS_Start(void)
{
    while(ScannerF.TXD.BusyFlag==sSET)//等待发送完成
    {
      vTaskDelay(5);
    }
    if(HAL_UART_Abort(&ScannerF.UartHandle)!= HAL_OK)//停止
    {
      User_Error_Handler(__FILE__,__LINE__,26102);
    }
    ScannerF.RXD.BusyFlag=sSET;
    ScannerF_RxBuffer_CLR();//初始化缓冲区
    ScannerF.RXD.BufferByte = 0x00;
    ScannerF.RXD.BufferIdx = 0;
    ScannerF.RXD.Received = 0;
    ScannerF.RXD.Time=0;
    if(HAL_UART_Receive_IT(&ScannerF.UartHandle, (uint8_t *)&ScannerF.RXD.BufferByte, 1)!= HAL_OK)//启动接收
    {
      User_Error_Handler(__FILE__,__LINE__,26102);
    }
}

/**
  * @brief  Configure the UART peripheral ,串口8初始化
  * @param  Put the USART peripheral in the Asynchronous mode (UART Mode) 
                   UART configured as follows:
                   - Word Length = 8 Bits
                   - Stop Bit = One Stop bit
                   - Parity = None
                   - BaudRate = 9600 baud
                   - Hardware flow control disabled (RTS and CTS signals) 
  * @retval 
  */
void ScannerF_Init(void)
{
  //USART5_Config();
  ScannerF_TxBuffer_CLR();
  ScannerF_RxBuffer_CLR();
  
  ScannerF.UartHandle.Instance  = UART5;
  ScannerF.UartHandle.Init.BaudRate = 9600;
  ScannerF.UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  ScannerF.UartHandle.Init.StopBits = UART_STOPBITS_1;
  ScannerF.UartHandle.Init.Parity = UART_PARITY_NONE;
  ScannerF.UartHandle.Init.Mode = UART_MODE_TX_RX;
  ScannerF.UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  ScannerF.UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  ScannerF.UartHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  ScannerF.UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  
  if (HAL_UART_DeInit(&ScannerF.UartHandle) != HAL_OK)
  {
    User_Error_Handler(__FILE__,__LINE__,26102);
  }
  if (HAL_UART_Init(&ScannerF.UartHandle) != HAL_OK)
  {
    User_Error_Handler(__FILE__,__LINE__,26102);
  }  

  //HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
  //HAL_NVIC_EnableIRQ(UART5_IRQn);
   /*启动接收*/
  ScannerF_UART_RXS_Start();//启动电源后接收到第一个字节
}

/**
  * @brief 串口中断接收回调
  * @param  
  * @retval NONE
  */
void ScannerF_RX_Callback(void)
{
  ScannerF.RXD.Received = 1;
  ScannerF.RXD.Buffer[ScannerF.RXD.BufferIdx] = ScannerF.RXD.BufferByte;
  ScannerF.RXD.BufferIdx++;
  if(HAL_UART_Receive_IT(&ScannerF.UartHandle, (uint8_t *)&ScannerF.RXD.BufferByte, 1)!= HAL_OK)//启动接收
  {
    User_Error_Handler(__FILE__,__LINE__,56102);
  }
}

void UART5_IRQHandler()
{
    HAL_UART_IRQHandler(&ScannerF.UartHandle);	
}


/**
  * @brief 接收完成扫描
  * @param  
  * @retval NONE
  */
static Fuction_StatusDef ScannerF_RX_Scan_Process(void)
{
  while(ScannerF.RXD.Received == 0)//等待接收到数据
  {
    vTaskDelay(USART_SCAN_TICK);
    ScannerF.RXD.Time+=USART_SCAN_TICK;    
    if(ScannerF.RXD.Time>USART_TIMEOUT)//超时检测
    {
      ScannerF.RXD.BusyFlag=sRST;
      ScannerF_RxBuffer_CLR();//初始化缓冲区
      if(HAL_UART_Abort(&ScannerF.UartHandle)!= HAL_OK)//停止接收
      {
        User_Error_Handler(__FILE__,__LINE__,26102);
      }
      return F_ERROR;
    }
  }
  while(1)//等待数据全部接收完成
  {
    ScannerF.RXD.Received = 0;
    vTaskDelay(USART_SCAN_TICK);
    if(ScannerF.RXD.Received == 0)//数据已经接收完成
    {
      break;
    }
  }
  /*扫描缓冲区*/
  for(uint16_t counti=0;counti<Buffer_Length;counti++)
  {
    if((ScannerF.RXD.Buffer[0]==0x02)&&(ScannerF.RXD.Buffer[counti]==0x0d)&&(ScannerF.RXD.Buffer[counti+1]==0x0a))//数据/指令接收完成
    {
      ScannerF.RXD.BusyFlag=sRST;
      return F_OK;
    }
  }
  return F_ERROR;
}

/**
  * @brief 发送软复位
  * @param  
  * @retval NONE
  */
Fuction_StatusDef ScannerF_RESET(void)
{
  char order[]="H";
  char rxorder[]="S";
  /*写TX缓存*/
  ScannerF_TxBuffer_CLR();
  ScannerF.TXD.Buffer[0]=0x02;
  strcat((char *)ScannerF.TXD.Buffer,order);
  ScannerF.TXD.Buffer[strlen(order)+1]=0x0D;
  ScannerF.TXD.Buffer[strlen(order)+2]=0x0A;

  /*启动接收*/
  ScannerF_UART_RXS_Start();

  /*启动发送*/
  UART_TXS_Start();
  
  /*等待2s*/
  vTaskDelay(2000);

  /*扫描接收字段*/
  {
  if(ScannerF_RX_Scan_Process() == F_ERROR)
    return F_ERROR;
  }

  /*扫描命令反馈字段*/
  if(memcmp((char*)&ScannerF.RXD.Buffer[1],rxorder,strlen(rxorder))==0)
  {
    return F_OK;
  }
  else
  {
    return F_ERROR;
  } 
}

/**
  * @brief 发送恢复出厂设置
  * @param  
  * @retval NONE
  */
Fuction_StatusDef ScannerF_DEFALT(void)
{
  char order[]="PC20";
  char rxorder[]="PS0";
  /*写TX缓存*/
  ScannerF_TxBuffer_CLR();
  ScannerF.TXD.Buffer[0]=0x02;
  strcat((char *)ScannerF.TXD.Buffer,order);
  ScannerF.TXD.Buffer[strlen(order)+1]=0x0D;
  ScannerF.TXD.Buffer[strlen(order)+2]=0x0A;
  
  /*启动接收*/
  ScannerF_UART_RXS_Start();

  /*启动发送*/
  UART_TXS_Start();
  
  /*扫描接收字段*/
  {
  if(ScannerF_RX_Scan_Process() == F_ERROR)
    return F_ERROR;
  }

  /*扫描命令反馈字段*/
  if(memcmp((char*)&ScannerF.RXD.Buffer[1],rxorder,strlen(rxorder))==0)
  {
    return F_OK;
  }
  else
  {
    return F_ERROR;
  } 
}

/**
  * @brief 发送停止扫描
  * @param  
  * @retval NONE
  */
static void ScannerF_Stop_Scan(void)
{
  char order[]="-";
  /*写TX缓存*/
  ScannerF_TxBuffer_CLR();
  ScannerF.TXD.Buffer[0]=0x02;
  strcat((char *)ScannerF.TXD.Buffer,order);
  ScannerF.TXD.Buffer[strlen(order)+1]=0x0D;
  ScannerF.TXD.Buffer[strlen(order)+2]=0x0A;
  
  /*启动接收*/
  ScannerF_UART_RXS_Start();

  /*启动发送*/
  UART_TXS_Start();
}

/**
  * @brief 发送启动扫描
  * @param  
  * @retval NONE
  */
Fuction_StatusDef ScannerF_Start_Scan(void)
{
  char order[]="+";
  /*写TX缓存*/
  ScannerF_TxBuffer_CLR();
  ScannerF.TXD.Buffer[0]=0x02;
  strcat((char *)ScannerF.TXD.Buffer,order);
  ScannerF.TXD.Buffer[strlen(order)+1]=0x0D;
  ScannerF.TXD.Buffer[strlen(order)+2]=0x0A;
  
  /*启动接收*/
  ScannerF_UART_RXS_Start();

  /*启动发送*/
  UART_TXS_Start();
  
  /*扫描接收字段*/
  if(ScannerF_RX_Scan_Process() == F_ERROR)
  {
    //停止扫描
    ScannerF_Stop_Scan();
    return F_ERROR;
  }
  return F_OK;
}

/**
  * @brief 扫描器初始化
  * @param  
  * @retval NONE
  */
Fuction_StatusDef ScannerF_INIT(void)
{
  ScannerF_Init();
  if(ScannerF_RESET()==F_ERROR)
  {
    return F_ERROR;
  }  
  if(ScannerF_DEFALT()==F_ERROR)
  {
    return F_ERROR;
  }
  return F_OK;
}

/*TEST*/
void ScannerF_TEST(void)
{

}
