/**
******************************************************************************
* @文件    user_scanner_s.C
* @作者    
* @作者    
* @版本    V0.0.1
******************************************************************************
样本条码扫描器

******************************************************************************
*/


/* Includes ------------------------------------------------------------------*/
#include "user_scanner.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
Scanner_Uart_typ  ScannerS;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  缓冲区初始化
  * @param  
  * @retval NONE
  */
static void ScannerS_TxBuffer_CLR(void)
{
    //初始化串口状态
    ScannerS.TXD.BusyFlag = sRST;
    //配置缓冲区为空
    memset((void *)ScannerS.TXD.Buffer,0,Buffer_Length);
}

static void ScannerS_RxBuffer_CLR(void)
{
    //初始化串口状态
    ScannerS.RXD.BusyFlag = sRST;
    //配置缓冲区为空
    memset((void *)ScannerS.RXD.Buffer,0,Buffer_Length);
}

/**
  * @brief  启动发送
  * @param  
  * @retval NONE
  */
static void UART_TXS_Start(void)
{
  ScannerS.TXD.BusyFlag=sSET;
  HAL_UART_Transmit_IT(&ScannerS.UartHandle, (uint8_t *)ScannerS.TXD.Buffer, strlen((char *)ScannerS.TXD.Buffer));
}

/**
  * @brief  启动接收
  * @param  
  * @retval NONE
  */
void ScannerS_UART_RXS_Start(void)
{
    while(ScannerS.TXD.BusyFlag==sSET)//等待发送完成
    {
      vTaskDelay(5);
    }
    if(HAL_UART_Abort(&ScannerS.UartHandle)!= HAL_OK)//停止
    {
      User_Error_Handler(__FILE__,__LINE__,26104);
    }
    ScannerS.RXD.BusyFlag=sSET;
    ScannerS_RxBuffer_CLR();//初始化缓冲区
    ScannerS.RXD.BufferByte = 0x00;
    ScannerS.RXD.BufferIdx = 0;
    ScannerS.RXD.Received = 0;
    ScannerS.RXD.Time=0;
    if(HAL_UART_Receive_IT(&ScannerS.UartHandle, (uint8_t *)&ScannerS.RXD.BufferByte, 1)!= HAL_OK)//启动接收
    {
      User_Error_Handler(__FILE__,__LINE__,26104);
    }
}

/**
  * @brief  Configure the UART peripheral ,串口1初始化
  * @param  Put the USART peripheral in the Asynchronous mode (UART Mode) 
                   UART configured as follows:
                   - Word Length = 8 Bits
                   - Stop Bit = One Stop bit
                   - Parity = None
                   - BaudRate = 115200 baud
                   - Hardware flow control disabled (RTS and CTS signals) 
  * @retval 
  */
void ScannerS_Init(void)
{
  ScannerS_TxBuffer_CLR();
  ScannerS_RxBuffer_CLR();
  
  ScannerS.UartHandle.Instance  = UART8;
  ScannerS.UartHandle.Init.BaudRate = 115200;
  ScannerS.UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  ScannerS.UartHandle.Init.StopBits = UART_STOPBITS_1;
  ScannerS.UartHandle.Init.Parity = UART_PARITY_NONE;
  ScannerS.UartHandle.Init.Mode = UART_MODE_TX_RX;
  ScannerS.UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  ScannerS.UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  ScannerS.UartHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  ScannerS.UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  
  if (HAL_UART_DeInit(&ScannerS.UartHandle) != HAL_OK)
  {
    User_Error_Handler(__FILE__,__LINE__,26104);
  }
  if (HAL_UART_Init(&ScannerS.UartHandle) != HAL_OK)
  {
    User_Error_Handler(__FILE__,__LINE__,26104);
  }
  
   /*启动接收*/
  ScannerS_UART_RXS_Start();//启动电源后接收到第一个字节
}

/**
  * @brief 串口中断接收回调
  * @param  
  * @retval NONE
  */
void ScannerS_RX_Callback(void)
{
  ScannerS.RXD.Received = 1;
  ScannerS.RXD.Buffer[ScannerS.RXD.BufferIdx] = ScannerS.RXD.BufferByte;
  ScannerS.RXD.BufferIdx++;
  if(HAL_UART_Receive_IT(&ScannerS.UartHandle, (uint8_t *)&ScannerS.RXD.BufferByte, 1)!= HAL_OK)//启动接收
  {
    User_Error_Handler(__FILE__,__LINE__,56102);
  }
}


/**
  * @brief 接收完成扫描
  * @param  
  * @retval NONE
  */
static Fuction_StatusDef ScannerS_RX_Scan_Process(void)
{
  while(ScannerS.RXD.Received == 0)//等待接收到数据
  {
    vTaskDelay(USART_SCAN_TICK);
    ScannerS.RXD.Time+=USART_SCAN_TICK;    
    if(ScannerS.RXD.Time>USART_TIMEOUT)//超时检测
    {
      ScannerS.RXD.BusyFlag=sRST;
      ScannerS_RxBuffer_CLR();//初始化缓冲区
      if(HAL_UART_Abort(&ScannerS.UartHandle)!= HAL_OK)//停止接收
      {
        User_Error_Handler(__FILE__,__LINE__,26104);
      }
      return F_ERROR;
    }
  }
  while(1)//等待数据全部接收完成
  {
    ScannerS.RXD.Received = 0;
    vTaskDelay(USART_SCAN_TICK);
    if(ScannerS.RXD.Received == 0)//数据已经接收完成
    {
      break;
    }
  }
  /*扫描缓冲区*/
  for(uint16_t counti=0;counti<Buffer_Length;counti++)
  {
    if(((ScannerS.RXD.Buffer[counti]==0x06)||(ScannerS.RXD.Buffer[counti]==0x05))
          &&(ScannerS.RXD.Buffer[counti+1]==0x2e))//指令接收完成
    {
      ScannerS.RXD.BusyFlag=sRST;
      return F_OK;
    }
    else if((ScannerS.RXD.Buffer[counti]==0x0d)&&(ScannerS.RXD.Buffer[counti+1]==0x0a))//数据接收完成
    {
      ScannerS.RXD.BusyFlag=sRST;
      return F_OK;
    }
  }
  return F_ERROR;
}

/**
  * @brief 发送恢复出厂设置/用户设置
  * @param  
  * @retval NONE
  */
Fuction_StatusDef ScannerS_DEFALT(void)
{
  char order[]="DEFALT.";
  /*启动接收*/
  ScannerS_UART_RXS_Start();
  
  /*写TX缓存*/
  ScannerS_TxBuffer_CLR();
  ScannerS.TXD.Buffer[0]=0X16;
  ScannerS.TXD.Buffer[1]=0X4D;
  ScannerS.TXD.Buffer[2]=0X0D;
  
  strcat((char *)ScannerS.TXD.Buffer,order);
  
  /*启动发送*/
  UART_TXS_Start();
  
  /*扫描接收字段*/
  return ScannerS_RX_Scan_Process();
}

/**
  * @brief 232控制模式
扫描引擎编程为以115,200波特，无奇偶校验，8位数据位，1位停止位的RS-232接口，并添加一个CR LF的后缀。
  * @param  
  * @retval NONE
  */
Fuction_StatusDef ScannerS_232(void)
{
  char order[]="PAP232.";
  /*启动接收*/
  ScannerS_UART_RXS_Start();
  
  /*写TX缓存*/
  ScannerS_TxBuffer_CLR();
  ScannerS.TXD.Buffer[0]=0X16;
  ScannerS.TXD.Buffer[1]=0X4D;
  ScannerS.TXD.Buffer[2]=0X0D;
  
  strcat((char *)ScannerS.TXD.Buffer,order);
  
  /*启动发送*/
  UART_TXS_Start();
  
  /*扫描接收字段*/
  return ScannerS_RX_Scan_Process();
}

/**
  * @brief 发送启动扫描
  * @param  
  * @retval NONE
  */
Fuction_StatusDef ScannerS_Start_Scan(void)
{
  /*启动接收*/
  ScannerS_UART_RXS_Start();
  
  /*写TX缓存*/
  ScannerS_TxBuffer_CLR();
  ScannerS.TXD.Buffer[0]=0X16;
  ScannerS.TXD.Buffer[1]=0X54;
  ScannerS.TXD.Buffer[2]=0X0D;
  
  /*启动发送*/
  UART_TXS_Start();
  
  /*扫描接收字段*/
  return ScannerS_RX_Scan_Process();
}

/**
  * @brief 发送停止扫描
  * @param  
  * @retval NONE
  */
void ScannerS_Stop_Scan(void)
{
  /*启动接收*/
  ScannerS_UART_RXS_Start();
  
  /*写TX缓存*/
  ScannerS_TxBuffer_CLR();
  ScannerS.TXD.Buffer[0]=0X16;
  ScannerS.TXD.Buffer[1]=0X55;
  ScannerS.TXD.Buffer[2]=0X0D;
  
  /*启动发送*/
  UART_TXS_Start();

}

/**
  * @brief 扫描器重置
  * @param  
  * @retval NONE
  */
Fuction_StatusDef ScannerS_Reset(void)
{
  ScannerS_Init();
  if(ScannerS_DEFALT()==F_ERROR)
  {
    return F_ERROR;
  }

  if(ScannerS_232()==F_ERROR)
  {
    return F_ERROR;
  }
  return F_OK;
}

/*TEST*/
void ScannerS_TEST(void)
{

}

/*****************************END OF FILE*****************************/
