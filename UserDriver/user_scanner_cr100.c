/**
******************************************************************************
* @文件    user_scanner_cr100.C
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
Scanner_Uart_typ  ScannerCR100;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  缓冲区初始化
  * @param  
  * @retval NONE
  */
static void ScannerCR100_TxBuffer_CLR(void)
{
    //初始化串口状态
    ScannerCR100.TXD.BusyFlag = sRST;
    //配置缓冲区为空
    memset((void *)ScannerCR100.TXD.Buffer,0,Buffer_Length);
}

static void ScannerCR100_RxBuffer_CLR(void)
{
    //初始化串口状态
    ScannerCR100.RXD.BusyFlag = sRST;
    //配置缓冲区为空
    memset((void *)ScannerCR100.RXD.Buffer,0,Buffer_Length);
}

/**
  * @brief  启动发送
  * @param  
  * @retval NONE
  */
static void UART_TXS_Start(void)
{
  ScannerCR100.TXD.BusyFlag=sSET;
  HAL_UART_Transmit_IT(&ScannerCR100.UartHandle, (uint8_t *)ScannerCR100.TXD.Buffer, strlen((char *)ScannerCR100.TXD.Buffer));
}

/**
  * @brief  启动接收
  * @param  
  * @retval NONE
  */
void ScannerCR100_UART_RXS_Start(void)
{
    while(ScannerCR100.TXD.BusyFlag==sSET)//等待发送完成
    {
      vTaskDelay(5);
    }
    if(HAL_UART_Abort(&ScannerCR100.UartHandle)!= HAL_OK)//停止
    {
      User_Error_Handler(__FILE__,__LINE__,26102);
    }
    ScannerCR100.RXD.BusyFlag=sSET;
    ScannerCR100_RxBuffer_CLR();//初始化缓冲区
    ScannerCR100.RXD.BufferByte = 0x00;
    ScannerCR100.RXD.BufferIdx = 0;
    ScannerCR100.RXD.Received = 0;
    ScannerCR100.RXD.Time=0;
    if(HAL_UART_Receive_IT(&ScannerCR100.UartHandle, (uint8_t *)&ScannerCR100.RXD.BufferByte, 1)!= HAL_OK)//启动接收
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
void ScannerCR100_Init(void)
{
  ScannerCR100_TxBuffer_CLR();
  ScannerCR100_RxBuffer_CLR();
  
  ScannerCR100.UartHandle.Instance  = UART8;
  ScannerCR100.UartHandle.Init.BaudRate = 9600;
  ScannerCR100.UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  ScannerCR100.UartHandle.Init.StopBits = UART_STOPBITS_1;
  ScannerCR100.UartHandle.Init.Parity = UART_PARITY_NONE;
  ScannerCR100.UartHandle.Init.Mode = UART_MODE_TX_RX;
  ScannerCR100.UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  ScannerCR100.UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  ScannerCR100.UartHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  ScannerCR100.UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  
  if (HAL_UART_DeInit(&ScannerCR100.UartHandle) != HAL_OK)
  {
    User_Error_Handler(__FILE__,__LINE__,26102);
  }
  if (HAL_UART_Init(&ScannerCR100.UartHandle) != HAL_OK)
  {
    User_Error_Handler(__FILE__,__LINE__,26102);
  }
  
   /*启动接收*/
  ScannerCR100_UART_RXS_Start();//启动电源后接收到第一个字节
}

/**
  * @brief 串口中断接收回调
  * @param  
  * @retval NONE
  */
void ScannerCR100_RX_Callback(void)
{
  ScannerCR100.RXD.Received = 1;
  ScannerCR100.RXD.Buffer[ScannerCR100.RXD.BufferIdx] = ScannerCR100.RXD.BufferByte;
  ScannerCR100.RXD.BufferIdx++;
  if(HAL_UART_Receive_IT(&ScannerCR100.UartHandle, (uint8_t *)&ScannerCR100.RXD.BufferByte, 1)!= HAL_OK)//启动接收
  {
    User_Error_Handler(__FILE__,__LINE__,56102);
  }
}


/**
  * @brief 接收完成扫描
  * @param  
  * @retval NONE
  */
static Fuction_StatusDef ScannerCR100_RX_Scan_Process(void)
{
  while(ScannerCR100.RXD.Received == 0)//等待接收到数据
  {
    vTaskDelay(USART_SCAN_TICK);
    ScannerCR100.RXD.Time+=USART_SCAN_TICK;    
    if(ScannerCR100.RXD.Time>USART_TIMEOUT)//超时检测
    {
      ScannerCR100.RXD.BusyFlag=sRST;
      ScannerCR100_RxBuffer_CLR();//初始化缓冲区
      if(HAL_UART_Abort(&ScannerCR100.UartHandle)!= HAL_OK)//停止接收
      {
        User_Error_Handler(__FILE__,__LINE__,26102);
      }
      return F_ERROR;
    }
  }
  while(1)//等待数据全部接收完成
  {
    ScannerCR100.RXD.Received = 0;
    vTaskDelay(USART_SCAN_TICK);
    if(ScannerCR100.RXD.Received == 0)//数据已经接收完成
    {
      break;
    }
  }
  /*扫描缓冲区*/
  for(uint16_t counti=0;counti<Buffer_Length;counti++)
  {
    if((ScannerCR100.RXD.Buffer[0]==0x02)&&(ScannerCR100.RXD.Buffer[counti]==0x0d)&&(ScannerCR100.RXD.Buffer[counti+1]==0x0a))//数据/指令接收完成
    {
      ScannerCR100.RXD.BusyFlag=sRST;
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
Fuction_StatusDef ScannerCR100_RESET(void)
{
  char order[]="H";
  char rxorder[]="S";
  /*写TX缓存*/
  ScannerCR100_TxBuffer_CLR();
  ScannerCR100.TXD.Buffer[0]=0x02;
  strcat((char *)ScannerCR100.TXD.Buffer,order);
  ScannerCR100.TXD.Buffer[strlen(order)+1]=0x0D;
  ScannerCR100.TXD.Buffer[strlen(order)+2]=0x0A;
  
  /*启动接收*/
  ScannerCR100_UART_RXS_Start();

  /*启动发送*/
  UART_TXS_Start();
  
  /*等待2s*/
  vTaskDelay(2000);

  /*扫描接收字段*/
  {
  if(ScannerCR100_RX_Scan_Process() == F_ERROR)
    return F_ERROR;
  }

  /*扫描命令反馈字段*/
  if(memcmp((char*)&ScannerCR100.RXD.Buffer[1],rxorder,strlen(rxorder))==0)
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
Fuction_StatusDef ScannerCR100_DEFALT(void)
{
  char order[]="PC20";
  char rxorder[]="PS0";
  /*写TX缓存*/
  ScannerCR100_TxBuffer_CLR();
  ScannerCR100.TXD.Buffer[0]=0x02;
  strcat((char *)ScannerCR100.TXD.Buffer,order);
  ScannerCR100.TXD.Buffer[strlen(order)+1]=0x0D;
  ScannerCR100.TXD.Buffer[strlen(order)+2]=0x0A;
  
  /*启动接收*/
  ScannerCR100_UART_RXS_Start();

  /*启动发送*/
  UART_TXS_Start();
  
  /*扫描接收字段*/
  
  if(ScannerCR100_RX_Scan_Process() == F_ERROR)
  {
    return F_ERROR;
  }
    
  /*扫描命令反馈字段*/
  if(memcmp((char*)&ScannerCR100.RXD.Buffer[1],rxorder,strlen(rxorder))==0)
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
static void ScannerCR100_Stop_Scan(void)
{
  char order[]="-";
  /*写TX缓存*/
  ScannerCR100_TxBuffer_CLR();
  ScannerCR100.TXD.Buffer[0]=0x02;
  strcat((char *)ScannerCR100.TXD.Buffer,order);
  ScannerCR100.TXD.Buffer[strlen(order)+1]=0x0D;
  ScannerCR100.TXD.Buffer[strlen(order)+2]=0x0A;
  
  /*启动接收*/
  ScannerCR100_UART_RXS_Start();

  /*启动发送*/
  UART_TXS_Start();
}

/**
  * @brief 发送启动扫描
  * @param  
  * @retval NONE
  */
Fuction_StatusDef ScannerCR100_Start_Scan(void)
{
  char order[]="+";
  /*写TX缓存*/
  ScannerCR100_TxBuffer_CLR();
  ScannerCR100.TXD.Buffer[0]=0x02;
  strcat((char *)ScannerCR100.TXD.Buffer,order);
  ScannerCR100.TXD.Buffer[strlen(order)+1]=0x0D;
  ScannerCR100.TXD.Buffer[strlen(order)+2]=0x0A;
  
  /*启动接收*/
  ScannerCR100_UART_RXS_Start();

  /*启动发送*/
  UART_TXS_Start();
  
  /*扫描接收字段*/
  if(ScannerCR100_RX_Scan_Process() == F_ERROR)
  {
    //停止扫描
    ScannerCR100_Stop_Scan();
    return F_ERROR;
  }
  return F_OK;
}

/**
  * @brief 扫描器初始化
  * @param  
  * @retval NONE
  */
Fuction_StatusDef ScannerCR100_INIT(void)
{
  ScannerCR100_Init();
  if(ScannerCR100_RESET()==F_ERROR)
  {
    return F_ERROR;
  }  
  if(ScannerCR100_DEFALT()==F_ERROR)
  {
    return F_ERROR;
  }
  return F_OK;
}

/*TEST*/
void ScannerCR100_TEST(void)
{

}

/*****************************END OF FILE*****************************/
