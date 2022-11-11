/**
******************************************************************************
* @文件    user_can.C
* @作者    GENGXU
* @作者    耿旭
* @版本    V0.0.1
******************************************************************************





******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "user_can.h"
#include "../SystemInfo/user_rtos_api.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan;
FDCAN_FilterTypeDef sFilterConfig;

uint32_t CanFilterID1 = 0;
uint32_t CanFilterID2 = 0;
uint32_t CanBrt = 1000; //uint32_t CanBrt=500;//默认500K，取值100，500，800，1000，通过跳线选择

uint32_t CanError = 0; //Can的错误状态标志
                       //bitx:0--无错误，1--发生错误
                       //bit0--接收中断发生错误

xSemaphoreHandle FDCAN_Mutex = NULL; //Can发送控制信号量，获取权限后使用。

FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
QueueHandle_t CanRxReady; //接收状态队列，单个队列，长度1字节，接收数据后将0x0f写入。
uint8_t CanRxData[8];
uint8_t CanTxData[8];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the FDCAN MSP.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @retval None
  */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  FDCANx_TX_GPIO_CLK_ENABLE();
  FDCANx_RX_GPIO_CLK_ENABLE();

  /* Select PLL1Q as source of FDCANx clock */
  RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  RCC_PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

  /* Enable FDCANx clock */
  FDCANx_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* FDCANx TX GPIO pin configuration  */
  GPIO_InitStruct.Pin = FDCANx_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = FDCANx_TX_AF;
  HAL_GPIO_Init(FDCANx_TX_GPIO_PORT, &GPIO_InitStruct);

  /* FDCANx RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = FDCANx_RX_PIN;
  GPIO_InitStruct.Alternate = FDCANx_RX_AF;
  HAL_GPIO_Init(FDCANx_RX_GPIO_PORT, &GPIO_InitStruct);

  /*##-3- Configure the NVIC #################################################*/
  /* NVIC for FDCANx */
  HAL_NVIC_SetPriority(FDCANx_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(FDCANx_IRQn);
}

/**
  * @brief  DeInitializes the FDCAN MSP.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @retval None
  */
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
  /*##-1- Reset peripherals ##################################################*/
  FDCANx_FORCE_RESET();
  FDCANx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* Configure FDCANx Tx as alternate function  */
  HAL_GPIO_DeInit(FDCANx_TX_GPIO_PORT, FDCANx_TX_PIN);

  /* Configure FDCANx Rx as alternate function  */
  HAL_GPIO_DeInit(FDCANx_RX_GPIO_PORT, FDCANx_RX_PIN);

  /*##-3- Disable the NVIC for FDCANx ########################################*/
  HAL_NVIC_DisableIRQ(FDCANx_IRQn);
}

/**
  * @brief  初始化FDCAN1
FDCAN使用PLL1Q，频率200MHz
  * @param  
  * @retval 0--ok;1--fail
  */
uint8_t FDCAN1_Moudle_Init(void)
{
  HAL_FDCAN_DeInit(&hfdcan); //先清除以前的设置

  hfdcan.Instance = FDCANx;
  hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC; //传统模式
  hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan.Init.AutoRetransmission = DISABLE; //关闭自动重传，传统模式下一定要关闭
  hfdcan.Init.TransmitPause = DISABLE;      //关闭传输暂停
  hfdcan.Init.ProtocolException = DISABLE;  //关闭协议异常处理

  hfdcan.Init.NominalPrescaler = 10;    /* tq = NominalPrescaler x (1/fdcan_ker_ck) */
                                        /*在分频值位12时，tq=1/20MHz*/
  hfdcan.Init.NominalSyncJumpWidth = 8; /*!< Specifies the maximum number of time quanta the FDCAN
                                            hardware is allowed to lengthen or shorten a bit to perform
                                            resynchronization.
                                            This parameter must be a number between 1 and 128 */

  if (CanBrt == 100)
  {
    hfdcan.Init.NominalTimeSeg1 = 174; /* NominalTimeSeg1 = Propagation_segment + Phase_segment_1 
                                         < Specifies the number of time quanta in Bit Segment 1.
                                           This parameter must be a number between 2 and 256 */
    hfdcan.Init.NominalTimeSeg2 = 25;  /*!< Specifies the number of time quanta in Bit Segment 2.
                                           This parameter must be a number between 2 and 128 */
  }
  else if (CanBrt == 500)
  {
    hfdcan.Init.NominalTimeSeg1 = 31;
    hfdcan.Init.NominalTimeSeg2 = 8;
  }
  else if (CanBrt == 800)
  {
    hfdcan.Init.NominalTimeSeg1 = 18;
    hfdcan.Init.NominalTimeSeg2 = 6;
  }
  else if (CanBrt == 1000)
  {
    hfdcan.Init.NominalTimeSeg1 = 14;
    hfdcan.Init.NominalTimeSeg2 = 5;
  }
  else
  {
    return 1;
  }

  /*ID设置，在此之前需要进行初始化*/
  CanFilterID2 = CanFilterID1;

  hfdcan.Init.MessageRAMOffset = 0;
  hfdcan.Init.StdFiltersNbr = 1;                    //标准信息ID滤波器编号
  hfdcan.Init.ExtFiltersNbr = 0;                    //扩展信息ID滤波器编号
  hfdcan.Init.RxFifo0ElmtsNbr = 1;                  //接收FIFO0元素编号
  hfdcan.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8; //接收FIFO0元素大小：8字节
  hfdcan.Init.RxFifo1ElmtsNbr = 0;
  hfdcan.Init.RxBuffersNbr = 0;                          //接收缓冲编号
  hfdcan.Init.TxEventsNbr = 0;                           //发送事件编号
  hfdcan.Init.TxBuffersNbr = 0;                          //发送缓冲编号
  hfdcan.Init.TxFifoQueueElmtsNbr = 10;                  //发送FIFO序列元素编号
  hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION; //发送FIFO序列模式
  hfdcan.Init.TxElmtSize = FDCAN_DATA_BYTES_8;           //发送大小:8字节
  if (HAL_FDCAN_Init(&hfdcan) != HAL_OK)
    return 1; //初始化FDCAN

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID; //标准ID
  sFilterConfig.FilterIndex = 0;            //滤波器索引
                                            //sFilterConfig.FilterType = FDCAN_FILTER_DUAL;//滤波器类型
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; //过滤器0关联到FIFO0
  sFilterConfig.FilterID1 = CanFilterID1;               //32位ID
  sFilterConfig.FilterID2 = CanFilterID2;               //
  if (HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig) != HAL_OK)
    return 1;

  /* Configure global filter to reject all non-matching frames */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan) != HAL_OK)
    return 1;
  if (HAL_FDCAN_ActivateNotification(&hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    return 1;

  /*初始化缓冲区、初始化错误状态*/
  memset(CanRxData, 0, sizeof(uint8_t) * 8);
  memset(CanTxData, 0, sizeof(uint8_t) * 8);
  FDCAN_Mutex = xSemaphoreCreateMutex();
  if (FDCAN_Mutex == NULL)
    return 1;
  CanRxReady = xQueueCreate(1, sizeof(uint8_t));
  if (CanRxReady == NULL)
    return 1;
  CanError = 0;
  return 0;
}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
  *                     This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  uint8_t rx_ready;
  rx_ready = 0x0f;

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retreive Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, CanRxData) != HAL_OK)
    {
      /* Reception Error */
      UserSetBit(CanError, 0);
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      UserSetBit(CanError, 0);
    }

    if (xQueueSendFromISR(CanRxReady, (void *)&rx_ready, &xHigherPriorityTaskWoken) != pdTRUE) //队列中存在消息未被读取，报错。
    {
      memset(CanRxData, 0, sizeof(uint8_t) * 8);
      UserSetBit(CanError, 0);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/**
  * @brief  执行数据收发
  * @param  
  *Timeout：超时计数，单位ms，表示通讯最长的等待时间，当该值为0时，表示忽略接收数据，直接返回
  *CommState：Can通讯状态，bitall=0--就绪，bit0=1--成功，bit1=1--接收超时，bit2=1--ID不匹配，bit3=1--发送失败，bit4=1--接口权限未获取，bit5=1--接口权限释放失败
  *TargetID：帧的目标通讯ID
  *TxLen：发送帧长度
  *TxBuf：发送缓冲区
  *RxLen：接收到的帧长度
  *RxBuf：接收缓冲区
  * @retval 0--ok;1--fail;
  */
uint8_t FDCAN1_Communicate(const uint32_t Timeout, uint32_t *CommState, const uint32_t TargetID,
                           const uint8_t TxLen, uint8_t *TxBuf, uint8_t *RxLen, uint8_t *RxBuf)
{
  uint8_t rx_ready;

  //  vTaskDelay(10);
  *CommState = 0;
  /*获取CAN通讯权限*/
  if (xUserSemaphoreTake(FDCAN_Mutex, 0) != pdTRUE)
  {
    Bit_Set_U32t(CommState, 4, 1);
    return 1;
  }

  /*执行发送*/
  memset(CanRxData, 0, sizeof(uint8_t) * 8);
  memset(CanTxData, 0, sizeof(uint8_t) * 8);
  xQueueReceive(CanRxReady, (void *)&rx_ready, 0); //强制读取一次队列，清空队列内容。
  CanError = 0;

  for (uint32_t cnti = 0; cnti < TxLen; cnti++)
  {
    CanTxData[cnti] = *(TxBuf + cnti);
  }
  TxHeader.Identifier = TargetID;
  TxHeader.IdType = FDCAN_STANDARD_ID;     //标准ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME; //数据帧
  TxHeader.DataLength = TxLen << 16;       //数据长度
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           //关闭速率切换
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            //传统的CAN模式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; //无发送事件
  TxHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &TxHeader, CanTxData) != HAL_OK)
  {
    /*释放通讯权限*/
    if (xUserSemaphoreGive(FDCAN_Mutex) != pdTRUE)
    {
      Bit_Set_U32t(CommState, 5, 1);
      return 1;
    }
    Bit_Set_U32t(CommState, 3, 1);
    return 1;
  }
  /*等待完成接收*/
  if (Timeout == 0) //忽略接收数据，直接退出
  {
    /*释放通讯权限*/
    if (xUserSemaphoreGive(FDCAN_Mutex) != pdTRUE)
    {
      Bit_Set_U32t(CommState, 5, 1);
      return 1;
    }
    return 0;
  }
  if (xQueueReceive(CanRxReady, (void *)&rx_ready, Timeout) != pdTRUE)
  {
    /*释放通讯权限*/
    if (xUserSemaphoreGive(FDCAN_Mutex) != pdTRUE)
    {
      Bit_Set_U32t(CommState, 5, 1);
      return 1;
    }
    Bit_Set_U32t(CommState, 1, 1);
    return 1;
  }

  /*数据处理*/
  if ((RxHeader.Identifier != TargetID) && (TargetID != 0x07ff)) // ID不匹配
  {
    Bit_Set_U32t(CommState, 2, 1);
    /*释放通讯权限*/
    if (xUserSemaphoreGive(FDCAN_Mutex) != pdTRUE)
    {
      Bit_Set_U32t(CommState, 5, 1);
      return 1;
    }
    return 1;
  }
  else
  {
    *RxLen = RxHeader.DataLength >> 16;
    for (uint32_t cntk = 0; cntk < *RxLen; cntk++)
    {
      *(RxBuf + cntk) = CanRxData[cntk];
    }
    /*释放通讯权限*/
    if (xUserSemaphoreGive(FDCAN_Mutex) != pdTRUE)
    {
      Bit_Set_U32t(CommState, 5, 1);
      return 1;
    }
    *CommState = 1;

    return 0;
  }
}

uint8_t myFDCAN1_Communicate(const uint32_t Timeout, uint32_t *CommState, const uint32_t TargetID,
                           const uint8_t TxLen, uint8_t *TxBuf, uint8_t *RxLen, uint8_t *RxBuf)
{
  uint8_t rx_ready;

  *CommState = 0;
  /*获取CAN通讯权限*/
  if (xUserSemaphoreTake(FDCAN_Mutex, 0) != pdTRUE)
  {
    Bit_Set_U32t(CommState, 4, 1);
    return 1;
  }

  /*执行发送*/
  memset(CanRxData, 0, sizeof(uint8_t) * 8);
  memset(CanTxData, 0, sizeof(uint8_t) * 8);
  xQueueReceive(CanRxReady, (void *)&rx_ready, 0); //强制读取一次队列，清空队列内容。
  CanError = 0;

  for (uint32_t cnti = 0; cnti < TxLen; cnti++)
  {
    CanTxData[cnti] = *(TxBuf + cnti);
  }
  TxHeader.Identifier = TargetID;
  TxHeader.IdType = FDCAN_STANDARD_ID;     //标准ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME; //数据帧
  TxHeader.DataLength = TxLen << 16;       //数据长度
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           //关闭速率切换
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            //传统的CAN模式
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; //无发送事件
  TxHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &TxHeader, CanTxData) != HAL_OK)
  {
    /*释放通讯权限*/
    if (xUserSemaphoreGive(FDCAN_Mutex) != pdTRUE)
    {
      Bit_Set_U32t(CommState, 5, 1);
      return 1;
    }
    Bit_Set_U32t(CommState, 3, 1);
    return 1;
  }
  /*等待完成接收*/
  if (Timeout == 0) //忽略接收数据，直接退出
  {
    /*释放通讯权限*/
    if (xUserSemaphoreGive(FDCAN_Mutex) != pdTRUE)
    {
      Bit_Set_U32t(CommState, 5, 1);
      return 1;
    }
    return 0;
  }
  if (xQueueReceive(CanRxReady, (void *)&rx_ready, Timeout) != pdTRUE)
  {
    /*释放通讯权限*/
    if (xUserSemaphoreGive(FDCAN_Mutex) != pdTRUE)
    {
      Bit_Set_U32t(CommState, 5, 1);
      return 1;
    }
    Bit_Set_U32t(CommState, 1, 1);
    return 1;
  }

  /*数据处理*/
  if ((RxHeader.Identifier != TargetID) && (TargetID != 0x07ff)) // ID不匹配
  {
    Bit_Set_U32t(CommState, 2, 1);
    /*释放通讯权限*/
    if (xUserSemaphoreGive(FDCAN_Mutex) != pdTRUE)
    {
      Bit_Set_U32t(CommState, 5, 1);
      return 1;
    }
    return 1;
  }
  else
  {
    *RxLen = RxHeader.DataLength >> 16;
    for (uint32_t cntk = 0; cntk < *RxLen; cntk++)
    {
      *(RxBuf + cntk) = CanRxData[cntk];
    }
    /*释放通讯权限*/
    if (xUserSemaphoreGive(FDCAN_Mutex) != pdTRUE)
    {
      Bit_Set_U32t(CommState, 5, 1);
      return 1;
    }
    *CommState = 1;

    return 0;
  }
}

/*****************************END OF FILE*****************************/
