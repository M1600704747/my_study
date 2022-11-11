/**
******************************************************************************
* @文件    user_gpm.C
* @作者    GENGXU
* @版本    V0.0.1
******************************************************************************
171016：
增加led初始化和led操作代码。
增加GPIO时钟初始化代码，统一初始化GPIO时钟。

200331:
配置UART1为信息打印的接口，重写printf函数。

******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "user_gpm.h"
#include "user_app.h"
#include "user_cmdfun.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static IWDG_HandleTypeDef IwdgHandle;
static CRC_HandleTypeDef CrcHandle;
UART_HandleTypeDef UartHandle;

extern uint8_t LogSaveToSD;
extern DeviceAlarmStruDef ErrorAlarm;
// static int UserErrLine;
char LogBuffer[130];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/**
 * @brief  GPIO时钟初始化代码
 * @param
 * @retval
 */
void MCU_GPIO_CLK_ENABLE(void)
{
  while (1)
  {
  }
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
static GPIO_TypeDef *UserLED_PORT[UserLEDn] = {UserLED1_GPIO_PORT, UserLED2_GPIO_PORT, UserLED3_GPIO_PORT, UserLED4_GPIO_PORT, UserLED5_GPIO_PORT};
static const uint32_t UserLED_PIN[UserLEDn] = {UserLED1_PIN, UserLED2_PIN, UserLED3_PIN, UserLED4_PIN, UserLED5_PIN};
/**
 * @brief  Configures LED GPIO.
 * @param  Led: Specifies the Led to be configured.
 * @retval None
 */
void ULED_Init(UserLed_TypeDef Led)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /*所有LED均为gpioj端口*/
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = UserLED_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(UserLED_PORT[Led], &GPIO_InitStruct);

  HAL_GPIO_WritePin(UserLED_PORT[Led], UserLED_PIN[Led], GPIO_PIN_RESET);
}

/**
 * @brief  Turns selected LED On.
 * @param  Led: Specifies the Led to be set on.
 * @retval None
 */
void ULED_On(UserLed_TypeDef Led)
{
  HAL_GPIO_WritePin(UserLED_PORT[Led], UserLED_PIN[Led], GPIO_PIN_SET);
}

/**
 * @brief  Turns selected LED Off.
 * @param  Led: Specifies the Led to be set off.
 * @retval None
 */
void ULED_Off(UserLed_TypeDef Led)
{
  HAL_GPIO_WritePin(UserLED_PORT[Led], UserLED_PIN[Led], GPIO_PIN_RESET);
}

/**
 * @brief  Toggles the selected LED.
 * @param  Led: Specifies the Led to be toggled.
 * @retval None
 */
void ULED_Toggle(UserLed_TypeDef Led)
{
  HAL_GPIO_TogglePin(UserLED_PORT[Led], UserLED_PIN[Led]);
}

/**
 * @brief  LED初始化程序
 * @param  Led: Specifies the Led to be toggled.
 * @retval None
 */
void ULED_INIT_ALL(void)
{
  ULED_Init(UserLED1);
  ULED_Init(UserLED2);
  ULED_Init(UserLED3);
  ULED_Init(UserLED4);
  ULED_Init(UserLED5);

  for (uint32_t cnti = 0; cnti < 5; cnti++)
  {
    ULED_On(UserLED1);
    ULED_On(UserLED2);
    ULED_On(UserLED3);
    ULED_On(UserLED4);
    ULED_On(UserLED5);
    HAL_Delay(200);
    ULED_Off(UserLED1);
    ULED_Off(UserLED2);
    ULED_Off(UserLED3);
    ULED_Off(UserLED4);
    ULED_Off(UserLED5);
    HAL_Delay(200);
  }
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/**
 * @brief  初始化用户按钮
 * @param  None
 * @retval None
 */
void UserButtonInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /*所有LED均为gpioj端口*/
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);
}

/**
 * @brief  读取用户按钮状态
 * @param  None
 * @retval 0--low，1--high
 */
uint8_t UserButtonRead(void)
{
  return (HAL_GPIO_ReadPin(GPIOJ, GPIO_PIN_5) == GPIO_PIN_RESET) ? 0 : 1;
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/**
 * @brief  初始化用户开关
 * @param  None
 * @retval None
 */
void UserSwitchInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /*SW1*/
  __HAL_RCC_GPIOI_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  __HAL_RCC_GPIOJ_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*SW2、SW3、SW4*/
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
 * @brief  读取用户开关状态
 *SW1:BIT0
 *SW2:BIT1
 *SW3:BIT2
 *SW4:BIT3
 * @param  None
 * @retval num
 */
uint8_t UserSwitchRead(void)
{
  uint8_t num = 0;
  /*SW1*/
  if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_8) == GPIO_PIN_SET)
  {
    UserSetBit(num, 0);
  }
  /*SW2*/
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
  {
    UserSetBit(num, 1);
  }
  /*SW3*/
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET)
  {
    UserSetBit(num, 2);
  }
  /*SW4*/
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_SET)
  {
    UserSetBit(num, 3);
  }

  return num;
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/**
 * @brief  时间数据写入
 * @param
 * *time：需要写入的时间
 * uint8_t year;//年，取值1~200表示1901~2100年
 * uint8_t month;//月，取值1~12
 * * uint8_t day;//日，取值1~31
 * uint8_t hour;//小时，取值0~23，24小时制
 * uint8_t minute;//分钟，取值0~59
 * uint8_t second;//秒，取值0~59
 * uint8_t decisecond;//十分之一秒，取值0~9
 * uint8_t millisecond;//千分之一秒，取值0~99
 * @retval num
 */
void TimeWrite(TimeStructDef *time,
               uint8_t year,
               uint8_t month,
               uint8_t day,
               uint8_t hour,
               uint8_t minute,
               uint8_t second,
               uint8_t decisecond,
               uint8_t millisecond)
{
  time->year = year;
  time->month = month;
  time->day = day;
  time->hour = hour;
  time->minute = minute;
  time->second = second;
  time->decisecond = decisecond;
  //  time->millisecond=millisecond;
}

/**
 * @brief  毫秒数据累加
 * @param
 * *time：需要写入的时间
 * @retval num
 */
void TimeAddMS(TimeStructDef *time)
{
  //  uint8_t daymax=0;
  //  uint16_t year_tmp=time->year+1900;
  //
  //  if((time->month==1)||(time->month=3)||(time->month=5)||(time->month=7)||(time->month=8)||(time->month=10)||(time->month=12))
  //  {
  //    daymax=31;
  //  }
  //  else if(time->month==2)
  //  {
  //    if(((year_tmp%4 == 0) && (year_tmp%100 != 0)) || (year_tmp%400 == 0))
  //    {
  //      daymax=29;
  //    }
  //    else
  //    {
  //      daymax=28;
  //    }
  //  }
  //  else
  //  {
  //    daymax=30;
  //  }
  //
  //  time->millisecond++;
  //
  //  if(time->millisecond>99)
  //  {
  //    time->millisecond=0;
  //    time->decisecond++;
  //  }
  //  if(time->decisecond>9)
  //  {
  //    time->decisecond=0;
  //    time->second++;
  //  }
  //  if(time->second>59)
  //  {
  //    time->second=0;
  //    time->minute++;
  //  }
  //  if(time->minute>59)
  //  {
  //    time->minute=0;
  //    time->hour++;
  //  }
  //  if(time->hour>23)
  //  {
  //    time->hour=0;
  //    time->day++;
  //  }
  //  if(time->day>daymax)
  //  {
  //    time->day=1;
  //    time->month++;
  //  }
  //  if(time->month>12)
  //  {
  //    time->month=1;
  //    time->year++;
  //  }
}

/**
 * @brief  十分之一秒数据累加
 * @param
 * *time：需要写入的时间
 * @retval num
 */
void TimeAddDS(TimeStructDef *time)
{
  uint8_t daymax = 0;
  uint16_t year_tmp = time->year + 2000;

  if ((time->month == 1) || (time->month == 3) || (time->month == 5) || (time->month == 7) || (time->month == 8) || (time->month == 10) || (time->month == 12))
  {
    daymax = 31;
  }
  else if (time->month == 2)
  {
    if (((year_tmp % 4 == 0) && (year_tmp % 100 != 0)) || (year_tmp % 400 == 0))
    {
      daymax = 29;
    }
    else
    {
      daymax = 28;
    }
  }
  else
  {
    daymax = 30;
  }

  time->decisecond++;

  if (time->decisecond > 9)
  {
    time->decisecond = 0;
    time->second++;
  }
  if (time->second > 59)
  {
    time->second = 0;
    time->minute++;
  }
  if (time->minute > 59)
  {
    time->minute = 0;
    time->hour++;
  }
  if (time->hour > 23)
  {
    time->hour = 0;
    time->day++;
  }
  if (time->day > daymax)
  {
    time->day = 1;
    time->month++;
  }
  if (time->month > 12)
  {
    time->month = 1;
    time->year++;
  }
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/**
 * @brief  初始化DWT
 * @param  无
 * @retval 无
 * @note   使用延时函数前，必须调用本函数
 */
uint32_t User_SysClockParam = 0;
void DWT_Init(void)
{
  User_SysClockParam = (HAL_RCC_GetSysClockFreq() / 1000000);

  DWT_LAR |= DWT_LAR_UNLOCK;

  /* 使能DWT外设 */
  DEM_CR |= (uint32_t)DEM_CR_TRCENA;

  /* DWT CYCCNT寄存器计数清0 */
  DWT_CYCCNT = (uint32_t)0u;

  /* 使能Cortex-M DWT CYCCNT寄存器 */
  DWT_CR |= (uint32_t)DWT_CR_CYCCNTENA;
}

/**
 * @brief  读取当前时间戳
 * @param  无
 * @retval 当前时间戳，即DWT_CYCCNT寄存器的值
 */
uint32_t CPU_TS_TmrRd(void)
{
  return ((uint32_t)DWT_CYCCNT);
}

/**
  * @brief  采用CPU的内部计数实现精确延时，32位计数器
  * @param  us : 延迟长度，单位1 us
  * @retval 无
  * @note   使用本函数前必须先调用 CPU_TS_TmrInit 函数使能计数器，
            最大延时值为8秒，即8*1000*1000
  */
void User_delay_us(uint32_t us)
{
  uint32_t ticks;
  uint32_t told, tnow, tcnt = 0;

  ticks = us * User_SysClockParam; /* 需要的节拍数 */
  tcnt = 0;
  told = (uint32_t)CPU_TS_TmrRd(); /* 刚进入时的计数器值 */
  while (1)
  {
    tnow = (uint32_t)CPU_TS_TmrRd();
    if (tnow != told)
    {
      /* 32位计数器是递增计数器 */
      if (tnow > told)
      {
        tcnt += tnow - told;
      }
      /* 重新装载 */
      else
      {
        tcnt += UINT32_MAX - told + tnow;
      }
      told = tnow;
      /*时间超过/等于要延迟的时间,则退出 */
      if (tcnt >= ticks)
        break;
    }
  }
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#ifndef SDRAM_DEVICE_ADDR
#define SDRAM_DEVICE_ADDR 0xD0000000U
#endif
/**
 * @brief  SDRAM连续地址写入uint8数据
 * @param
 *StartAddr：相对SDRAM_DEVICE_ADDR的偏移地址
 *Buffer：待写入的数据
 *Size：数据长度
 * @retval 当前时间戳，即DWT_CYCCNT寄存器的值
 */
void UserSDRAM_Write_U8(uint32_t StartAddr, uint8_t *Buffer, uint32_t Size)
{
  for (uint32_t cnti = 0; cnti < Size; cnti++)
  {
    *(__IO uint8_t *)(SDRAM_DEVICE_ADDR + StartAddr + cnti) = *(Buffer + cnti);
  }
}

/**
 * @brief  SDRAM连续地址读取uint8数据
 * @param
 *StartAddr：相对SDRAM_DEVICE_ADDR的偏移地址
 *Buffer：待读取的数据缓冲区
 *Size：数据长度
 * @retval 当前时间戳，即DWT_CYCCNT寄存器的值
 */
void UserSDRAM_Read_U8(uint32_t StartAddr, uint8_t *Buffer, uint32_t Size)
{
  for (uint32_t cnti = 0; cnti < Size; cnti++)
  {
    *(Buffer + cnti) = *(__IO uint8_t *)(SDRAM_DEVICE_ADDR + StartAddr + cnti);
  }
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/**
 * @brief UART MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration
 * @param huart: UART handle pointer
 * @retval None
 */
void User_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();

  /* Select SysClk as source of USART1 clocks */
  RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART16;
  RCC_PeriphClkInit.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

  /* Enable USARTx clock */
  USARTx_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;

  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Alternate = USARTx_RX_AF;

  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief UART MSP De-Initialization
 *        This function frees the hardware resources used in this example:
 *          - Disable the Peripheral's clock
 *          - Revert GPIO and NVIC configuration to their default state
 * @param huart: UART handle pointer
 * @retval None
 */
void User_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  /*##-1- Reset peripherals ##################################################*/
  USARTx_FORCE_RESET();
  USARTx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);
  HAL_NVIC_DisableIRQ(USART1_IRQn);
}

/**
 * @brief  UART1初始化
 * @param  None
 * @retval 0--ok;1--fail
 */
uint8_t Uart1_Comm_Init(void)
{
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits (7 data bit + 1 parity bit) :
                      BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
      - Stop Bit    = One Stop bit
      - Parity      = ODD parity
      - BaudRate    = 256000 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance = USARTx;

  UartHandle.Init.BaudRate = 256000;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  User_UART_MspInit(&UartHandle);
  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    return 1;
  }
  return 0;
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/**
 * @brief  串口调试线程
 * @param  None
 * @retval None
 */
extern uint8_t debug_cmd[200];
extern uint8_t debug_mode;
void Uart1DebugTask(void *pvParameters)
{
  uint8_t CmdIdx = 0;
  uint8_t CmdBufferByte = 0;
  uint8_t CmdBuffer[200];
  memset(CmdBuffer, 0, sizeof(CmdBuffer));
  if (HAL_UART_Receive_IT(&UartHandle, (uint8_t *)&CmdBufferByte, 1) != HAL_OK) //启动接收
  {
    User_Error_Handler(__FILE__, __LINE__, 26101);
  }
  for (;;)
  {
    /* 等待通知，进入阻塞 */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    printf("%c", (char)CmdBufferByte);
    CmdBuffer[CmdIdx] = CmdBufferByte;
    CmdBufferByte = 0;
    CmdIdx++;

    if (HAL_UART_Receive_IT(&UartHandle, (uint8_t *)&CmdBufferByte, 1) != HAL_OK) //启动接收
    {
      User_Error_Handler(__FILE__, __LINE__, 26101);
    }

    if (CmdIdx > 1)
    {
      if (CmdBuffer[CmdIdx - 2] == 0x0D && CmdBuffer[CmdIdx - 1] == 0x0A) //回车
      {
        //处理
        if (debug_mode == 0)
        {
          if (memcmp(CmdBuffer, "DEBUG", 5) == 0)
          {
            debug_mode = 1;
            //进入调试模式
            xTaskCreate(DebugCtlTask, "DebugCtlTask", configMINIMAL_STACK_SIZE * 10, NULL, osPriorityHigh + 3, NULL);
          }
        }
        else
        {
          memcpy(debug_cmd, CmdBuffer, strlen((char *)CmdBuffer));
        }
        memset(CmdBuffer, 0, sizeof(CmdBuffer));
        CmdIdx = 0;
      }
    }
  }
}

/**
 * @brief 串口中断接收回调
 * @param
 * @retval NONE
 */
void Debug_RX_Callback(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(Uart1DebugTaskHandle, &xHigherPriorityTaskWoken);
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/**
  * @brief  This function is executed in case of error occurrence.
            需要在LED、OS初始化之后使用
  * @param  None
  * @retval None
  */
void User_Error_Handler(char *function, int line, uint32_t erridx)
{
  char UserErrFunc[50];
  int UserErrLine;
  uint32_t errorcode = erridx;

  UBaseType_t uxSavedInterruptStatus;

  if (56102 == erridx)
  {
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    errorcode = 26102;
  }
  else
  {
    taskENTER_CRITICAL();
  }

  ///
  ULED_On(UserLED5);
  DeviceStatusChg = Error;
#if 1
  for (uint16_t cnti = 0; cnti < 50; cnti++)
  {
    if (*(function + cnti) == 0)
    {
      break;
    }
    else
    {
      UserErrFunc[cnti] = *(function + cnti);
    }
  }
#else
  memset(UserErrFunc, 0, sizeof(UserErrFunc));
  memcpy(UserErrFunc, function, sizeof(UserErrFunc));
#endif

  UserErrLine = line;
  printf("\n********************************************************");
  printf("\nError:%s\t%d", UserErrFunc, UserErrLine);
  printf("\n********************************************************");
  //记录日志
  memset(LogBuffer, 0, sizeof(LogBuffer));
  sprintf(LogBuffer, "ERROR CODE %d, SYS %s %ld", erridx, UserErrFunc, UserErrLine);
  WorkRecordReadWrite(0, 0, LogBuffer);

  // SD_Log_Write();//日志文件写入sd卡
  LogSaveToSD = 1;
  ErrorAlarm.alarm = 1;
  ErrorAlarm.level = 2;
  ErrorAlarm.code = errorcode;
  //
  if (56102 == erridx)
  {
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
  }
  else
  {
    taskEXIT_CRITICAL();
  }
  //发送告警
  while (EthSendAlarm((DeviceAlarmStruDef *)&ErrorAlarm) != 0)
  {
    vTaskDelay(100);
  }

  while (1)
  {
    vTaskDelay(1);
  }
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/**
  * @brief  看门狗初始化
  * @param  //prer:分频数:IWDG_PRESCALER_4~IWDG_PRESCALER_256
            //rlr:自动重装载值,0~0XFFF.
            //时间计算(大概):Tout=((4*2^prer)*rlr)/32 (ms).

  * @retval 1--初始化失败，0--初始化成功
  */
uint8_t IWDG_Init_Start(uint8_t prer, uint16_t rlr)
{
  IwdgHandle.Instance = IWDG1;
  IwdgHandle.Init.Prescaler = prer; //设置 IWDG 分频系数
  IwdgHandle.Init.Reload = rlr;     //重装载值
  IwdgHandle.Init.Window = IWDG_WINDOW_DISABLE;

  if (HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
  {
    return 1;
  }
  else
    return 0;
}

/**
 * @brief  喂狗
 * @param
 * @retval 1--喂狗失败，0--喂狗成功
 */
uint8_t IWDG_Feed(void)
{
  if (HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK)
  {
    return 1;
  }
  else
    return 0;
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/**
  * @brief  位设置
  * @param * data：要设置的数
             bit:0~31
             value：0--对应位设置为0，1--对应位设置为1
  * @retval NONE
  */
void Bit_Set_U32t(uint32_t *data, uint8_t bit, uint8_t value)
{
  uint32_t tmp = *data;
  if (value == 0)
  {
    UserClrBit(tmp, bit);
  }
  else
    UserSetBit(tmp, bit);
  *data = tmp;
}

/**
 * @brief  位读取
 * @param bit:0~31
 * @retval 0-reset，1-set
 */
uint8_t Bit_Get_U32t(uint32_t data, uint8_t bit)
{
  uint32_t tmp1 = 0;
  uint32_t tmp2 = data;

  UserSetBit(tmp1, bit);
  if ((tmp1 & tmp2) == 0)
  {
    return 0;
  }
  else
    return 1;
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/**
 * @brief uint32_t限定范围数据增加
 *返回x和y的和，如果x+y大于max，则从0开始重新计算
 * @param x,y：进行相加的数，max：允许的最大数值
 * @retval 增加后的数据
 */
uint32_t U32T_LimitAdd(const uint32_t x, const uint32_t y, uint32_t max)
{
  return (((x + y) < max) ? (x + y) : (x + y - max));
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/**
 * @brief uint32_t转为uint8_t的数组用于发送
 * @param None
 * @retval 0--ok;1--fail
 */
void uint32_To_Array(const uint32_t data_32, uint8_t *data_8)
{
  union
  {
    uint32_t data_i;
    uint8_t data_u[4];
  } data_tmp;

  data_tmp.data_i = data_32;
  *(data_8 + 0) = data_tmp.data_u[3];
  *(data_8 + 1) = data_tmp.data_u[2];
  *(data_8 + 2) = data_tmp.data_u[1];
  *(data_8 + 3) = data_tmp.data_u[0];
}

/**
 * @brief  uint8_t的数组转换为uint32
 * @param
 * @retval NONE
 */
uint32_t Array_To_uint32(const uint8_t *data_8)
{
  union
  {
    uint32_t data_i;
    uint8_t data_u[4];
  } data_tmp;

  data_tmp.data_u[0] = *(data_8 + 3);
  data_tmp.data_u[1] = *(data_8 + 2);
  data_tmp.data_u[2] = *(data_8 + 1);
  data_tmp.data_u[3] = *(data_8 + 0);
  return data_tmp.data_i;
}

/**
 * @brief  float转换为uint8
 * @param
 * @retval NONE
 */
void float_to_uint8(const float data_f, uint8_t *data_8)
{
  union
  {
    float data_i;
    uint8_t data_u[4];
  } data_tmp;

  data_tmp.data_i = data_f;
  *(data_8 + 0) = data_tmp.data_u[0];
  *(data_8 + 1) = data_tmp.data_u[1];
  *(data_8 + 2) = data_tmp.data_u[2];
  *(data_8 + 3) = data_tmp.data_u[3];
}

/**
 * @brief  uint8转换为float
 * @param
 * @retval NONE
 */
float uint8_to_float(const uint8_t *data_8)
{
  union
  {
    float data_i;
    uint8_t data_u[4];
  } data_tmp;

  data_tmp.data_u[0] = *(data_8 + 0);
  data_tmp.data_u[1] = *(data_8 + 1);
  data_tmp.data_u[2] = *(data_8 + 2);
  data_tmp.data_u[3] = *(data_8 + 3);
  return data_tmp.data_i;
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/**
 * @brief  CRC初始化
 * @param
 * @retval 1--初始化失败，0--初始化成功
 */
uint8_t User_CRC_Init(void)
{
  /*##-1- Configure the CRC peripheral #######################################*/
  CrcHandle.Instance = CRC;
  /* The default polynomial is not used. It is required to defined it in CrcHandle.Init.GeneratingPolynomial*/
  CrcHandle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  /* Set the value of the polynomial */
  CrcHandle.Init.GeneratingPolynomial = 0x8005;
  /* The user-defined generating polynomial generates a 16-bit long CRC */
  CrcHandle.Init.CRCLength = CRC_POLYLENGTH_16B;
  /* The default init value is used */
  CrcHandle.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  CrcHandle.Init.InitValue = 0xFFFF;
  /* The input data are not inverted，与输入的数据长度保持一致 */
  CrcHandle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  /* The output data are not inverted */
  CrcHandle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  /* The input data are 8-bit long */
  CrcHandle.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&CrcHandle) != HAL_OK)
  {
    return 1;
  }
  else
    return 0;
}

/**
 * @brief  CRC计算
 * @param
 * @retval NONE
 */
uint32_t CRC_Value_Computer(uint8_t *aDataBuffer, uint32_t BUFFER_SIZE)
{
  return HAL_CRC_Calculate(&CrcHandle, (uint32_t *)aDataBuffer, BUFFER_SIZE);
}

/*****************************END OF FILE*****************************/
