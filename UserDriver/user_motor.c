/**
******************************************************************************
* @文件    user_motor.C
* @作者    
* @版本    
******************************************************************************

******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "user_motor.h"
#include "user_can.h"

static uint32_t FDCANCOMSTATE;

//static uint8_t tx[8]; //发送
//static uint8_t rx[8]; //接收

static void MotorWorkRecordReadWrite(int data1, int data2, const uint32_t rcdr, char *info)
{
  WorkRecordReadWrite(0, rcdr, info);
  //User_Error_Handler(__FILE__, rcdr, 26100);
}

static void CoverI32ToU8_B(const int_fast32_t data_int, uint8_t *data_8)
{
  union
  {
    int_fast32_t data_i;
    uint8_t data_u[4];
  } data_tmp;

  data_tmp.data_i = data_int;
  *(data_8 + 0) = data_tmp.data_u[3];
  *(data_8 + 1) = data_tmp.data_u[2];
  *(data_8 + 2) = data_tmp.data_u[1];
  *(data_8 + 3) = data_tmp.data_u[0];
}

static void CoverU16ToU8_B(const uint16_t data_16, uint8_t *data_8)
{
  union
  {
    uint32_t data_i;
    uint8_t data_u[2];
  } data_tmp;

  data_tmp.data_i = data_16;
  *(data_8 + 0) = data_tmp.data_u[1];
  *(data_8 + 1) = data_tmp.data_u[0];
}

static void CoverFloatToU8_B(const float data_float, uint8_t *data_8)
{
  union
  {
    float data_f;
    uint8_t data_u[4];
  } data_tmp;

  data_tmp.data_f = data_float;
  *(data_8 + 0) = data_tmp.data_u[3];
  *(data_8 + 1) = data_tmp.data_u[2];
  *(data_8 + 2) = data_tmp.data_u[1];
  *(data_8 + 3) = data_tmp.data_u[0];
}

static void CoverU32ToU8_B(const uint32_t data_32, uint8_t *data_8)
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

static int_fast32_t CoverU8ToI32_B(const uint8_t *data_8)
{
  union
  {
    int_fast32_t data_i;
    uint8_t data_u[4];
  } data_tmp;

  data_tmp.data_u[0] = *(data_8 + 3);
  data_tmp.data_u[1] = *(data_8 + 2);
  data_tmp.data_u[2] = *(data_8 + 1);
  data_tmp.data_u[3] = *(data_8 + 0);
  return data_tmp.data_i;
}

static uint16_t CoverU8ToU16_B(const uint8_t *data_8)
{
  union
  {
    uint16_t data_i;
    uint8_t data_u[2];
  } data_tmp;

  data_tmp.data_u[0] = *(data_8 + 1);
  data_tmp.data_u[1] = *(data_8 + 0);
  return data_tmp.data_i;
}

static uint32_t CoverU8ToU32_B(const uint8_t *data_8)
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
* @brief 数据收发加等待权限获取
* @param
* @retval
	0--ok，1--fail
* @note
*/
uint8_t FDCAN1_CommunicateHandle(uint32_t timeout, uint8_t boardid, uint8_t txlen, uint8_t *txbuff, uint8_t *rxlen, uint8_t *rxbuff)
{
  uint32_t timeoutcnt = 0;

  vTaskDelay(5);

  for (;;)
  {
    FDCANCOMSTATE = 0;
    if (FDCAN1_Communicate(timeout, &FDCANCOMSTATE, boardid, txlen, txbuff, rxlen, rxbuff) == 1) //fail
    {
      if (Bit_Get_U32t(FDCANCOMSTATE, 4) == 1) //接口权限未获取
      {
        __nop(); //直到权限取得
      }
      if (Bit_Get_U32t(FDCANCOMSTATE, 1) == 1) //接收超时
      {
        timeoutcnt++;
        if (timeoutcnt > 3) //发送总次数不超过3次
        {
              return 1;
        }
      }
      if (Bit_Get_U32t(FDCANCOMSTATE, 2) == 1) //ID不匹配
      {
        MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:FDCAN ID unmatched");
        __nop(); //重发
      }
      if (Bit_Get_U32t(FDCANCOMSTATE, 3) == 1) //发送失败
      {
        MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:FDCAN Send Fail");
        __nop(); //重发
      }
      if (Bit_Get_U32t(FDCANCOMSTATE, 5) == 1) //权限无法释放
      {
        MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:FDCAN MutexGive Fail");
        return 1;
      }
    }
    else
    {
      return 0;
    }
    vTaskDelay(1);
  }
}

/**
* @brief 步进电机控制参数初始化
* @param
*stpm：电机控制结构体
* @retval
* @note
*/

/**
* @brief 直流电机控制参数初始化
* @param
*dcm：电机控制结构体
* @retval
* @note
*/
void DcmDrvParamInit(DCMStruDef *dcm)
{
#if 1
  memset(dcm, 0, sizeof(DCMStruDef));
#else
  dcm->BoardID = 0;
  dcm->WorkState = 0;
  dcm->WorkState_N = 0;
  dcm->Error = 0;
  dcm->Warning = 0;
  dcm->Mode = 0;
  dcm->CLMode = 0;
  dcm->ActualParam = 0;
  dcm->KP = 0;
  dcm->KI = 0;
  dcm->EP = 0;
  dcm->PPR = 0;
  dcm->TargetParam = 0;
  dcm->CLState = 0;
  dcm->ActualAmp = 0;
  dcm->TSAmp = 0;
#endif
}

/**
* @brief 板卡固件编号查询
* @param
BoardID：板卡ID号
*Ver:版本号指针，格式“V:X.Y.Z”
* @retval
0--ok，1--fail
* @note
*/
uint8_t BoardN_VersionGet(uint8_t BoardID, uint8_t *Ver)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 1;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 3;
  FDCAN1_CommunicateHandle(timeout, BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  memcpy(Ver, &rx[1], 7);
  return 0;
}

//NEW------------------------------------------------------------------------/
/******************************************************************************/
/*                 特殊指令                                                    */
/******************************************************************************/
/**
* @brief 发送心跳包（广播）
* @param
* @retval
0--ok，1--fail
* @note
*/
uint8_t BoardN_Heartbeat(void)
{
  uint32_t timeout = 0;          //超时计数无返回值
  uint8_t txlen = 4;             //发送的帧长度
  uint8_t tx[8] = {0};           //发送
  uint8_t rxlen = 0;             //接收的帧长度
  uint8_t rx[8] = {0};           //接收
  uint16_t BroadcastID = 0x07ff; //广播ID

  tx[0] = 255;
  tx[1] = 254;
  tx[2] = 255;
  tx[3] = 254;
  FDCAN1_CommunicateHandle(timeout, BroadcastID, txlen, tx, &rxlen, rx);
  return 0;
}

/**
* @brief 板卡重启
* @param
*BoardID：板卡ID号
* @retval
0--ok，1--fail
* @note
*/
uint8_t BoardN_Reboot(uint8_t BoardID)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 4;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收

  tx[0] = 254;
  tx[1] = 0;
  tx[2] = 254;
  tx[3] = 0;
  FDCAN1_CommunicateHandle(timeout, BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {    
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  return 0;
}

/******************************************************************************/
/*                 状态查询指令                                                */
/******************************************************************************/
/**
* @brief 步进电机驱动状态查询，访问后刷新步进电机的工作状态、引脚电平(STOPR,STOPL,EMG)
* @param
*stpm：电机控制结构体
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_DrvStateGet(STPMStruDef *stpm)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  uint16_t u16Temp = 0;
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);

  tx[0] = 1;
  tx[1] = stpm->DrvIndex << 4;
  tx[1] |= 0;

  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);

  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  u16Temp = CoverU8ToU16_B(&rx[1]);
  stpm->WorkState_N = (u16Temp >> 14) & 0x03;
  stpm->Error = (u16Temp >> 13) & 0x01;
  stpm->Warning = (u16Temp >> 12) & 0x01;
  stpm->ErrorCode = u16Temp & 0x0fff;
  stpm->ClpState_N = UserReadBit(rx[3], 6);
  stpm->EmgState = UserReadBit(rx[3], 4);
  stpm->StopL.InptState = UserReadBit(rx[3], 3);
  stpm->StopR.InptState = UserReadBit(rx[3], 2);
  stpm->PulseState = UserReadBit(rx[3], 1);
  stpm->STPOut = UserReadBit(rx[3], 0);
  return 0;
}

/**
* @brief 步进电机编码器位置及闭环控制状态读取
* @param
*stpm：电机控制结构体
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_EncoderPositionRead(STPMStruDef *stpm)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  uint16_t u16Temp = 0;
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 1;
  tx[1] = stpm->DrvIndex << 4;
  tx[1] |= 2;

  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);

  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->EncoderPos = CoverU8ToU32_B(&rx[4]);
  u16Temp = CoverU8ToU16_B(&rx[1]);
  stpm->WorkState_N = (u16Temp >> 14) & 0x03;
  stpm->Error = (u16Temp >> 13) & 0x01;
  stpm->Warning = (u16Temp >> 12) & 0x01;
  stpm->ErrorCode = u16Temp & 0x0fff;
  stpm->ClpState_N = UserReadBit(rx[3], 6);
  stpm->EmgState = UserReadBit(rx[3], 4);
  stpm->StopL.InptState = UserReadBit(rx[3], 3);
  stpm->StopR.InptState = UserReadBit(rx[3], 2);
  stpm->PulseState = UserReadBit(rx[3], 1);
  stpm->STPOut = UserReadBit(rx[3], 0);

  return 0;
}

/**
* @brief 步进电机实际位置读取
* @param
*stpm：电机控制结构体
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_XactualGet(STPMStruDef *stpm)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  uint16_t u16Temp = 0;
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 1;
  tx[1] = stpm->DrvIndex << 4;
  tx[1] |= 1;

  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);

  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Xactual = CoverU8ToU32_B(&rx[4]);
  u16Temp = CoverU8ToU16_B(&rx[1]);
  stpm->WorkState_N = (u16Temp >> 14) & 0x03;
  stpm->Error = (u16Temp >> 13) & 0x01;
  stpm->Warning = (u16Temp >> 12) & 0x01;
  stpm->ErrorCode = u16Temp & 0x0fff;
  stpm->ClpState_N = UserReadBit(rx[3], 6);
  stpm->EmgState = UserReadBit(rx[3], 4);
  stpm->StopL.InptState = UserReadBit(rx[3], 3);
  stpm->StopR.InptState = UserReadBit(rx[3], 2);
  stpm->PulseState = UserReadBit(rx[3], 1);
  stpm->STPOut = UserReadBit(rx[3], 0);
  return 0;
}

/**
* @brief 步进电机实际速度读取
* @param
*stpm：电机控制结构体
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_VactualGet(STPMStruDef *stpm)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  uint16_t u16Temp = 0;
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 1;
  tx[1] = stpm->DrvIndex << 4;
  ;
  tx[1] |= 3;

  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);

  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Vactual = CoverU8ToU32_B(&rx[4]);
  u16Temp = CoverU8ToU16_B(&rx[1]);
  stpm->WorkState_N = (u16Temp >> 14) & 0x03;
  stpm->Error = (u16Temp >> 13) & 0x01;
  stpm->Warning = (u16Temp >> 12) & 0x01;
  stpm->ErrorCode = u16Temp & 0x0fff;
  stpm->ClpState_N = UserReadBit(rx[3], 6); //uint8_t
  stpm->EmgState = UserReadBit(rx[3], 4);
  stpm->speedStatus = UserReadBit(rx[3], 5);
  stpm->StopL.InptState = UserReadBit(rx[3], 3);
  stpm->StopR.InptState = UserReadBit(rx[3], 2);
  stpm->PulseState = UserReadBit(rx[3], 1);
  stpm->STPOut = UserReadBit(rx[3], 0);
  return 0;
}

/**
* @brief 步进电机实际加速度读取
* @param
*stpm：电机控制结构体
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_AactualGet(STPMStruDef *stpm)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  uint16_t u16Temp = 0;
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 1;
  tx[1] = stpm->DrvIndex << 4;
  tx[1] |= 4;

  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);

  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Aactual = CoverU8ToU32_B(&rx[4]);
  u16Temp = CoverU8ToU16_B(&rx[1]);
  stpm->WorkState_N = (u16Temp >> 14) & 0x03;
  stpm->Error = (u16Temp >> 13) & 0x01;
  stpm->Warning = (u16Temp >> 12) & 0x01;
  stpm->ErrorCode = u16Temp & 0x0fff;
  stpm->ClpState_N = UserReadBit(rx[3], 6);
  stpm->EmgState = UserReadBit(rx[3], 4);
  stpm->StopL.InptState = UserReadBit(rx[3], 3);
  stpm->StopR.InptState = UserReadBit(rx[3], 2);
  stpm->PulseState = UserReadBit(rx[3], 1);
  stpm->STPOut = UserReadBit(rx[3], 0);
  return 0;
}

/**
* @brief 步进电机边沿检测计数器读取
* @param
*stpm：电机控制结构体
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_ScntGet(STPMStruDef *stpm)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度

  uint8_t tx[8] = {0}; //发送
  uint8_t rxlen = 0;   //接收的帧长度
  uint8_t rx[8] = {0}; //接收
  uint16_t u16Temp = 0;
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 1;
  tx[1] = stpm->DrvIndex << 4;
  tx[1] |= 10;

  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);

  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->PulseCnt = CoverU8ToU32_B(&rx[4]);
  u16Temp = CoverU8ToU16_B(&rx[1]);
  stpm->WorkState_N = (u16Temp >> 14) & 0x03;
  stpm->Error = (u16Temp >> 13) & 0x01;
  stpm->Warning = (u16Temp >> 12) & 0x01;
  stpm->ErrorCode = u16Temp & 0x0fff;
  stpm->ClpState_N = UserReadBit(rx[3], 6);
  stpm->EmgState = UserReadBit(rx[3], 4);
  stpm->StopL.InptState = UserReadBit(rx[3], 3);
  stpm->StopR.InptState = UserReadBit(rx[3], 2);
  stpm->PulseState = UserReadBit(rx[3], 1);
  stpm->STPOut = UserReadBit(rx[3], 0);
  return 0;
}

/**
* @brief 板卡错误代码读取和重置
* @param
*stpm：电机控制结构体
mode：0--仅读取，1--读取并重置warning
* @retval
0--ok，1--fail
* @note
*/
uint8_t BoardN_ErrorCodeGet(DCMStruDef *dcm, STPMStruDef *stpm, uint8_t mode)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 4;
  tx[1] = stpm->DrvIndex << 4;
  ;
  if (mode == 0)
  {
    tx[1] |= 0x80;
  }
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  dcm->ErrorCode = rx[3];
  stpm->ErrorCode = CoverU8ToU32_B(&rx[4]);

  return 0;
}

/**
* @brief 步进电机运动轨迹读取
* @param
*stpm：电机控制结构体
dataindex：数据索引，取值1-10000
*shift：位移参数
*speed：速度参数
*acc_encoder：加速度、编码器参数
* @retval 
0--ok，1--fail
* @note 
*/
uint8_t StpmN_RCDRead(STPMStruDef *stpm, const uint16_t dataindex, int_fast32_t *shift, int_fast32_t *speed, int_fast32_t *acc_encoder)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 4;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收

  //memset(rx, 0, 8);
  //memset(tx, 0, 8);

  tx[0] = 5;
  CoverU16ToU8_B(dataindex, &tx[1]);
  /*位移*/
  tx[3] = 1;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    return 1;
  }
  *shift = CoverU8ToI32_B(&rx[4]);
  /*速度*/
  tx[3] = 2;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    return 1;
  }
  *speed = CoverU8ToI32_B(&rx[4]);
  /*加速度、编码器*/
  tx[3] = 3;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    return 1;
  }
  *acc_encoder = CoverU8ToI32_B(&rx[4]);

  return 0;
}
/******************************************************************************/
/*                 参数配置指令                                                */
/******************************************************************************/
/**
* @brief 心跳包设置
* @param
BoardID：板卡ID号
stopmode:0-定位模式下，超时等待运行完成，1-超时立即停机
releasemode：0-停机后不释放，1-停机后释放
HBtimeout：超时时间，单位ms
* @retval
0--ok，1--fail
* @note
*/
uint8_t BoardN_HeartbeatSet(uint8_t BoardID, uint8_t stopmode, uint8_t releasemode, uint32_t HBtimeout)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 11;
  if (stopmode == 1)
  {
    UserSetBit(tx[1], 6);
  }
  if (releasemode == 1)
  {
    UserSetBit(tx[1], 5);
  }
  CoverU32ToU8_B(HBtimeout, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  return 0;
}

/**
* @brief 直流电机模式配置
* @param
*dcm：电机控制结构体
mode：0--直流无刷，1--直流有刷，2--电磁铁
clmode：0--开环，1--闭环
kp：PI比例项，实际值为KP除以100（0.00~50.00）
ki：PI积分项，实际值为KI除以100（0.00~50.00）
ppr:单圈脉冲数
* @retval
0--ok，1--fail
* @note
*/
uint8_t DcmN_DrvModeSet(DCMStruDef *dcm, const uint8_t mode, const uint8_t clmode, const uint16_t kp, const uint16_t ki, const uint8_t ppr)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 12;
  tx[1] |= (mode << 5);
  tx[1] |= (clmode << 4);
  CoverU16ToU8_B(kp, &tx[2]);
  CoverU16ToU8_B(ki, &tx[4]);
  tx[6] = ppr;
  FDCAN1_CommunicateHandle(timeout, dcm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  dcm->Mode = mode;
  dcm->CLMode = clmode;
  dcm->KP = kp;
  dcm->KI = ki;
  dcm->PPR = ppr;
  return 0;
}

/**
* @brief 直流电机运行电流阈值设置
* @param
*dcm：电机控制结构体
* @retval
0--ok，1--fail
* @note
*/
uint8_t DcmN_ThAmpSet(DCMStruDef *dcm, const float amp)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 13;
  tx[1] = 0;
  CoverFloatToU8_B(amp, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, dcm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  dcm->TSAmp = amp;

  return 0;
}

/**
* @brief 步进电机电流参数配置
* @param
*stpm：电机控制结构体
drvcurrent：驱动电流
hldcurrent：保持电流
stddelay：延迟参数，单位1us
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CurrentSet(STPMStruDef *stpm, const float drvcurrent, const float hldcurrent, const uint32_t stddelay)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  uint32_t paramd_tmp = 0;
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  if (!((drvcurrent > 0.0f) && (drvcurrent <= 3.3f)))
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (!((hldcurrent > 0.0f) && (hldcurrent <= 3.3f)))
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }

  tx[0] = 14;
  tx[1] |= stpm->DrvIndex << 4;
  paramd_tmp = 2; //向下匹配电流
  CoverU32ToU8_B(paramd_tmp, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 14;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 1; //运行电流
  CoverFloatToU8_B(drvcurrent, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->DrvScl = drvcurrent;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 14;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 2; //保持电流
  CoverFloatToU8_B(hldcurrent, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->HOLDScl = hldcurrent;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 14;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 3; //切换延迟
  CoverU32ToU8_B(stddelay, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->StdDelay = stddelay;
  return 0;
}

/**
* @brief 步进电机软使能配置
* @param
*stpm：电机控制结构体
curstage：软使能台阶数
inttime：间隔时间
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_SoftEnableSet(STPMStruDef *stpm, const uint32_t curstage, const uint32_t inttime)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 14;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 5; //台阶数
  CoverU32ToU8_B(curstage, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->CurStage = curstage;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 14;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 6; //间隔
  CoverU32ToU8_B(inttime, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->IntTime = inttime;
  return 0;
}

/**
* @brief 步进电机细分配置
* @param
*stpm：电机控制结构体
microstep：细分参数,取值256、128、64、32、16、8、4、2、1
fullsteprnd：转动一圈的全步值
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_MicroStepSet(STPMStruDef *stpm, const uint32_t microstep, const uint32_t fullsteprnd)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 15;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 0; //细分设置
  CoverU32ToU8_B(microstep, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->MicroStep = microstep;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 15;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 1; //全步设置
  CoverU32ToU8_B(fullsteprnd, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  return 0;
}

/**
* @brief 步进电机运行方向入配置
* @param
*stpm：电机控制结构体
enable：0--正向，1--反向
polarity：
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_RunDirSet(STPMStruDef *stpm, const uint8_t dir)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 15;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 2; //方向
  CoverU32ToU8_B(dir, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }

  return 0;
}

/**
* @brief 步进电机ZeroWait写入
* @param
*stpm：电机控制结构体
param：待写入的参数
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_ZeroWaitSet(STPMStruDef *stpm, const uint32_t param)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 15;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 3; //过零延迟
  CoverU32ToU8_B(param, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->ZeroWait = param;
  return 0;
}

/**
* @brief 步进电机编码器单元配置
* @param
*stpm：电机控制结构体
line：编码器线数
dir：编码器与电机旋转方向关系 0-同向 1-反向
mode：编码器模式 0-差分模式 1-单端模式
ratiose：编码器轴端齿数
ratiosm：电机轴端齿数
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_EncoderBasicSet(STPMStruDef *stpm, const uint16_t line, const uint8_t dir,
                              const uint8_t mode,
                              const uint16_t ratiose, const uint16_t ratiosm)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 4;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 16;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 0; //编码器线数
  CoverU16ToU8_B(line, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->EncoderLine = line;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 16;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 1; //编码器方向
  CoverU16ToU8_B(dir, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->EncoderDir = dir;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 16;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 2; //模式
  CoverU16ToU8_B(mode, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->EncoderDir = mode;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 16;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 3; //编码器端齿数
  CoverU16ToU8_B(ratiose, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->RatioSE = ratiose;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 16;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 4; //电机轴端齿数
  CoverU16ToU8_B(ratiosm, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->RatioSM = ratiosm;
  return 0;
}

uint8_t StpmN_EncoderBasicSetReg1(STPMStruDef *stpm, const uint16_t line,
                                  const uint16_t ratiose, const uint16_t ratiosm)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 4;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 16;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 0; //编码器线数
  CoverU16ToU8_B(line, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->EncoderLine = line;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 16;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 3; //编码器端齿数
  CoverU16ToU8_B(ratiose, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->RatioSE = ratiose;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 16;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 4; //电机轴端齿数
  CoverU16ToU8_B(ratiosm, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->RatioSM = ratiosm;
  return 0;
}

uint8_t StpmN_EncoderBasicSetReg2(STPMStruDef *stpm, const uint8_t dir, const uint8_t mode)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 4;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 16;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 1; //编码器方向
  CoverU16ToU8_B(dir, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->EncoderDir = dir;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 16;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 2; //模式
  CoverU16ToU8_B(mode, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->EncoderDir = mode;

  return 0;
}

/**
* @brief 步进电机闭环控制参数配置
* @param
*stpm：电机控制结构体
stabletime：稳定时间，单位ms UI16
allowpositindiff：允许位置差，对应于编码器 UI16
stablecnt：允许闭环修正稳定次数 UI16
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CloseParaSet(STPMStruDef *stpm, const uint16_t stabletime,
                           const uint16_t stablecnt, const uint16_t allowpositindiff)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 4;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 17;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 0; //整定间隔
  CoverU16ToU8_B(stabletime, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->StableTime = stabletime;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 17;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 1; //整定次数
  CoverU16ToU8_B(stablecnt, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->StableCnt = stablecnt;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 17;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 2; //允许偏差
  CoverU16ToU8_B(allowpositindiff, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->AllowPositinDiff = allowpositindiff;
  return 0;
}

/**
* @brief 步进电机圆周运动模式配置
* @param
*stpm：电机控制结构体
driving：电机轴齿数
driven：从动轴齿数
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CircularSet(STPMStruDef *stpm, const uint16_t driving, const uint16_t driven)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 4;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 18;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 0; //电机轴
  CoverU16ToU8_B(driving, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->DrivingRatio = driving;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 18;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 1; //从动轴
  CoverU16ToU8_B(driven, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->DrivenRatio = driven;
  return 0;
}

/**
* @brief 步进电机斜坡模式写入
* @param
*stpm：电机控制结构体
slopeindex：斜坡数据索引，取值0-15分别对应1-16号斜坡
slopemode：斜坡模式，0--无斜坡，1--梯形斜坡，2--S形斜坡
posmode：位置模式，0--速度模式，1--位置模式
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_SlopeSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint8_t slopemode, const uint8_t posmode)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 19;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= slopeindex;
  tx[2] = 1;
  CoverU32ToU8_B(posmode, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Slope[slopeindex].PosModeEnable = posmode;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));

  tx[0] = 19;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= slopeindex;
  tx[2] = 2;
  CoverU32ToU8_B(slopemode, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Slope[slopeindex].SlopeMode = slopemode;
  return 0;
}

/**
* @brief 步进电机Vstart写入
* @param
*stpm：电机控制结构体
slopeindex：斜坡数据索引，取值0-15分别对应1-16号斜坡
param：待写入的参数
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_VstartSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 19;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= slopeindex;
  tx[2] = 3;
  CoverU32ToU8_B(param, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Slope[slopeindex].Vstart = param;
  return 0;
}

/**
* @brief 步进电机Vstop写入
* @param
*stpm：电机控制结构体
slopeindex：斜坡数据索引，取值0-15分别对应1-16号斜坡
param：待写入的参数
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_VstoptSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 19;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= slopeindex;
  tx[2] = 15;
  CoverU32ToU8_B(param, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Slope[slopeindex].Vstop = param;
  return 0;
}

/**
* @brief 步进电机Vbreak写入
* @param
*stpm：电机控制结构体
slopeindex：斜坡数据索引，取值0-4分别对应1-5号斜坡
param：待写入的参数
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_VbreaktSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 19;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= slopeindex;
  tx[2] = 5;
  CoverU32ToU8_B(param, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Slope[slopeindex].Vbreak = param;
  return 0;
}

/**
* @brief 步进电机Amax写入
* @param
*stpm：电机控制结构体
slopeindex：斜坡数据索引，取值0-4分别对应1-5号斜坡
param：待写入的参数
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_AmaxtSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 19;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= slopeindex;
  tx[2] = 8;
  CoverU32ToU8_B(param, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Slope[slopeindex].Amax = param;
  return 0;
}

/**
* @brief 步进电机Dmax写入
* @param
*stpm：电机控制结构体
slopeindex：斜坡数据索引，取值0-4分别对应1-5号斜坡
param：待写入的参数
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_DmaxtSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 19;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= slopeindex;
  tx[2] = 10;
  CoverU32ToU8_B(param, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Slope[slopeindex].Dmax = param;
  return 0;
}

/**
* @brief 步进电机Astart写入
* @param
*stpm：电机控制结构体
slopeindex：斜坡数据索引，取值0-4分别对应1-5号斜坡
param：待写入的参数
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_AstarttSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 19;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= slopeindex;
  tx[2] = 4;
  CoverU32ToU8_B(param, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Slope[slopeindex].Astart = param;
  return 0;
}

/**
* @brief 步进电机Dfinal写入
* @param
*stpm：电机控制结构体
slopeindex：斜坡数据索引，取值0-4分别对应1-5号斜坡
param：待写入的参数
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_DfinaltSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 19;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= slopeindex;
  tx[2] = 14;
  CoverU32ToU8_B(param, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Slope[slopeindex].Dfinal = param;
  return 0;
}

/**
* @brief 步进电机Bow1写入
* @param
*stpm：电机控制结构体
slopeindex：斜坡数据索引，取值0-15分别对应1-16号斜坡
param：待写入的参数
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_Bow1tSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 19;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= slopeindex;
  tx[2] = 17;
  CoverU32ToU8_B(param, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Slope[slopeindex].Bow1 = param;
  return 0;
}

/**
* @brief 步进电机Bow2写入
* @param
*stpm：电机控制结构体
slopeindex：斜坡数据索引，取值0-15分别对应1-16号斜坡
param：待写入的参数
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_Bow2tSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 19;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= slopeindex;
  tx[2] = 18;
  CoverU32ToU8_B(param, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Slope[slopeindex].Bow2 = param;
  return 0;
}

/**
* @brief 步进电机Bow3写入
* @param
*stpm：电机控制结构体
slopeindex：斜坡数据索引，取值0-15分别对应1-16号斜坡
param：待写入的参数
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_Bow3tSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 19;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= slopeindex;
  tx[2] = 19;
  CoverU32ToU8_B(param, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Slope[slopeindex].Bow3 = param;
  return 0;
}

/**
* @brief 步进电机Bow4写入
* @param
*stpm：电机控制结构体
slopeindex：斜坡数据索引，取值0-15分别对应1-16号斜坡
param：待写入的参数
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_Bow4tSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 19;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= slopeindex;
  tx[2] = 20;
  CoverU32ToU8_B(param, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Slope[slopeindex].Bow4 = param;
  return 0;
}

/**
* @brief 斜坡数据确认并使能
* @param
*stpm：电机控制结构体
slopeindex：斜坡数据索引，取值0-15分别对应1-16号斜坡
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_SlopeCheck(STPMStruDef *stpm, const uint8_t slopeindex)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 19;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= slopeindex;
  tx[2] = 0;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  return 0;
}

/**
* @brief 步进电机Vmax写入
* @param
*stpm：电机控制结构体
speedindex:速度数据索引，取值0-15分别对应1-16号速度
param：待写入的参数
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_VmaxtSet(STPMStruDef *stpm, const uint8_t speedindex, const int_fast32_t param)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 20;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= speedindex;
  CoverI32ToU8_B(param, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Vmax[speedindex] = param;
  return 0;
}

/**
* @brief 步进电机SSL输入配置
* @param
*stpm：电机控制结构体
enable：停止开关使能，0--不使能，1--使能
polarity：触发极性，0--低电平，1--高电平
latchen:锁存使能，0--不使能，1--使能
latchcfg:锁存配置，0--锁存跳变有效电平，1--锁存跳变无效电平
dmax:减速度值，触发停机的减速度
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_SSLSet(STPMStruDef *stpm, const uint8_t enable, const uint8_t polarity, const uint8_t latchen, const uint8_t latchcfg, const uint32_t dmax)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 21;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 0; //ssl

  if (polarity == 1) //高电平
  {
    UserSetBit(tx[2], 7);
  }
  if (enable == 1)
  {
    UserSetBit(tx[2], 6);
  }
  if (latchen == 1)
  {
    UserSetBit(tx[2], 5);
  }
  if (latchcfg == 1)
  {
    UserSetBit(tx[2], 4);
  }
  CoverU32ToU8_B(dmax, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  return 0;
}

/**
* @brief 步进电机SSR输入配置
* @param
*stpm：电机控制结构体
enable：停止开关使能，0--不使能，1--使能
polarity：触发极性，0--低电平，1--高电平
latchen:锁存使能，0--不使能，1--使能
latchcfg:锁存配置，0--锁存跳变有效电平，1--锁存跳变无效电平
dmax:减速度值，触发停机的减速度
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_SSRSet(STPMStruDef *stpm, const uint8_t enable, const uint8_t polarity, const uint8_t latchen, const uint8_t latchcfg, const uint32_t dmax)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 21;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 1; //ssr

  if (polarity == 1) //高电平
  {
    UserSetBit(tx[2], 7);
  }
  if (enable == 1)
  {
    UserSetBit(tx[2], 6);
  }
  if (latchen == 1)
  {
    UserSetBit(tx[2], 5);
  }
  if (latchcfg == 1)
  {
    UserSetBit(tx[2], 4);
  }
  CoverU32ToU8_B(dmax, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  return 0;
}

/**
* @brief 步进电机EMG输入配置
* @param
*stpm：电机控制结构体
enable：急停开关使能，0--不使能，1--使能
polarity：触发极性，0--低电平，1--高电平
filter:滤波值，范围0-9，值越小越灵敏
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_SemgSet(STPMStruDef *stpm, const uint8_t enable, const uint8_t polarity, const uint8_t filter)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 21;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 2; //semg

  if (polarity == 1) //高电平
  {
    UserSetBit(tx[2], 7);
  }
  if (enable == 1)
  {
    UserSetBit(tx[2], 3);
  }
  CoverU32ToU8_B(filter, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->EmgEnable = enable;
  stpm->EmgPolarity = polarity;
  stpm->EmgFilter = filter;
  return 0;
}

/**
* @brief 步进电机边沿检测计数器配置
* @param
*stpm：电机控制结构体
polarity：计数光电传感器触发极性，=0表示不进行记录，=1表示记录上升沿，=2表示记录下降沿，=3表示上升沿和下降沿均记录
scnt：需要更改的计数值
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_ScntSet(STPMStruDef *stpm, const uint8_t polarity, const uint32_t scnt)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 21;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 3; //scnt

  tx[2] |= (polarity << 1);
  tx[2] |= 0x01;
  CoverU32ToU8_B(scnt, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->PulseCnt = scnt;
  stpm->PulsePolarity = polarity;
  return 0;
}

/**
* @brief 步进电机单次运行超时配置
* @param
*stpm：电机控制结构体
mode：超时停止模式，0-超时后缓停，1-超时后急停
time：超时时间，单位ms
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_TimeoutSet(STPMStruDef *stpm, const uint8_t mode, const uint32_t time)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 23;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= (mode & 0x01);

  CoverU32ToU8_B(time, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->TimeOutMode = mode;
  stpm->TimeOut = time;
  return 0;
}

/**
* @brief 步进电机实际位置写入
* @param
*stpm：电机控制结构体
xactual：实际位置
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_XactualSet(STPMStruDef *stpm,const int_fast32_t xactual)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 24;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 0; //实际位置
  CoverI32ToU8_B(xactual, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Xactual = xactual;

  return 0;
}

/**
* @brief 步进电机目标位置写入
* @param
*stpm：电机控制结构体
xtarget：目标位置
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_XtargetSet(STPMStruDef *stpm, const int_fast32_t xtarget)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 24;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 1; //目标位置
  CoverI32ToU8_B(xtarget, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Xtarget = xtarget;

  return 0;
}

/**
* @brief 步进电机编码器位置写入
* @param
*stpm：电机控制结构体
encoderpos:写入编码器目标位置值
* @retval
0--ok，1--fail
* @note
*/

/**
* @brief 步进电机编码器位置写入
* @param
*stpm：电机控制结构体
encoderpos:写入编码器目标位置值
* @retval
0--ok，1--fail
* @note
*/

uint8_t StpmN_EncoderPositionSet(STPMStruDef *stpm, const int32_t encoderpos)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 24;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 2; //编码器位移
  CoverI32ToU8_B(encoderpos, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);

  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }

  return 0;
}

/**
* @brief 步进电机速度写入
* @param
*stpm：电机控制结构体
Vmax：速度
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_VmaxSet(STPMStruDef *stpm, const int_fast32_t Vmax)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 24;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 3; //速度
  CoverI32ToU8_B(Vmax, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  return 0;
}

/**
* @brief 步进电机运动轨迹记录功能配置
* @param
*stpm：电机控制结构体
stpmindex：电机驱动的索引，取值0-3对应驱动1-4#
enable：使能控制，0--不使能轨迹记录，1--使能轨迹记录
savemode：保存模式，0--单次模式，1--循环模式
datamode：数据选择，0--记录加速度运动轨迹，1--记录编码器位置轨迹
* @retval 
0--ok，1--fail
* @note 
*/
uint8_t StpmN_RCDConfig(STPMStruDef *stpm, const uint8_t enable, const uint8_t savemode, const uint8_t datamode)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 25;
  tx[1] |= stpm->DrvIndex << 4;
  if (enable == 1)
  {
    UserSetBit(tx[1], 3);
  }
  if (savemode == 1)
  {
    UserSetBit(tx[1], 2);
  }
  if (datamode == 1)
  {
    UserSetBit(tx[1], 1);
  }
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    return 1;
  }
  if (rx[1] != 0)
  {
    return 1;
  }
  return 0;
}

/**
* @brief 步进电机回HOME速度配置
* @param
*stpm：电机控制结构体
slopeindex：回home调用的斜坡
speed：回home调用的速度，符号为方向
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_GoHomeSpeedSet(STPMStruDef *stpm, const uint8_t slopeindex, const int_fast32_t speed)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  uint8_t dir = 0;
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 26;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 0; //斜坡索引
  CoverU32ToU8_B(slopeindex, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->HomeSlope = slopeindex;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 26;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 1; //回HOME速度
  CoverU32ToU8_B(speed, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Vhome = speed;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 26;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 2; //回HOME方向
  if (speed < 0)
  {
    dir = 0;
  }
  else
  {
    dir = 1;
  }
  CoverU32ToU8_B(dir, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->HomeDir = dir;
  return 0;
}

/**
* @brief 步进电机Home偏移量、回Home清除编码器数据的等待时间设置
* @param
*stpm：电机控制结构体
offset：home传感器相对0位的偏移量
cirdelay:回Home清除编码器数据的等待时间
sdebounce：防抖位移量
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_HomeOffsetSet(STPMStruDef *stpm, const uint32_t offset, const uint32_t cirdelay, const uint32_t sdebounce)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 26;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 3; //偏移量
  CoverU32ToU8_B(offset, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->HomeOffset = offset;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 26;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 4; //稳定时间
  CoverU32ToU8_B(cirdelay, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->HomeCirDelay = cirdelay;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 26;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 5; //防抖位移量
  CoverU32ToU8_B(sdebounce, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->HomeSdebounce = sdebounce;
  return 0;
}

/**
* @brief 步进电机组合运动1位置配置
* @param
*stpm：电机控制结构体
clmode：0开环运行，1闭环运行
spdchgpos：变速位置
finalpos:最终位置
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CM1PosSet(STPMStruDef *stpm, const uint8_t clmode, const int_fast32_t spdchgpos, const int_fast32_t finalpos)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 30;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 0; //运行模式
  tx[2] = 1;  //CM1
  CoverU32ToU8_B((clmode & 0x01), &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->CM1Mode = (clmode & 0x01);

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 30;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 1; //变速位置
  tx[2] = 1;  //CM1
  CoverI32ToU8_B(spdchgpos, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->CM1SpdChgPos = spdchgpos;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 30;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 2; //最终位置
  tx[2] = 1;  //CM1
  CoverI32ToU8_B(finalpos, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->CM1FinalPos = finalpos;
  return 0;
}

/**
* @brief 步进电机组合运动1速度配置
* @param
*stpm：电机控制结构体
slopeindex：调用的斜坡编号
spd1：第一段速度
spd2：第二段速度
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CM1SpeedSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t spd1, const uint32_t spd2)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 30;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 3; //斜坡
  tx[2] = 1;  //CM1
  CoverU32ToU8_B((slopeindex & 0x0f), &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->CM1Slope = (slopeindex & 0x0f);

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 30;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 4; //第一段速度
  tx[2] = 1;  //CM1
  CoverU32ToU8_B(spd1, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->CM1Spd1 = spd1;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 30;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 5; //第二段速度
  tx[2] = 1;  //CM1
  CoverU32ToU8_B(spd2, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->CM1Spd2 = spd2;
  return 0;
}

/**
* @brief 步进电机组合运动2位置配置
* @param
*stpm：电机控制结构体
clmode：0开环运行，1闭环运行
spdchgpos：变速位置
finalpos:最终位置
shieldpos:屏蔽位置
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CM2PosSet(STPMStruDef *stpm, const uint8_t clmode, const int_fast32_t spdchgpos, const int_fast32_t finalpos, const int_fast32_t shieldpos)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 30;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 0; //运行模式
  tx[2] = 2;  //CM2
  CoverU32ToU8_B((clmode & 0x01), &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->CM2Mode = (clmode & 0x01);

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 30;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 1; //变速位置
  tx[2] = 2;  //CM2
  CoverI32ToU8_B(spdchgpos, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->CM2SpdChgPos = spdchgpos;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 30;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 2; //最终位置
  tx[2] = 2;  //CM2
  CoverI32ToU8_B(finalpos, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->CM2FinalPos = finalpos;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 30;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 7; //屏蔽位置
  tx[2] = 2;  //CM2
  CoverI32ToU8_B(shieldpos, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->CM2ShieldPos = shieldpos;
  return 0;
}

/**
* @brief 步进电机组合运动2速度配置
* @param
*stpm：电机控制结构体
slopeindex：调用的斜坡编号
spd1：第一段速度
spd2：第二段速度
dec：触发传感器后的减速度，0为急停
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CM2SpeedSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t spd1, const uint32_t spd2, const uint32_t dec)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 30;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 3; //斜坡
  tx[2] = 2;  //CM2
  CoverU32ToU8_B((slopeindex & 0x0f), &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->CM2Slope = (slopeindex & 0x0f);

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 30;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 4; //第一段速度
  tx[2] = 2;  //CM2
  CoverU32ToU8_B(spd1, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->CM2Spd1 = spd1;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 30;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 5; //第二段速度
  tx[2] = 2;  //CM2
  CoverU32ToU8_B(spd2, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->CM2Spd2 = spd2;

  memset(rx, 0, sizeof(rx));
  memset(tx, 0, sizeof(tx));
  tx[0] = 30;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 6; //减速度
  tx[2] = 2;  //CM2
  CoverU32ToU8_B(dec, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->CM2Dec = dec;
  return 0;
}

/******************************************************************************/
/*                 运动控制指令                                                */
/******************************************************************************/
/**
* @brief 初始化直流电机模块
* @param
*dcm：电机控制结构体
* @retval
0--ok，1--fail
* @note
*/
uint8_t DcmN_DrvInit(DCMStruDef *dcm)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 31;
  tx[1] = 0x40;
  FDCAN1_CommunicateHandle(timeout, dcm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  return 0;
}

/**
* @brief 直流电机运行控制
* @param
*dcm：电机控制结构体
speed：速度（当开环模式表示PWM）
* @retval
0--ok，1--fail
* @note
*/
uint8_t DcmN_BDCSpeedSet(DCMStruDef *dcm, const float speed)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  if (dcm->Mode != 1)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  tx[0] = 31;
  CoverFloatToU8_B(speed, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, dcm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  dcm->TargetParam = speed;
  return 0;
}

/**
* @brief 电磁铁通断电配置
* @param
*dcm：电机控制结构体
state=0(OFF),state=1(ON)
* @retval
0--ok，1--fail
* @note
*/
uint8_t DcmN_ElectromagnetSet(DCMStruDef *dcm, const uint8_t state)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  if (dcm->Mode != 2)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  tx[0] = 31;
  if (state == 0)
  {
    tx[1] = 0;
  }
  else
  {
    tx[1] = 0x20;
  }
  FDCAN1_CommunicateHandle(timeout, dcm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  return 0;
}

/**
* @brief 无刷电机速度配置
* @param
*dcm：电机控制结构体
brk：0--break关闭，1--break打开
speed：速度（当开环模式表示PWM）
* @retval
0--ok，1--fail
* @note
*/
uint8_t DcmN_BLDCSpeedSet(DCMStruDef *dcm, const uint8_t brk, const float speed)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 6;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  if (dcm->Mode != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  tx[0] = 31;
  if (brk == 0)
  {
    tx[1] = 0;
  }
  else
  {
    tx[1] = 0x10;
  }
  CoverFloatToU8_B(speed, &tx[2]);
  FDCAN1_CommunicateHandle(timeout, dcm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  dcm->TargetParam = speed;
  return 0;
}

/**
* @brief 步进电机驱动复位/参数激活
* @param
*stpm：电机控制结构体
operation：0-复位步进驱动，1-驱动参数激活
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_DrvRstInit(STPMStruDef *stpm, const uint8_t operation)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收

  //memset(rx, 0, 8);
  //memset(tx, 0, 8);

  tx[0] = 32;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= (operation & 0x01);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  return 0;
}

/**
* @brief 步进电机输出使能/释放
* @param
*stpm：电机控制结构体
operation：0-释放步进电机的输出，1-使能步进电机的输出
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_OutEnableSet(STPMStruDef *stpm, const uint8_t operation)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收

  StpmN_DrvStateGet(stpm);

  if (operation == 0)
  {

    if (stpm->WorkState_N == 0 || stpm->WorkState_N == 1)
    {
      return 0;
    }
    else if (stpm->WorkState_N == 3)
    {
      return StpmN_Stop(stpm, 1, 1, 0);
    }
  }
  else
  {
    if (stpm->WorkState_N == 2)
    {
      return 0;
    }
    else if (stpm->WorkState_N == 3)
    {
      return StpmN_Stop(stpm, 1, 0, 0);
    }
    else if (stpm->WorkState_N == 0)
    {
      return 1;
    }
  }

  tx[0] = 32;
  tx[1] |= stpm->DrvIndex << 4;
  if (operation == 0)
  {
    tx[1] |= 3;
  }
  else if (operation == 1)
  {
    tx[1] |= 2;
  }
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }

  //等待使能完成
  if (operation == 1)
  {
    while (1)
    {
      vTaskDelay(5);
      StpmN_DrvStateGet(stpm);
      if (stpm->WorkState_N == 2)
      {
        break;
      }
    }
  }
  return 0;
}

/**
* @brief 步进电机运行
* @param
*stpm：电机控制结构体
clmode：0开环模式，1闭环模式
cirmode：0直线模式，1圆周模式
relmode：0绝对位置模式，1相对位置模式
slopeindex：斜坡数据索引，取值0-15分别对应1-16号斜坡
speedindex:速度数据索引，取值0-15分别对应1-16号速度
xtarget:目标位置/相对运行位置
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_Run(STPMStruDef *stpm, const uint8_t clmode,
                  const uint8_t cirmode, const uint8_t relmode,
                  const uint8_t slopeindex, const uint8_t speedindex,
                  const uint32_t xtarget)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 7;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 33;
  tx[1] |= stpm->DrvIndex << 4;
  if (clmode == 1)
  {
    UserSetBit(tx[1], 3);
  }
  if (cirmode == 1)
  {
    UserSetBit(tx[1], 2);
  }
  if (relmode == 1)
  {
    UserSetBit(tx[1], 1);
  }
  tx[2] = (((slopeindex & 0x0f) << 4) | (speedindex & 0x0f));
  CoverU32ToU8_B(xtarget, &tx[3]);
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
   // //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Xtarget = xtarget;
  return 0;
}

/**
* @brief 步进电机停止
* @param
*stpm：电机控制结构体
hardstop：0缓停，1急停
release：0停止后不释放，1停止释放
warn：0不触发告警，1触发告警
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_Stop(STPMStruDef *stpm, const uint8_t hardstop, const uint8_t release, const uint8_t warn)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 34;
  tx[1] |= stpm->DrvIndex << 4;
  if (hardstop == 1)
  {
    UserSetBit(tx[1], 3);
  }
  if (release == 1)
  {
    UserSetBit(tx[1], 2);
  }
  if (warn == 1)
  {
    UserSetBit(tx[1], 1);
  }
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  return 0;
}

/**
* @brief 步进电机电平输出
* @param
*stpm：电机控制结构体
output：0输出低电平，1输出高电平
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_StpOutSet(STPMStruDef *stpm, const uint8_t output)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 35;
  tx[1] |= stpm->DrvIndex << 4;
  if (output == 1)
  {
    UserSetBit(tx[1], 0);
  }
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  return 0;
}

/**
* @brief 步进电机回HOME指令配置
* @param
*stpm：电机控制结构体
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_GoHomeSet(STPMStruDef *stpm)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 36;
  tx[1] |= stpm->DrvIndex << 4;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  return 0;
}

/**
* @brief 步进电机回HOME状态查询
* @param
*stpm：电机控制结构体
*status：回Home状态，0--操作已完成，1--操作执行中，2--操作失败
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_GoHomeGet(STPMStruDef *stpm, uint8_t *status)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 2;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 36;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 0x80;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  *status = rx[2] & 0x03;
  return 0;
}

/**
* @brief 组合运动1指令配置
* @param
*stpm：电机控制结构体
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CM1Set(STPMStruDef *stpm)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 3;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 40;
  tx[1] |= stpm->DrvIndex << 4;
  tx[2] = 1;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Xtarget = stpm->CM1FinalPos;
  return 0;
}

/**
* @brief 组合运动1状态查询
* @param
*stpm：电机控制结构体
*status：组合运动1状态，0--操作已完成，1--操作执行中，2--操作失败
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CM1Get(STPMStruDef *stpm, uint8_t *status)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 3;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 40;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 0x80;
  tx[2] = 1;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  *status = rx[3] & 0x03;
  return 0;
}

/**
* @brief 组合运动2指令配置
* @param
*stpm：电机控制结构体
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CM2Set(STPMStruDef *stpm)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 3;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 40;
  tx[1] |= stpm->DrvIndex << 4;
  tx[2] = 2;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  if (rx[1] != 0)
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  stpm->Xtarget = stpm->CM2FinalPos;
  return 0;
}

/**
* @brief 组合运动2状态查询
* @param
*stpm：电机控制结构体
*status：组合运动1状态，0--操作已完成，1--操作执行中，2--操作失败
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CM2Get(STPMStruDef *stpm, uint8_t *status)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 3;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 40;
  tx[1] |= stpm->DrvIndex << 4;
  tx[1] |= 0x80;
  tx[2] = 2;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  *status = rx[3] & 0x03;
  return 0;
}

/**
* @brief 组合运动指令1锁存位置读取
* @param
*stpm：电机控制结构体
*latchpos：锁存位置
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CM1LatchPosGet(STPMStruDef *stpm, int_fast32_t *latchpos)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 4;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 41;
  tx[1] |= stpm->DrvIndex << 4;
  tx[2] = 1;
  tx[3] = 0;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  *latchpos = CoverU8ToI32_B(&rx[4]);
  return 0;
}

/**
* @brief 组合运动指令1实际停止位置读取
* @param
*stpm：电机控制结构体
*stoppos：实际停止位置
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CM1StopPosGet(STPMStruDef *stpm, int_fast32_t *stoppos)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 4;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 41;
  tx[1] |= stpm->DrvIndex << 4;
  tx[2] = 1;
  tx[3] = 1;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  *stoppos = CoverU8ToI32_B(&rx[4]);
  return 0;
}

/**
* @brief 组合运动指令1变速完成位置读取
* @param
*stpm：电机控制结构体
*spd：实际停止位置
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CM1SpdChgedPosGet(STPMStruDef *stpm, int_fast32_t *spdchgedpos)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 4;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 41;
  tx[1] |= stpm->DrvIndex << 4;
  tx[2] = 1;
  tx[3] = 2;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  *spdchgedpos = CoverU8ToI32_B(&rx[4]);
  return 0;
}

/**
* @brief 组合运动指令2触发位置读取
* @param
*stpm：电机控制结构体
*latchpos：锁存位置
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CM2TriggerPosGet(STPMStruDef *stpm, int_fast32_t *triggerpos)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 4;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 41;
  tx[1] |= stpm->DrvIndex << 4;
  tx[2] = 2;
  tx[3] = 0;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  *triggerpos = CoverU8ToI32_B(&rx[4]);
  return 0;
}

/**
* @brief 组合运动指令2实际停止位置读取
* @param
*stpm：电机控制结构体
*stoppos：实际停止位置
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CM2StopPosGet(STPMStruDef *stpm, int_fast32_t *stoppos)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 4;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 41;
  tx[1] |= stpm->DrvIndex << 4;
  tx[2] = 2;
  tx[3] = 1;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  *stoppos = CoverU8ToI32_B(&rx[4]);
  return 0;
}

/**
* @brief 组合运动指令2变速完成位置读取
* @param
*stpm：电机控制结构体
*spd：实际停止位置
* @retval
0--ok，1--fail
* @note
*/
uint8_t StpmN_CM2SpdChgedPosGet(STPMStruDef *stpm, int_fast32_t *spdchgedpos)
{
  uint32_t timeout = 10; //超时计数10ms
  uint8_t txlen = 4;     //发送的帧长度
  uint8_t tx[8] = {0};   //发送
  uint8_t rxlen = 0;     //接收的帧长度
  uint8_t rx[8] = {0};   //接收
  //memset(rx, 0, 8);
  //memset(tx, 0, 8);
  tx[0] = 41;
  tx[1] |= stpm->DrvIndex << 4;
  tx[2] = 2;
  tx[3] = 2;
  FDCAN1_CommunicateHandle(timeout, stpm->BoardID, txlen, tx, &rxlen, rx);
  if (rx[0] != tx[0])
  {
    //User_Error_Handler(__FILE__, __LINE__, 26100);
    return 1;
  }
  *spdchgedpos = CoverU8ToI32_B(&rx[4]);
  return 0;
}

/**
  * @brief 等待普通运行完成
  * @param  *stpm：电机控制结构体
            timeout：超时时间，单位ms
  * @retval 0--ok;1--commfail;2--timeout;3--runfail
*/
uint8_t WaitingNormalCplt(STPMStruDef *stpm, uint32_t timeout)
{
  uint32_t timecnt = 0;
  for (;;)
  {
    if (timeout != 0)
    {
      timecnt++;
      if ((timecnt * 10) > timeout)
      {
        return 2;
      }
      vTaskDelay(10);
    }
    if (StpmN_DrvStateGet(stpm) != 0)
    {
      return 1;
    }
    else
    {
      if (stpm->Error == 1)
      {
        return 3;
      }
      if (stpm->WorkState_N == 2)
      {
        break;
      }
      else if (timeout == 0)
      {
        return 2;
      }
    }
  }
  return 0;
}

/**
  * @brief 等待闭环运行完成
  * @param  *stpm：电机控制结构体
            timeout：超时时间，单位ms
  * @retval 0--ok;1--commfail;2--timeout;3--runfail
*/
uint8_t WaitingCLCplt(STPMStruDef *stpm, uint32_t timeout)
{
  uint32_t timecnt = 0;
  for (;;)
  {
    if (timeout != 0)
    {
      if ((timecnt * 10) > timeout)
      {
        return 2;
      }
      timecnt++;
      vTaskDelay(10);
    }
    if (StpmN_DrvStateGet(stpm) != 0)
    {
      return 1;
    }
    else
    {
      if (stpm->Error == 1)
      {
        return 3;
      }
      if (stpm->WorkState_N == 2)
      {
        break;
      }
      else if (timeout == 0)
      {
        return 2;
      }
    }
  }
  if (stpm->ClpState_N == 1)
  {
    return 3;
  }
  return 0;
}

/**
  * @brief 等待CM1运行完成
  * @param  *stpm：电机控制结构体
            timeout：超时时间，单位ms
  * @retval 0--ok;1--commfail;2--timeout;3--runfail
*/
uint8_t WaitingCM1Cplt(STPMStruDef *stpm, uint32_t timeout)
{
  uint32_t timecnt = 0;
  uint8_t excutor_status = 1;
  int_fast32_t postemp = INT32_MAX;
  for (;;)
  {
    if (timeout != 0)
    {
      if ((timecnt * 10) > timeout)
      {
        return 2;
      }
      timecnt++;
      vTaskDelay(10);
    }
    if (StpmN_CM1Get(stpm, &excutor_status) != 0)
    {
      return 1;
    }
    else
    {
      if (excutor_status == 2)
      {
        return 3;
      }
      else if (excutor_status == 0)
      {
        break;
      }
      else if (timeout == 0)
      {
        return 2;
      }
    }
  }
  if (StpmN_DrvStateGet(stpm) != 0)
  {
    return 1;
  }
  if (stpm->Error == 1)
  {
    return 3;
  }
  if (StpmN_CM1LatchPosGet(stpm, &postemp) != 0)
  {
    return 1;
  }
  if (postemp == INT32_MAX) //没有所存到位置
  {
    if ((stpm->StopL.InptState) != (stpm->Set.hom_pol)) //不在HOME位
    {
      return 3;
    }
    else
    {
      return 0;
    }
  }
  //检查运行位置
  else if ((abs(abs(postemp) - stpm->Set.hom_dev)) > stpm->Set.hom_drf)
  {
    return 3;
  }
  else
  {
    return 0;
  }
}
#if 0
/**
* @brief 电机参数初始化
* @param
* @retval
0--ok，1--fail
* @note
*/
uint8_t MotorBoardConfig(MotorStruDef *motors)
{
  static STPMStruDef WASHP3_Motor;
  StpmDrvParamInit(&WASHP3_Motor, motors);

  /* 电机参数初始化注意点

  step 1 复位驱动器
  step 2 设置参数
  step 3 激活参数
  

  设置斜坡：
  设置参数后需要 docheck
  
  */

  for (;;)
  {
    /*步进驱动复位*/
    if (StpmN_DrvRstInit(&WASHP3_Motor, 0) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    for (;;)
    {
      StpmN_DrvStateGet(&WASHP3_Motor);
      vTaskDelay(10);
      if (WASHP3_Motor.WorkState_N == 0)
        break;
    }

    /*斜坡模式设置，梯形-六点模式，位移模式*/
    if (StpmN_SlopeSet(&WASHP3_Motor, 0, 1, 1) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor: StepMotor Error");
      break;
    }

    /*vstart,vstop,vbreak,amax,dmax,astart,dfinal数据写入*/
    if (StpmN_VstartSet(&WASHP3_Motor, 0, WASHP3_Motor.Set.slo[0].Vstart) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    //WASHP3_Motor.Set.slo[0].Vstop = motors->Vstop;
    if (StpmN_VstoptSet(&WASHP3_Motor, 0, WASHP3_Motor.Set.slo[0].Vstop) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    //WASHP3_Motor.Set.slo[0].Vbreak = motors->Vbreak;
    if (StpmN_VbreaktSet(&WASHP3_Motor, 0, WASHP3_Motor.Set.slo[0].Vbreak) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    //WASHP3_Motor.Set.slo[0].Amax = motors->AMax;
    if (StpmN_AmaxtSet(&WASHP3_Motor, 0, WASHP3_Motor.Set.slo[0].Amax) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    // WASHP3_Motor.Set.slo[0].Dmax = motors->Dmax;
    if (StpmN_DmaxtSet(&WASHP3_Motor, 0, WASHP3_Motor.Set.slo[0].Dmax) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    //WASHP3_Motor.Set.slo[0].Astart = motors->Astart;
    if (StpmN_AstarttSet(&WASHP3_Motor, 0, WASHP3_Motor.Set.slo[0].Astart) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    //WASHP3_Motor.Set.slo[0].Dfinal = motors->Dfinal;
    if (StpmN_DfinaltSet(&WASHP3_Motor, 0, WASHP3_Motor.Set.slo[0].Dfinal) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    /*斜坡数据确认并使能*/
    if (StpmN_SlopeCheck(&WASHP3_Motor, 0) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    /*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/

    /*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
    /*斜坡模式设置，梯形-六点模式，位移模式*/
    if (StpmN_SlopeSet(&WASHP3_Motor, 1, 1, 1) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor: StepMotor Error");
      break;
    }
    /*vstart,vstop,vbreak,amax,dmax,astart,dfinal数据写入*/
    if (StpmN_VstartSet(&WASHP3_Motor, 1, WASHP3_Motor.Set.slo[1].Vstart) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    if (StpmN_VstoptSet(&WASHP3_Motor, 1, WASHP3_Motor.Set.slo[1].Vstop) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    if (StpmN_VbreaktSet(&WASHP3_Motor, 1, WASHP3_Motor.Set.slo[1].Vbreak) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    if (StpmN_AmaxtSet(&WASHP3_Motor, 1, WASHP3_Motor.Set.slo[1].Amax) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    if (StpmN_DmaxtSet(&WASHP3_Motor, 1, WASHP3_Motor.Set.slo[1].Dmax) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    if (StpmN_AstarttSet(&WASHP3_Motor, 1, WASHP3_Motor.Set.slo[1].Astart) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    if (StpmN_DfinaltSet(&WASHP3_Motor, 1, WASHP3_Motor.Set.slo[1].Dfinal) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    /*斜坡数据确认并使能*/
    if (StpmN_SlopeCheck(&WASHP3_Motor, 1) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }

    /*Vmax写入*/
    for (uint8_t idx = 0; idx < 5; idx++)
    {
      if (StpmN_VmaxtSet(&WASHP3_Motor, idx, WASHP3_Motor.Set.v_max[idx]) == 1)
      {
        MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
        break;
      }
    }
    /*过零延迟写入，100us过零延迟时间*/
    if (StpmN_ZeroWaitSet(&WASHP3_Motor, 100) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    /*电流参数配置，默认切换延时1000us*/
    if (StpmN_CurrentSet(&WASHP3_Motor, WASHP3_Motor.Set.drv_i, WASHP3_Motor.Set.hld_i, 1000) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    /*细分参数配置*/
    if (StpmN_MicroStepSet(&WASHP3_Motor, WASHP3_Motor.Set.stp_d, WASHP3_Motor.Set.stp_r) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    /*回Home专用斜坡模式设置，T-6P模式，位移模式*/
    if (StpmN_SlopeSet(&WASHP3_Motor, 4, 1, 1) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor: StepMotor Error");
      break;
    }
    /*vstart,vstop,vbreak,amax,dmax,astart,dfinal数据写入*/
    if (StpmN_VstartSet(&WASHP3_Motor, 4, WASHP3_Motor.Set.slo[4].Vstart) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    if (StpmN_VstoptSet(&WASHP3_Motor, 4, WASHP3_Motor.Set.slo[4].Vstop) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    if (StpmN_VbreaktSet(&WASHP3_Motor, 4, WASHP3_Motor.Set.slo[4].Vbreak) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    if (StpmN_AmaxtSet(&WASHP3_Motor, 4, WASHP3_Motor.Set.slo[4].Amax) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    if (StpmN_DmaxtSet(&WASHP3_Motor, 4, WASHP3_Motor.Set.slo[4].Dmax) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    if (StpmN_AstarttSet(&WASHP3_Motor, 4, WASHP3_Motor.Set.slo[4].Astart) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    if (StpmN_DfinaltSet(&WASHP3_Motor, 4, WASHP3_Motor.Set.slo[4].Dfinal) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    /*回Home专用斜坡数据确认并使能*/
    if (StpmN_SlopeCheck(&WASHP3_Motor, 4) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    /*回Home速度配置*/
    if (StpmN_GoHomeSpeedSet(&WASHP3_Motor, 4, WASHP3_Motor.Set.v_home) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    /*Home偏移量设置*/
    if (StpmN_HomeOffsetSet(&WASHP3_Motor, WASHP3_Motor.Set.hom_dev, 0, WASHP3_Motor.Set.hom_eli) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    /*Home传感器极性配置*/
    if (StpmN_SSLSet(&WASHP3_Motor, 0, WASHP3_Motor.Set.hom_pol, 0, 0, 0) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    /*组合1运行（快速回0）速度斜坡配置*/
    if (StpmN_CM1SpeedSet(&WASHP3_Motor, 0, 0, 0) != 0)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    /*组合1运行（快速回0）位置配置*/
    if (StpmN_CM1PosSet(&WASHP3_Motor, 0, 0, 0) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    /*参数激活*/
    if (StpmN_DrvRstInit(&WASHP3_Motor, 1) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }
    /*电机输出使能*/
    if (StpmN_OutEnableSet(&WASHP3_Motor, 1) == 1)
    {
      MotorWorkRecordReadWrite(0, 0, __LINE__, "user_excutor:StepMotor Error");
      break;
    }

    /*正常返回*/
    return 0;
  }
  /*出错跳出到这里*/
  return 1;
}

// StpmN_Run
#endif

/*****************************END OF FILE*****************************/
