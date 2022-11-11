/**
******************************************************************************
* @文件    user_data.C
* @作者    GENGXU
* @版本    V0.0.1
******************************************************************************
本文件定义了整机运行控制参数的保存方法，提供了参数保存和读取的接口


存储器映射：（相对地址）
Block1：
0x0000000~0x0000003     uint32_t 类型数据，表示当前Block已编程次数。
0x0000004~0x000000f     保留区域
0x0000010~BlockSizeMax  格式“Vx.x.x.xxxx”，未使用区域保留

Block2：
0x0000000~0x0000003     uint32_t 类型数据，表示当前Block已编程次数。
0x0000004~0x000000f     保留区域
0x0000010~BlockSizeMax  IP地址、网关地址、掩码、外部CAN网络ID

Block3：
0x0000000~0x0000003     uint32_t 类型数据，表示当前Block已编程次数。
0x0000004~0x000000f     保留区域
0x0000010~BlockSizeMax  仪器编号（20位ASCII）、主控板序号（20位ASCII）

Block4：
0x0000000~0x0000003     uint32_t 类型数据，表示当前Block已编程次数。
0x0000004~0x000000f     保留区域
0x0000010~BlockSizeMax  仪器运行参数

******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "user_data.h"
#include "stm32h743i_eval_nor.h"
#include "user_app.h"
#include "user_excutor.h"

extern uint8_t SetMotorParamFromPC(const uint16_t motorname, StepMotorParams *params);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BlockSize 0x20000 //128KB模式
//#define BlockSize        0x10000//64KW模式
//#define FWInfoAddr         NOR_DEVICE_ADDR
//#define CommInfoAddr      (NOR_DEVICE_ADDR+1*BlockSize)
//#define MacInfoAddr       (NOR_DEVICE_ADDR+2*BlockSize)
//#define ParamInfoAddr     (NOR_DEVICE_ADDR+3*BlockSize)

#define NORFLASHREAD 1
#define NORFLASHWRITE 0

#define FWInfoAddr 0
#define CommInfoAddr (1 * BlockSize)
#define MacInfoAddr (2 * BlockSize)
#define ParamInfoAddr (3 * BlockSize)

#define ParamInADDR (ParamInfoAddr + 0x10)

#define MotorParamAddr (20 * BlockSize)
#define MotorParamOffset (BlockSize)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint32_t TrackNumber;
/*Static IP ADDRESS*/
extern uint32_t IP_ADDR0;
extern uint32_t IP_ADDR1;
extern uint32_t IP_ADDR2;
extern uint32_t IP_ADDR3;

/*NETMASK*/
extern uint32_t NETMASK_ADDR0;
extern uint32_t NETMASK_ADDR1;
extern uint32_t NETMASK_ADDR2;
extern uint32_t NETMASK_ADDR3;

/*Gateway Address*/
extern uint32_t GW_ADDR0;
extern uint32_t GW_ADDR1;
extern uint32_t GW_ADDR2;
extern uint32_t GW_ADDR3;

/*CAN ID设置*/
//extern uint32_t ExCanID;

/*参数*/
//extern ChainInStruDef ChainIn;
//extern ChainBackStruDef ChainBack;
extern volatile TrackStruDef Track[MaxTrackNumber];
extern volatile TableStruDef Table;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  芯片检验
  检查芯片是否被正确编程过
  检查芯片中的程序版本是否和代码中的程序版本（ProjectVersion）一致
  * @param  None
  * @retval 0--ok,1--fail,2--芯片数据不合法
  */
uint8_t UserFlashCheck(void)
{
  union
  {
    uint8_t a[4];
    uint16_t b[2];
    uint32_t c;
  } Param32;

  /*Block3*/
  if (BSP_NOR_ReadData(0, MacInfoAddr, &Param32.b[0], 2) != BSP_ERROR_NONE)
    return 1;
  if ((Param32.a[0] == 0xff) && (Param32.a[1] == 0xff) && (Param32.a[2] == 0xff) && (Param32.a[3] == 0xff))
    return 2;

  /*Block4*/
  if (BSP_NOR_ReadData(0, ParamInfoAddr, &Param32.b[0], 2) != BSP_ERROR_NONE)
    return 1;
  if ((Param32.a[0] == 0xff) && (Param32.a[1] == 0xff) && (Param32.a[2] == 0xff) && (Param32.a[3] == 0xff))
    return 2;

  return 0;
}

/**
  * @brief  程序版本号读写
  * @param  rw=0，表示写入，rw=1，表示读取
  * @retval 0--ok,1--fail
  */

/**
  * @brief  网络信息读写
  * @param  rw=0，表示写入，rw=1，表示读取
  * @retval 0--ok,1--fail
  */
uint8_t UserEthernetInfoRW(uint8_t rw)
{
  uint32_t BlockEraseCnt = 0;
  union
  {
    uint8_t a[4];
    uint16_t b[2];
    uint32_t c;
  } Param32;
  uint16_t datatmp[26] = {0};

  if (BSP_NOR_ReadData(0, CommInfoAddr, &Param32.b[0], 2) != BSP_ERROR_NONE)
    return 1;
  if ((Param32.a[0] == 0xff) && (Param32.a[1] == 0xff) && (Param32.a[2] == 0xff) && (Param32.a[3] == 0xff))
    BlockEraseCnt = 0;
  else
    BlockEraseCnt = Param32.c;

  if (rw == 0)
  {
    if (BSP_NOR_EraseBlock(0, CommInfoAddr) != BSP_ERROR_NONE)
      return 1;
    BlockEraseCnt++;
    Param32.c = BlockEraseCnt;
    if (BSP_NOR_WriteData(0, CommInfoAddr, &Param32.b[0], 2) != BSP_ERROR_NONE)
      return 1;

    Param32.c = IP_ADDR0;
    datatmp[0] = Param32.b[0];
    datatmp[1] = Param32.b[1];
    Param32.c = IP_ADDR1;
    datatmp[2] = Param32.b[0];
    datatmp[3] = Param32.b[1];
    Param32.c = IP_ADDR2;
    datatmp[4] = Param32.b[0];
    datatmp[5] = Param32.b[1];
    Param32.c = IP_ADDR3;
    datatmp[6] = Param32.b[0];
    datatmp[7] = Param32.b[1];

    Param32.c = NETMASK_ADDR0;
    datatmp[8] = Param32.b[0];
    datatmp[9] = Param32.b[1];
    Param32.c = NETMASK_ADDR1;
    datatmp[10] = Param32.b[0];
    datatmp[11] = Param32.b[1];
    Param32.c = NETMASK_ADDR2;
    datatmp[12] = Param32.b[0];
    datatmp[13] = Param32.b[1];
    Param32.c = NETMASK_ADDR3;
    datatmp[14] = Param32.b[0];
    datatmp[15] = Param32.b[1];

    Param32.c = GW_ADDR0;
    datatmp[16] = Param32.b[0];
    datatmp[17] = Param32.b[1];
    Param32.c = GW_ADDR1;
    datatmp[18] = Param32.b[0];
    datatmp[19] = Param32.b[1];
    Param32.c = GW_ADDR2;
    datatmp[20] = Param32.b[0];
    datatmp[21] = Param32.b[1];
    Param32.c = GW_ADDR3;
    datatmp[22] = Param32.b[0];
    datatmp[23] = Param32.b[1];

    if (BSP_NOR_WriteData(0, CommInfoAddr + 0x0000010, &datatmp[0], 26) != BSP_ERROR_NONE)
      return 1;
  }
  else
  {
    if (BSP_NOR_ReadData(0, CommInfoAddr + 0x0000010, &datatmp[0], 26) != BSP_ERROR_NONE)
      return 1;
    Param32.b[0] = datatmp[0];
    Param32.b[1] = datatmp[1];
    IP_ADDR0 = Param32.c;
    Param32.b[0] = datatmp[2];
    Param32.b[1] = datatmp[3];
    IP_ADDR1 = Param32.c;
    Param32.b[0] = datatmp[4];
    Param32.b[1] = datatmp[5];
    IP_ADDR2 = Param32.c;
    Param32.b[0] = datatmp[6];
    Param32.b[1] = datatmp[7];
    IP_ADDR3 = Param32.c;

    Param32.b[0] = datatmp[8];
    Param32.b[1] = datatmp[9];
    NETMASK_ADDR0 = Param32.c;
    Param32.b[0] = datatmp[10];
    Param32.b[1] = datatmp[11];
    NETMASK_ADDR1 = Param32.c;
    Param32.b[0] = datatmp[12];
    Param32.b[1] = datatmp[13];
    NETMASK_ADDR2 = Param32.c;
    Param32.b[0] = datatmp[14];
    Param32.b[1] = datatmp[15];
    NETMASK_ADDR3 = Param32.c;

    Param32.b[0] = datatmp[16];
    Param32.b[1] = datatmp[17];
    GW_ADDR0 = Param32.c;
    Param32.b[0] = datatmp[18];
    Param32.b[1] = datatmp[19];
    GW_ADDR1 = Param32.c;
    Param32.b[0] = datatmp[20];
    Param32.b[1] = datatmp[21];
    GW_ADDR2 = Param32.c;
    Param32.b[0] = datatmp[22];
    Param32.b[1] = datatmp[23];
    GW_ADDR3 = Param32.c;
  }
  return 0;
}

/**
  * @brief  设备信息读写
  * @param  rw=0，表示写入，rw=1，表示读取
  * @retval 0--ok,1--fail
  */
uint8_t UserMacInfoRW(uint8_t rw)
{
  uint32_t BlockEraseCnt = 0;
  uint16_t data16[42] = {0};
  char datach[42] = {0};
  union
  {
    uint8_t a[4];
    uint16_t b[2];
    uint32_t c;
  } Param32;

  if (BSP_NOR_ReadData(0, MacInfoAddr, &Param32.b[0], 2) != BSP_ERROR_NONE)
    return 1;
  if ((Param32.a[0] == 0xff) && (Param32.a[1] == 0xff) && (Param32.a[2] == 0xff) && (Param32.a[3] == 0xff))
    BlockEraseCnt = 0;
  else
    BlockEraseCnt = Param32.c;

  if (rw == 0)
  {
    if (BSP_NOR_EraseBlock(0, MacInfoAddr) != BSP_ERROR_NONE)
      return 1;
    BlockEraseCnt++;
    Param32.c = BlockEraseCnt;
    if (BSP_NOR_WriteData(0, MacInfoAddr, &Param32.b[0], 2) != BSP_ERROR_NONE)
      return 1;

    // strcat(datach,MacSN);
    // strcat(datach,CtrlBoardSN);
    for (uint32_t cnti = 0; cnti < 20; cnti++)
    {
      datach[cnti] = MacSN[cnti];
    }
    datach[20] = 0;
    for (uint32_t cnti = 0; cnti < 20; cnti++)
    {
      datach[cnti + 21] = CtrlBoardSN[cnti];
    }
    datach[41] = 0;
    for (uint32_t cnti = 0; cnti < 42; cnti++)
    {
      data16[cnti] = (uint16_t)datach[cnti];
    }
    if (BSP_NOR_WriteData(0, MacInfoAddr + 0x0000010, &data16[0], 42) != BSP_ERROR_NONE)
      return 1;
  }
  else
  {
    if (BSP_NOR_ReadData(0, MacInfoAddr + 0x0000010, &data16[0], 42) != BSP_ERROR_NONE)
      return 1;

    for (uint32_t cnti = 0; cnti < 21; cnti++)
    {
      datach[cnti] = (char)data16[cnti];
    }
    memset(MacSN, 0, 20);
    strcpy(MacSN, datach);
    for (uint32_t cnti = 0; cnti < 21; cnti++)
    {
      datach[cnti] = (char)data16[cnti + 21];
    }
    memset(CtrlBoardSN, 0, 20);
    strcpy(CtrlBoardSN, datach);
  }
  return 0;
}

/**
  * @brief  参数信息读写
  * @param  rw=0，表示写入，rw=1，表示读取
  * @retval 0--ok,1--fail
  */
#define TableParamnum 23
#define SingleTrackParamnum 29

uint8_t UserParamInfoRW(uint8_t rw)
{
  uint32_t Paramnum = 200; //TableParamnum + SingleTrackParamnum * MaxTrackNumber;
  uint32_t BlockEraseCnt = 0;
  uint16_t data16[2];
  uint32_t cntk = 0;
  union
  {
    uint8_t a[4];
    uint16_t b[2];
    uint32_t c;
  } Param32;

  union
  {
    uint16_t b[2];
    int_fast32_t c;
  } Param[200];

  Param32.c = 0;

  for (uint32_t cntx = 0; cntx < Paramnum; cntx++)
  {
    Param[cntx].c = 0;
  }

  if (BSP_NOR_ReadData(0, ParamInfoAddr, &Param32.b[0], 2) != BSP_ERROR_NONE)
    return 1;
  if ((Param32.a[0] == 0xff) && (Param32.a[1] == 0xff) && (Param32.a[2] == 0xff) && (Param32.a[3] == 0xff))
    BlockEraseCnt = 0;
  else
    BlockEraseCnt = Param32.c;

  if (rw == NORFLASHWRITE)
  {
    if (BSP_NOR_EraseBlock(0, ParamInfoAddr) != BSP_ERROR_NONE)
      return 1;
    BlockEraseCnt++;
    Param32.c = BlockEraseCnt;
    if (BSP_NOR_WriteData(0, ParamInfoAddr, &Param32.b[0], 2) != BSP_ERROR_NONE)
      return 1;
    cntk = 0;

    //进样链条
    Param[cntk++].c = ChainIn.Offset[0];
    Param[cntk++].c = ChainIn.Offset[1];

    //返回链条
    Param[cntk++].c = ChainBack.Offset[0][0];
    Param[cntk++].c = ChainBack.Offset[0][1];
    Param[cntk++].c = ChainBack.Offset[1][0];
    Param[cntk++].c = ChainBack.Offset[1][1];

    //进样推板电机（扫码）
    Param[cntk++].c = Table.PBI.Pos[1];
    Param[cntk++].c = Table.PBI.Pos[2];
    Param[cntk++].c = Table.PBI.Pos[3];
    Param[cntk++].c = Table.PBI.Pos[4];
    Param[cntk++].c = Table.PBI.Pos[5];
    Param[cntk++].c = Table.PBI.Pos[6];
    Param[cntk++].c = Table.PBI.Pos[7];
    Param[cntk++].c = Table.PBI.Pos[8];
    Param[cntk++].c = Table.PBI.Pos[9];
    Param[cntk++].c = Table.PBI.Pos[10];
    Param[cntk++].c = Table.PBI.Pos[11];
    Param[cntk++].c = Table.PBI.Pos[12];

    //进样推送电机
    Param[cntk++].c = Table.PPI.Pos[1];

    //返回推板电机
    Param[cntk++].c = Table.PBB.Pos[1];
    Param[cntk++].c = Table.PBB.Pos[2];

    //返回推送回收仓电机
    Param[cntk++].c = Table.PPB.Pos[1];

    cntk++;
    //轨道参数
    for (uint8_t Tid = 0; Tid < MaxTrackNumber; Tid++)
    {
      //等待挡板电机
      Param[cntk++].c = Track[Tid].WBF.Pos[1];

      //常规通道挡板电机
      Param[cntk++].c = Track[Tid].NPB.Pos[1];
      Param[cntk++].c = Track[Tid].NPB.Pos[2];
      Param[cntk++].c = Track[Tid].NPB.Pos[3];
      Param[cntk++].c = Track[Tid].NPB.Pos[4];
      Param[cntk++].c = Track[Tid].NPB.Pos[5];
      Param[cntk++].c = Track[Tid].NPB.Pos[6];
      Param[cntk++].c = Track[Tid].NPB.Pos[7];
      Param[cntk++].c = Track[Tid].NPB.Pos[8];
      Param[cntk++].c = Track[Tid].NPB.Pos[9];
      Param[cntk++].c = Track[Tid].NPB.Pos[10];
      Param[cntk++].c = Track[Tid].NPB.Pos[11];

      //急诊通道挡板电机
      Param[cntk++].c = Track[Tid].EPB.Pos[1];
      Param[cntk++].c = Track[Tid].EPB.Pos[2];
      Param[cntk++].c = Track[Tid].EPB.Pos[3];
      Param[cntk++].c = Track[Tid].EPB.Pos[4];
      Param[cntk++].c = Track[Tid].EPB.Pos[5];
      Param[cntk++].c = Track[Tid].EPB.Pos[6];
      Param[cntk++].c = Track[Tid].EPB.Pos[7];
      Param[cntk++].c = Track[Tid].EPB.Pos[8];
      Param[cntk++].c = Track[Tid].EPB.Pos[9];
      Param[cntk++].c = Track[Tid].EPB.Pos[10];

      //变轨电机
      Param[cntk++].c = Track[Tid].MTC.Pos[1];
      Param[cntk++].c = Track[Tid].MTC.Pos[2];
      Param[cntk++].c = Track[Tid].MTC.Pos[3];

      //变轨挡板电机
      Param[cntk++].c = Track[Tid].MBF.Pos[1];

      //返回通道阻挡推板电机
      Param[cntk++].c = Track[Tid].BPB.Pos[1];
      Param[cntk++].c = Track[Tid].BPB.Pos[2];
    }

    for (uint32_t cnti = 0; cnti < Paramnum; cnti++)
    {
      data16[0] = Param[cnti].b[0];
      data16[1] = Param[cnti].b[1];
      if (BSP_NOR_WriteData(0, ParamInADDR + cnti * 4, &data16[0], 2) != BSP_ERROR_NONE)
        return 1;
    }
  }
  else
  {
    for (uint32_t cnti = 0; cnti < Paramnum; cnti++)
    {
      if (BSP_NOR_ReadData(0, ParamInADDR + cnti * 4, &data16[0], 2) != BSP_ERROR_NONE)
        return 1;
      Param[cnti].b[0] = data16[0];
      Param[cnti].b[1] = data16[1];
    }

    cntk = 0;
    //进样链条
    ChainIn.Offset[0] = Param[cntk++].c;
    ChainIn.Offset[1] = Param[cntk++].c;

    //返回链条
    ChainBack.Offset[0][0] = Param[cntk++].c;
    ChainBack.Offset[0][1] = Param[cntk++].c;
    ChainBack.Offset[1][0] = Param[cntk++].c;
    ChainBack.Offset[1][1] = Param[cntk++].c;

    //进样推板电机（扫码）
    Table.PBI.Pos[1] = Param[cntk++].c;
    Table.PBI.Pos[2] = Param[cntk++].c;
    Table.PBI.Pos[3] = Param[cntk++].c;
    Table.PBI.Pos[4] = Param[cntk++].c;
    Table.PBI.Pos[5] = Param[cntk++].c;
    Table.PBI.Pos[6] = Param[cntk++].c;
    Table.PBI.Pos[7] = Param[cntk++].c;
    Table.PBI.Pos[8] = Param[cntk++].c;
    Table.PBI.Pos[9] = Param[cntk++].c;
    Table.PBI.Pos[10] = Param[cntk++].c;
    Table.PBI.Pos[11] = Param[cntk++].c;
    Table.PBI.Pos[12] = Param[cntk++].c;

    //进样推送电机
    Table.PPI.Pos[1] = Param[cntk++].c;

    //返回推板电机
    Table.PBB.Pos[1] = Param[cntk++].c;
    Table.PBB.Pos[2] = Param[cntk++].c;

    //返回推送回收仓电机
    Table.PPB.Pos[1] = Param[cntk++].c;

    //轨道数
    cntk++;

    //轨道参数
    for (uint8_t Tid = 0; Tid < MaxTrackNumber; Tid++)
    {
      //等待挡板电机
      Track[Tid].WBF.Pos[1] = Param[cntk++].c;

      //常规通道挡板电机
      Track[Tid].NPB.Pos[1] = Param[cntk++].c;
      Track[Tid].NPB.Pos[2] = Param[cntk++].c;
      Track[Tid].NPB.Pos[3] = Param[cntk++].c;
      Track[Tid].NPB.Pos[4] = Param[cntk++].c;
      Track[Tid].NPB.Pos[5] = Param[cntk++].c;
      Track[Tid].NPB.Pos[6] = Param[cntk++].c;
      Track[Tid].NPB.Pos[7] = Param[cntk++].c;
      Track[Tid].NPB.Pos[8] = Param[cntk++].c;
      Track[Tid].NPB.Pos[9] = Param[cntk++].c;
      Track[Tid].NPB.Pos[10] = Param[cntk++].c;
      Track[Tid].NPB.Pos[11] = Param[cntk++].c;

      //急诊通道挡板电机
      Track[Tid].EPB.Pos[1] = Param[cntk++].c;
      Track[Tid].EPB.Pos[2] = Param[cntk++].c;
      Track[Tid].EPB.Pos[3] = Param[cntk++].c;
      Track[Tid].EPB.Pos[4] = Param[cntk++].c;
      Track[Tid].EPB.Pos[5] = Param[cntk++].c;
      Track[Tid].EPB.Pos[6] = Param[cntk++].c;
      Track[Tid].EPB.Pos[7] = Param[cntk++].c;
      Track[Tid].EPB.Pos[8] = Param[cntk++].c;
      Track[Tid].EPB.Pos[9] = Param[cntk++].c;
      Track[Tid].EPB.Pos[10] = Param[cntk++].c;

      //变轨电机
      Track[Tid].MTC.Pos[1] = Param[cntk++].c;
      Track[Tid].MTC.Pos[2] = Param[cntk++].c;
      Track[Tid].MTC.Pos[3] = Param[cntk++].c;

      //变轨挡板电机
      Track[Tid].MBF.Pos[1] = Param[cntk++].c;

      //返回通道阻挡推板电机
      Track[Tid].BPB.Pos[1] = Param[cntk++].c;
      Track[Tid].BPB.Pos[2] = Param[cntk++].c;
    }
  }

  return 0;
}

static uint16_t subcheckcode(uint8_t *tmp, uint16_t length)
{

  uint16_t rts;
  rts = 0;

  for (uint16_t i = 0; i < length; i++)
  {
    rts += tmp[i];
  }
	
	return rts;
}




/**
  * @brief  参数信息读写
  * @param  rw=0，表示写入，rw=1，表示读取
  * @retval 0--ok,1--fail
  */

uint8_t UserMotorParamInfoRW(uint8_t rw, uint16_t motorname, StepMotorParams *tmp)
{
  uint16_t subcheck;

  uint16_t tmpbuff[1024]={0};

  SubCode errorcode;

  if (rw == 0)
  {
    memset(tmpbuff,0,sizeof(StepMotorParams));

    subcheck = subcheckcode((uint8_t *)tmp, sizeof(StepMotorParams));
    errorcode.High = (uint8_t)(subcheck >> 8);
    errorcode.Low = (uint8_t)(subcheck);

    memcpy(tmpbuff,tmp,sizeof(StepMotorParams));

    if (BSP_NOR_EraseBlock(0, MotorParamAddr + motorname * MotorParamOffset) != BSP_ERROR_NONE)
      return 1;

    if (BSP_NOR_WriteData(0, MotorParamAddr + motorname * MotorParamOffset, (uint16_t *)&errorcode, 1) != BSP_ERROR_NONE)
    {
      return 1;
    }

    if (BSP_NOR_WriteData(0, MotorParamAddr + motorname * MotorParamOffset + 0x10, (uint16_t *)tmpbuff, sizeof(StepMotorParams) / 2) != BSP_ERROR_NONE)
    {
      return 1;
    }
    else
    {
      memset(tmpbuff, 0, sizeof(StepMotorParams));
      if (BSP_NOR_ReadData(0, MotorParamAddr + motorname * MotorParamOffset + 0x10, (uint16_t *)tmpbuff, sizeof(StepMotorParams) / 2) != BSP_ERROR_NONE)
      {
        return 1;
      }

      return 0;
    }

  }
  else if (rw == 1)
  {

    if (BSP_NOR_ReadData(0, MotorParamAddr + motorname * MotorParamOffset , (uint16_t *)&errorcode,1) != BSP_ERROR_NONE)
    {
      return 1;
    }

    if (BSP_NOR_ReadData(0, MotorParamAddr + motorname * MotorParamOffset + 0x10, (uint16_t *)tmpbuff, sizeof(StepMotorParams) / 2) != BSP_ERROR_NONE)
    {
      return 1;
    }

    memcpy(tmp,tmpbuff,sizeof(StepMotorParams));

    subcheck = subcheckcode((uint8_t *)tmp, sizeof(StepMotorParams));

    if ((errorcode.High != (uint8_t)(subcheck >> 8)) && errorcode.Low != (uint8_t)(subcheck))
    {
      SetMotorParamFromPC(motorname, tmp);
      return 1;
    }
    else
    {
      SetMotorParamFromPC(motorname, tmp);
      return 0;

    }
  }

  return 0;
}

/*****************************END OF FILE*****************************/
