/**
******************************************************************************
* @文件    user_motor.H
* @作者    
* @版本    
******************************************************************************
新驱动板接口
V0.1.1-20210730
仅用于接口开发
******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_MOTOR_H
#define __USER_MOTOR_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "user_gpm.h"




  /* Exported types ------------------------------------------------------------*/
  typedef struct
  {
    uint8_t Enable;    //使能标志，0--不使能，1--使能
    uint8_t EffLevel;  //有效电平，0--低电平，1--高电平
    uint8_t InptState; //输入状态，0--低电平，1--高电平
  } StopSwitch;

  typedef struct
  {
    uint8_t SlopeMode;     //斜坡形式，0--无斜坡，1--梯形斜坡，2--S形斜坡
    uint8_t PosModeEnable; //位置模式使能，0--不使能，1--使能
    uint32_t Vstart;       //起始速度值
    uint32_t Vstop;        //停止速度
    uint32_t Vbreak;       //断点速度
    uint32_t Amax;         //最大加速度
    uint32_t Dmax;         //最大减速度
    uint32_t Astart;       //起始加速度
    uint32_t Dfinal;       //末端减速度
    uint32_t Bow1;         //弓形值1~4
    uint32_t Bow2;
    uint32_t Bow3;
    uint32_t Bow4;
  } SlopeStruDef;

  typedef struct
  {
    SlopeStruDef slo[5]; //斜坡参数
    uint8_t slos;        //配置的斜坡数
    int32_t v_max[5];    //最大速度
    uint8_t vmaxs;       //配置的速度数
    float drv_i;         //驱动电流：0~3.3
    float hld_i;         //保持电流：0~3.3
    uint32_t stp_d;      //细分数
    uint16_t stp_r;      //电机整圈整步数
    int32_t v_home;      //回home速度
    int32_t hom_dev;     //home偏移量，触发home方向光耦多走距离，逻辑home位
    uint8_t hom_pol;     //传感器极性：0-低电平触发；1-高电平触发；
    uint32_t hom_eli;    //回home远离光耦距离，消抖
    uint32_t hom_drf;    //快速回home偏离误差允许位移(OLD_NU)
    uint8_t hom_otf;     //快速回home修正使能：0-直接报错；1-自动修正(OLD_NU)
    uint8_t r_ena;       //圆周使能(OLD_NU)
    uint32_t r_stp;      //圆周整圈步(OLD_NU)
    uint16_t r_driving;  //圆周驱动轴比例(NEW)1
    uint16_t r_driven;   //圆周从动轴比例(NEW)?
    uint8_t e_ena;       //编码器使能(OLD_NU)
    uint16_t e_line;     //编码器线数(NEW)1000
    uint16_t e_ratioe;   //编码器轴比例(NEW)1
    uint16_t e_ratiom;   //编码器电机轴比例(NEW)1
    uint8_t e_dir;       //编码器方向
    uint8_t e_mod;       //编码器模式：0-差分；1-单端；
    int32_t e_res;       //从动轴转一圈对应编码器转动圈数*编码器分辨率(OLD_NU)
    float e_stp;         //步进电机单圈微步数/(编码器分辨率*4)(OLD_NU)
    uint16_t e_sdt;      //闭环稳定时间
    uint16_t e_sdn;      //闭环稳定次数
    uint16_t e_drf;      //闭环允许稳定偏差
    uint8_t sto_ena;     //堵转检测使能
    int32_t sto_v;       //堵转检测起始速度
    uint8_t pus_mod;     //传感器计数模式：0-不进行记录，1-记录上升沿，2-记录下降沿，3-上升沿和下降沿均记录

  } Motset_StrDef;

  typedef struct
  {
    uint8_t BoardID;   //所在板卡的ID
    uint8_t DrvIndex;  //驱动接口索引,取值0-3对应驱动接口1-4
    uint8_t WorkState; //工作状态，0--RESET，1--READY，2--RUN，3--ERR(OLD_NU)

    uint8_t WorkState_N; //工作状态，0--RESET，1--Released，2--STOP，3--Working(NEW)
    uint8_t RunMode;     //上一次运行的模式，0--空，1--回home，2--CM1，3--CM2，4--普通运行，5--闭环运行

    uint8_t Error;      //错误状态(NEW)
    uint8_t Warning;    //告警状态(NEW)
    uint16_t ErrorCode; //错误代码

    /*-----------------------------------------------------------------------*/
    StopSwitch StopL;      //左停开关
    StopSwitch StopR;      //右停开关
    uint8_t HardStop;      //0--软停机，1--硬停机
    uint32_t DStop;        //软停机调用的减速度
    uint8_t LatchEnable;   //0--不使能锁存，1--使能锁存STOPL，2--使能STOPR
    uint8_t LatchPolarity; //0--锁存有效电平，1--锁存非有效电平
    int_fast32_t XLatch;   //锁存位置，只读
    uint8_t STPOut;        //电平输出端口，0--低电平，1--高电平
    uint8_t PulsePolarity; //计数光电传感器触发极性，=0表示不进行记录，=1表示记录上升沿，=2表示记录下降沿，=3表示上升沿和下降沿均记录
    uint32_t PulseCnt;     //计数光电传感器计数值
    uint8_t PulseState;    //计数传感器当前状态
    /*-----------------------------------------------------------------------*/
    SlopeStruDef Slope[16]; //预制斜坡
    float DrvScl;           //驱动电流，0-3.3A，分辨率3.3/256A
    float HOLDScl;          //保持电流，0-3.3A，分辨率3.3/256A
    float StdDelay;         //驱动电流切换到保持电流的延时时间，单位us，分辨率50ns

    uint32_t CurStage; //释放到使能阶段电流抬升阶数
    uint32_t IntTime;  //释放到使能抬升间隔

    uint32_t MicroStep; //细分参数，取值256、128、64、32、16、8、4、2、1
    /*-----------------------------------------------------------------------*/
    uint8_t CIREnable;     //圆周运动使能，0--不使能，1--使能
    uint32_t Xrange;       //圆周运动一圈的步数
    uint16_t DrivingRatio; //电机轴齿数比
    uint16_t DrivenRatio;  //从动轴齿数比
    /*-----------------------------------------------------------------------*/
    int_fast32_t Xactual;  //当前实际位置
    int_fast32_t Vactual;  //当前实际速度，只读
    int_fast32_t Aactual;  //当前实际加速度，只读
    int_fast32_t Vmax[16]; //最大速度
    int_fast32_t Xtarget;  //目标位置
    uint32_t ZeroWait;     //过零延迟，单位us
    /*-----------------------------------------------------------------------*/
    uint8_t EmgEnable;   //急停开关使能，0--不使能，1--使能
    uint8_t EmgPolarity; //触发极性，0--下降沿，1--上升沿，2--下降沿保持，3--上升沿保持
    uint8_t EmgState;    //引脚状态，0--低电平，1--高电平
    uint8_t EmgFilter;   //滤波设置，0-9
    uint8_t TimeOutMode; //超时停止模式，0-超时后缓停，1-超时后急停
    uint32_t TimeOut;    //运行限时，0--不设置，X--设置速度不为0时间为Xms
    /*-----------------------------------------------------------------------*/
    uint8_t AutoStop;       //堵转自停功能
    int_fast32_t StopSpeed; //监控堵转自停的起始速度值
    /*-----------------------------------------------------------------------*/
    uint8_t HomeSlope;       //回Home的斜坡
    int_fast32_t Vhome;      //回Home的速度
    uint8_t HomeDir;         //回Home的方向
    int_fast32_t HomeOffset; //Home位置与传感器之间的偏差
    uint32_t HomeCirDelay;   //回Home清除编码器数据的等待时间
    uint32_t HomeSdebounce;  //防抖位移量

    uint8_t HomePolarity; //Home传感器电平，0--低电平，1--高电平
    uint8_t Autohome;     //自动回Home，0--不自动回Home，1--自动回Home
    uint32_t ZeroErr;     //传感器与homeoffset间的最大允许误差

    /*-----------------------------------------------------------------------*/
    uint8_t CM1Mode;           //运行模式
    int_fast32_t CM1SpdChgPos; //变速位置
    int_fast32_t CM1FinalPos;  //最终位置
    uint8_t CM1Slope;          //调用的斜坡编号
    uint32_t CM1Spd1;          //第一段速度
    uint32_t CM1Spd2;          //第二段速度

    uint8_t CM2Mode;           //运行模式
    int_fast32_t CM2SpdChgPos; //变速位置
    int_fast32_t CM2FinalPos;  //最终位置
    int_fast32_t CM2ShieldPos; //屏蔽位
    uint8_t CM2Slope;          //调用的斜坡编号
    uint32_t CM2Spd1;          //第一段速度
    uint32_t CM2Spd2;          //第二段速度
    uint32_t CM2Dec;           //触发传感器的减速度
    /*-----------------------------------------------------------------------*/
    uint8_t EncoderMode;  //编码器工作基础模式，0--差分，1--单端
    uint8_t EncoderDir;   //编码器运行方向，0--同向，1--反向
    int32_t EncoderRes;   //从动轴转一圈对应编码器转动圈数*编码器分辨率。
    uint16_t EncoderLine; //编码器线数
    uint16_t RatioSE;     //编码器轴端齿数
    uint16_t RatioSM;     //电机轴端齿数
    float StepRatio;      //表示步进电机单圈微步数/（编码器分辨率*4）

    uint16_t StableTime;       //稳定时间，单位ms
    uint16_t StableCnt;        //最大稳定次数
    uint16_t AllowPositinDiff; //允许位置差，1/（编码器分辨率*4）

    int32_t XtargetCL; //编码器目标位置，编码器给出的脉冲值

    uint8_t speedStatus; //速度到达预设速度标志

    uint8_t ClpCtrlState; //闭环运行状态，0--关闭，1--打开

    uint8_t ClpState;   //闭环稳定状态，0--稳定，1--未稳定
    uint8_t ClpState_N; //闭环稳定状态，0--稳定，1--错误

    int32_t EncoderPos; //编码器位置，编码器当前的实际脉冲值

    Motset_StrDef Set; //设置参数
  } STPMStruDef;       //步进电机控制参数，只读，当运行相应的程序后，根据运行结果自动更新

  typedef struct
  {
    uint8_t w_mod;  //工作模式，0 = 直流无刷，1 = 直流有刷，2 = 电磁
    uint8_t c_mod;  //控制模式，0 = 开环，1 = 闭环(仅无刷支持)
    float v_max[5]; //最大速度，开环以PWM配置，闭环以r/min配置，取值-100~100
    float CurThr;   //直流电机电流阈值
    uint16_t kp;    //比例值，取值0~5000，实际效果0~50
    uint16_t ki;    //积分值，取值0~5000，实际效果0~50
    uint16_t ep;    //调节偏差比例，取值0~50000，实际效果0~50
  } Dmtset_StrDef;

  typedef struct
  {
    uint8_t BoardID;   //所在板卡的ID
    uint8_t WorkState; //工作状态，0--RESET，1--READY，2--RUN，3--ERR

    uint8_t WorkState_N; //工作状态，0--RESET，1--SLEEP，2--RUN，3--Brake
    uint8_t Error;
    uint8_t Warning;

    uint8_t Mode;      //0--直流无刷，1--直流有刷，2--电磁铁
    uint8_t CLMode;    //闭环控制模式，0--开环，1--闭环
    uint8_t ErrorCode; //错误代码
    float ActualParam; //实际参数，当属于电机模式时，表示闭环速度或开环PWM，当属于电磁铁模式时，0--低电平，非0表示高电平，只读
    /*-----------------------------------------------------------------------*/
    uint16_t KP; //比例项
    uint16_t KI; //积分项
    uint16_t EP; //偏差值

    uint8_t PPR; //单圈脉冲
    /*-----------------------------------------------------------------------*/
    float TargetParam; //目标参数
    uint8_t CLState;   //闭环稳定性
    /*-----------------------------------------------------------------------*/
    float ActualAmp; //实际电流
    float TSAmp;     //电流控制阈值

    Dmtset_StrDef Set; //设置参数
  } DCMStruDef;        //直流电机控制参数，只读，当运行相应的程序后，根据运行结果自动更新

  typedef struct
  {
    float duty;      // 输出占空比，范围：0.00~100.00
    uint16_t speed;  //实际转速
    uint8_t fanmode; //反馈模式  =0：堵转反馈模式；=1：速度反馈模式
  } FanCtrlStruDef;  //风扇控制和状态读取

  /* Exported variables ---------------------------------------------------------*/
  

  /* Exported functions ------------------------------------------------------- */
  /******************************************************************************/
  /*                 电机驱动板公共代码                                         */
  /******************************************************************************/
  //New
  uint8_t FDCAN1_CommunicateHandle(uint32_t timeout, uint8_t boardid, uint8_t txlen, uint8_t *tx, uint8_t *rxlen, uint8_t *rx);
      uint8_t BoardN_Heartbeat(void);
  uint8_t BoardN_Reboot(uint8_t BoardID);
  uint8_t StpmN_DrvStateGet(STPMStruDef *stpm);
  uint8_t StpmN_EncoderPositionRead(STPMStruDef *stpm);
  uint8_t StpmN_XactualGet(STPMStruDef *stpm);
  uint8_t StpmN_VactualGet(STPMStruDef *stpm);
  uint8_t StpmN_AactualGet(STPMStruDef *stpm);
  uint8_t StpmN_ScntGet(STPMStruDef *stpm);
  uint8_t DcmN_DrvStateGet(DCMStruDef *dcm);
  uint8_t DcmN_RunAmpGet(DCMStruDef *dcm);
  uint8_t BoardN_VersionGet(uint8_t BoardID, uint8_t *Ver);
  uint8_t BoardN_ErrorCodeGet(DCMStruDef *dcm, STPMStruDef *stpm, uint8_t mode);
  uint8_t StpmN_RCDRead(STPMStruDef *stpm, const uint16_t dataindex, int_fast32_t *shift, int_fast32_t *speed, int_fast32_t *acc_encoder);

  uint8_t BoardN_HeartbeatSet(uint8_t BoardID, uint8_t stopmode, uint8_t releasemode, uint32_t HBtimeout);
  uint8_t DcmN_DrvModeSet(DCMStruDef *dcm, const uint8_t mode, const uint8_t clmode, const uint16_t kp, const uint16_t ki, const uint8_t ppr);
  uint8_t DcmN_ThAmpSet(DCMStruDef *dcm, const float amp);
  uint8_t StpmN_CurrentSet(STPMStruDef *stpm, const float drvcurrent, const float hldcurrent, const uint32_t stddelay);
  uint8_t StpmN_SoftEnableSet(STPMStruDef *stpm, const uint32_t curstage, const uint32_t inttime);
  uint8_t StpmN_MicroStepSet(STPMStruDef *stpm, const uint32_t microstep, const uint32_t fullsteprnd);
  uint8_t StpmN_RunDirSet(STPMStruDef *stpm, const uint8_t dir);
  uint8_t StpmN_ZeroWaitSet(STPMStruDef *stpm, const uint32_t param);
  uint8_t StpmN_EncoderBasicSetReg2(STPMStruDef *stpm, const uint8_t dir, const uint8_t mode);
  uint8_t StpmN_EncoderBasicSetReg1(STPMStruDef *stpm, const uint16_t line,
                                    const uint16_t ratiose, const uint16_t ratiosm);
  uint8_t StpmN_EncoderBasicSet(STPMStruDef *stpm, const uint16_t line, const uint8_t dir, const uint8_t mode, const uint16_t ratiose, const uint16_t ratiosm);
  uint8_t StpmN_CloseParaSet(STPMStruDef *stpm, const uint16_t stabletime, const uint16_t stablecnt, const uint16_t allowpositindiff);
  uint8_t StpmN_CircularSet(STPMStruDef *stpm, const uint16_t driving, const uint16_t driven);
  uint8_t StpmN_SlopeSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint8_t slopemode, const uint8_t posmode);
  uint8_t StpmN_VstartSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param);
  uint8_t StpmN_VstoptSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param);
  uint8_t StpmN_VbreaktSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param);
  uint8_t StpmN_AmaxtSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param);
  uint8_t StpmN_DmaxtSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param);
  uint8_t StpmN_AstarttSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param);
  uint8_t StpmN_DfinaltSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param);
  uint8_t StpmN_Bow1tSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param);
  uint8_t StpmN_Bow2tSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param);
  uint8_t StpmN_Bow3tSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param);
  uint8_t StpmN_Bow4tSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t param);
  uint8_t StpmN_SlopeCheck(STPMStruDef *stpm, const uint8_t slopeindex);
  uint8_t StpmN_VmaxtSet(STPMStruDef *stpm, const uint8_t speedindex, const int_fast32_t param);
  uint8_t StpmN_SSLSet(STPMStruDef *stpm, const uint8_t enable, const uint8_t polarity, const uint8_t latchen, const uint8_t latchcfg, const uint32_t dmax);
  uint8_t StpmN_SSRSet(STPMStruDef *stpm, const uint8_t enable, const uint8_t polarity, const uint8_t latchen, const uint8_t latchcfg, const uint32_t dmax);
  uint8_t StpmN_SemgSet(STPMStruDef *stpm, const uint8_t enable, const uint8_t polarity, const uint8_t filter);
  uint8_t StpmN_ScntSet(STPMStruDef *stpm, const uint8_t polarity, const uint32_t scnt);
  uint8_t StpmN_TimeoutSet(STPMStruDef *stpm, const uint8_t mode, const uint32_t time);
  uint8_t StpmN_XactualSet(STPMStruDef *stpm, const int_fast32_t xactual);
  uint8_t StpmN_XtargetSet(STPMStruDef *stpm, const int_fast32_t xtarget);
  uint8_t StpmN_EncoderPositionSet(STPMStruDef *stpm, const int32_t encoderpos);
  uint8_t StpmN_VmaxSet(STPMStruDef *stpm, const int_fast32_t Vmax);
  uint8_t StpmN_RCDConfig(STPMStruDef *stpm, const uint8_t enable, const uint8_t savemode, const uint8_t datamode);
  uint8_t StpmN_GoHomeSpeedSet(STPMStruDef *stpm, const uint8_t slopeindex, const int_fast32_t speed);
  uint8_t StpmN_HomeOffsetSet(STPMStruDef *stpm, const uint32_t offset, const uint32_t cirdelay, const uint32_t sdebounce);
  uint8_t StpmN_CM1PosSet(STPMStruDef *stpm, const uint8_t clmode, const int_fast32_t spdchgpos, const int_fast32_t finalpos);
  uint8_t StpmN_CM1SpeedSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t spd1, const uint32_t spd2);
  uint8_t StpmN_CM2PosSet(STPMStruDef *stpm, const uint8_t clmode, const int_fast32_t spdchgpos, const int_fast32_t finalpos, const int_fast32_t shieldpos);
  uint8_t StpmN_CM2SpeedSet(STPMStruDef *stpm, const uint8_t slopeindex, const uint32_t spd1, const uint32_t spd2, const uint32_t dec);

  uint8_t DcmN_DrvInit(DCMStruDef *dcm);
  uint8_t DcmN_BDCSpeedSet(DCMStruDef *dcm, const float speed);
  uint8_t DcmN_ElectromagnetSet(DCMStruDef *dcm, const uint8_t state);
  uint8_t DcmN_BLDCSpeedSet(DCMStruDef *dcm, const uint8_t brk, const float speed);
  uint8_t StpmN_DrvRstInit(STPMStruDef *stpm, const uint8_t operation);
  uint8_t StpmN_OutEnableSet(STPMStruDef *stpm, const uint8_t operation);
  uint8_t StpmN_Run(STPMStruDef *stpm, const uint8_t clmode, const uint8_t cirmode, const uint8_t relmode, const uint8_t slopeindex, const uint8_t speedindex, const uint32_t xtarget);
  uint8_t StpmN_Stop(STPMStruDef *stpm, const uint8_t hardstop, const uint8_t release, const uint8_t warn);
  uint8_t StpmN_StpOutSet(STPMStruDef *stpm, const uint8_t output);
  uint8_t StpmN_GoHomeSet(STPMStruDef *stpm);
  uint8_t StpmN_GoHomeGet(STPMStruDef *stpm, uint8_t *status);
  uint8_t StpmN_CM1Set(STPMStruDef *stpm);
  uint8_t StpmN_CM1Get(STPMStruDef *stpm, uint8_t *status);
  uint8_t StpmN_CM2Set(STPMStruDef *stpm);
  uint8_t StpmN_CM2Get(STPMStruDef *stpm, uint8_t *status);
  uint8_t StpmN_CM1LatchPosGet(STPMStruDef *stpm, int_fast32_t *latchpos);
  uint8_t StpmN_CM1StopPosGet(STPMStruDef *stpm, int_fast32_t *stoppos);
  uint8_t StpmN_CM1SpdChgedPosGet(STPMStruDef *stpm, int_fast32_t *spdchgedpos);
  uint8_t StpmN_CM2TriggerPosGet(STPMStruDef *stpm, int_fast32_t *triggerpos);
  uint8_t StpmN_CM2StopPosGet(STPMStruDef *stpm, int_fast32_t *stoppos);
  uint8_t StpmN_CM2SpdChgedPosGet(STPMStruDef *stpm, int_fast32_t *spdchgedpos);

  uint8_t WaitingNormalCplt(STPMStruDef *stpm, uint32_t timeout);
  uint8_t WaitingCLCplt(STPMStruDef *stpm, uint32_t timeout);
  uint8_t WaitingCM1Cplt(STPMStruDef *stpm, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif
/*****************************END OF FILE*****************************/
