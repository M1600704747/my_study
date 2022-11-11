/**
******************************************************************************
* @文件    user_excutor.H
* @作者    GENGXU
* @版本    V0.0.1
******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_EXCUTOR_H
#define __USER_EXCUTOR_H

/* Includes ------------------------------------------------------------------*/
#include "user_gpm.h"
#include "user_can.h"
/* Private types ------------------------------------------------------------*/

typedef struct
{
    __IO uint8_t Mode;      //0速度1位移2无效3闭环位移    斜坡模式，0--无斜坡，1--梯形斜坡，2--S形斜坡
    __IO uint8_t SlopeMode; //位置模式，0--速度模式，1--位置模式
    __IO uint32_t VBreak;
    __IO uint32_t AStart;
    __IO uint32_t VStart;
    __IO uint32_t VStop;
    __IO uint32_t DFinal;
    __IO int_fast32_t Vmax;
    __IO uint32_t AMax;
    __IO uint32_t DMax;
} STPM_SLOPE_PARAM;

typedef struct
{
    // 运动参数

    __IO uint8_t Mode; //0速度1位移2无效3闭环位移

    __IO STPM_SLOPE_PARAM run;
    __IO STPM_SLOPE_PARAM home;
    __IO STPM_SLOPE_PARAM fasthome;
    __IO STPM_SLOPE_PARAM stopslope;
    __IO STPM_SLOPE_PARAM closerun;

    // 编码器
    __IO uint8_t EncEn; //编码器使能
    __IO uint8_t Dir;   // 编码器方向
    __IO float EncRes;  // 细分数 * 单圈步数 / 编码器线数
    __IO uint8_t EncDiff;
    __IO uint8_t EncDir;
    __IO uint16_t signscale;
    __IO uint16_t line;
    __IO uint16_t ratiose;
    __IO uint16_t ratiosm;

    // 电机
    __IO float DrvCur;  // 运行电流
    __IO float HoldCur; //保持电流

    // 驱动器
    __IO uint16_t StepDiv; // 细分
    __IO uint8_t HMSenPol; //home 传感器极性

    __IO uint32_t HOffset; // home 偏移量
    __IO uint32_t ASOffset; //防抖位移
   
    __IO uint32_t FZAllowError; //允许误差
    __IO uint16_t ESTC;         // 稳定时间
    __IO uint16_t ESTB;
    __IO uint16_t ESCNT;  //estc:定位整定时间，estb:允许位置差，escnt：允许最大整定次数

} MotorParamDef; //电机参数变量

typedef enum
{
    _motor_none = 0,        //空
    _motor_stop,            //停止
    _motor_rls,             //释放
    _motor_home,            //回HOME
    _motor_run_pos,         //位移运行
    _motor_run_spd,         //速度运行
    _motor_runrelative_pos, //相对位移运行
    _motor_enable_ctr, //电机使能
    //_motor_run_pos_cl,  //闭环位移运行
} MotorCtlStatusDef; //电机控制状态

typedef enum
{
    _motor_idle = 0,  //已经停止，空闲
    _motor_rlsd,      //已经释放
    _motor_running,   //正在运行
    _motor_goinghome, //回HOME运行中
    _motor_error,     //错误
    _motor_enable,     //电机使能
} MotorStatusDef;     //电机实际运行状态

typedef enum
{
    _none = 0,
    _stopl,
    _stopr,
    _exin0,
    _exin1,
    _exin2,
    _exin3,
} SensorIfsDef; //工作传感器输入接口

typedef struct
{
    uint8_t cmd;
    uint8_t pos;
    uint8_t status;
    uint32_t value;
} MotorCtrCMD;

typedef struct
{
    uint8_t status;
    uint8_t sensor;
} MotorStatus;

typedef struct
{
    __IO uint8_t BoardID;    //驱动板CANID号
    __IO uint8_t ChannelID;  //板上电机通道号，0-3
    __IO uint8_t MotorLogID; //记录中的ID，高4位代表设备号，低4位代表电机号，与连线图
    __IO uint16_t MotorRenameID; //记录中的ID，高4位代表设备号，低4位代表电机号，与连线图
    __IO SemaphoreHandle_t Mutex; //互斥量
    __IO MotorParamDef Param; //参数

    __IO MotorCtlStatusDef Ctl; //控制
    __IO MotorStatusDef Status;  //状态
    __IO MotorCtrCMD ctrcmd;
    __IO MotorStatus sta;

    __IO int_fast32_t Vmax;   //速度
    __IO int_fast32_t FZVmax; //快回0速度
    //  StpmN_XactualGet
    __IO int_fast32_t Pos[14]; //运行位置微步
    __IO int_fast32_t PosTemp; //临时运行微步
    __IO uint8_t CurPos;       //当前所在位置编号
    __IO uint8_t NxtPos;       //下一个所在位置编号

    __IO SensorIfsDef WorkSensor[2]; //工作传感器检测选择，分别对应POS位置中的位置1、2
    __IO uint8_t InitCmlt;      //初始化完成，0未完成，1完成
    __IO TaskHandle_t TaskHandle; //电机控制线程句柄
} MotorStruDef;                 //电机变量

typedef enum
{
    _chain_idle = 0,
    _chain_running,
    _chain_error,
} ChainStatusDef; //进样链条状态枚举

typedef enum
{
    _cctl_stop = 0,  //刚开机或停止运行
    _cctl_run,       //运行
    _cctl_userpause, //用户按下暂停
    _cctl_autopause, //自动暂停
    _cctl_wait,      //自动暂停
} ChainInCtlDef;     //进样链条运行控制状态枚举

typedef enum
{
    _crst_Idle = 0,      //空
    _crst_Blocked,       //运行被阻挡
    _crst_SampleBlocked, //直接发现样本退出
    _crst_Sample,        //运行到样本
    _crst_Stop,          //外部原因停止
} ChainRstDef;           //链条运行返回状态枚举

typedef struct
{
    __IO SemaphoreHandle_t Mutex; //链条互斥量
    //使用该互斥量，不需要使用电机本身互斥量
    ChainStatusDef Status;
    __IO int_fast32_t Offset[2]; //运行偏移微步
    ChainInCtlDef RunCtl;        //链条运行指令
    __IO uint8_t UserGoBack;     //倒退
} ChainInStruDef;                //进样链条变量

typedef struct
{
    __IO int_fast32_t RunPos; //运行的格数
    __IO uint8_t StopCtl;     //0运行到最终位停 1运行到样本检测位停
    __IO uint8_t Align;       //当前对齐位置，0-进样 1-返回 2-回收通道
    __IO uint8_t RunMode;     //在使能编码器的情况下，0-编码器模式，1-传统模式，非使能编码器模式情况下无效
    __IO ChainRstDef RstReturn; //返回值
    __IO uint32_t RunCnt;     //向前移动的格数
} ChainCtlStruDef;            //进样链条控制变量

typedef enum
{
    _crrs_Idle = 0, //空
                    //    _crrs_Checked,
    _crrs_Prepared,
    _crrs_CIReady,
    _crrs_PPCplt,
} ChainReRunStatus; //链条运行返回状态枚举

typedef struct
{
    SemaphoreHandle_t Mutex; //链条互斥量
    //使用该互斥量，不需要使用电机本身互斥量
    ChainStatusDef Status;
    __IO int_fast32_t Offset[2][2]; //运行偏移微步
    __IO uint8_t CurAlign;          //当前对齐位置，0-进样1-返回 2-回收通道
                                    //    __IO uint8_t CurAlignPos;//当前样本对齐位，0-标准位1-标准位前一位
    __IO uint8_t SampleInWaiting;   //进样正在等待链条
    __IO uint8_t SampleBackWaiting; //进样正在等待链条
    __IO uint8_t SampleRecWaiting;  //回收正在等待链条
    //__IO uint8_t SampleRecWaiting;//回收正在等待链条互斥量
    __IO uint8_t FrameList[30];
    __IO uint8_t RecoveryFullAlarm[30];     // 回收区满报警 0：未报过警，1：报过警
    __IO uint8_t FrameCount;
    ChainReRunStatus ReRunStatus;
    __IO uint8_t ReRunWaitingID;

} ChainBackStruDef; //返回链条变量

/* Exported constants --------------------------------------------------------*/
#define MotorCtl_STACK_SIZE (configMINIMAL_STACK_SIZE * 2)
#define MotorCtl_Priority osPriorityAboveNormal + 3
#define ChainCtl_Priority osPriorityHigh + 3
#define SampleCtl_STACK_SIZE  (configMINIMAL_STACK_SIZE * 10)
#define SampleCtl_Priority osPriorityNormal + 3
#define MotorTimeout 1000
#define SampleTimeout 1000
#define MotorMaxTry 3 //电机运行尝试次数

#define ChainBackSingleStep 1000 //单格编码器步长
#define ChainBackAllowError 5    //允许偏差
#define ChainBackAdjustTimes 5   //整定次数

extern volatile ChainInStruDef ChainIn;
extern volatile ChainBackStruDef ChainBack;

/* Exported macro ------------------------------------------------------------*/
/* Exported variables ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */


uint8_t MAIN_EXIN_Get(uint8_t sensorid);
void MAIN_EXIN_Init(void);

void LED_Set(uint8_t LEDID, uint8_t LED_Status);
#endif
/*****************************END OF FILE*****************************/
