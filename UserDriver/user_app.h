/**
******************************************************************************
* @文件    user_app.H
* @作者    SUYANG
* @版本    V0.0.1
******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_APP_H
#define __USER_APP_H

#ifdef __cplusplus
extern "C"
{
#endif

//主控板版本定义
//#define __BoardVersion02//主控板__BoardVersion02 与新版本IO定义不同

//仪器版本定义
//#define __DeviceVersion01//__DeviceVersion01 2×Honeywell扫码器 链条扫描架号 扫码区扫描瓶号
#define __DeviceVersion02 //__DeviceVersion02 Leuze扫码器 扫码区扫描架号、瓶号

//部件定义
#define __USEChainBackEncoder //返回链条编码器模式
#define __USE43D3111D0        //扫码编码器电机为STD-43D3111（1000线差分编码器）
#define __USE43D3111D1        //轨道1的编码器电机为STD-43D3111（1000线差分编码器）
#define __USE43D3111D2//轨道2的编码器电机为STD-43D3111（1000线差分编码器）
#define __USE43D3111D3 //轨道3的编码器电机为STD-43D3111（1000线差分编码器）
#define __USE43D3111D4 //轨道4的编码器电机为STD-43D3111（1000线差分编码器）

//轨道变轨挡板机械部件定义
//#define __USE_NEWMBFD1//轨道1的变轨挡板钣金件为加厚版本
//#define __USE_NEWMBFD2//轨道2的变轨挡板钣金件为加厚版本
//#define __USE_NEWMBFD3//轨道3的变轨挡板钣金件为加厚版本
//#define __USE_NEWMBFD4//轨道4的变轨挡板钣金件为加厚版本

#define ScanErrorCode 28103 //<scan error
#define RecoveryFullCode 28101
#define UserPausedRerunWaitingCode 28102
#define FrameLostAlarmCode 28200
#define UnknownFrameAlarmCode 28300
#define InitAlarmCode 20200


/* Exported macro ------------------------------------------------------------*/
#define MaxTrackNumber 4   //最大允许接入轨道数
#define MaxFrameNumber 100 //最大允许的样本架数
#define IdleCntAim 10      //样本回收启动空闲时间设置。时间约为n*100ms


/* Includes ------------------------------------------------------------------*/
#include "user_gpm.h"
#include "user_excutor.h"
#include "main.h"
    /* Private types ------------------------------------------------------------*/

    typedef enum
    {
        _NULL = 0,
        None,
        StandBy,
        _Reset,
        Run,
        Pause,
        Stop,
        Error,
    } DeviceStatusDef; //设备状态枚举

    typedef struct
    {
        __IO uint8_t WarnID[20];   //AlarmString
        __IO uint8_t Value[3][20]; //AlarmInfo
    } DeviceWarnDef;               //设备告警结构体

    typedef enum
    {
        ENorm = 0,
        EEmg,
        EBack,
    } TrackEnterExitDef; //轨道出入口定义

    typedef struct
    {
        __IO uint8_t TrackID;            //0-3
        TrackEnterExitDef TrackEntrance; //样本轨道入口
        TrackEnterExitDef TrackExit;     //样本轨道出口
        __IO uint8_t AddingEn;           //样本是否在本节轨道加样
    } SampleDispatchDef;                 //样本轨道调度结构体

    typedef struct
    {
        __IO uint8_t BeltRun;
        __IO uint8_t BeltStop;
        __IO uint8_t BeltPause;
        __IO uint32_t BeltCnt;
    } TrackBeltStruDef; //样本轨道调度结构体

    //W等待通道N常规通道E急诊通道B返回通道M多通道
    //BF挡板电机PB位置挡板电机TC变轨电机BL长传送带电机BS短传送带电机
    typedef struct
    {
        MotorStruDef WBF; //等待挡板电机
        MotorStruDef NPB; //常规通道挡板电机
        MotorStruDef NBL; //常规通道长传送带电机
        MotorStruDef NBS; //常规通道短传送带电机

        MotorStruDef EPB; //急诊通道挡板电机
        MotorStruDef EBL; //急诊通道长传送带电机
        MotorStruDef EBS; //急诊通道短传送带电机

        MotorStruDef MTC; //变轨电机
        MotorStruDef MBF; //变轨挡板电机

        MotorStruDef BPB; //返回通道阻挡推板电机
        MotorStruDef BBL; //返回通道长传送带电机
        MotorStruDef BBS; //返回通道短传送带电机

        __IO uint8_t EPBWKCnt[10]; //急诊挡板的工作传感器计数，0-位置0到位置1工作传感器计数

        __IO uint8_t BdCanID[3]; //板卡ID

        //轨道样本记录
        __IO uint8_t WaitSID;  //等待位的id
        __IO uint8_t NormSID;  //常规位的id
        __IO uint8_t EmgSID;   //急诊位的id
        
        __IO uint8_t NormNextSID;
        __IO uint8_t EmgNextSID;

        //轨道传送带结构体
        __IO TrackBeltStruDef NLBelt;
        __IO TrackBeltStruDef ELBelt;
        __IO TrackBeltStruDef BLBelt;
        __IO TrackBeltStruDef NSBelt;
        __IO TrackBeltStruDef ESBelt;
        __IO TrackBeltStruDef BSBelt;

    } TrackStruDef; //轨道变量

    typedef enum
    {
        //槽型光耦
        WBF_HM, //等待挡板电机0位
        WBF_WK, //等待挡板电机工作位
        NPB_HM, //常规通道挡板电机0位
        NPB_WK, //常规通道挡板电机工作位

        EPB_HM, //急诊通道挡板电机0位
        EPB_WK, //急诊通道挡板电机工作位

        MTC_HM, //变轨电机0位
        MTC_WK, //变轨电机工作位
        MTC_SH, //变轨电机弹簧0位
        MTC_SW, //变轨电机弹簧工作位

        MBF_HM, //变轨挡板电机0位
        MBF_WK, //变轨挡板电机工作位

        BPB_HM, //返回通道阻挡推板电机0位
        BPB_WK, //返回通道阻挡推板电机工作位

        //反射光耦
        //W等待通道N常规通道E急诊通道B返回通道M多通道
        //EN入口MO移动MI中部EX出口
        W_EN, //等待通道入口检测

        N_MO, //常规通道样本移动检测
        N_MI, //常规通道变轨前检测
        N_EX, //常规通道出口检测

        E_EN, //急诊通道入口检测
        E_MO, //急诊通道样本移动检测
        E_MI, //急诊通道变轨前检测
        E_EX, //急诊通道出口检测

        B_EN,         //返回通道入口检测
        B_MI,         //返回通道变轨后检测
    } TrackSensorDef; //轨道传感器

    //CH链条电机PB推板电机PP推送电机
    //I进样电机B返回电机
    //F前进R后退
    typedef struct
    {
        MotorStruDef CHIF; //进样链条前进电机
        MotorStruDef CHIR; //进样链条后退电机
        MotorStruDef CHBF; //返回链条前进电机
        MotorStruDef CHBR; //返回链条后退电机

        MotorStruDef PBI; //进样推板电机（扫码）
        MotorStruDef PPI; //进样推送电机

        MotorStruDef PBB; //返回推板电机
        MotorStruDef PPB; //返回推送回收仓电机

        uint8_t BdCanID[2]; //板卡ID
    } TableStruDef;         //样本台变量

    typedef enum
    {
        //槽型光耦
        CHIF_S, //进样链条前进电机位置检测
        CHIR_S, //进样链条后退电机位置检测
        CHBF_S, //返回链条前进电机位置检测
        CHBR_S, //返回链条后退电机位置检测

        PBI_HM,  //进样推板电机（扫码）HOME
        PBI_WK1, //进样推板电机（扫码）码盘工作，仅第一版本有效
        PBI_WK2, //进样推板电机最后工作，第二版本仪器中，作为中间工作位

        PPI_HM, //进样推送电机HOME
        PPI_WK, //进样推送电机工作

        PBB_HM,  //返回推板电机HOME
        PBB_WK1, //返回推板电机工作1
        PBB_WK2, //返回推板电机工作2

        PPB_HM, //返回推送回收仓电机HOME
                //    PPB_WK,//返回推送回收仓电机工作

        //反射光耦
        ChainIn_FS, //进样链条正向终端
        ChainIn_RS, //进样链条反向终端
        ChainIn_SS, //进样链条样本检测

        ChainBack_FS,  //返回链条正向终端
        ChainBack_RS,  //返回链条反向终端
        ChainBack_SFS, //返回链条样本检测前
        ChainBack_SRS, //返回链条样本检测后

        Scan_HS, //扫码区高检测
        Scan_LS, //扫码区低检测
        Scan_OS, //扫码区出口检测

        Recy_ES, //回收区入口检测
        Recy_FS, //回收区满检测

        Norm_SS, //常规轨道样本检测
        Emg_SS,  //急诊轨道样本检测
        Back_SS, //返回轨道样本检测

        _BUT_Pause, //暂停按钮
        _BUT_Back,  //后退按钮
        TypeOfTestTube,

    } TableSensorDef; //样本台传感器

    typedef struct
    {
        __IO uint8_t Type;          //类型，0空1低2高
        __IO uint8_t SampleIDA[50]; //样本条码
    } SampleDef;                    //整机样本结构体

    typedef struct
    {
        __IO uint8_t Inuse;            //使用中
        __IO uint8_t FramSID;          //样本架序号，与外围Frame[n]一致
        __IO uint32_t FrameID;         //样本架号,十进制
        __IO uint8_t FrameIDA[50];     //样本架条码ASCII
        SampleDef Sample[10];          //10个样本序列
        SampleDispatchDef SD;          //样本架调度
        __IO uint8_t FrameDevice;      //1-第一轨道2-第二轨道3-第三轨道4-第四轨道
                                       //11样本台进样12样本台扫码13样本台返回
        __IO uint8_t FrmNextDevice;    //13样本台，1-4轨道
        __IO uint8_t FrmSampleNextPos; //下个样本位置
        __IO uint8_t FrmSampleCurPos;  //加样位置：当前样本位置
        __IO uint8_t FrmAddingEmg;     //0常规通道加样 1急诊通道加样
        __IO uint8_t FrmMovStart;      //1启动运行 ， 0 表示可以移动
        __IO uint8_t ReceiveStatus;    //样本接收端接收许可。0-空，1-接收许可，2-接收完成
        __IO uint8_t ReStatus;         //回收、重测状态，0空，1回收，2重测，
        __IO uint8_t ResetStatus;         //回收、重测状态，0空，1回收，2重测，
        __IO uint8_t Recorde; // 重复架号标志  1 轨道自己处理
        //样本架所有样本都回收时样本架置为回收，样本架有任意样本重测时样本架置为重测
    } FrameDef; //整机样本架结构体

    typedef struct
    {
        __IO uint8_t But_Status;      //按键当前状态
        __IO uint8_t But_Status_Prev; //按键上个状态
        __IO uint8_t But_Cnt;         //按键防抖计数
        __IO uint8_t LED_Status;      //LED状态，0灭，1亮，2闪
        __IO uint8_t LED_Status_Prev; //上一个LED状态，0灭，1亮，2闪
        __IO uint8_t LED_Cnt;         //LED闪烁计数
    } TableButStruDef;                //样本台按键变量

    typedef struct
    {
        uint8_t alarm;    //告警,0空 1告警
        uint8_t level;    //级别，0info 1warn 2error
        uint32_t info;    //其他信息，如需要
        uint32_t pos;     //错误位置，01-99，如需要
        uint32_t code;    //代号
    } DeviceAlarmStruDef; //告警结构体

    /* Exported constants --------------------------------------------------------*/
    extern DeviceStatusDef DeviceStatus;
    extern DeviceStatusDef DeviceStatusChg;
    extern uint32_t TrackNumber;
    extern __IO uint8_t FrmBarcodeSend;
    extern volatile TrackStruDef Track[4];
    extern volatile TableStruDef Table;
    extern __IO uint8_t SystemEmergencyStop; //紧急停机

    extern __IO uint8_t recodessid; //add 2021-10-30 for GetScannerSampleInfo

    extern volatile FrameDef Frame[MaxFrameNumber];

    /* Exported variables ------------------------------------------------------------*/

    /* Exported functions ------------------------------------------------------- */
    //void TrackSampleCtrlTask(void *pvParameters);
    //void TableSampleCtrlTask(void *pvParameters);
    void TableBackCtrlTask(void *pvParameters);
    void DebugCtlTask(void *pvParameters);
    
    
    void DeviceCtrlTask(void *pvParameters);
    //void ScannerInitTask(void *pvParameters);
    void TrakIDInit(void);

    void SensorInit(void);
    //void Frame_Init(uint8_t FrameSID);
    uint8_t Motor_All_Home(uint8_t init);
    void Motor_All_Stop(uint8_t mode);
    
    uint8_t FullSemaphoreTake(TickType_t block_time);
    uint8_t FullSemaphoreGive(void);
    //uint8_t Track_Belt_Run(uint8_t trackid);
    //uint8_t Track_Belt_Stop(uint8_t trackid);
    
    uint8_t MotorMTCCtrl(uint8_t TrackID, uint8_t posid);
    
    uint8_t Debug_Motor_Conv(uint8_t device, uint8_t motor, MotorStruDef **MotorSt);
    

#endif
    /*****************************END OF FILE*****************************/
