/**
******************************************************************************
* @文件    user_cmdfun.c
* @作者
* @版本    V0.0.1
******************************************************************************


******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "user_cmdfun.h"
#include "../Tasks/user_chainbacktask.h"
#include "../Tasks/user_chainintask.h"
#include "../Tasks/user_cmd_api.h"
#include "../Tasks/user_cmd_api_20220621.h"
#include "../Tasks/user_motor_api.h"
#include "../lib/user_motor_params.h"

extern uint8_t Motor_Param_Set(void);
extern uint8_t FullInit(void);
extern uint8_t Table_Sensor_Get(TableSensorDef SensorID);
extern uint8_t Track_Sensor_Get(uint8_t TrackID, TrackSensorDef SensorID);
extern uint8_t SetMotorParamFromPC(const uint16_t motorname, StepMotorParams *params);

extern uint8_t MotorCtrl(MotorStruDef *Motor, MotorCtlStatusDef Ctl, uint8_t PosID);
extern uint8_t CpyMotorToParamStruct(MotorStruDef *motor, StepMotorParams *params);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t testu32 = 0;
__IO uint8_t EthDbgMode = 0;		   //网络调试模式
__IO uint8_t RACKRUNMODE = 0xff;	   //上位机运行模式
static __IO uint8_t EthDbgTestRun = 0; //测试运行

extern uint8_t namebuff[30];

typedef struct
{
    __IO uint8_t ResetStatus[3];
    __IO uint8_t SSID[3];
    __IO uint32_t racksid[3];

} RackStatus;

typedef struct
{
    __IO uint8_t ResetStatus;
    __IO uint32_t dev;
    __IO uint8_t SSID;
} DEVRESETStatus;

DEVRESETStatus rackdev[5][4];

extern RNG_HandleTypeDef RngHandle;
extern __IO uint8_t Test_Mode;

extern uint32_t TxID;

extern char LastSource[25]; //最近一条的指令来源

extern DeviceAlarmStruDef RecoveryFullAlarm;
extern DeviceAlarmStruDef RecoveryFulltoStopAlarm;
extern DeviceAlarmStruDef UserPausedRerunWaitingAlarm;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static uint8_t EthDbgEnterDebugMode(void);
static void EthCmdEnableScanMode(void *pvParameters);
static void EthCmdGetRackArriveLocationResultTask(void *pvParameters);
static void EthCmdEnterDebugModeTask(void *pvParameters);
static void EthDbgMotorRunTask(void *pvParameters);
static void EthCmdResetTask(void *pvParameters);
static void EthCmdDeviceSetTimeTask(void *pvParameters);
static void EthCmdDeviceSetMotorParamTask(void *pvParameters);
static uint8_t EthDbgSetParam(EthPkgDef *pkg);
static void EthCmdAlarmTask(void *pvParameters);
static void EthCmdScanTrackBarcodeTask(void *pvParameters);
static void EthCmdTrackGotoDeviceTask(void *pvParameters);
static void EthCmdGetInstrumentStatusTask(void *pvParameters);

//

static void EthCMDRsetRackResultTask(void *pvParameters);
static void RsetRackResultTask(void *pvParameters, void *pvParameters1);
static void TrackGotoDeviceTask(void *pvParameters);
static void EthCmdRsetRackCmd(void *pvParameters);
static void EthCmdGetMotorParaTask(void *pvParameters);

//
static void EthCmdDeviceVersionTask(void *pvParameters);
static uint8_t PackageMotorParam(uint16_t motorname, uint8_t *str);
static void EthCmdMotorInitCmd(void *pvParameters);
static void EthCmdSaveParamFlashTask(void *pvParameters);
static void EthCmdTrackGotoRecycleTask(void *pvParameters);
static void EthDbgScanBarcodeTask(void *pvParameters);
static void EthCmdSystemResetTask(void *pvParameters);
static void EthCmdGetRackLocationTask(void *pvParameters);

/**
 * @brief  接收数据包指令处理
 *对接收数据包中的数据进行处理，分析指令。
 *如需进行异步操作，则在该函数中建立操作线程；操作线程将待发送数据写入发送队列并更新缓冲区，检测发送状态，决定是否重发
 * @param
 **pkg：等待处理的数据包
 **str：截取的command片段
 * @retval 0--需要返回ack，1--需要返回nak
 */
uint8_t EthRecvCmdDeal(EthPkgDef *pkg, char *str)
{
    uint32_t handle_index = 0;

    if (UserEthRecvCmdDeal(pkg, str) == 0)
    {
        return 0;
    }
    else if (UserEth20220621RecvCmdDeal(pkg, str) == 0)
    {
        return 0;
    }

    if (strcmp(str, "GetInstrumentStatus") == 0) //获取仪器运行状态
    {
        handle_index = EthHandleIndexGet();
        if (handle_index == 0xff) //没有空余的线程句柄了
        {
            return 1;
        }
        //返回状态
        pkg->dealtask = NULL; //接收包没有需要的数据，由网络线程清空接收包即可
        xUserTaskCreate(EthCmdGetInstrumentStatusTask, "EthCmdGetInstrumentStatusTask", configMINIMAL_STACK_SIZE, NULL, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
        return 0;
    }
    else if (strcmp((char *)str, "TrackEmerRack") == 0) //急诊下发命令
    {
        if(DeviceStatus == _Reset)
				{
					//工作记录
					memset(LogBuffer, 0, sizeof(LogBuffer));
					sprintf(LogBuffer, "DeviceStatus _Reset No TrackEmerRack");
					WorkRecordReadWrite(0, 0, LogBuffer);
				}
				else
				{
					emg_flg = 10;
					//工作记录
					memset(LogBuffer, 0, sizeof(LogBuffer));
					sprintf(LogBuffer, "After TrackEmerRack DeviceStatusChg = %d,DeviceStatus = %d\n",DeviceStatusChg,DeviceStatus);
					WorkRecordReadWrite(0, 0, LogBuffer);
				}
        return 0;
    }
    else if (strcmp((char *)str, "DeviceVersion") == 0) //下位机版本信息获取
    {
        //仪器版本号返回
        handle_index = EthHandleIndexGet();
        if (handle_index == 0xff) //没有空余的线程句柄了
        {
            return 1;
        }
        pkg->dealtask = NULL; //接收包没有需要的数据，由网络线程清空接收包即可
        xUserTaskCreate(EthCmdDeviceVersionTask, "EthCmdDeviceVersionTask", configMINIMAL_STACK_SIZE * 1, NULL, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
        return 0;
    }
    else if (strcmp(str, "SystemReset") == 0) //系统复位
    {
        xUserTaskCreate(EthCmdSystemResetTask, "EthCmdSystemResetTask", configMINIMAL_STACK_SIZE * 1, NULL, osPriorityNormal, NULL);
        return 0;
    }
    else if (strcmp((char *)str, "CheckRsetRack") == 0) //进入调试模式
    {
        handle_index = EthHandleIndexGet();
        if (handle_index == 0xff) //没有空余的线程句柄了
        {
            return 1;
        }
        //返回状态
        pkg->dealtask = &EthCMDTaskHandle[handle_index]; //接收包没有需要的数据，由网络线程清空接收包即可
        xUserTaskCreate(EthCMDRsetRackResultTask, "EthCMDRsetRackResultTask", configMINIMAL_STACK_SIZE, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
        return 0;

        // return
    }
    else if (strcmp((char *)str, "TrackGotoRecycle") == 0) //回回收区、复测
    {
        //走到回收区/复测
        handle_index = EthHandleIndexGet();
        if (handle_index == 0xff) //没有空余的线程句柄了
        {
            return 1;
        }
        pkg->dealtask = &EthCMDTaskHandle[handle_index];
        xUserTaskCreate(EthCmdTrackGotoRecycleTask, "EthCmdTrackGotoRecycleTask", configMINIMAL_STACK_SIZE * 2, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
        return 0;
    }
    else if (strcmp((char *)str, "TrackGotoDevice") == 0) //移动
    {
        //走到设备
        handle_index = EthHandleIndexGet();
        if (handle_index == 0xff) //没有空余的线程句柄了
        {
            return 1;
        }
        pkg->dealtask = &EthCMDTaskHandle[handle_index];
        xUserTaskCreate(EthCmdTrackGotoDeviceTask, "EthCmdTrackGotoDeviceTask", configMINIMAL_STACK_SIZE * 2, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
        return 0;
    }
    else if (strcmp((char *)str, "GetRackLocation") == 0) //查询样本架位置
    {
        handle_index = EthHandleIndexGet();
        if (handle_index == 0xff) //没有空余的线程句柄了
        {
            return 1;
        }
        pkg->dealtask = &EthCMDTaskHandle[handle_index];
        xUserTaskCreate(EthCmdGetRackLocationTask, "EthCmdGetRackLocationTask", configMINIMAL_STACK_SIZE * 2, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
        return 0;
    }
    else if (strcmp((char *)str, "GetRackArriveLocation") == 0) //查询样本架位置
    {
        handle_index = EthHandleIndexGet();
        if (handle_index == 0xff) //没有空余的线程句柄了
        {
            return 1;
        }
        pkg->dealtask = &EthCMDTaskHandle[handle_index];
        xUserTaskCreate(EthCmdGetRackArriveLocationResultTask, "EthCmdGetRackArriveLocationResultTask", configMINIMAL_STACK_SIZE * 2, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);

        return 0;
    }
    else if (strcmp((char *)str, "UpdateInstrument") == 0) //进入网络升级模式
    {
        xUserTaskCreate(EthOTATask, "EthOTATask", configMINIMAL_STACK_SIZE * 1, NULL, osPriorityNormal, NULL);
        return 0;
    }
    else if (strcmp((char *)str, "EnableScanMode") == 0) //进入调试模式
    {
        handle_index = EthHandleIndexGet();
        if (handle_index == 0xff) //  没有空余的线程句柄了
        {
            return 1;
        }
        //返回状态
        pkg->dealtask = &EthCMDTaskHandle[handle_index];
        xUserTaskCreate(EthCmdEnableScanMode, "EthCmdEnableScanMode", configMINIMAL_STACK_SIZE * 1, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
        return 0;
    }
    else if (strcmp((char *)str, "RsetRack") == 0) //单模块重置
    {

        handle_index = EthHandleIndexGet();
        if (handle_index == 0xff) //没有空余的线程句柄了
        {
            return 1;
        }
        pkg->dealtask = &EthCMDTaskHandle[handle_index];
        xUserTaskCreate(EthCmdRsetRackCmd, "EthCmdRsetRackCmd", configMINIMAL_STACK_SIZE, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);

        return 0;
    }
    else if (strcmp(str, "SNGet") == 0) //获取设备信息 MACSN BOARDSN etc.
    {
        handle_index = EthHandleIndexGet();
        if (handle_index == 0xff) //没有空余的线程句柄了
        {
            return 1;
        }
        //返回状态
        pkg->dealtask = NULL; //接收包没有需要的数据，由网络线程清空接收包即可
        xUserTaskCreate(EthCmdDeviceInfoTask, "EthCmdDeviceInfoTask", configMINIMAL_STACK_SIZE * 2, NULL, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
        return 0;
    }
    else if (strcmp((char *)str, "ScanTrackBarcode") == 0) //返回当前轨道条码
    {
        //返回当前轨道条码
        return EthCmdScanTrackBarcode();
    }
    else if (strcmp((char *)str, "GetEnableScanModeResult") == 0) //进入调试模式
    {
        handle_index = EthHandleIndexGet();
        if (handle_index == 0xff) //  没有空余的线程句柄了
        {
            return 1;
        }
        //返回状态
        pkg->dealtask = &EthCMDTaskHandle[handle_index];
        xUserTaskCreate(EthCmdEnableScanMode, "EthCmdEnableScanMode", configMINIMAL_STACK_SIZE * 1, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
        return 0;
    }
    else
    {
        if ((DeviceStatus == Error) || (DeviceStatus == Stop))
        {
            return 1;
        }
        else if (strcmp((char *)str, "Reset") == 0) //仪器复位
        {
            //系统初始化（完整）
            handle_index = EthHandleIndexGet();
            if (handle_index == 0xff) //没有空余的线程句柄了
            {
                return 1;
            }
            pkg->dealtask = NULL; //接收包没有需要的数据，由网络线程清空接收包即可
            xUserTaskCreate(EthCmdResetTask, "EthCmdResetTask", configMINIMAL_STACK_SIZE * 2, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
            return 0;
        }

        else if (strcmp(str, "SetTime") == 0) //设置系统时间
        {
            handle_index = EthHandleIndexGet();
            if (handle_index == 0xff) //没有空余的线程句柄了
            {
                return 1;
            }
            //返回状态
            pkg->dealtask = &EthCMDTaskHandle[handle_index];
            xUserTaskCreate(EthCmdDeviceSetTimeTask, "EthCmdDeviceSetTimeTask", configMINIMAL_STACK_SIZE * 1, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);

            return 0;
        }
        else if (strcmp((char *)str, "QuickInit") == 0) //仪器快速复位
        {
            //系统初始化（快速）
            handle_index = EthHandleIndexGet();
            if (handle_index == 0xff) //没有空余的线程句柄了
            {
                return 1;
            }
            pkg->dealtask = NULL; //接收包没有需要的数据，由网络线程清空接收包即可
            xUserTaskCreate(EthQuickInitTask, "EthQuickInitTask", configMINIMAL_STACK_SIZE * 1, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
            return 0;
        }

        else if (strcmp((char *)str, "SetMotorPara") == 0) //设置电机参数
        {
            //电机参数
            handle_index = EthHandleIndexGet();
            if (handle_index == 0xff) //没有空余的线程句柄了
            {
                return 1;
            }
            pkg->dealtask = &EthCMDTaskHandle[handle_index];
            xUserTaskCreate(EthCmdDeviceSetMotorParamTask, "EthCmdDeviceSetMotorParamTask", configMINIMAL_STACK_SIZE * 1,
                            (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
            return 0;
        }
        else if (strcmp((char *)str, "GetMotorPara") == 0) //获取电机参数
        {
            handle_index = EthHandleIndexGet();
            if (handle_index == 0xff) //没有空余的线程句柄了
            {
                return 1;
            }
            pkg->dealtask = &EthCMDTaskHandle[handle_index];
            xUserTaskCreate(EthCmdGetMotorParaTask, "EthCmdGetMotorParaTask", configMINIMAL_STACK_SIZE * 1, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
            return 0;
        }
        else if (strcmp((char *)str, "test_start") == 0) //启动测试
        {
            return EthCmdTestStart();
        }
        else if (strcmp((char *)str, "chaininstatus") == 0) //发送样本台进样链条状态
        {
            return EthSendChainInStatus();
        }
        else if (strcmp((char *)str, "CancelDo") == 0) //停止测试
        {
            EthCmdCancelDo();
            return 0;
        }
        // debug软件接口
        else if (strcmp((char *)str, "EnterDebugMode") == 0) //进入调试模式
        {
            handle_index = EthHandleIndexGet();
            if (handle_index == 0xff) //没有空余的线程句柄了
            {
                return 1;
            }
            //返回状态
            pkg->dealtask = NULL; //接收包没有需要的数据，由网络线程清空接收包即可
            xUserTaskCreate(EthCmdEnterDebugModeTask, "EthCmdEnterDebugModeTask", configMINIMAL_STACK_SIZE * 1, NULL, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
            return 0;
        }
        else if (strcmp((char *)str, "ParamSave") == 0) //参数保存
        {
            handle_index = EthHandleIndexGet();
            if (handle_index == 0xff) //没有空余的线程句柄了
            {
                return 1;
            }
            //返回状态
            pkg->dealtask = NULL; //接收包没有需要的数据，由网络线程清空接收包即可
            xUserTaskCreate(EthCmdSaveParamFlashTask, "EthCmdSaveParamFlashTask", configMINIMAL_STACK_SIZE * 1, NULL, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
            return 0;
        }
        else if (strcmp((char *)str, "ExitDebugMode") == 0) //退出调试模式
        {
            return EthDbgExitDebugMode();
        }
        else if (strcmp((char *)str, "GetPos") == 0) //获取保存的位置
        {
            return EthDbgGetPos(pkg);
        }
        else if (strcmp((char *)str, "SetPos") == 0) //设置保存的位置
        {
            return EthDbgSetPos(pkg);
        }
        else if (strcmp((char *)str, "MotorRun") == 0) //电机运行
        {
            return EthDbgMotorRun(pkg);
        }
        else if (strcmp((char *)str, "SensorGet") == 0) //获取传感器
        {
            return EthDbgSensorGet(pkg);
        }
        else if (strcmp((char *)str, "SetParam") == 0) //设置参数
        {
            return EthDbgSetParam(pkg);
        }
        else if (strcmp((char *)str, "GetParam") == 0) //获取参数
        {
            return EthDbgGetParam(pkg);
        }

        else if (strcmp((char *)str, "DeviceTestRun") == 0) //测试运行
        {
            return EthDbgDeviceTestRun(pkg);
        }
        else if (strcmp((char *)str, "SNSet") == 0) //测试运行
        {
            return EthDbgSetParam(pkg);
        }
        else if (strcmp((char *)str, "ScanBarcode") == 0) //扫码
        {
            //扫码
            handle_index = EthHandleIndexGet();
            if (handle_index == 0xff) //没有空余的线程句柄了
            {
                return 1;
            }
            pkg->dealtask = NULL; //接收包没有需要的数据，由网络线程清空接收包即可
            xUserTaskCreate(EthDbgScanBarcodeTask, "EthDbgScanBarcodeTask", configMINIMAL_STACK_SIZE * 1, NULL, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
            return 0;
        }
    }

    return 1;
}

/**
 * @brief 查询样本架在加样位位置（CmdEnterDebugMode）
 * @param  FrameData:上位机传入的样本架信息(str)
 * @retval None
 */
static void EthCmdSaveParamFlashTask(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    //(void *)pvParameters;
    uint8_t result_flg; //设置时间返回标记 ，0：表示成功  1，表示失败
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();

    uint32_t SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, SaveParamFlash Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }

    result_flg = EthDbgParamSave();

    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>SaveFlashResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
	<result></result>\n\
  </data>\n\
</root>");

    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", result_flg);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    WorkRecordReadWrite(0, 0, "INFO ETH TXCMD DeviceVersionResult");
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS SaveParamFlash");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        WorkRecordReadWrite(0, 0, "INFO ETH RXACK SaveParamFlash");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK SaveParamFlash");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 获取电机参数
 * @param  None
 * @retval None
 */
static void EthCmdGetMotorParaTask(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    EthPkgDef *pkg = (void *)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint16_t motorname = 0;
    uint8_t paraname = 0;

    StepMotorParams stepmotorparam;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    WorkRecordReadWrite(0, 0, "INFO ETH RXCMD GetMotorParaResult");

    memset(strtmp, 0, EthDataLen);

    if (EthXmlCmpFind(pkg->data, "motorname", (char *)strtmp) != 0)
    {
    }

    motorname = atol((char *)strtmp);

    memset(strtmp, 0, EthDataLen);

    if (EthXmlCmpFind(pkg->data, "paraname", (char *)strtmp) != 0)
    {
    }

    paraname = atol((char *)strtmp);

    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, DeviceVersion Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }

    // EthPkgBuf_Send[SendIdx].cmd = EthDevVer;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>GetMotorParaResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
      <motorname></motorname>\n\
	  <paraname></paraname>\n\
	  <paradata></paradata>\n\
  </data>\n\
</root>");

    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", motorname);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "motorname", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", paraname);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "paraname", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    //打包参数数据  yfxiao 2021-11-09

    // 需要修改

    // PackageMotorParam(motorname, strtmp);
    usermotorparam.PackageProtocol(motorname, 0, strtmp);

    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "paradata", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    WorkRecordReadWrite(0, 0, "INFO ETH TXCMD GetMotorParaResult");
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < 1; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS GetMotorParaResult");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        WorkRecordReadWrite(0, 0, "INFO ETH RXACK GetMotorParaResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK GetMotorParaResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 获取下位机版本信息（CmdDeviceVersion）
 * @param  None
 * @retval None
 */
static void EthCmdDeviceVersionTask(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    (void)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    WorkRecordReadWrite(0, 0, "INFO ETH RXCMD DeviceVersion");
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, DeviceVersion Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    // EthPkgBuf_Send[SendIdx].cmd = EthDevVer;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>DeviceVersionResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <ver></ver>\n\
  </data>\n\
</root>");

    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    memcpy((char *)strtmp, ProjectVersion, strlen(ProjectVersion));
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "ver", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    WorkRecordReadWrite(0, 0, "INFO ETH TXCMD DeviceVersionResult");
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS DeviceVersionResult");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        WorkRecordReadWrite(0, 0, "INFO ETH RXACK DeviceVersionResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK DeviceVersionResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 设置系统时间
 * @param  None
 * @retval None
 */
static void EthCmdDeviceSetTimeTask(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    EthPkgDef *pkg = (void *)pvParameters;
    uint8_t result_flg; //设置时间返回标记 ，0：表示成功  1，表示失败
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    result_flg = EthDbgSetParam(pkg);
    WorkRecordReadWrite(0, 0, "INFO ETH RXCMD SETTIME");
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, DeviceVersion Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }

    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>SetTimeResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <status></status>\n\
  </data>\n\
</root>");

    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    if (result_flg)
    {
        memcpy((char *)strtmp, "Fail", 4);
    }
    else
    {
        memcpy((char *)strtmp, "Pass", 4);
    }

    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "status", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    WorkRecordReadWrite(0, 0, "INFO ETH TXCMD DeviceVersionResult");
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS DeviceVersionResult");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        WorkRecordReadWrite(0, 0, "INFO ETH RXACK DeviceVersionResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK DeviceVersionResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/*
 * 解析电机参数 yfxiao
 */
static void split(char *src, const char *separator, char **dest, int *num)
{
    char *pNext;
    int count = 0;
    if (src == NULL || strlen(src) == 0)
        return;
    if (separator == NULL || strlen(separator) == 0)
        return;
    pNext = strtok(src, separator);
    while (pNext != NULL)
    {
        *dest++ = pNext;
        ++count;
        pNext = strtok(NULL, separator);
    }
    *num = count;
}

/*

获取电机参数
*/
static uint8_t PackageParamToPC(uint8_t *strtmp, StepMotorParams *params)
{
    uint8_t i = 0;
    for (i = 0; i < 100; i++)
    {
        switch (i)
        {
        case BOARDID:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->board.boardid);
        }
        break;
        case MOTORID:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->board.motorid);
        }
        break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%f",);}break;
        case RUNCURRENT:
        {
            sprintf((char *)strtmp, "%s%f;", strtmp, params->runcur);
        }
        break;
        case HOLDCURRENT:
        {
            sprintf((char *)strtmp, "%s%f;", strtmp, params->holdcur); //
        }
        break;
        case MICROSTEP:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->mstep);
        }
        break;
        case RUNMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->runmode);
        }
        break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        case RUNRUNDIR:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.dir);
        }
        break;
        case RUNSLOPEMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.SlopeMode);
        }
        break;
        case RUNCLOSEMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.clmode);
        }
        break;
        case RUNBREAKSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.VBreak);
        }
        break;
        case RUNSTARTACC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.AStart);
        }
        break;
        case RUNSTARTSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.VStart);
        }
        break;
        case RUNSTOPSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.VStop);
        }
        break;
        case RUNDFINA:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.DFinal);
        }
        break;
        case RUNRUNSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.Vmax);
        }
        break;
        case RUNRUNACC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.AMax);
        }
        break;
        case RUNRUNDEC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.DMax);
        }
        break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        case HOMERUNDIR:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.dir);
        }
        break;
        case HOMESLOPEMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.SlopeMode);
        }
        break;
        case HOMECLOSEMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.clmode);
        }
        break;
        case HOMEBREAKSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.VBreak);
        }
        break;
        case HOMESTARTACC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.AStart);
        }
        break;
        case HOMESTARTSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.VStart);
        }
        break;
        case HOMESTOPSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.VStop);
        }
        break;
        case HOMEDFINA:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.DFinal);
        }
        break;
        case HOMERUNSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.Vmax);
        }
        break;
        case HOMERUNACC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.AMax);
        }
        break;
        case HOMERUNDEC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.DMax);
        }
        break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        case FASETHOMERUNDIR:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.dir);
        }
        break;
        case FASETHOMESLOPEMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.SlopeMode);
        }
        break;
        case FASETHOMECLOSEMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.clmode);
        }
        break;
        case FASETHOMEBREAKSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.VBreak);
        }
        break;
        case FASETHOMESTARTACC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.AStart);
        }
        break;
        case FASETHOMESTARTSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.VStart);
        }
        break;
        case FASETHOMESTOPSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.VStop);
        }
        break;
        case FASETHOMEDFINA:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.DFinal);
        }
        break;
        case FASETHOMERUNSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.Vmax);
        }
        break;
        case FASETHOMERUNACC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.AMax);
        }
        break;
        case FASETHOMERUNDEC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.DMax);
        }
        break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        case ENCODEENABLE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.enable);
        }
        break;
        case ENCODEDIR:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.Dir);
        }
        break;
        case ENCODEENCRES:
        {
            sprintf((char *)strtmp, "%s%f;", strtmp, params->encode.EncRes);
        }
        break;
        case ENCODEENCODEMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.EncDiff);
        }
        break;
        case ENCODESIGNSCALE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.signscale);
        }
        break;
        case ENCODELINE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.line);
        }
        break;
        case ENCODERATIOS:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.ratiose);
        }
        break;
        case ENCODERATIOSM:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.ratiosm);
        }
        break;
        case ENCODEESTC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.ESTC);
        }
        break;
        case ENCODEESTB:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.ESTB);
        }
        break;
        case ENCODEESCNT:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.ESCNT);
        }
        break;
        case ENCODEASOFFSET:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.ASOffset);
        }
        break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        case HOMEPOLE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->homeparam.HMSenPol);
        }
        break;
        case HOMEOFFSET:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->homeparam.HOffset);
        }
        break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;

        default:
        {
            sprintf((char *)strtmp, "%s0;", strtmp);
        }
        break;
        }
    }

    return 0;
}

/*

获取电机参数
*/
static uint8_t PCGetParam(uint8_t *strtmp, StepMotorParams *params)
{
    uint8_t i = 0;
    // uint8_t strtmp[2048] ;

    for (i = 0; i < 100; i++)
    {
        switch (i)
        {
        case BOARDID:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->board.boardid);
        }
        break;
        case MOTORID:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->board.motorid);
        }
        break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%f",);}break;
        case RUNCURRENT:
        {
            sprintf((char *)strtmp, "%s%f;", strtmp, params->runcur);
        }
        break;
        case HOLDCURRENT:
        {
            sprintf((char *)strtmp, "%s%f;", strtmp, params->holdcur); //
        }
        break;
        case MICROSTEP:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->mstep);
        }
        break;
        case RUNMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->runmode);
        }
        break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        case RUNRUNDIR:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.dir);
        }
        break;
        case RUNSLOPEMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.SlopeMode);
        }
        break;
        case RUNCLOSEMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.clmode);
        }
        break;
        case RUNBREAKSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.VBreak);
        }
        break;
        case RUNSTARTACC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.AStart);
        }
        break;
        case RUNSTARTSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.VStart);
        }
        break;
        case RUNSTOPSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.VStop);
        }
        break;
        case RUNDFINA:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.DFinal);
        }
        break;
        case RUNRUNSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.Vmax);
        }
        break;
        case RUNRUNACC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.AMax);
        }
        break;
        case RUNRUNDEC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->run.DMax);
        }
        break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        case HOMERUNDIR:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.dir);
        }
        break;
        case HOMESLOPEMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.SlopeMode);
        }
        break;
        case HOMECLOSEMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.clmode);
        }
        break;
        case HOMEBREAKSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.VBreak);
        }
        break;
        case HOMESTARTACC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.AStart);
        }
        break;
        case HOMESTARTSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.VStart);
        }
        break;
        case HOMESTOPSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.VStop);
        }
        break;
        case HOMEDFINA:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.DFinal);
        }
        break;
        case HOMERUNSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.Vmax);
        }
        break;
        case HOMERUNACC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.AMax);
        }
        break;
        case HOMERUNDEC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->home.DMax);
        }
        break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        case FASETHOMERUNDIR:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.dir);
        }
        break;
        case FASETHOMESLOPEMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.SlopeMode);
        }
        break;
        case FASETHOMECLOSEMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.clmode);
        }
        break;
        case FASETHOMEBREAKSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.VBreak);
        }
        break;
        case FASETHOMESTARTACC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.AStart);
        }
        break;
        case FASETHOMESTARTSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.VStart);
        }
        break;
        case FASETHOMESTOPSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.VStop);
        }
        break;
        case FASETHOMEDFINA:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.DFinal);
        }
        break;
        case FASETHOMERUNSPEED:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.Vmax);
        }
        break;
        case FASETHOMERUNACC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.AMax);
        }
        break;
        case FASETHOMERUNDEC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->fasthome.DMax);
        }
        break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        case ENCODEENABLE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.enable);
        }
        break;
        case ENCODEDIR:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.Dir);
        }
        break;
        case ENCODEENCRES:
        {
            sprintf((char *)strtmp, "%s%f;", strtmp, params->encode.EncRes);
        }
        break;
        case ENCODEENCODEMODE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.EncDiff);
        }
        break;
        case ENCODESIGNSCALE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.signscale);
        }
        break;
        case ENCODELINE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.line);
        }
        break;
        case ENCODERATIOS:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.ratiose);
        }
        break;
        case ENCODERATIOSM:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.ratiosm);
        }
        break;
        case ENCODEESTC:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.ESTC);
        }
        break;
        case ENCODEESTB:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.ESTB);
        }
        break;
        case ENCODEESCNT:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.ESCNT);
        }
        break;
        case ENCODEASOFFSET:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->encode.ASOffset);
        }
        break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        case HOMEPOLE:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->homeparam.HMSenPol);
        }
        break;
        case HOMEOFFSET:
        {
            sprintf((char *)strtmp, "%s%ld;", strtmp, params->homeparam.HOffset);
        }
        break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;
        // case		:{//sprintf((char*)strtmp,"%ld",);}break;

        default:
        {
            sprintf((char *)strtmp, "%s0;", strtmp);
        }
        break;
        }
    }

    return 0;
}

static uint8_t ParamSaveInRam(const uint16_t motorname, uint8_t paramname, uint8_t *param, StepMotorParams *params)
{
    uint8_t paramnumber = 0;
    uint8_t *paramtmp[100] = {0};
    uint64_t tmp;
    int count;

    memset(params, 0, sizeof(StepMotorParams));

    if (paramname == 0)
    {
        split((char *)param, ";", (char **)paramtmp, &count);
    }

    for (; paramname < count; paramname++)
    {

        switch (paramname)
        {
        case BOARDID:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->board.boardid = (uint8_t)(tmp);
        }
        break;
        case MOTORID:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->board.motorid = (uint8_t)(tmp);
        }
        break;
        // case		:{}break;
        // case		:{}break;
        case RUNCURRENT:
        {
            params->runcur = (float)(atof((char *)paramtmp[paramname]));
        }
        break;
        case HOLDCURRENT:
        {
            params->holdcur = (float)(atof((char *)paramtmp[paramname])); // holdcur
        }
        break;
        case MICROSTEP:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->mstep = (uint16_t)(tmp);
        }
        break;
        case RUNMODE:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->runmode = (uint8_t)(tmp);
        }
        break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        case RUNRUNDIR:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->run.dir = (uint8_t)(tmp);
        }
        break;
        case RUNSLOPEMODE:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->run.SlopeMode = (uint8_t)(tmp);
        }
        break;
        case RUNCLOSEMODE:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->run.clmode = (uint8_t)(tmp);
        }
        break;
        case RUNBREAKSPEED:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->run.VBreak = (uint32_t)(tmp);
        }
        break;
        case RUNSTARTACC:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->run.AStart = (uint32_t)(tmp);
        }
        break;
        case RUNSTARTSPEED:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->run.VStart = (uint32_t)(tmp);
        }
        break;
        case RUNSTOPSPEED:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->run.VStop = (uint32_t)(tmp);
        }
        break;
        case RUNDFINA:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->run.DFinal = (uint32_t)(tmp);
        }
        break;
        case RUNRUNSPEED:
        {
            tmp = atoi((char *)paramtmp[paramname]);
            params->run.Vmax = (int_fast32_t)(tmp);
        }
        break;
        case RUNRUNACC:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->run.AMax = (uint32_t)(tmp);
        }
        break;
        case RUNRUNDEC:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->run.DMax = (uint32_t)(tmp);
        }
        break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        case HOMERUNDIR:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->home.dir = (uint8_t)(tmp);
        }
        break;
        case HOMESLOPEMODE:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->home.SlopeMode = (uint8_t)(tmp);
        }
        break;
        case HOMECLOSEMODE:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->home.clmode = (uint8_t)(tmp);
        }
        break;
        case HOMEBREAKSPEED:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->home.VBreak = (uint32_t)(tmp);
        }
        break;
        case HOMESTARTACC:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->home.AStart = (uint32_t)(tmp);
        }
        break;
        case HOMESTARTSPEED:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->home.VStart = (uint32_t)(tmp);
        }
        break;
        case HOMESTOPSPEED:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->home.VStop = (uint32_t)(tmp);
        }
        break;
        case HOMEDFINA:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->home.DFinal = (uint32_t)(tmp);
        }
        break;
        case HOMERUNSPEED:
        {
            tmp = atoi((char *)paramtmp[paramname]);
            params->home.Vmax = (int_fast32_t)(tmp);
        }
        break;
        case HOMERUNACC:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->home.AMax = (uint32_t)(tmp);
        }
        break;
        case HOMERUNDEC:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->home.DMax = (uint32_t)(tmp);
        }
        break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        case FASETHOMERUNDIR:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->fasthome.dir = (uint8_t)(tmp);
        }
        break;
        case FASETHOMESLOPEMODE:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->fasthome.SlopeMode = (uint8_t)(tmp);
        }
        break;
        case FASETHOMECLOSEMODE:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->fasthome.clmode = (uint8_t)(tmp);
        }
        break;
        case FASETHOMEBREAKSPEED:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->fasthome.VBreak = (uint32_t)(tmp);
        }
        break;
        case FASETHOMESTARTACC:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->fasthome.AStart = (uint32_t)(tmp);
        }
        break;
        case FASETHOMESTARTSPEED:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->fasthome.VStart = (uint32_t)(tmp);
        }
        break;
        case FASETHOMESTOPSPEED:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->fasthome.VStop = (uint32_t)(tmp);
        }
        break;
        case FASETHOMEDFINA:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->fasthome.DFinal = (uint32_t)(tmp);
        }
        break;
        case FASETHOMERUNSPEED:
        {
            tmp = atoi((char *)paramtmp[paramname]);
            params->fasthome.Vmax = (int_fast32_t)(tmp);
        }
        break;
        case FASETHOMERUNACC:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->fasthome.AMax = (uint32_t)(tmp);
        }
        break;
        case FASETHOMERUNDEC:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->fasthome.DMax = (uint32_t)(tmp);
        }
        break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        case ENCODEENABLE:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->encode.enable = (uint8_t)(tmp);
        }
        break;
        case ENCODEDIR:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->encode.Dir = (uint8_t)(tmp);
        }
        break;
        case ENCODEENCRES:
        {
            tmp = atof((char *)paramtmp[paramname]);
            params->encode.EncRes = (float)(atof((char *)paramtmp[paramname]));
        }
        break;
        case ENCODEENCODEMODE:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->encode.EncDiff = (uint8_t)(tmp);
        }
        break;
        case ENCODESIGNSCALE:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->encode.signscale = (uint16_t)(tmp);
        }
        break;
        case ENCODELINE:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->encode.line = (uint16_t)(tmp);
        }
        break;
        case ENCODERATIOS:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->encode.ratiose = (uint16_t)(tmp);
        }
        break;
        case ENCODERATIOSM:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->encode.ratiosm = (uint16_t)(tmp);
        }
        break;
        case ENCODEESTC:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->encode.ESTC = (uint16_t)(tmp);
        }
        break;
        case ENCODEESTB:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->encode.ESTB = (uint16_t)(tmp);
        }
        break;
        case ENCODEESCNT:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->encode.ESCNT = (uint16_t)(tmp);
        }
        break;
        case ENCODEASOFFSET:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->encode.ASOffset = (uint32_t)(tmp);
        }
        break;
        // case		:{}break;
        // case		:{}break;
        case HOMEPOLE:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->homeparam.HMSenPol = (uint8_t)(tmp);
        }
        break;
        case HOMEOFFSET:
        {
            tmp = atol((char *)paramtmp[paramname]);
            params->homeparam.HOffset = (uint32_t)(tmp);
        }
        break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;
        // case		:{}break;

        default:
            break;
        }
    }

    SetMotorParamFromPC(motorname, params);

    return 0;
}

static uint8_t PackageMotorParam(uint16_t motorname, uint8_t *str)
{
    StepMotorParams params;

    memset(&params, 0, sizeof(StepMotorParams));

    switch (motorname)
    {
    case TABLE_CHAININ_F:
    {
        CpyMotorToParamStruct((MotorStruDef *)&Table.CHIF, &params);
    }
    break;
    case TABLE_CHAININ_B:
    {
        CpyMotorToParamStruct((MotorStruDef *)&Table.CHIR, &params);
    }
    break;
    case TABLE_SCANMOTOR:
    {
        CpyMotorToParamStruct((MotorStruDef *)&Table.PBI, &params);
    }
    break;
    case TABLE_SAMPLEINMOTOR:
    {
        CpyMotorToParamStruct((MotorStruDef *)&Table.PPI, &params);
    }
    break;
    case TABLE_CHAINBACK_F:
    {
        CpyMotorToParamStruct((MotorStruDef *)&Table.CHBF, &params);
    }
    break;
    case TABLE_CHAINBACK_B:
    {
        CpyMotorToParamStruct((MotorStruDef *)&Table.CHBR, &params);
    }
    break;
    case TABLE_BACKMOTR:
    {
        CpyMotorToParamStruct((MotorStruDef *)&Table.PBB, &params);
    }
    break;
    case TABLE_RECYLEMOTOR:
    {
        CpyMotorToParamStruct((MotorStruDef *)&Table.PPB, &params);
    }
    break;
    case RACK_WAITPOSMOTOR:
    {
        // for (uint8_t i = 0; i < MaxTrackNumber; i++)
        {
            CpyMotorToParamStruct((MotorStruDef *)&Track[0].WBF, &params);
        }
    }
    break;
    case RACK_BACKBLOCKMOTOR:
    {
        // for (uint8_t i = 0; i < MaxTrackNumber; i++)
        {
            CpyMotorToParamStruct((MotorStruDef *)&Track[0].BPB, &params);
        }
    }
    break;
    case RACK_NORMALLONGMOTOR:
    {
        // for (uint8_t i = 0; i < MaxTrackNumber; i++)
        {
            CpyMotorToParamStruct((MotorStruDef *)&Track[0].NBL, &params);
        }
    }
    break;
    case RACK_EMGLONGMOTOR:
    {
        // for (uint8_t i = 0; i < MaxTrackNumber; i++)
        {
            CpyMotorToParamStruct((MotorStruDef *)&Track[0].EBL, &params);
        }
    }
    break;
    case RACK_CHANGRACKMOTOR:
    {
        // for (uint8_t i = 0; i < MaxTrackNumber; i++)
        {
            CpyMotorToParamStruct((MotorStruDef *)&Track[0].MTC, &params);
        }
    }
    break;
    case RACK_NORMALBLOCKMOTOR:
    {
        // for (uint8_t i = 0; i < MaxTrackNumber; i++)
        {
            CpyMotorToParamStruct((MotorStruDef *)&Track[0].MBF, &params);
        }
    }
    break;
    case RACK_EMGBLOCKMOTOR:
    {
        // for (uint8_t i = 0; i < MaxTrackNumber; i++)
        {
            CpyMotorToParamStruct((MotorStruDef *)&Track[0].EPB, &params);
        }
    }
    break;
    case RACK_BACKLONGMOTOR:
    {
        // for (uint8_t i = 0; i < MaxTrackNumber; i++)
        {
            CpyMotorToParamStruct((MotorStruDef *)&Track[0].BBL, &params);
        }
    }
    break;
    case RACK_NORMALSHORTMOTOR:
    {
        // for (uint8_t i = 0; i < MaxTrackNumber; i++)
        {
            CpyMotorToParamStruct((MotorStruDef *)&Track[0].NBS, &params);
        }
    }
    break;
    case RACK_EMGSHORTMOTOR:
    {
        // for (uint8_t i = 0; i < MaxTrackNumber; i++)
        {
            CpyMotorToParamStruct((MotorStruDef *)&Track[0].EBS, &params);
        }
    }
    break;
    case RACK_BACKSHORTMOTOR:
    {
        // for (uint8_t i = 0; i < MaxTrackNumber; i++)
        {
            CpyMotorToParamStruct((MotorStruDef *)&Track[0].BBS, &params);
        }
    }
    break;
    case RACK_CHANGBLOCKMOTOR:
    {
        // for (uint8_t i = 0; i < MaxTrackNumber; i++)
        {
            CpyMotorToParamStruct((MotorStruDef *)&Track[0].MBF, &params);
        }
    }
    break;
    default:
        break;
    }

    PCGetParam(str, &params);
    return 0;
}

/**
 * @brief 设置电机参数
 * @param  None
 * @retval None
 */
static void EthCmdDeviceSetMotorParamTask(void *pvParameters)
{
    EthPkgDef *pkg = (void *)pvParameters;
    uint8_t errorflg = 0; //设置时间返回标记 ，0：表示成功  1，表示失败
    uint8_t strtmp[EthDataLen] = {0};
    uint16_t motorname;
    uint8_t paramname;

    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();

    StepMotorParams params;
    memset(strtmp, 0, EthDataLen);

    if (EthXmlCmpFind(pkg->data, "motorname", (char *)strtmp) != 0)
    {
        errorflg = 1;
    }

    if (errorflg == 0)
    {

        motorname = atol((char *)strtmp);
        if (EthXmlCmpFind(pkg->data, "paraname", (char *)strtmp) != 0)
        {
            errorflg = 1;
        }

        if (errorflg == 0)
        {
            paramname = atol((char *)strtmp);
            memset(strtmp, 0, EthDataLen);
            if (EthXmlCmpFind(pkg->data, "paradata", (char *)strtmp) != 0)
            {
                errorflg = 1;
            }

            if (errorflg == 0)
            {
                // ParamSaveInRam(motorname, paramname, strtmp, &params);
                ///< (const uint8_t motorid, const uint8_t paramid, char *param)
                usermotorparam.ParsingProtocol(motorname, paramname, strtmp);
            }
        }
    }

    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 仪器编号、主板编号、IP地址读取（CmdDeviceVersion）
 * @param  None
 * @retval None
 */
void EthCmdDeviceInfoTask(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    (void)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    WorkRecordReadWrite(0, 0, "INFO ETH RXCMD DeviceInfo");
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, DeviceVersion Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    // EthPkgBuf_Send[SendIdx].cmd = EthDevVer;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>SNGetResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <macaddr></macaddr>\n\
    <macsn></macsn>\n\
    <ctrlboardsn></ctrlboardsn>\n\
    <ipaddr></ipaddr>\n\
    <ipcomport></ipcomport>\n\
    <software></software>\n\
  </data>\n\
</root>");

    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%02X:%02X:%02X:%02X:%02X:%02X",
            ETH_MAC_ADDR0, ETH_MAC_ADDR1, ETH_MAC_ADDR2, ETH_MAC_ADDR3, ETH_MAC_ADDR4, ETH_MAC_ADDR5);

    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "macaddr", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    memcpy((char *)strtmp, MacSN, 20);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "macsn", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    memcpy((char *)strtmp, CtrlBoardSN, 20); // MacSN
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "ctrlboardsn", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    memcpy((char *)strtmp, EthSource, strlen(EthSource));
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "ipaddr", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    } // yfxiao

    memset(strtmp, 0, EthDataLen);
    memcpy((char *)strtmp, "7", 2);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "ipcomport", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    } // yfxiao

    memset(strtmp, 0, EthDataLen);
    memcpy((char *)strtmp, ProjectVersion, strlen(ProjectVersion));
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "software", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    } // yfxiao

    memset(strtmp, 0, EthDataLen);
    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    WorkRecordReadWrite(0, 0, "INFO ETH TXCMD DeviceInfoResult");
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS DeviceVersionResult");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        WorkRecordReadWrite(0, 0, "INFO ETH RXACK DeviceVersionResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK DeviceVersionResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 上传告警信息入口
 * @param  DeviceAlarmStruDef* AlarmStru 告警结构体
 * @retval None
 */
uint8_t EthSendAlarm(DeviceAlarmStruDef *AlarmStru)
{
    uint8_t handle_index = EthHandleIndexGet();
    if (handle_index == 0xff) //没有空余的线程句柄了
    {
        return 1;
    }
    if (AlarmStru->code == RecoveryFullCode)
    {
        while (EthSendRecoveryFulltoStopAlarm()!=0)
        {
            vTaskDelay(10);
        }
    }
    xUserTaskCreate(EthCmdAlarmTask, "EthCmdAlarmTask", configMINIMAL_STACK_SIZE * 2, (void *)AlarmStru, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
    return 0;
}

/**
 * @brief 上传告警信息（CmdAlarm）
 * @param  None
 * @retval None
 */
static void EthCmdAlarmTask(void *pvParameters)
{
    DeviceAlarmStruDef *alarmtemp = (DeviceAlarmStruDef *)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    while (1)
    {
        SendIdx = EthSendIndexGet();
        if (SendIdx != UINT32_MAX)
        {
            break;
        }
        vTaskDelay(10);
    }
    // EthPkgBuf_Send[SendIdx].cmd = EthAlarm;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>Alarm</command>\n\
  <priority>9</priority>\n\
  <data>\n\
    <level></level>\n\
    <code></code>\n\
    <info></info>\n\
  </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //错误级别
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%d", alarmtemp->level);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "level", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //错误代号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", alarmtemp->code); // yfxiao
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "code", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //其他信息
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", alarmtemp->info);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "info", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    WorkRecordReadWrite(0, 0, "INFO ETH TXCMD Alarm");
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS Alarm");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        WorkRecordReadWrite(0, 0, "INFO ETH RXACK Alarm");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK Alarm");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 系统重启（CmdSystemReset）
 * @param  None
 * @retval None
 */
static void EthCmdSystemResetTask(void *pvParameters)
{
    (void)pvParameters;
    WorkRecordReadWrite(0, 0, "INFO ETH RXCMD SystemReset");
    SD_Log_Write();
    vTaskDelay(500);
    //系统复位
    HAL_NVIC_SystemReset();
    while (1)
    {
    }
}

/**
 * @brief 系统进入升级模式（CmdOTA）
 * @param  None
 * @retval None
 */
uint32_t backupram __attribute__((at(0x38800000)));
void EthOTATask(void *pvParameters)
{
    (void)pvParameters;
    //定义备份域

    //打开备份域SRAM
    PWR->CR1 |= PWR_CR1_DBP;
    while ((PWR->CR1 & PWR_CR1_DBP) == RESET)
    {
    }
    __HAL_RCC_BKPRAM_CLK_ENABLE();
    backupram = 0xa55aa55a; //告知bootloader进行升级
    WorkRecordReadWrite(0, 0, "INFO ETH RXCMD OTA");
    SD_Log_Write();
    vTaskDelay(500);
    //系统复位
    HAL_NVIC_SystemReset();
    while (1)
    {
    }
}

// /****************************************************************************/

/**
 * @brief 通知轨道开始运行（CmdTestStart）
 * @param  None
 * @retval 0成功 1失败
 */
uint8_t EthCmdTestStart(void)
{
    if (((DeviceStatus == StandBy) || (DeviceStatus == Pause)) || (ChainIn.RunCtl == _cctl_userpause || ChainIn.RunCtl == _cctl_autopause)) //&& (ChainIn.RunCtl != _cctl_run)
    {
        DeviceStatusChg = Run;
        while (DeviceStatus != Run)
        {
            vTaskDelay(1);
        }
        ChainIn.RunCtl = _cctl_run;
        // yfxiao 05-14
        // scanner_flg = 3;
        WorkRecordReadWrite(0, 0, "INFO ETH RXCMD TestStart OK");
        return 0;
    }
    WorkRecordReadWrite(0, 0, "INFO ETH RXCMD TestStart FAIL");
    return 1;
}

/**
 * @brief 停止测试（CmdCancelDo）
 * @param  None
 * @retval 0成功 1失败
 */
void EthCmdCancelDo(void)
{
    taskENTER_CRITICAL();

    if (DeviceStatus == None)
    {
        DeviceStatus = Stop;
    }
    else
    {
        DeviceStatusChg = Stop;
    }
    taskEXIT_CRITICAL();
    WorkRecordReadWrite(0, 0, "INFO ETH RXCMD CancelDo");
}

/**
 * @brief 通知轨道线程发送当前样本信息（CmdScanTrackBarcode）
 * @param  None
 * @retval 0--成功 1--失败
 */
uint8_t EthCmdScanTrackBarcode(void)
{
    //已经发送过才会重发
    if (FrmBarcodeSend == 1)
    {
        FrmBarcodeSend = 0;
        WorkRecordReadWrite(0, 0, "INFO ETH RXCMD ScanTrackBarcode OK");
        return 0;
    }
    else
    {
        WorkRecordReadWrite(0, 0, "INFO ETH RXCMD ScanTrackBarcode FAIL");
        return 1;
    }
}

/**
 * @brief 将系统状态发送给上位机（CmdGetInstrumentStatus）
 * @param  None
 * @retval None
 */
static void EthCmdGetInstrumentStatusTask(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    (void)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t TxIDTemp = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    // WorkRecordReadWrite(0, 0, "INFO ETH RXCMD GetInstrumentStatus");
    //获取发送包
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, GetInstrumentStatus Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    // EthPkgBuf_Send[SendIdx].cmd = EthStatus;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    TxIDTemp = EthPkgBuf_Send[SendIdx].cmdid;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    // EthPkgBuf_Send[SendIdx].dealtask=&EthCmdGetInstrumentStatusTaskHandle;
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>InstrumentStatus</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <status></status>\n\
    <operation></operation>\n\
  </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //系统状态
    switch (DeviceStatus)
    {
    case _NULL:
    case None:
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "status", 4, "None") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
        break;

    case StandBy:
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "status", 4, "StandBy") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
        break;

    case _Reset:
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "status", 4, "Reset") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
        break;

    case Run:
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "status", 4, "Run") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
        break;

    case Pause:
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "status", 4, "Pause") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
        break;

    case Stop:
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "status", 4, "Stop") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
        break;

    case Error:
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "status", 4, "Error") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
        break;

    default:
        break;
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    //工作日志记录

    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "WARN ETH TXRS InstrumentStatus TxID %u", TxIDTemp);
            WorkRecordReadWrite(0, 0, LogBuffer);
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "INFO ETH RXACK InstrumentStatus TxID %u", TxIDTemp);
        WorkRecordReadWrite(0, 0, LogBuffer);
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    else
    {
        //发送失败
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "WARN ETH TXNAK InstrumentStatus TxID %u", TxIDTemp);
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 将扫描结果发送给上位机接口
 * @param  uint8_t SID：样本架SID号
 * @retval None
 */
BarcodeInfo sendbarcode;
uint8_t EthSendBarcode(uint8_t *sid, uint8_t autotmp)
{

    uint8_t handle_index = EthHandleIndexGet();
    if (handle_index == 0xff) //没有空余的线程句柄了
    {
        return 1;
    }
    sendbarcode.sid = (uint8_t)*sid;
    sendbarcode.isauto = autotmp;
    xUserTaskCreate(EthCmdScanTrackBarcodeTask, "EthCmdScanTrackBarcodeTask", configMINIMAL_STACK_SIZE * 2, (void *)&sendbarcode, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
    return 0;
}

/**
 * @brief 自动上报开始按键触发（ButtonTarget）
 * @param  None
 * @retval None
 */
void EthCmdButtonPuress(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    (void)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    WorkRecordReadWrite(0, 0, "INFO ETH RXCMD ButtonTarget");
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, ButtonTarget Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    // EthPkgBuf_Send[SendIdx].cmd = EthDevVer;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>ButtonTarget</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <button>StartButton</button>\n\
  </data>\n\
</root>");

    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    WorkRecordReadWrite(0, 0, "INFO ETH TXCMD ButtonTarget");
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS ButtonTargetResult");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        WorkRecordReadWrite(0, 0, "INFO ETH RXACK ButtonTargetResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK ButtonTargetResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 自动上报开始按键触发状态（ButtonTarget）
 * @param  void
 * @retval None
 */
uint8_t EthSendButtonStart(void)
{
    uint8_t handle_index = EthHandleIndexGet();
    if (handle_index == 0xff) //没有空余的线程句柄了
    {
        return 1;
    }
    xUserTaskCreate(EthCmdButtonPuress, "EthCmdAlarmTask", configMINIMAL_STACK_SIZE * 2, NULL, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
    return 0;
}

/**
 * @brief 发送进样链条状态（chaininstatus）
 * @param  void
 * @retval None
 */
static void EthCmdChainInStatus(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    (void)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    WorkRecordReadWrite(0, 0, "INFO ETH RXCMD chaininstatus");
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, chaininstatus Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    // EthPkgBuf_Send[SendIdx].cmd = EthDevVer;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>chaininstatus</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <status></status>\n\
  </data>\n\
</root>"); // yfxiao 2021-07-19

    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    if (ChainIn.RunCtl == _cctl_autopause || ChainIn.RunCtl == _crst_Idle)
    {
        memcpy((char *)strtmp, "null", 4);
    }
    else if (ChainIn.RunCtl == _cctl_run && waitscannerflg == 0)
    {
        memcpy((char *)strtmp, "run", 4);
    }
    else if (ChainIn.RunCtl == _cctl_stop)
    {
        memcpy((char *)strtmp, "stop", 4);
    }
    else if (ChainIn.RunCtl == _cctl_run && waitscannerflg == 1)
    {
        memcpy((char *)strtmp, "wait", 4);
    }
    else
    {
        memcpy((char *)strtmp, "null", 4);
    }

    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "status", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    WorkRecordReadWrite(0, 0, "INFO ETH TXCMD chaininstatusResult");
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS chaininstatusResult");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        WorkRecordReadWrite(0, 0, "INFO ETH RXACK chaininstatusResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK chaininstatusResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 将进样链条状态发送给上位机
 * @param  void
 * @retval None
 */
uint8_t EthSendChainInStatus(void)
{
    uint8_t handle_index = EthHandleIndexGet();
    if (handle_index == 0xff) //没有空余的线程句柄了
    {
        return 1;
    }
    xUserTaskCreate(EthCmdChainInStatus, "EthCmdChainInStatus", configMINIMAL_STACK_SIZE * 2, NULL, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
    return 0;
}

#if 1

/**
 * @brief 将样本架到达等待位发送给上位机接口
 * @param  uint8_t SID：样本架SID号
 * @retval None
 */
uint8_t EthSendWaitpos(uint8_t *sid) // yfxiao 2021-04-26
{
    uint8_t handle_index = EthHandleIndexGet();
    if (handle_index == 0xff) //没有空余的线程句柄了
    {
        return 1;
    }
    xUserTaskCreate(EthCmdTrackWaitposTask, "EthCmdTrackWaitposTask", configMINIMAL_STACK_SIZE * 2, (void *)sid, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
    return 0;
}

/**
 * @brief 将样本架到达等待位发送给上位机(TrackGotoDeviceResult)
 * @param  SID：样本架SID号
 * @retval None
 */
void EthCmdTrackWaitposTask(void *pvParameters)
{
    uint8_t SID = *(uint8_t *)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, ScanTrackBarcode Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }

    if (Frame[SID].Recorde == 1)
    {
        memset(strtmp, 0, EthDataLen);
        sprintf((char *)strtmp, "frame id %ld is goto recyle zone", Frame[SID].FramSID);
        WorkRecordReadWrite(0, 0, (char *)strtmp);
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id>2</id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>TrackGotoDeviceResult</command>\n\
  <priority>normal</priority>\n\
  <data>\n\
    <rack></rack>\n\
    <device></device>\n\
    <emer></emer>\n\
    <pos></pos>\n\
    <state>wait</state>\n\
    <result>true</result>\n\
  </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //样本架号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", (uint32_t)Frame[SID].FrameID);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "rack", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //样本架号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", (uint32_t)Frame[SID].FrameDevice);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "device", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    WorkRecordReadWrite(0, 0, "INFO ETH TXCMD TrackGotoDeviceResult");
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < 1; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, 3174 / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS TrackGotoDeviceResult");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        WorkRecordReadWrite(0, 0, "INFO ETH RXACK TrackGotoDeviceResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK ScanTrackBarcodeResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}
#endif

/**
 * @brief 将扫描结果发送给上位机（CmdScanTrackBarcodeResult）
 * @param  SID：样本架SID号
 * @retval None
 */
static void EthCmdScanTrackBarcodeTask(void *pvParameters)
{
    BarcodeInfo *barcodeinfo = (void *)pvParameters;
    uint8_t SID = barcodeinfo->sid;
    uint8_t i = 0;
    DEVRESETStatus tmp;
    uint8_t strtmp[EthDataLen] = {0};
    uint8_t strtmp2[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, ScanTrackBarcode Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }

    if (EthDbgMode == 0 && barcodeinfo->isauto == 0) //主动上报并且在正常测试状态需要判断架子是否重复
    {
        for (i = 0; i < MaxFrameNumber; i++)
        {
            if (i != SID && Frame[i].Inuse == 1 && Frame[i].Recorde == 0 && Frame[i].FrameID == Frame[SID].FrameID)
            {
                tmp.SSID = SID;
                Frame[SID].Recorde = 1;
                tmp.dev = 12;
                tmp.ResetStatus = 10;
                xUserTaskCreate(TrackGotoDeviceTask, "TrackGotoDeviceTask", configMINIMAL_STACK_SIZE, (void *)&tmp, SampleCtl_Priority, NULL);
            }
        }
    }

    // EthPkgBuf_Send[SendIdx].cmd = EthScanTrackBarcode;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>ScanTrackBarcodeResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <auto></auto>\n\
    <rack></rack>\n\
  </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    //是否是主动上报
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%d", (uint8_t)barcodeinfo->isauto);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "auto", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    //样本架号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", (uint32_t)Frame[SID].FrameID);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "rack", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    for (uint8_t i = 0; i < 10; i++)
    {
        if (Frame[SID].Sample[i].Type == 0) //无样本
        {
            continue;
        }
        memset(strtmp, 0, EthDataLen);
        strcpy((char *)strtmp, "\
  <result>\n\
    <pos></pos>\n\
    <type></type>\n\
    <barcode></barcode>\n\
  </result>\n");

        //位置号
        memset(strtmp2, 0, EthDataLen);
        sprintf((char *)strtmp2, "%d", i + 1);
        if (EthXmlCmpAdd((char *)strtmp, "pos", 4, (char *)strtmp2) == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }

        //类型
        if (Frame[SID].Sample[i].Type == 1) //微量杯
        {
            if (EthXmlCmpAdd((char *)strtmp, "type", 4, "low") == 1)
            {
                //错误处理
                WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
                EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
            }
        }
        else if (Frame[SID].Sample[i].Type == 2) //采血管
        {
            if (EthXmlCmpAdd((char *)strtmp, "type", 4, "high") == 1)
            {
                //错误处理
                WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
                EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
            }
        }

        //条码
        if (EthXmlCmpAdd((char *)strtmp, "barcode", 4, (char *)Frame[SID].Sample[i].SampleIDA) == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }

        //总
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "data", 5, (char *)strtmp) == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    WorkRecordReadWrite(0, 0, "INFO ETH TXCMD ScanTrackBarcodeResult");
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS ScanTrackBarcodeResult");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        WorkRecordReadWrite(0, 0, "INFO ETH RXACK ScanTrackBarcodeResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
        //判断是否重复，如果重复送回到回收区
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK ScanTrackBarcodeResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 查询样本架在加样位位置（CmdEnterDebugMode）
 * @param  FrameData:上位机传入的样本架信息(str)
 * @retval None
 */
static void EthCmdEnableScanMode(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    EthPkgDef *pkg = (void *)pvParameters;

    uint8_t tmp[5];

    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();

    if (EthXmlCmpFind(pkg->data, "mode", (char *)tmp) == 0)
    {
        RACKRUNMODE = (uint8_t)(atol((char *)tmp));
    }

    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief  获取上位机是否需要扫码
 * @param  void
 * @retval None  static void EthCmdGetEnableScanMode(void *pvParameters)
 */
static void EthCmdGetEnableScanMode(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    (void)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    WorkRecordReadWrite(0, 0, "INFO ETH RXCMD GetEnableScanMode");
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, ButtonTarget Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    // EthPkgBuf_Send[SendIdx].cmd = EthDevVer;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>GetEnableScanMode</command>\n\
  <priority>1</priority>\n\
</root>");

    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    WorkRecordReadWrite(0, 0, "INFO ETH TXCMD GetEnableScanMode");
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS ButtonTargetResult");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        WorkRecordReadWrite(0, 0, "INFO ETH RXACK ButtonTargetResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK ButtonTargetResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief  获取是否需要扫码
 * @param  void
 * @retval None  EthCmdGetEnableScanMode
 */
uint8_t EthSendGetEnableScanMode(void)
{
    uint8_t handle_index = EthHandleIndexGet();
    if (handle_index == 0xff) //没有空余的线程句柄了
    {
        return 1;
    }
    xUserTaskCreate(EthCmdGetEnableScanMode, "EthCmdGetEnableScanMode", configMINIMAL_STACK_SIZE * 2, NULL, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
    return 0;
}

/**
 * @brief 查询样本架在加样位位置（CmdEnterDebugMode）
 * @param  FrameData:上位机传入的样本架信息(str)
 * @retval None
 */
static void EthCmdEnterDebugModeTask(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    EthPkgDef *pkg = (void *)pvParameters;

    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    EthDbgSetParam(pkg);
    WorkRecordReadWrite(0, 0, "INFO ETH RXCMD SETTIME");
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, DeviceVersion Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }

    EthDbgEnterDebugMode();

    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>EnterDebugModeResult</command>\n\
  <priority>normal</priority>\n\
  <data>\n\
    <status></status>\n\
  </data>\n\
</root>");

    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);

    memcpy((char *)strtmp, "true", 4);

    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "status", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    WorkRecordReadWrite(0, 0, "INFO ETH TXCMD EthCmdEnterDebugModeTask");
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS EthCmdEnterDebugModeTask");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        WorkRecordReadWrite(0, 0, "INFO ETH RXACK EthCmdEnterDebugModeTask");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK EthCmdEnterDebugModeTask");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 查询样本架在加样位位置（CmdIsRackArriveLocationResult）
 * @param  FrameData:上位机传入的样本架信息(str)
 * @retval None
 */
static void EthCmdGetRackArriveLocationResultTask(void *pvParameters)
{
    EthPkgDef *pkg = (void *)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    uint8_t ErrorFlag = 0;
    uint32_t FrameIDTemp = 0;
    uint8_t FrameSID = 0;
    uint8_t emgflg = 0;
    uint32_t device = 0;
    uint32_t pos = 0;
    emgflg = 10;
    device = 10;
    pos = 100;
    //    uint8_t u8Temp = 0;

    //获取发送包
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, GetRackLocation Ignored");
        //清空接收包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }

    //解析相关内容
    //解析架号
    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "rack", (char *)strtmp) != 0)
    {
        ErrorFlag |= 1;
    }
    FrameIDTemp = atol((char *)strtmp);

    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "device", (char *)strtmp) != 0)
    {
        ErrorFlag |= 1;
    }
    device = atol((char *)strtmp);

    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "aisle", (char *)strtmp) != 0)
    {
        ErrorFlag |= 1;
    }

    if (strcmp((char *)strtmp, "emg") == 0)
    {
        emgflg = 1;
    }
    else if (strcmp((char *)strtmp, "normal") == 0) //
    {
        emgflg = 0;
    }
    else
    {
        emgflg = 0;
    }

    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "pos", (char *)strtmp) != 0)
    {
        ErrorFlag |= 1;
    }
    pos = atol((char *)strtmp);

    if (FrameIDTemp == 0)
    {
        ErrorFlag |= 1;
    }
    else //找样本
    {
        for (uint8_t i = 0; i < MaxFrameNumber; i++)
        {
            if (Frame[i].FrameID == FrameIDTemp && Frame[i].Inuse == 1 && Frame[i].Recorde == 0) // Recorde
            {
                FrameSID = i;
                break;
            }
        }
        if (FrameSID == 0)
        {
            ErrorFlag |= 1; //没找到
        }
    }

    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)pkg->source);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>GetRackArriveLocationResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <rack></rack>\n\
    <result></result>\n\
	<device></device>\n\
    <aisle></aisle>\n\
	<pos></pos>\n\
  </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //样本架号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", (uint32_t)FrameIDTemp);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "rack", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //样本架号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", (uint32_t)device);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "device", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    if (emgflg)
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "aisle", 4, "emg") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }
    else
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "aisle", 4, "normal") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }

    //样本架号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", (uint32_t)pos);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "pos", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    if (Frame[FrameSID].FrameDevice == 0) //异常
    {
        ErrorFlag |= 1;
    }

    //成功
    if (ErrorFlag == 0)
    {
        //位置
        //还需要判断等待位的情况
        if ((Frame[FrameSID].FrmMovStart == 0)) //非运行中
        {
            if (Frame[FrameSID].FrmAddingEmg == emgflg && device == Frame[FrameSID].FrameDevice && Frame[FrameSID].FrmSampleCurPos == pos)
            {
                if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "true") == 1)
                {
                    //错误处理
                    WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
                    EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
                }
            } // 2021-08-18
            else
            {
                if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "false") == 1)
                {
                    //错误处理
                    WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
                    EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
                }
            }
        }
        else //运行中
        {
            if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "moving") == 1)
            {
                //错误处理
                WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
                EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
            }
        }
    }
    else
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "false") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //工作记录
    memset(LogBuffer, 0, sizeof(LogBuffer));
    sprintf(LogBuffer, "INFO ETH TXCMD GetRackArriveLocationResult FrameID:%ld", FrameIDTemp);
    WorkRecordReadWrite(0, 0, LogBuffer);

    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "WARN ETH TXRS GetRackLocationResult FrameID:%ld", FrameIDTemp);
            WorkRecordReadWrite(0, 0, LogBuffer);
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "INFO ETH RXACK GetRackLocationResult FrameID:%ld", FrameIDTemp);
        WorkRecordReadWrite(0, 0, LogBuffer);
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
    else
    {
        //发送失败
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "WARN ETH TXNAK GetRackLocationResult FrameID:%ld", FrameIDTemp);
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}
/**
 * @brief 查询样本架位置（CmdGetRackLocation）
 * @param  FrameData:上位机传入的样本架信息(str)
 * @retval None
 */
static void EthCmdGetRackLocationTask(void *pvParameters)
{
    EthPkgDef *pkg = (void *)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    uint8_t ErrorFlag = 0;
    uint32_t FrameIDTemp = 0;
    uint8_t FrameSID = 0;
    //    uint8_t u8Temp = 0;

    //获取发送包
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, GetRackLocation Ignored");
        //清空接收包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }

    //解析相关内容
    //解析架号
    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "rack", (char *)strtmp) != 0)
    {
        ErrorFlag |= 1;
    }
    FrameIDTemp = atol((char *)strtmp);
    if (FrameIDTemp == 0)
    {
        ErrorFlag |= 1;
    }
    else //找样本
    {
        for (uint8_t i = 0; i < MaxFrameNumber; i++)
        {
            if (Frame[i].FrameID == FrameIDTemp && Frame[i].Inuse == 1 && Frame[i].Recorde == 0)
            {
                FrameSID = i;
                break;
            }
        }
        if (FrameSID == 0)
        {
            ErrorFlag |= 1; //没找到
        }
    }

    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)pkg->source);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>GetRackLocationResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <rack></rack>\n\
    <result></result>\n\
    <location></location>\n\
    <center></center>\n\
  </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //样本架号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", (uint32_t)FrameIDTemp);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "rack", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    if (Frame[FrameSID].FrameDevice == 0) //异常
    {
        ErrorFlag |= 1;
    }

    //成功
    if (ErrorFlag == 0)
    {
        //结果
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "true") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
        //位置
        //还需要判断等待位的情况
        if ((Frame[FrameSID].FrmMovStart == 0)) //非运行中
        {
            if (Frame[FrameSID].FrameDevice < MaxTrackNumber) //轨道的情况
            {
                if (Frame[FrameSID].FrmAddingEmg == 0) //常规
                {
                    //位置信息
                    memset(strtmp, 0, EthDataLen);
                    sprintf((char *)strtmp, "%dA", Frame[FrameSID].FrameDevice);
                    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "location", 4, (char *)strtmp) == 1)
                    {
                        //错误处理
                        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
                        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
                    }
                }
                else //急诊
                {
                    //位置信息
                    memset(strtmp, 0, EthDataLen);
                    sprintf((char *)strtmp, "%dE", Frame[FrameSID].FrameDevice);
                    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "location", 4, (char *)strtmp) == 1)
                    {
                        //错误处理
                        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
                        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
                    }
                }
                //加样位置
                memset(strtmp, 0, EthDataLen);
                sprintf((char *)strtmp, "%d", Frame[FrameSID].FrmSampleCurPos);
                if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "center", 4, (char *)strtmp) == 1)
                {
                    //错误处理
                    WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
                    EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
                }
            }
            else if (Frame[FrameSID].FrameDevice == 12) //扫码区
            {
                //位置信息
                if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "location", 4, "0A") == 1)
                {
                    //错误处理
                    WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
                    EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
                }
            }
            else if (Frame[FrameSID].FrameDevice == 13) //缓存区
            {
                //位置信息
                if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "location", 4, "0B") == 1)
                {
                    //错误处理
                    WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
                    EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
                }
            }
        }
        else if (Frame[FrameSID].FrmSampleCurPos == 0xff) //在等待位
        {
            if (Frame[FrameSID].FrameDevice < MaxFrameNumber) //轨道的情况
            {
                if (Frame[FrameSID].FrmAddingEmg == 0) //常规
                {
                    //位置信息
                    memset(strtmp, 0, EthDataLen);
                    sprintf((char *)strtmp, "%dB", Frame[FrameSID].FrameDevice);
                    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "location", 4, (char *)strtmp) == 1)
                    {
                        //错误处理
                        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
                        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
                    }
                }
            }
        }
        else //运行中
        {
            if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "location", 4, "moving") == 1)
            {
                //错误处理
                WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
                EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
            }
        }
    }
    else
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "false") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //工作记录
    memset(LogBuffer, 0, sizeof(LogBuffer));
    sprintf(LogBuffer, "INFO ETH TXCMD GetRackLocationResult FrameID:%ld", FrameIDTemp);
    WorkRecordReadWrite(0, 0, LogBuffer);

    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "WARN ETH TXRS GetRackLocationResult FrameID:%ld", FrameIDTemp);
            WorkRecordReadWrite(0, 0, LogBuffer);
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "INFO ETH RXACK GetRackLocationResult FrameID:%ld", FrameIDTemp);
        WorkRecordReadWrite(0, 0, LogBuffer);
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
    else
    {
        //发送失败
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "WARN ETH TXNAK GetRackLocationResult FrameID:%ld", FrameIDTemp);
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 样本架进入进入样本回收区/重测区（CmdTrackGotoRecycle）
 * @param  FrameData:上位机传入的样本架信息(str)
 * @retval None
 */
static void EthCmdTrackGotoRecycleTask(void *pvParameters)
{
    EthPkgDef *pkg = (void *)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    uint8_t ErrorFlag = 0;
    uint32_t FrameIDTemp = 0;
    uint8_t FrameSID = 0;
    uint8_t u8Temp = 0;

    //获取发送包
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, TrackGotoRecycle Ignored");
        //清空接收包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
    //解析相关内容
    //解析架号
    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "rack", (char *)strtmp) != 0)
    {
        ErrorFlag |= 1;
    }
    FrameIDTemp = atol((char *)strtmp);
    if (FrameIDTemp == 0)
    {
        ErrorFlag |= 1;
    }
    else //找样本
    {
        for (uint8_t i = 0; i < MaxFrameNumber; i++)
        {
            if (Frame[i].FrameID == FrameIDTemp && Frame[i].Inuse == 1 && Frame[i].Recorde == 0)
            {
                FrameSID = i;
                break;
            }
        }
        if (FrameSID == 0)
        {
            ErrorFlag |= 1; //没找到
        }
    }

    //如果已经发送过指令，报错
    if (ErrorFlag == 0)
    {
        // while (Frame[FrameSID].FrmMovStart != 0)
        // {
        //     vTaskDelay(100);
        // }
        if (Frame[FrameSID].ReStatus != 0) //已经发送过指令
        {
            ErrorFlag |= 1;
        }
    }

    //解析回收/复测
    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "rerun", (char *)strtmp) != 0)
    {
        ErrorFlag |= 1;
    }

    if (strcmp((char *)strtmp, "true") == 0)
    {
        u8Temp = 2;
    }
    else if (strcmp((char *)strtmp, "false") == 0)
    {
        u8Temp = 1;
    }
    else
    {
        ErrorFlag |= 1;
    }

    memset(LogBuffer, 0, sizeof(LogBuffer));
    sprintf(LogBuffer, "INFO ETH RXCMD TrackGotoRecycle FrameID:%ld Rerun:%d Error:%d", FrameIDTemp, u8Temp, ErrorFlag);
    WorkRecordReadWrite(0, 0, LogBuffer);

    if (ErrorFlag == 0) //无错误
    {
        Frame[FrameSID].ReStatus = u8Temp;
        //等待运行完成
        while ((Frame[FrameSID].Inuse != 0) || (Frame[FrameSID].ReStatus != 0))
        {
            vTaskDelay(500);
        }
        if (Frame[FrameSID].ReStatus == 0xff) //错误
        {
            ErrorFlag |= 1;
        }
    }

    // EthPkgBuf_Send[SendIdx].cmd = EthTrackGotoRecycle;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)pkg->source);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>TrackGotoRecycleResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <rack></rack>\n\
    <rerun></rerun>\n\
    <result></result>\n\
  </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //样本架号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", (uint32_t)FrameIDTemp);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "rack", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //回收/复测
    if (u8Temp == 2) //复测
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "rerun", 4, "true") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }
    else if (u8Temp == 1) //回收
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "rerun", 4, "false") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }

    //成功、失败
    if (ErrorFlag == 0)
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "true") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }
    else
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "false") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //工作记录
    memset(LogBuffer, 0, sizeof(LogBuffer));
    sprintf(LogBuffer, "INFO ETH TXCMD TrackGotoRecycleResult FrameID:%ld Error:%d", FrameIDTemp, ErrorFlag);
    WorkRecordReadWrite(0, 0, LogBuffer);
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "WARN ETH TXRS TrackGotoRecycleResult FrameID:%ld", FrameIDTemp);
            WorkRecordReadWrite(0, 0, LogBuffer);
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "INFO ETH RXACK TrackGotoRecycleResult FrameID:%ld", FrameIDTemp);
        WorkRecordReadWrite(0, 0, LogBuffer);
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
    else
    {
        //发送失败
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "WARN ETH TXNAK TrackGotoRecycleResult FrameID:%ld", FrameIDTemp);
        WorkRecordReadWrite(0, 0, LogBuffer);
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 样本架从扫描处前往设备加样位置（CmdTrackGotoDevice）
 * @param  FrameData:上位机传入的样本架信息(str)
 * @retval None
 */
static void EthCmdTrackGotoDeviceTask(void *pvParameters)
{
    EthPkgDef *pkg = (void *)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    uint8_t ErrorFlag = 0;
    uint32_t FrameIDTemp = 0;
    uint8_t FrameSID = 0;
    uint8_t u8Temp = 0;

    //获取发送包
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, TrackGotoDevice Ignored");
        //清空接收包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }

    //解析相关内容
    //解析架号
    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "rack", (char *)strtmp) != 0)
    {
        ErrorFlag |= 1;
    }
    FrameIDTemp = atol((char *)strtmp);
    if (FrameIDTemp == 0)
    {
        ErrorFlag |= 1;
    }
    else //找样本
    {
        for (uint8_t i = 0; i < MaxFrameNumber; i++)
        {
            if (Frame[i].FrameID == FrameIDTemp && Frame[i].Inuse == 1 && Frame[i].Recorde == 0)
            {
                FrameSID = i;
                break;
            }
        }
        if (FrameSID == 0)
        {
            ErrorFlag |= 1; //没找到
        }
    }

    //如果在运行中，等待运行完成
    if (ErrorFlag == 0)
    {
        while (Frame[FrameSID].FrmMovStart == 1)
        {
            vTaskDelay(100);
        }
    }
    if (Frame[FrameSID].FrmMovStart == 2) //失败
    {
        ErrorFlag |= 1;
    }

    //解析设备号
    if (ErrorFlag == 0)
    {
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "device", (char *)strtmp) != 0)
        {
            ErrorFlag |= 1;
        }
        u8Temp = atoi((char *)strtmp);
        if (u8Temp == 0) //样本台
        {
            if (Frame[FrameSID].FrameDevice == 13) //已经在样本台了
            {
                ErrorFlag |= 1;
            }
            else
            {
                Frame[FrameSID].FrmNextDevice = 13; //样本台回收位置为13
                if (scanner_flg != 3)
                {
                    scanner_flg = 1;
                }
            }
        }
        else if (u8Temp <= TrackNumber) // 1-4轨道
        {
            Frame[FrameSID].FrmNextDevice = u8Temp;
        }
        else
        {
            ErrorFlag |= 1;
            if (scanner_flg != 3)
            {
                scanner_flg = 1;
            }
        }
    }
    //解析急诊
    if (ErrorFlag == 0 && Frame[FrameSID].FrameDevice == 12) //仅在扫码后第一次发送有效
    {
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "emer", (char *)strtmp) != 0)
        {
            ErrorFlag |= 1;
        }
        if (strcmp((char *)strtmp, "true") == 0)
        {
            Frame[FrameSID].FrmAddingEmg = 1;
        }
        else if (strcmp((char *)strtmp, "false") == 0)
        {
            Frame[FrameSID].FrmAddingEmg = 0;
        }
        else
        {
            ErrorFlag |= 1;
        }
    }
    //解析运行的样本位置
    if (ErrorFlag == 0 && Frame[FrameSID].FrmNextDevice != 13) //返回不需要解析位置
    {
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "pos", (char *)strtmp) != 0)
        {
            ErrorFlag |= 1;
        }
        u8Temp = atoi((char *)strtmp);
        if (u8Temp <= 10 && u8Temp != 0) //样本位置
        {
            Frame[FrameSID].FrmSampleNextPos = u8Temp;
        }
        else
        {
            ErrorFlag |= 1;
        }
    }
    //工作记录
    memset(LogBuffer, 0, sizeof(LogBuffer));
    sprintf(LogBuffer, "INFO ETH RXCMD TrackGotoDevice FrameID:%ld Device:%d Pos:%d Emer:%d Error:%d",
            FrameIDTemp, Frame[FrameSID].FrmNextDevice, Frame[FrameSID].FrmSampleNextPos, Frame[FrameSID].FrmAddingEmg, ErrorFlag);
    WorkRecordReadWrite(0, 0, LogBuffer);

    if (ErrorFlag == 0) //无错误
    {
        //启动运行
        Frame[FrameSID].FrmMovStart = 1;
        //等待运行完成
        while (Frame[FrameSID].FrmMovStart != 0)
        {
            vTaskDelay(1000);
            if (Frame[FrameSID].FrmMovStart == 0xff) //错误
            {
                ErrorFlag |= 1;
                Frame[FrameSID].FrmMovStart = 0;
                break;
            }
        }
    }

    if (Frame[FrameSID].Recorde == 1)
    {
        memset(strtmp, 0, EthDataLen);
        sprintf((char *)strtmp, "frame id %ld is goto recyle zone", Frame[FrameSID].FramSID);
        WorkRecordReadWrite(0, 0, (char *)strtmp);
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    // EthPkgBuf_Send[SendIdx].cmd = EthTrackGotoDevice;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)pkg->source);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>TrackGotoDeviceResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <rack></rack>\n\
    <device></device>\n\
    <emer></emer>\n\
    <pos></pos>\n\
    <state>destination</state>\n\
    <result></result>\n\
  </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //样本架号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", (uint32_t)FrameIDTemp);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "rack", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    //设备号
    memset(strtmp, 0, EthDataLen);
    if (Frame[FrameSID].FrmNextDevice == 13) //回收
    {
        sprintf((char *)strtmp, "%d", 0);
    }
    else
    {
        sprintf((char *)strtmp, "%d", Frame[FrameSID].FrmNextDevice);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "device", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    //急诊
    if (Frame[FrameSID].FrmAddingEmg == 1)
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "emer", 4, "true") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }
    else
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "emer", 4, "false") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }
    //位置号
    memset(strtmp, 0, EthDataLen);
    if (Frame[FrameSID].FrmNextDevice == 13) //回收
    {
        sprintf((char *)strtmp, "%d", 0);
    }
    else
    {
        sprintf((char *)strtmp, "%d", Frame[FrameSID].FrmSampleNextPos);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "pos", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    //成功、失败
    if (ErrorFlag == 0)
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "true") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }
    else
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "false") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //工作记录
    memset(LogBuffer, 0, sizeof(LogBuffer));
    sprintf(LogBuffer, "INFO ETH TXCMD TrackGotoDeviceResult FrameID:%ld Error:%d", FrameIDTemp, ErrorFlag);
    WorkRecordReadWrite(0, 0, LogBuffer);

    if (Frame[FrameSID].Recorde == 1)
    {
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "WARN ETH TXRS TrackGotoDeviceResult FrameID:%ld", FrameIDTemp);
            WorkRecordReadWrite(0, 0, LogBuffer);
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "INFO ETH RXACK TrackGotoDeviceResult FrameID:%ld", FrameIDTemp);
        WorkRecordReadWrite(0, 0, LogBuffer);
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
    else
    {
        //发送失败
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "WARN ETH TXNAK TrackGotoDeviceResult FrameID:%ld", FrameIDTemp);
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 样本台初始化（CmdReset）
 * @param  None
 * @retval None
 */
static void EthCmdResetTask(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    (void)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    uint8_t ErrorFlag = 0;
    WorkRecordReadWrite(0, 0, "INFO ETH RXCMD Reset");
    //获取发送包
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, Reset Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }

    while (DeviceStatus == NULL)
    {
        UserTaskDelay(10);
    }

    //获取仪器状态
    if (DeviceStatus != None)
    {
        ErrorFlag = 0xf0; //不在初开机状态
    }
    if (ErrorFlag == 0)
    {
        DeviceStatusChg = _Reset;
        //电机参数写入
        if (Motor_Param_Set())
        {
            ErrorFlag = 0xf1;
        }
    }
    if (ErrorFlag == 0)
    {
        //获取所有互斥量
        if (FullSemaphoreTake(0))
        {
            ErrorFlag = 0xf2;
        }
    }
    //完整复位
    if (ErrorFlag == 0)
    {
        ErrorFlag = FullInit();
        if(ErrorFlag != 0)
        {
            WorkRecordReadWrite(0, 0, "FullInit failed.");
        }
    }
    //标准复位
    if (ErrorFlag == 0)
    {
        ErrorFlag = Motor_All_Home(1);
        if(ErrorFlag != 0)
        {
            WorkRecordReadWrite(0, 0, "Motor_All_Home failed.");
        }
    }
    //归还所有互斥量
    if (ErrorFlag == 0)
    {
        if (FullSemaphoreGive())
        {
            ErrorFlag = 0xf3;
        }
    }
    if (ErrorFlag == 0)
    {
        DeviceStatusChg = StandBy;
        // DeviceStatus = StandBy;
    }
    else if (DeviceStatus == _Reset && ErrorFlag != 0xf0) //不是因为状态错误导致的失败
    {
        DeviceStatusChg = None;
        // DeviceStatus = None;
    }

    initflg = 1;

    // EthPkgBuf_Send[SendIdx].cmd = EthReset;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>ResetResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <result></result>\n\
    <info></info>\n\
  </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //成功、失败
    if (ErrorFlag == 0)
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "true") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
    }
    else
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "false") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
    }

    //错误代号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%d", ErrorFlag);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "info", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //工作记录
    memset(LogBuffer, 0, sizeof(LogBuffer));
    sprintf(LogBuffer, "INFO ETH TXCMD ResetResult Error:0x%02X", ErrorFlag);
    WorkRecordReadWrite(0, 0, LogBuffer);
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS ResetResult");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        DeviceStatusChg = Pause;
        WorkRecordReadWrite(0, 0, "INFO ETH RXACK ResetResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK ResetResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 样本台快速初始化（CmdQuickInit）
 * @param  None
 * @retval None
 */
void EthQuickInitTask(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    (void)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    uint8_t ErrorFlag = 0;
    WorkRecordReadWrite(0, 0, "INFO ETH RXCMD QuickInit");

    //获取发送包
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, QuickInit Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }

    //获取仪器状态
    if (DeviceStatus != None)
    {
        ErrorFlag = 0xf0; //不在初开机状态
    }

    //无错误，运行
    if (ErrorFlag == 0)
    {
        DeviceStatusChg = _Reset;
        // DeviceStatus = _Reset;
        //电机参数写入
        if (Motor_Param_Set())
        {
            ErrorFlag = 0xf1;
        }
    }
    if (ErrorFlag == 0)
    {
        //获取所有互斥量
        if (FullSemaphoreTake(0))
        {
            ErrorFlag = 0xf2;
        }
    }
    //标准电机复位
    if (ErrorFlag == 0)
    {
        ErrorFlag = Motor_All_Home(1);
    }
    //归还所有互斥量
    if (ErrorFlag == 0)
    {
        if (FullSemaphoreGive())
        {
            ErrorFlag = 0xf3;
        }
    }
    if (ErrorFlag == 0)
    {
        DeviceStatusChg = StandBy;
        // DeviceStatus = StandBy;
    }
    else if (DeviceStatus == _Reset && ErrorFlag != 0xf0) //不是因为状态错误导致的失败
    {
        DeviceStatusChg = None;
        // DeviceStatus = None;
    }

    // EthPkgBuf_Send[SendIdx].cmd = EthQuickInit;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>QuickInitResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <result></result>\n\
    <info></info>\n\
  </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //成功、失败
    if (ErrorFlag == 0)
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "true") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
    }
    else
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "false") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
    }

    //错误代号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%d", ErrorFlag);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "info", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //工作记录
    memset(LogBuffer, 0, sizeof(LogBuffer));
    sprintf(LogBuffer, "INFO ETH TXCMD QuickInitResult Error:0x%02X", ErrorFlag);
    WorkRecordReadWrite(0, 0, LogBuffer);
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS QuickInitResult");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        WorkRecordReadWrite(0, 0, "INFO ETH RXACK QuickInitResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK QuickInitResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/* Debug模式函数---------------------------------------------------------*/
/**
 * @brief 进入调试模式
 * @param  None
 * @retval None
 */
static uint8_t EthDbgEnterDebugMode(void)
{
    if (EthDbgTestRun != 0) //在整机运行中
    {
        //停机
        // EthCmdCancelDo();
        // EthDbgTestRun = 0;
        return 0;
    }
#if 1

    EthDbgMode = 1;

#else
    if (Motor_Param_Set())
    {
        return 1;
    }

    EthDbgMode = 1;
#endif

    return 0;
}

/**
 * @brief 退出调试模式
 * @param  None
 * @retval None
 */
uint8_t EthDbgExitDebugMode(void)
{
    if (EthDbgMode == 0)
    {
        return 1;
    }
    EthDbgMode = 0;
    return 0;
}

/**
 * @brief 电机参数解析
 * @param  data：网络包数据，device_id：设备id，motor_id：电机ID,pos_id:位置ID
 * @retval 0-成功，1-失败
 */
uint8_t EthDbgMotResolve(uint8_t *data, uint8_t *device_id, uint8_t *motor_id, uint8_t *pos_id)
{
    uint8_t mot_pos_num[][12] = {{2, 0, 12, 1, 2, 2, 2, 1}, {1, 2, 0, 0, 3, 11, 11, 0, 0, 0, 0, 1}}; //分别对应样本台和轨道的工位数
    uint8_t device, motor, pos = 0;
    uint8_t strtmp[EthDataLen] = {0};
    //解析相关内容
    //解析设备号
    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind((char *)data, "device", (char *)strtmp) != 0)
    {
        return 1;
    }
    device = atol((char *)strtmp);
    if (device > 4)
    {
        return 1;
    }

    //解析电机序号
    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind((char *)data, "motor", (char *)strtmp) != 0)
    {
        return 1;
    }
    motor = atol((char *)strtmp);

    if (pos_id != NULL)
    {
        //解析位置序号
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind((char *)data, "pos", (char *)strtmp) != 0)
        {
            return 1;
        }
        pos = atol((char *)strtmp);
        if (pos > 12 || pos == 0)
        {
            return 1;
        }

        //电机位置判断
        if (pos > mot_pos_num[(device == 0 ? 0 : 1)][motor - 1])
        {
            return 1;
        }

        *pos_id = pos;
    }

    *device_id = device;
    *motor_id = motor;
    return 0;
}

/**
 * @brief 电机位置参数设置
 * @param  pkg:接收包
 * @retval None
 */
uint8_t EthDbgSetPos(EthPkgDef *pkg)
{
    uint8_t strtmp[EthDataLen] = {0};
    uint8_t device = 0;
    uint8_t motor = 0;
    uint8_t pos = 0;
    int_fast32_t value = 0;
    MotorStruDef **MotorStruTemp = NULL;
    MotorStruDef *MotorStruCtrl;
    if (EthDbgMode == 0) //不在debug模式
    {
        return 1;
    }
    //解析相关内容
    if (EthDbgMotResolve((uint8_t *)(pkg->data), &device, &motor, &pos))
    {
        return 1;
    }

    //解析位置参数
    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "value", (char *)strtmp) != 0)
    {
        return 1;
    }
    value = atol((char *)strtmp);

    //执行参数写入
    if (device == 0 && motor == 1) //进样链条
    {
        ChainIn.Offset[pos - 1] = value;
    }
    else if (device == 0 && motor == 5) //返回链条，前进
    {
        ChainBack.Offset[0][pos - 1] = value;
    }
    else if (device == 0 && motor == 6) //返回链条，后退
    {
        ChainBack.Offset[1][pos - 1] = value;
    }
    else //其他普通电机
    {
        if (Debug_Motor_Conv(device, motor, MotorStruTemp))
        {
            return 1;
        }
        MotorStruCtrl = *MotorStruTemp;
        MotorStruCtrl->Pos[pos] = value;
    }
    return 0;
}

/**
 * @brief 电机位置读取入口
 * @param  pkg:接收包
 * @retval None
 */
uint8_t EthDbgGetPos(EthPkgDef *pkg)
{
    uint8_t handle_index = 0;
    //  uint8_t strtmp[EthDataLen] = {0};
    uint8_t device = 0;
    uint8_t motor = 0;
    uint8_t pos = 0;
    //  int_fast32_t value = 0;

    if (EthDbgMode == 0) //不在debug模式
    {
        return 1;
    }
    //判断输入参数准确性
    if (EthDbgMotResolve((uint8_t *)(pkg->data), &device, &motor, &pos))
    {
        return 1;
    }

    //建立发送参数线程
    handle_index = EthHandleIndexGet();
    if (handle_index == 0xff) //没有空余的线程句柄了
    {
        return 1;
    }
    pkg->dealtask = &EthCMDTaskHandle[handle_index];
    xUserTaskCreate(EthDbgGetPosTask, "EthDbgGetPosTask", configMINIMAL_STACK_SIZE * 2, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
    return 0;
}

/**
 * @brief 电机位置参数读取发送线程
 * @param  pkg:接收包
 * @retval None
 */
void EthDbgGetPosTask(void *pvParameters)
{
    EthPkgDef *pkg = (void *)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint8_t device = 0;
    uint8_t motor = 0;
    uint8_t pos = 0;
    int_fast32_t value = 0;
    MotorStruDef **MotorStruTemp = NULL;
    MotorStruDef *MotorStruCtrl;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    //获取发送包
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, DbgGetPos Ignored");
        //清空接收包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
    // EthPkgBuf_Send[SendIdx].cmd = DbgGetPos;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
 <version>1</version>\n\
 <id></id>\n\
 <source></source>\n\
 <target></target>\n\
 <command>GetPosResult</command>\n\
 <priority>1</priority>\n\
 <data>\n\
   <device></device>\n\
   <motor></motor>\n\
   <pos></pos>\n\
   <value></value>\n\
 </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        //清空接收包和发送包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        //清空接收包和发送包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        //清空接收包和发送包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    if (EthDbgMotResolve((uint8_t *)(pkg->data), &device, &motor, &pos))
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH DbgMotResolve Error");
        //清空接收包和发送包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //设备号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%d", device);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "device", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //电机号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%d", motor);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "motor", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //位置号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%d", pos);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "pos", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //参数值读取
    if (device == 0 && motor == 1) //进样链条
    {
        value = ChainIn.Offset[pos - 1];
    }
    else if (device == 0 && motor == 5) //返回链条，前进
    {
        value = ChainBack.Offset[0][pos - 1];
    }
    else if (device == 0 && motor == 6) //返回链条，后退
    {
        value = ChainBack.Offset[1][pos - 1];
    }
    else //其他电机
    {
        if (Debug_Motor_Conv(device, motor, MotorStruTemp) == 0)
        {
            MotorStruCtrl = *MotorStruTemp;
            value = MotorStruCtrl->Pos[pos];
        }
    }

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", value);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "value", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS DbgGetPosResult");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK DbgGetPosResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 电机运行入口
 * @param  pkg:接收包
 * @retval None
 */
uint8_t EthDbgMotorRun(EthPkgDef *pkg)
{
    uint8_t handle_index = 0;
    //  uint8_t strtmp[EthDataLen] = {0};
    uint8_t device = 0;
    uint8_t motor = 0;
    // uint8_t pos = 0;
    //   int_fast32_t value = 0;

    if (EthDbgMode == 0) //不在debug模式
    {
        return 1;
    }
    //判断输入参数准确性
    if (EthDbgMotResolve((uint8_t *)(pkg->data), &device, &motor, NULL))
    {
        return 1;
    }

    //建立发送参数线程
    handle_index = EthHandleIndexGet();
    if (handle_index == 0xff) //没有空余的线程句柄了
    {
        return 1;
    }
    pkg->dealtask = &EthCMDTaskHandle[handle_index];
    xUserTaskCreate(EthDbgMotorRunTask, "EthCmdGetEnableScanMode", configMINIMAL_STACK_SIZE * 2, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
    return 0;
}

/**
 * @brief 电机运行控制发送线程
 * @param  pkg:接收包
 * @retval None
 */
static void EthDbgMotorRunTask(void *pvParameters)
{
    EthPkgDef *pkg = (void *)pvParameters;

    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint8_t device = 0;
    uint8_t motor = 0;
    int_fast32_t value = 0;
    uint8_t param = 0;
    uint32_t timeout_cnt = 0;
    ChainCtlStruDef DebugTemp;
    MotorStruDef **MotorStruTemp = NULL;
    MotorStruDef *MotorStruCtrl;
    //  MotorCtlStatusDef MotCtlStruTemp;
    uint8_t ctrl = 0; // 1普通运行 2回home 3快回0 4停止
    uint8_t rtn = 0;  // 0正常 1错误
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    //获取发送包
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, DbgMotorRun Ignored");
        //清空接收包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
    // EthPkgBuf_Send[SendIdx].cmd = DbgGetPos;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
 <version>1</version>\n\
 <id></id>\n\
 <source></source>\n\
 <target></target>\n\
 <command>MotorRunResult</command>\n\
 <priority>1</priority>\n\
 <data>\n\
   <device></device>\n\
   <motor></motor>\n\
   <result></result>\n\
 </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        //清空接收包和发送包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        //清空接收包和发送包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        //清空接收包和发送包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    if (EthDbgMotResolve((uint8_t *)(pkg->data), &device, &motor, NULL))
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH DbgMotResolve Error");
        //清空接收包和发送包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //解析指令
    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "control", (char *)strtmp) != 0)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        //清空接收包和发送包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (strcmp((char *)strtmp, "run") == 0) //运行
    {
        ctrl = 1;
    }
    else if (strcmp((char *)strtmp, "home") == 0) //回HOME
    {
        ctrl = 2;
    }
    else if (strcmp((char *)strtmp, "fzero") == 0) //快回0
    {
        ctrl = 3;
    }
    else if (strcmp((char *)strtmp, "stop") == 0) //停止
    {
        ctrl = 4;
    }
    else if (strcmp((char *)strtmp, "release") == 0) //释放
    {
        ctrl = 5;
    }
    else
    {
        rtn = 1;
    }

    if (ctrl == 1)
    {
        //解析位置参数
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "step", (char *)strtmp) != 0)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH DbgMotResolve Error");
            //清空接收包和发送包并删除线程
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
        value = atol((char *)strtmp);

        //解析其他参数
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "param", (char *)strtmp) != 0)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH DbgMotResolve Error");
            //清空接收包和发送包并删除线程
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
        param = atol((char *)strtmp);
    }

    //设备号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%d", device);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "device", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        //清空接收包和发送包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //电机号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%d", motor);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "motor", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        //清空接收包和发送包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    if (device == 0 && (motor == 1 || motor == 2) && ctrl == 1) //进样链条，前进，后退
    {
        if (motor == 1) //前进
        {
            DebugTemp.RunPos = value;
            DebugTemp.Align = param;
        }
        else if (motor == 2) //后退
        {
            DebugTemp.RunPos = -value;
            DebugTemp.Align = 0;
        }
        DebugTemp.StopCtl = 0;
        ChainIn.RunCtl = _cctl_run;
        xUserTaskCreate(ChainInCtrlTask, "ChainInCtrlTask", MotorCtl_STACK_SIZE * 2, (void *)&DebugTemp, ChainCtl_Priority, NULL);
        while (1)
        {
            UserTaskDelay(20);
            if (ChainIn.Status == _chain_idle)
            {
                break;
            }
        }
        ChainIn.RunCtl = _cctl_stop;
        rtn = 0;
    }
    else if (device == 0 && (motor == 5 || motor == 6) && ctrl == 1) //返回链条，前进，后退
    {

        if (motor == 5) //前进
        {
            DebugTemp.RunPos = value;
        }
        else if (motor == 6) //后退
        {
            DebugTemp.RunPos = -value;
        }
        DebugTemp.Align = param;
        DebugTemp.StopCtl = 0;
        xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&DebugTemp, ChainCtl_Priority, NULL);
        while (1)
        {
            UserTaskDelay(20);
            if (ChainBack.Status == _chain_idle)
            {
                break;
            }
        }
        rtn = 0;
    }
    else if (device == 0 && (motor == 1 || motor == 2) && ctrl == 5) //进样链条释放
    {
        if (MotorCtrl((MotorStruDef *)&Table.CHIF, _motor_stop, 0))
        {
            rtn = 1;
        }
        if (MotorCtrl((MotorStruDef *)&Table.CHIR, _motor_stop, 0))
        {
            rtn = 1;
        }
        if (MotorCtrl((MotorStruDef *)&Table.CHIF, _motor_rls, 0))
        {
        }
        if (MotorCtrl((MotorStruDef *)&Table.CHIR, _motor_rls, 0))
        {
        }
    }
    else if (device == 0 && (motor == 5 || motor == 6) && ctrl == 5) //返回链条释放
    {
        if (MotorCtrl((MotorStruDef *)&Table.CHBF, _motor_stop, 0))
        {
            rtn = 1;
        }
        if (MotorCtrl((MotorStruDef *)&Table.CHBR, _motor_stop, 0))
        {
            rtn = 1;
        }
        if (MotorCtrl((MotorStruDef *)&Table.CHBF, _motor_rls, 0))
        {
        }
        if (MotorCtrl((MotorStruDef *)&Table.CHBR, _motor_rls, 0))
        {
        }
    }
    else if (device == 0 && motor == 5 && ctrl == 2) //返回链条，回home
    {
        //释放后向电机
        if (MotorCtrl((MotorStruDef *)&Table.CHBR, _motor_stop, 0))
        {
            rtn = 1;
        }
        if (MotorCtrl((MotorStruDef *)&Table.CHBR, _motor_rls, 0))
        {
        }

        //回HOME
        if (rtn == 0)
        {
            if (MotorCtrl((MotorStruDef *)&Table.CHBF, _motor_home, 0))
            {
                rtn = 1;
            }
            timeout_cnt = 0;
            while (Table.CHBF.Status != _motor_idle)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > 500)
                {
                    rtn = 1;
                    break;
                }
            }
        }
    }
    else //其他普通电机
    {
        if (Debug_Motor_Conv(device, motor, MotorStruTemp))
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            //清空接收包和发送包并删除线程
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
        MotorStruCtrl = *MotorStruTemp;
        //电机运行
        switch (ctrl)
        {
        case 1:
            if (MotorStruCtrl->Param.Mode == 0) //速度模式
            {
                if (value == 0) //正向
                {
                    if (MotorCtrl(MotorStruCtrl, _motor_run_spd, 0))
                    {
                        rtn = 1;
                    }
                }
                else if (value == 1) //反向
                {
                    if (MotorCtrl(MotorStruCtrl, _motor_run_spd, 1))
                    {
                        rtn = 1;
                    }
                }
                else
                {
                    rtn = 1;
                }
            }
            else //位移模式或者闭环位移
            {
                MotorStruCtrl->PosTemp = value;
                if (MotorCtrl(MotorStruCtrl, _motor_run_pos, 0xff))
                {
                    rtn = 1;
                }
            }
            break;

        case 2:
            if (MotorCtrl(MotorStruCtrl, _motor_home, 0))
            {
                rtn = 1;
            }
            break;

        case 3:
            if (MotorCtrl(MotorStruCtrl, _motor_run_pos, 0))
            {
                rtn = 1;
            }
            break;

        case 4:
            if (MotorCtrl(MotorStruCtrl, _motor_stop, 0))
            {
                rtn = 1;
            }
            break;

        case 5:
            if (MotorCtrl(MotorStruCtrl, _motor_stop, 0))
            {
                rtn = 1;
            }
            if (MotorCtrl(MotorStruCtrl, _motor_rls, 0))
            {
                rtn = 1;
            }
            break;

        default:
            break;
        }

        //状态读取
        if (rtn == 0 && ctrl != 5)
        {
            timeout_cnt = 0;
            while (MotorStruCtrl->Status != _motor_idle)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > 500)
                {
                    rtn = 1;
                    break;
                }
            }
        }
    }
    if (rtn == 0)
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "true") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            //清空接收包和发送包并删除线程
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }
    else
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "false") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            //清空接收包和发送包并删除线程
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        //清空接收包和发送包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS DbgMotorRunResult");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK DbgMotorRunResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 反馈传感器状态入口
 * @param  pkg:接收包
 * @retval None
 */
uint8_t EthDbgSensorGet(EthPkgDef *pkg)
{
    uint8_t handle_index = 0;
    if (EthDbgMode == 0) //不在debug模式
    {
        return 1;
    }
    //建立发送参数线程
    handle_index = EthHandleIndexGet();
    if (handle_index == 0xff) //没有空余的线程句柄了
    {
        return 1;
    }
    pkg->dealtask = &EthCMDTaskHandle[handle_index];
    xUserTaskCreate(EthDbgSensorGetTask, "EthCmdGetEnableScanMode", configMINIMAL_STACK_SIZE * 2, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
    return 0;
}

/**
 * @brief 传感器状态追加到字符串
 * @param  status：传感器状态，str:追加的字符串
 * @retval None
 */
static void EthDbgSensorStatusAdd(uint8_t status, uint8_t *str)
{
    uint8_t statustemp[2] = {0};
    statustemp[0] = status + 0x30;
    strcat((char *)str, (char *)statustemp);
}

/**
 * @brief 反馈传感器状态线程
 * @param  None
 * @retval None
 */
void EthDbgSensorGetTask(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    EthPkgDef *pkg = (void *)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint8_t sensorstrtemp[50] = {0}; //传感器状态字符串
    int_fast32_t encodercnt[2] = {0};
    uint8_t device = 0;
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();

    //获取发送包
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, DbgSensorGet Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }

    //解析设备号
    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "device", (char *)strtmp) != 0)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, DbgSensorGet Ignored");
        //清空接收包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    device = atol((char *)strtmp);

    //传感器状态列举
    if (device == 0)
    {
        EthDbgSensorStatusAdd(Table_Sensor_Get(CHIF_S), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(CHIR_S), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(CHBF_S), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(CHBR_S), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(PBI_HM), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(PBI_WK1), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(PBI_WK2), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(PPI_HM), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(PPI_WK), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(PBB_HM), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(PBB_WK1), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(PBB_WK2), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(PPB_HM), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(ChainIn_FS), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(ChainIn_RS), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(ChainIn_SS), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(ChainBack_FS), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(ChainBack_RS), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(ChainBack_SFS), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(ChainBack_SRS), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(Scan_HS), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(Scan_LS), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(Scan_OS), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(Recy_ES), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(Recy_FS), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(Norm_SS), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(Emg_SS), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(Back_SS), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(_BUT_Pause), sensorstrtemp);
        EthDbgSensorStatusAdd(Table_Sensor_Get(_BUT_Back), sensorstrtemp);
        //扫码电机
        encodercnt[0] = STPM_Encoder_Get(Table.PBI.BoardID, Table.PBI.ChannelID);
        //返回链条电机
        encodercnt[1] = STPM_Encoder_Get(Table.CHBF.BoardID, Table.CHBF.ChannelID);
    }
    else if (device <= TrackNumber)
    {
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, WBF_HM), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, WBF_WK), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, NPB_HM), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, NPB_WK), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, EPB_HM), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, EPB_WK), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, MTC_HM), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, MTC_WK), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, MTC_SH), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, MTC_SW), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, MBF_HM), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, MBF_WK), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, BPB_HM), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, BPB_WK), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, W_EN), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, N_MO), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, N_MI), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, N_EX), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, E_EN), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, E_MO), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, E_MI), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, E_EX), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, B_EN), sensorstrtemp);
        EthDbgSensorStatusAdd(Track_Sensor_Get(device - 1, B_MI), sensorstrtemp);
        //常规挡板电机
        encodercnt[0] = STPM_Encoder_Get(Track[device - 1].NPB.BoardID, Track[device - 1].NPB.ChannelID);
        //急诊挡板电机
        encodercnt[1] = STPM_Encoder_Get(Track[device - 1].EPB.BoardID, Track[device - 1].EPB.ChannelID);
    }

    // EthPkgBuf_Send[SendIdx].cmd = EthStatus;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
 <version>1</version>\n\
 <id></id>\n\
 <source></source>\n\
 <target></target>\n\
 <command>SensorGetResult</command>\n\
 <priority>1</priority>\n\
 <data>\n\
   <device></device>\n\
   <sensor></sensor>\n\
   <encoder1></encoder1>\n\
   <encoder2></encoder2>\n\
 </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //设备号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%d", device);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "device", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //传感器信息
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "sensor", 4, (char *)sensorstrtemp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    //编码器1
    sprintf((char *)strtmp, "%ld", encodercnt[0]);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "encoder1", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    //编码器2
    sprintf((char *)strtmp, "%ld", encodercnt[1]);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "encoder2", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS DbgSensorGet");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK DbgSensorGet");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 扫码测试
 * @param  None
 * @retval None
 */
static void EthDbgScanBarcodeTask(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    (void)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    WorkRecordReadWrite(0, 0, "INFO ETH RXCMD ScanBarcode");

    //获取发送包
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, ScanBarcode Ignored");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }

    // EthPkgBuf_Send[SendIdx].cmd = DbgScanBarcode;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>ScanBarcodeResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <barcode></barcode>\n\
  </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //扫码
    memset(strtmp, 0, EthDataLen);
#ifdef __DeviceVersion01
    if (ScannerS_Start_Scan() == F_ERROR)
    {
        ScannerS_Stop_Scan();
    }
    else
    {
        for (uint8_t i = 0; i < Buffer_Length; i++)
        {
            if (ScannerS.RXD.Buffer[i] == 0x0d)
            {
                break;
            }
            strtmp[i] = ScannerS.RXD.Buffer[i];
        }
    }
#elif defined __DeviceVersion02
    if (ScannerCR100_Start_Scan() == F_OK)
    {
        for (uint8_t i = 0; i < Buffer_Length; i++)
        {
            if (ScannerCR100.RXD.Buffer[i + 1] == 0x0d)
            {
                break;
            }
            strtmp[i] = ScannerCR100.RXD.Buffer[i + 1];
        }
    }

#endif

    //条码号
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "barcode", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS DbgScanBarcodeResult");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK DbgScanBarcodeResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 其他参数设置
 * @param  pkg:接收包
 * @retval None
 */
static uint8_t EthDbgSetParam(EthPkgDef *pkg) // yfxiao
{
    uint8_t strtmp[EthDataLen] = {0};
    DbgParamDef paramtype = NoneParam;
    TimeStructDef DebugTime;
    int_fast32_t value = 0;

    if (EthXmlCmpFind(pkg->data, "param", (char *)strtmp) != 0)
    {
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "year", (char *)strtmp) != 0)
        {
            memset(strtmp, 0, EthDataLen);
            if (EthXmlCmpFind(pkg->data, "macaddr", (char *)strtmp) != 0)
            {
                return 1;
            }
            else
            {
                paramtype = MacSNParam;
            }
        }
        else
        {
            paramtype = TimeParam;
        }
    }

    if (strcmp((char *)strtmp, "Tracknum") == 0) //轨道数量
    {
        paramtype = TrackNumberParam;
    }
    else if (strcmp((char *)strtmp, "Time") == 0) //时间
    {
        paramtype = TimeParam;
    }
    else if (strcmp((char *)strtmp, "MacSN") == 0) //仪器序列号
    {
        paramtype = MacSNParam;
    }
    else if (strcmp((char *)strtmp, "CtrlBoardSN") == 0) //主控板序列号
    {
        paramtype = CtrlBoardSNParam;
    }
    switch (paramtype)
    {
    case TrackNumberParam:
        //解析参数
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "value", (char *)strtmp) != 0)
        {
            return 1;
        }
        value = atol((char *)strtmp);
        if (value < 0 || value > 4)
        {
            return 1;
        }
        TrackNumber = value;
        break;

    case TimeParam:
        //解析参数
        //年
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "year", (char *)strtmp) != 0)
        {
            return 1;
        }
        value = atol((char *)strtmp);
        if (value < 2000 || value > 2200)
        {
            return 1;
        }
        DebugTime.year = value - 2000;
        //月
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "month", (char *)strtmp) != 0)
        {
            return 1;
        }
        value = atol((char *)strtmp);
        if (value < 1 || value > 12)
        {
            return 1;
        }
        DebugTime.month = value;
        //日
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "day", (char *)strtmp) != 0)
        {
            return 1;
        }
        value = atol((char *)strtmp);
        if (value < 1 || value > 31)
        {
            return 1;
        }
        DebugTime.day = value;
        //小时
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "hour", (char *)strtmp) != 0)
        {
            return 1;
        }
        value = atol((char *)strtmp);
        if (value > 23)
        {
            return 1;
        }
        DebugTime.hour = value;
        //分钟
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "minute", (char *)strtmp) != 0)
        {
            return 1;
        }
        value = atol((char *)strtmp);
        if (value > 59)
        {
            return 1;
        }
        DebugTime.minute = value;
        //秒
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "second", (char *)strtmp) != 0)
        {
            return 1;
        }
        value = atol((char *)strtmp);
        if (value > 59)
        {
            return 1;
        }
        DebugTime.second = value;
        //设置系统时间
        if (SysTime.year == DebugTime.year && SysTime.month == DebugTime.month && SysTime.day == DebugTime.day && SysTime.hour == DebugTime.hour && SysTime.minute == DebugTime.minute)
        {
        }
        else
        {
            SysTime.year = DebugTime.year;
            SysTime.month = DebugTime.month;
            SysTime.day = DebugTime.day;
            SysTime.hour = DebugTime.hour;
            SysTime.minute = DebugTime.minute;
            SysTime.second = DebugTime.second;
            //时间写入RTC
            RTC_Time_Write();
            taskENTER_CRITICAL();
            memset(namebuff, 0, 30);
            taskEXIT_CRITICAL();
        }

        break;

    case MacSNParam:
    {
        //解析参数
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "macsn", (char *)strtmp) != 0)
        {
            return 1;
        }
        if ((strlen((char *)strtmp) == 0) || (strlen((char *)strtmp) > 20))
        {
            return 1;
        }
        memset(MacSN, 0, 20);
        strcpy(MacSN, (char *)strtmp);
        if (UserMacInfoRW(0))
        {
            return 1;
        }
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "ctrlboardsn", (char *)strtmp) != 0)
        {
            return 1;
        }
        if ((strlen((char *)strtmp) == 0) || (strlen((char *)strtmp) > 20))
        {
            return 1;
        }
        memset(CtrlBoardSN, 0, 20);
        strcpy(CtrlBoardSN, (char *)strtmp);
        if (UserMacInfoRW(0))
        {
            return 1;
        }
    }
    break;

    case CtrlBoardSNParam:
        //解析参数
        memset(strtmp, 0, EthDataLen);
        if (EthXmlCmpFind(pkg->data, "value", (char *)strtmp) != 0)
        {
            return 1;
        }
        if ((strlen((char *)strtmp) == 0) || (strlen((char *)strtmp) > 20))
        {
            return 1;
        }
        memset(CtrlBoardSN, 0, 20);
        strcpy(CtrlBoardSN, (char *)strtmp);
        if (UserMacInfoRW(0))
        {
            return 1;
        }
        break;

    default:
        return 1;
    }
    return 0;
}

/**
 * @brief 其他参数读取入口
 * @param  pkg:接收包
 * @retval None
 */
uint8_t EthDbgGetParam(EthPkgDef *pkg)
{
    uint8_t handle_index = 0;
    // if (EthDbgMode == 0) //不在debug模式
    // {
    //     return 1;
    // }

    //建立发送参数线程
    handle_index = EthHandleIndexGet();
    if (handle_index == 0xff) //没有空余的线程句柄了
    {
        return 1;
    }
    pkg->dealtask = &EthCMDTaskHandle[handle_index];
    xUserTaskCreate(EthDbgGetParamTask, "EthDbgGetParamTask", configMINIMAL_STACK_SIZE * 2, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
    return 0;
}

/**
 * @brief 其他参数读取发送线程
 * @param  pkg:接收包
 * @retval None
 */
void EthDbgGetParamTask(void *pvParameters)
{
    EthPkgDef *pkg = (void *)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    DbgParamDef paramtype = NoneParam;
    //获取发送包
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, DbgGetParam Ignored");
        //清空接收包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
    // EthPkgBuf_Send[SendIdx].cmd = DbgGetParam;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
 <version>1</version>\n\
 <id></id>\n\
 <source></source>\n\
 <target></target>\n\
 <command>GetParamResult</command>\n\
 <priority>1</priority>\n\
 <data>\n\
   <param></param>\n\
   <value></value>\n\
 </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        //清空接收包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        //清空接收包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        //清空接收包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //解析参数类型并处理
    if (EthXmlCmpFind(pkg->data, "param", (char *)strtmp) != 0)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH EthXmlCmpFind Error");
        //清空接收包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (strcmp((char *)strtmp, "Tracknum") == 0) //轨道数量
    {
        paramtype = TrackNumberParam;
    }
    else if (strcmp((char *)strtmp, "Time") == 0) //时间
    {
        paramtype = TimeParam;
    }
    else if (strcmp((char *)strtmp, "MacSN") == 0) //仪器序列号
    {
        paramtype = MacSNParam;
    }
    else if (strcmp((char *)strtmp, "CtrlBoardSN") == 0) //主控板序列号
    {
        paramtype = CtrlBoardSNParam;
    }

    switch (paramtype)
    {
    //写入参数
    case TrackNumberParam:
        //轨道参数
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "param", 4, "Tracknum") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            //清空接收包并删除线程
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
        //轨道数
        memset(strtmp, 0, EthDataLen);
        sprintf((char *)strtmp, "%d", TrackNumber);
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "value", 4, (char *)strtmp) == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            //清空接收包并删除线程
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
        break;

    case TimeParam:
        RTC_Time_Read();
        //时间
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "param", 4, "Time") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            //清空接收包并删除线程
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
        //时间信息
        memset(strtmp, 0, EthDataLen);
        sprintf((char *)strtmp, "%04d-%02d-%02d(%02d:%02d:%02d)",
                SysTime.year + 2000, SysTime.month, SysTime.day, SysTime.hour, SysTime.minute, SysTime.second);
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "value", 4, (char *)strtmp) == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            //清空接收包并删除线程
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
        break;

    case MacSNParam:
        UserMacInfoRW(1);
        //仪器序列号
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "param", 4, "MacSN") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            //清空接收包并删除线程
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
        //仪器序列号参数
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "value", 4, MacSN) == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
        break;

    case CtrlBoardSNParam:
        UserMacInfoRW(1);
        //控制板序列号
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "param", 4, "CtrlBoardSN") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
        //控制板序列号参数
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "value", 4, CtrlBoardSN) == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
        break;

    default:
        break;
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            WorkRecordReadWrite(0, 0, "WARN ETH TXRS DbgGetParamResult");
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
    else
    {
        //发送失败
        WorkRecordReadWrite(0, 0, "WARN ETH TXNAK DbgGetParamResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/**
 * @brief 保存参数
 * @param  None
 * @retval 0-成功 1-失败
 */
uint8_t EthDbgParamSave(void)
{

    if (UserParamInfoRW(0) || MotorParamInit(2))
    {
        return 1;
    }

    return 0;
}

/**
 * @brief 整机运行测试
 * @param  pkg:接收包
 * @retval None
 */
uint8_t EthDbgDeviceTestRun(EthPkgDef *pkg)
{
    //    uint8_t strtmp[EthDataLen] = {0};
    uint8_t handle_index = 0;
    //    uint8_t mode = 0;
    if (EthDbgMode == 0) //不在debug模式
    {
        return 1;
    }
    if (EthDbgTestRun != 0) //已经在跑了
    {
        return 1;
    }
    EthDbgTestRun = 1;
    handle_index = EthHandleIndexGet();
    if (handle_index == 0xff) //没有空余的线程句柄了
    {
        return 1;
    }
    pkg->dealtask = &EthCMDTaskHandle[handle_index];
    xUserTaskCreate(EthDbgDeviceTestRunTask, "EthDbgDeviceTestRunTask", configMINIMAL_STACK_SIZE, (void *)pkg, osPriorityNormal, &EthCMDTaskHandle[handle_index]);
    return 0;
}

/**
 * @brief 整机运行测试线程
 * @param  pkg:接收包
 * @retval None
 */
void EthDbgDeviceTestRunTask(void *pvParameters)
{
    EthPkgDef *pkg = (void *)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint8_t mode = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    //解析运行模式
    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "mode", (char *)strtmp) != 0)
    {
        // return 1;
    }
    mode = atol((char *)strtmp);
    if (mode > 2)
    {
        // return 1;
    }
    switch (mode)
    {
    case 0:
        Test_Mode = 0x10; //带轨道的测试模式
        break;

    case 1:
        Test_Mode = 0x03; //样本台自回收模式
        break;

    case 2:
        Test_Mode = 0x02; //样本台自循环模式
        break;

    default:
        Test_Mode = 0;
        break;
    }
    //随机数生成器
    /*## Configure the RNG peripheral #######################################*/
    RngHandle.Instance = RNG;

    /* DeInitialize the RNG peripheral */
    if (HAL_RNG_DeInit(&RngHandle) != HAL_OK)
    {
        //错误处理
        User_Error_Handler(__FILE__, __LINE__, 25110);
    }

    /* Initialize the RNG peripheral */
    if (HAL_RNG_Init(&RngHandle) != HAL_OK)
    {
        //错误处理
        User_Error_Handler(__FILE__, __LINE__, 25110);
    }
    // Motor_Param_Set();
    Motor_All_Home(1);
    DeviceStatusChg = StandBy;
    while (DeviceStatus != StandBy)
    {
        vTaskDelay(10);
    }
    DeviceStatusChg = Run;
    while (DeviceStatus != Run)
    {
        vTaskDelay(10);
    }
    ChainIn.RunCtl = _cctl_run;
    for (;;)
    {
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
}

static uint8_t FindSSID(uint32_t FrameIDTemp)
{
    uint8_t FrameSID = 0;
    for (uint8_t i = 0; i < MaxFrameNumber; i++)
    {
        if (Frame[i].FrameID == FrameIDTemp && Frame[i].Inuse == 1 && Frame[i].Recorde == 0)
        {
            FrameSID = i;
            Frame[i].Recorde = 1;
            break;
        }
    }

    return FrameSID;
}


/**
 * @brief 回收等待的样本架
 * @param device：要回收的样本架
 * @return uint8_t
 */
static void RsetWaitRack(uint8_t device)
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < MaxFrameNumber; i++)
    {
        if (Frame[i].Inuse == 1 && Frame[i].FrameDevice == 12 && Frame[i].FrmNextDevice == device)
        {
            Frame[i].FrmNextDevice = 13;
            break;
        }
    }
    taskEXIT_CRITICAL();
}


/**
 * @brief 获取下位机版本信息（CmdRsetRack）
 * @param  None
 * @retval None
 */

static uint8_t RsetRack(DEVRESETStatus *param, uint32_t FrameIDTemp, uint8_t dev)
{
    param->SSID = FindSSID(FrameIDTemp);

    if (param->SSID > 0)
    {
        param->ResetStatus = 10; //

        xUserTaskCreate(TrackGotoDeviceTask, "TrackGotoDeviceTask", configMINIMAL_STACK_SIZE, (void *)param, SampleCtl_Priority, NULL);
    }
    else
    {
        param->SSID = 0;
    }
    return param->SSID;
}

static void EthCmdRsetRackCmd(void *pvParameters)
{
    EthPkgDef *pkg = (void *)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};

    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    uint8_t ErrorFlag = 0;
    uint32_t FrameIDTemp = 0;

    uint8_t dev = 0;
    uint8_t i = 0;
    uint8_t rts = 0;

    //获取发送包

    //解析相关内容
    //解析架号
    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "device", (char *)strtmp) != 0)
    {
        ErrorFlag |= 1;
    }

    FrameIDTemp = atol((char *)strtmp);

    if (FrameIDTemp > 4)
    {
        //设备错误
        ErrorFlag |= 1;
    }
    else
    {
        dev = FrameIDTemp;
        memset(rackdev[dev], 0, sizeof(rackdev[dev]));
        for (i = 0; i < 4; i++)
        {
            rackdev[dev][i].dev = dev; //
            rackdev[dev][i].SSID = 0;  //
        }
    }

    //工作记录
    memset(LogBuffer, 0, sizeof(LogBuffer));
    sprintf(LogBuffer, "INFO ETH RXCMD RsetRack Device:%d Error:%d", dev, ErrorFlag);
    WorkRecordReadWrite(0, 0, LogBuffer);

    // 回收在扫码区等待的样本架
    // RsetWaitRack(dev);

    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "rack1", (char *)strtmp) != 0)
    {
        ErrorFlag |= 1;
    }
    else
    {
        FrameIDTemp = atol((char *)strtmp);

        if (FrameIDTemp)
        {
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO START RSET RACK:%d", FrameIDTemp);
            WorkRecordReadWrite(0, 0, LogBuffer);

            rts = RsetRack(&rackdev[dev][0], FrameIDTemp, dev);
        }
    }

    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "rack2", (char *)strtmp) != 0)
    {
        ErrorFlag |= 1;
    }
    else
    {
        FrameIDTemp = atol((char *)strtmp);
        if (FrameIDTemp)
        {
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO START RSET RACK:%d", FrameIDTemp);
            WorkRecordReadWrite(0, 0, LogBuffer);

            rts = RsetRack(&rackdev[dev][1], FrameIDTemp, dev);
        }
    }

    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "rack3", (char *)strtmp) != 0)
    {
        ErrorFlag |= 1;
    }
    else
    {
        FrameIDTemp = atol((char *)strtmp);
        if (FrameIDTemp)
        {
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO START RSET RACK:%d", FrameIDTemp);
            WorkRecordReadWrite(0, 0, LogBuffer);
            rts = RsetRack(&rackdev[dev][2], FrameIDTemp, dev);
        }
    }

    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "rack4", (char *)strtmp) != 0)
    {
        ErrorFlag |= 1;
    }
    else
    {
        FrameIDTemp = atol((char *)strtmp);
        if (FrameIDTemp)
        {
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO START RSET RACK:%d", FrameIDTemp);
            WorkRecordReadWrite(0, 0, LogBuffer);

            rts = RsetRack(&rackdev[dev][3], FrameIDTemp, dev);
        }
    }

    for (i = 0; i < 4; i++)
    {
        rackdev[0][i].ResetStatus = 0;
        rackdev[0][i].dev = dev;
    }

    if (rts != 0)
    {
        RsetRackResultTask((void *)&dev, (void *)pkg);
    }

    for (;;)
    {
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
}

/**
 * @brief 样本架从扫描处前往设备加样位置（CmdTrackGotoDevice）
 * @param  FrameData:上位机传入的样本架信息(str)
 * @retval None
 */
static void TrackGotoDeviceTask(void *pvParameters)
{
    DEVRESETStatus *param = (void *)pvParameters;

    uint8_t strtmp[EthDataLen] = {0};
    uint32_t NotifyReturn = 0;
    uint32_t SendIdx = 0;
    uint32_t CurrentTaskHandleIndex = EthHandleIndexGet();
    uint8_t ErrorFlag = 0;
    uint32_t FrameIDTemp = 0;
    uint8_t FrameSID = 0;

    // 获取当前线程，为了给发送包用，用来后续接任务通知
    if (CurrentTaskHandleIndex == 0xff) //没有空余的线程句柄了
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH EthHandleIndexGet None.");
        vTaskDelete(NULL);
    }
    EthCMDTaskHandle[CurrentTaskHandleIndex] = xTaskGetCurrentTaskHandle();

    //获取发送包
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, DbgGetParam Ignored");
        //清空接收包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }

    FrameSID = param->SSID;
    FrameIDTemp = Frame[FrameSID].FrameID;

    //如果在运行中，等待运行完成
    if (ErrorFlag == 0)
    {
        while (1)
        {
            // 除了扫码区的架子，其他位置的先等待完成
            if(Frame[FrameSID].FrmMovStart == 1)
            {
                if(Frame[FrameSID].FrameDevice == 12)
                {
                    break;
                } else
                {
                    vTaskDelay(100);
                }
            } else
            {
                break;
            }
        }
    }
    if (Frame[FrameSID].FrmMovStart == 2) //失败
    {
        ErrorFlag |= 1;
    }

    vTaskDelay(1000);

    if (Frame[FrameSID].FrameDevice != 13)
    {
        Frame[FrameSID].FrmNextDevice = 13;
        //工作记录
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "INFO ETH MySelf TrackGotoDevice FrameID:%ld Device:%d Pos:%d Emer:%d Error:%d",
                FrameIDTemp, Frame[FrameSID].FrmNextDevice, Frame[FrameSID].FrmSampleNextPos, Frame[FrameSID].FrmAddingEmg, ErrorFlag);
        WorkRecordReadWrite(0, 0, LogBuffer);

        if (ErrorFlag == 0) //无错误
        {
            //启动运行
            Frame[FrameSID].FrmMovStart = 1;
            //等待运行完成
            while (Frame[FrameSID].FrmMovStart != 0)
            {
                vTaskDelay(1000);
                if (Frame[FrameSID].FrmMovStart == 0xff) //错误
                {
                    ErrorFlag |= 1;
                    Frame[FrameSID].FrmMovStart = 0;
                    break;
                }
                if (Frame[FrameSID].FrameDevice == 13)
                {
                    break;
                }
            }
        }

        // 发送单模块重置时到缓存区后的应答
        EthPkgBuf_Send[SendIdx].cmdid = TxID++;
        EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
        strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
        strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
        strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>RsetRackDeviceResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <rack></rack>\n\
    <device></device>\n\
    <result></result>\n\
  </data>\n\
</root>");

        memset(strtmp, 0, EthDataLen);
        sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }

        //样本架号
        memset(strtmp, 0, EthDataLen);
        sprintf((char *)strtmp, "%ld", FrameIDTemp);
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "rack", 4, (char *)strtmp) == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }

        //设备号
        memset(strtmp, 0, EthDataLen);
        if (Frame[FrameSID].FrmNextDevice == 13) //回收
        {
            sprintf((char *)strtmp, "%d", 0);
        }
        else
        {
            sprintf((char *)strtmp, "%d", Frame[FrameSID].FrmNextDevice);
        }

        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "device", 4, (char *)strtmp) == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }

        //成功、失败
        if (ErrorFlag == 0)
        {
            if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "true") == 1)
            {
                //错误处理
                WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
                EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
            }
        }
        else
        {
            if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "false") == 1)
            {
                //错误处理
                WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
                EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
            }
        }

        if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }

        //工作记录
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "INFO ETH TXCMD RsetRackDeviceResult FrameID:%ld Error:%d", FrameIDTemp, ErrorFlag);
        WorkRecordReadWrite(0, 0, LogBuffer);

//		if (Frame[FrameSID].Recorde == 1)
//		{
//			EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
//		}

        //写入完成
        EthPkgBuf_Send[SendIdx].state = 2;
        //等待通知
        for (uint8_t i = 0; i < EthMaxTxTimes; i++)
        {
            NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
            if (NotifyReturn == RevAck)
            {
                break;
            }
            else
            {
                //重发
                memset(LogBuffer, 0, sizeof(LogBuffer));
                sprintf(LogBuffer, "WARN ETH TXRS RsetRackDeviceResult FrameID:%ld", FrameIDTemp);
                WorkRecordReadWrite(0, 0, LogBuffer);
                EthPkgBuf_Send[SendIdx].state = 2;
            }
        }
        if (NotifyReturn == RevAck)
        {
            //发送成功
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO ETH RXACK RsetRackDeviceResult FrameID:%ld", FrameIDTemp);
            WorkRecordReadWrite(0, 0, LogBuffer);
        }
        else
        {
            //发送失败
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "WARN ETH TXNAK RsetRackDeviceResult FrameID:%ld", FrameIDTemp);
        }
    }

    memset(LogBuffer, 0, sizeof(LogBuffer));
    sprintf(LogBuffer, "INFO ETH MySelf TrackGotoRecycle ssid :%d FrameID:%ld Error:%d", FrameSID, FrameIDTemp, ErrorFlag);
    WorkRecordReadWrite(0, 0, LogBuffer);


    taskENTER_CRITICAL();
    Frame[FrameSID].ReStatus = 1;
    taskEXIT_CRITICAL();
    //等待运行完成
    while ((Frame[FrameSID].Inuse != 0) || (Frame[FrameSID].ReStatus != 0)) // Recorde
    {
        vTaskDelay(500);
    }
    if (Frame[FrameSID].ReStatus == 0xff) //错误
    {
        ErrorFlag |= 1;
    }


    // 发送应答
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)LastSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>RsetRackRecycleResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <rack></rack>\n\
    <result></result>\n\
  </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //样本架号
    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", (uint32_t)FrameIDTemp);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "rack", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //成功、失败
    if (ErrorFlag == 0)
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "true") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
    }
    else
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "false") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
        }
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //工作记录
    memset(LogBuffer, 0, sizeof(LogBuffer));
    sprintf(LogBuffer, "INFO ETH TXCMD RsetRackRecycleResult FrameID:%ld Error:%d", FrameIDTemp, ErrorFlag);
    WorkRecordReadWrite(0, 0, LogBuffer);
    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "WARN ETH TXRS RsetRackRecycleResult FrameID:%ld", FrameIDTemp);
            WorkRecordReadWrite(0, 0, LogBuffer);
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "INFO ETH RXACK RsetRackRecycleResult FrameID:%ld", FrameIDTemp);
        WorkRecordReadWrite(0, 0, LogBuffer);
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
    else
    {
        //发送失败
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "WARN ETH TXNAK RsetRackRecycleResult FrameID:%ld", FrameIDTemp);
        WorkRecordReadWrite(0, 0, LogBuffer);
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }
}

/**
 * @brief 样本架从扫描处前往设备加样位置（CmdTrackGotoDevice）
 * @param  FrameData:上位机传入的样本架信息(str)
 * @retval None
 */
static void RsetRackResultTask(void *pvParameters, void *pvParameters1)
{
    EthPkgDef *pkg = (void *)pvParameters1;
    uint8_t *tmp = (void *)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};
    uint32_t SendIdx = 0;
    uint32_t NotifyReturn = 0;
    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();

    uint8_t dev = 0;

    dev = *tmp;

    //获取发送包
    SendIdx = EthSendIndexGet();
    if (SendIdx == UINT32_MAX)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH SendIndex Full, RsetRackResult Ignored");
        //清空接收包并删除线程
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }

    // EthPkgBuf_Send[SendIdx].cmd = EthTrackGotoDevice;
    EthPkgBuf_Send[SendIdx].cmdid = TxID++;
    EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
    strcpy((char *)EthPkgBuf_Send[SendIdx].source, (char *)EthSource);
    strcpy((char *)EthPkgBuf_Send[SendIdx].target, (char *)pkg->source);
    strcpy((char *)EthPkgBuf_Send[SendIdx].data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>RsetRackResult</command>\n\
  <priority>1</priority>\n\
  <data>\n\
    <device></device>\n\
    <result></result>\n\
  </data>\n\
</root>");

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", EthPkgBuf_Send[SendIdx].cmdid);
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    memset(strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%ld", dev);

    if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "device", 4, (char *)strtmp) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }

    if (rackdev[dev][0].ResetStatus == 0 && rackdev[dev][1].ResetStatus == 0 &&
            rackdev[dev][2].ResetStatus == 0 && rackdev[dev][3].ResetStatus == 0)
    {
        //复位完成发送复位完成
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "true") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }
    else if (rackdev[dev][0].ResetStatus == 0 || rackdev[dev][1].ResetStatus == 0 ||
             rackdev[dev][2].ResetStatus == 0 || rackdev[dev][3].ResetStatus == 0)
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "Resetting") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }
    else
    {
        if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "result", 4, "true") == 1)
        {
            //错误处理
            WorkRecordReadWrite(0, 0, "WARN ETH XmlCmpAdd Error");
            EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
        }
    }

    if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
    {
        //错误处理
        WorkRecordReadWrite(0, 0, "WARN ETH Encode Error");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, &EthPkgBuf_Send[SendIdx]);
    }

    //写入完成
    EthPkgBuf_Send[SendIdx].state = 2;
    //等待通知
    for (uint8_t i = 0; i < EthMaxTxTimes; i++)
    {
        NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
        if (NotifyReturn == RevAck)
        {
            break;
        }
        else
        {
            //重发
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "WARN ETH TXRS RsetRackResult ");
            WorkRecordReadWrite(0, 0, LogBuffer);
            EthPkgBuf_Send[SendIdx].state = 2;
        }
    }
    if (NotifyReturn == RevAck)
    {
        //发送成功
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "INFO ETH RXACK RsetRackResult");
        WorkRecordReadWrite(0, 0, LogBuffer);
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, NULL);
    }
    else
    {
        //发送失败
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "WARN ETH TXNAK RsetRackResult");
        EthDeleteCurrentTask(CurrentTaskHandleIndex, pkg, &EthPkgBuf_Send[SendIdx]);
    }
}

/**
 * @brief 样本架从扫描处前往设备加样位置（CmdTrackGotoDevice）
 * @param  FrameData:上位机传入的样本架信息(str)
 * @retval None
 */
static void EthCMDRsetRackResultTask(void *pvParameters)
{
    EthPkgDef *pkg = (void *)pvParameters;
    uint8_t strtmp[EthDataLen] = {0};

    uint32_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
    uint8_t ErrorFlag = 0;

    uint8_t dev = 0;

    memset(strtmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "device", (char *)strtmp) != 0)
    {
        ErrorFlag |= 1;
    }

    dev = atol((char *)strtmp);

    RsetRackResultTask((void *)&dev, (void *)pkg);
    for (;;)
    {
        //不应该执行到这一步
        EthDeleteCurrentTask(CurrentTaskHandleIndex, NULL, NULL);
    }
}

/*****************************END OF FILE*****************************/
