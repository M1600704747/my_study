/**
******************************************************************************
* @文件    user_app.C
* @作者    SUYANG
* @版本    V1.0.1
******************************************************************************
*轨道控制线程和函数

******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "user_app.h"
#include "../BspApi/user_sensor_api.h"
#include "../Tasks/user_chainbacktask.h"
#include "../Tasks/user_chainintask.h"
#include "../Tasks/user_cmd_api_20220621.h"
#include "../Tasks/user_motor_ctr.h"
#include "../Tasks/user_sample_in.h"
#include "stm32h743i_eval_nor.h"
#include "user_cmdfun.h"
#include "user_data.h"
#include "user_excutor.h"
#include "user_motor.h"
#include "user_rtc.h"
#include "user_scanner.h"
#include "user_sdlog.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
extern uint8_t Motor_Param_Set(void);
extern uint8_t MotorCtrl(MotorStruDef *Motor, MotorCtlStatusDef Ctl, uint8_t PosID);

uint32_t TrackNumber = 0;				 //所连接的轨道数,0-4
volatile FrameDef Frame[MaxFrameNumber]; //__attribute__((at(0x20000000 + 0x20000))); //__attribute__((at(SDRAM_DEVICE_ADDR+8*1024)));

volatile TrackStruDef Track[MaxTrackNumber]; // __attribute__((at(SDRAM_DEVICE_ADDR)));
volatile TableStruDef Table;				 // __attribute__((at(SDRAM_DEVICE_ADDR+16*1024)));
static TableButStruDef ButPause;			 //暂停按钮
static TableButStruDef ButBack;				 //后退按钮
DeviceStatusDef DeviceStatus = _NULL;
DeviceStatusDef DeviceStatusChg = _NULL;
static TaskHandle_t TableInCtrlTaskHandle;
static TaskHandle_t TableBackCtrlTaskTaskHandle;
static TaskHandle_t TrackSampleTransferTaskHandle;
QueueHandle_t TransferRequest; //转移队列，保存的内容为样本架序列编号，0-99

//设备错误告警
DeviceAlarmStruDef RecoveryFullAlarm;
DeviceAlarmStruDef RecoveryFulltoStopAlarm;
DeviceAlarmStruDef UserPausedRerunWaitingAlarm;
DeviceAlarmStruDef FrameLostAlarm;
DeviceAlarmStruDef UnknownFrameAlarm;
DeviceAlarmStruDef InitAlarm;
DeviceAlarmStruDef ErrorAlarm;
DeviceAlarmStruDef ScanErrorAlarm;

__IO uint32_t NoBarcodeFrameID = 0;
__IO uint8_t FrmBarcodeSend = 0;
__IO uint8_t SystemEmergencyStop = 0; //紧急停机

__IO uint8_t Test_Mode = 0x00;

__IO uint8_t recodessid = 0x00;

SemaphoreHandle_t framemutex; //

//
//调试测试模式、老化模式。
// 0x00-无
// 0x01-样本台自测试，随机
// 0x02-样本台自测试，循环
// 0x03-样本台自测试，回收
// 0x10-轨道自测试
__IO uint8_t DeviceFrameCnt = 0;
__IO uint32_t DeviceIdleCnt = 0;
RNG_HandleTypeDef RngHandle;

// extern ChainInStruDef ChainIn;
// extern ChainBackStruDef ChainBack;

extern uint32_t SysOnTime; //系统开机时间，单位s，溢出后清零

extern uint32_t MainSenGPIOPin[];
extern GPIO_TypeDef *MainSenGPIOPort[];
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void SampleDispatch(FrameDef *Frame);
static void TrackSampleCtrlTask(void *pvParameters);
static uint8_t MotorEPBCtrl(uint8_t TrackID, uint8_t posid);
static void ChainRerunWatingTask(void *pvParameters);
static void TableSampleCtrlTask(void *pvParameters);
// static void TableInCtrlTask(void *pvParameters);
static void TableButCtrlTask(void *pvParameters);
// static uint8_t TrackInitSampleBackFun(uint8_t TrackID, uint8_t mode);

/**
 * @brief  样本架信息清空/信息初始化
 * @param  FrameSID：样本架ID
 * @retval
 */
void Frame_Init(uint8_t FrameSID)
{
#if 0
    Frame[FrameSID].Inuse = 0;
    Frame[FrameSID].FramSID = 0;
    Frame[FrameSID].FrameID = 0;
    memset((void *)Frame[FrameSID].FrameIDA, 0, 50);
    for (uint8_t i = 0; i < 10; i++)
    {
        Frame[FrameSID].Sample[i].Type = 0;
        //Frame[FrameSID].Sample[i].SampleID = 0;
        memset((void *)Frame[FrameSID].Sample[i].SampleIDA, 0, 50);
    }
    Frame[FrameSID].SD.TrackID = 0;
    Frame[FrameSID].SD.TrackEntrance = ENorm;
    Frame[FrameSID].SD.TrackExit = ENorm;
    Frame[FrameSID].SD.AddingEn = 0;
    Frame[FrameSID].FrameDevice = 0;
    Frame[FrameSID].FrmNextDevice = 0;
    Frame[FrameSID].FrmSampleNextPos = 0;
    Frame[FrameSID].FrmSampleCurPos = 0;
    Frame[FrameSID].FrmAddingEmg = 0;
    Frame[FrameSID].FrmMovStart = 0;
    Frame[FrameSID].ReceiveStatus = 0;
    Frame[FrameSID].ReStatus = 0;
    Frame[FrameSID].Recorde = 0;
#else
    memset((void *)&Frame[FrameSID], 0, sizeof(Frame[FrameSID]));
#endif
}

/**
 * @brief  样本转移线程
 * @param  None
 * @retval None
 */
static void TrackSampleTransferTask(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    (void)pvParameters;
    uint8_t TransferID, PosNext;
    //初始化队列
    TransferRequest = xQueueCreate(10, sizeof(uint8_t));

    if (TransferRequest == NULL)
    {
        //错误处理
        User_Error_Handler(__FILE__, __LINE__, 25101);
    }
    for (;;)
    {
        xQueueReceive(TransferRequest, (void *)&TransferID, portMAX_DELAY); //读取队列，读取不到就阻塞
        //确认样本可用
        if (Frame[TransferID].Inuse == 0)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 25102);
        }
        //已经到极限位置轨道4的情况
        if (Frame[TransferID].FrameDevice == TrackNumber && Frame[TransferID].SD.TrackExit != EBack)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 25103);
        }
        //检查样本当前位置，检查样本出口确定下个入口
        //样本当前在轨道位置
        if (Frame[TransferID].FrameDevice != 0 && Frame[TransferID].FrameDevice <= 4)
        {
            switch (Frame[TransferID].SD.TrackExit)
            {
            case ENorm:
                //转移到下一个轨道的常规位置
                Frame[TransferID].SD.TrackEntrance = ENorm;
                PosNext = Frame[TransferID].FrameDevice + 1;
                break;
            case EEmg:
                //转移到下一个轨道的急诊位置
                Frame[TransferID].SD.TrackEntrance = EEmg;
                PosNext = Frame[TransferID].FrameDevice + 1;
                break;
            case EBack:
                //转移到前一个轨道的返回位置，如果是1轨道，转移给样本台
                Frame[TransferID].SD.TrackEntrance = EBack;
                PosNext = Frame[TransferID].FrameDevice - 1;
                if (PosNext == 0) //转移到样本台
                {
                    PosNext = 13;
                }
                break;
            default:
                break;
            }
        }
        //在样本台的情况
        else if (Frame[TransferID].FrameDevice == 11) //扫码区
        {
            PosNext = 12;
        }
        else if (Frame[TransferID].FrameDevice == 12) //扫码区进样
        {
            PosNext = 1;
        }
        //下个样本轨道的调度
        //样本台
        if (PosNext == 12 || PosNext == 13)
        {
            //样本位置记录
            Frame[TransferID].FrameDevice = PosNext;

            //建立样本台的处理线程
            xUserTaskCreate(TableSampleCtrlTask, "TableSampleCtrlTask", configMINIMAL_STACK_SIZE * 2, (void *)&Frame[TransferID], SampleCtl_Priority, NULL);
        }
        else
        {
            //样本位置记录
            Frame[TransferID].FrameDevice = PosNext;
            //样本调度信息
            Frame[TransferID].SD.TrackID = Frame[TransferID].FrameDevice - 1;
            Frame[TransferID].SD.TrackEntrance = Frame[TransferID].SD.TrackExit;
            if (Frame[TransferID].FrmNextDevice == PosNext) //下节轨道加样
            {
                Frame[TransferID].SD.AddingEn = 1;
            }
            else
            {
                Frame[TransferID].SD.AddingEn = 0;
            }
            //建立下个样本轨道的处理线程
            xUserTaskCreate(TrackSampleCtrlTask, "TrackSampleCtrlTask", configMINIMAL_STACK_SIZE * 10, (void *)&Frame[TransferID], SampleCtl_Priority, NULL);
        }
    }
}

/**
 * @brief  返回链条出现一个未知样本架
 * @param  None
 * @retval None
 */
static void ChainBackUnknownFrameDeal(void)
{
    uint8_t SID_Temp = 0;
    //标记未知样本
    taskENTER_CRITICAL();
    // xUserSemaphoreTake(framemutex, portMAX_DELAY);
    for (uint8_t i = 1; i < MaxFrameNumber; i++)
    {
        if (Frame[i].Inuse == 0) //未使用,初始化样本架数据
        {
            Frame[i].Inuse = 1;
            Frame[i].FramSID = i;
            SID_Temp = i;
            Frame[i].FrameDevice = 13;
            Frame[i].ReceiveStatus = 0;
            Frame[i].ReStatus = 1; //直接标记返回
            DeviceFrameCnt++;
            break;
        }
    }
    ChainBack.FrameList[ChainBack.FrameCount] = SID_Temp;
    ChainBack.RecoveryFullAlarm[ChainBack.FrameCount] = 0;
    ChainBack.FrameCount++;
    taskEXIT_CRITICAL();
    // xUserSemaphoreGive(framemutex);
}

/**
 * @brief  样本分发模块调度算法样本架运行路径空闲检查
 * @param  Frame：传入的样本信息地址
 * @retval uint8_t 0空闲 1忙
 */
static uint8_t SampleRouteCheck(FrameDef *Frame)
{
    //仅适用于下一个目标为轨道的情况
    if ((Frame->FrmNextDevice == 0) || (Frame->FrmNextDevice == 13) || (Frame->FrmNextDevice > TrackNumber))
    {
        return 0;
    }
    //先检查需要加样的轨道的情况
    if (Frame->FrmAddingEmg == 0) //常规加样
    {
        if (Track[Frame->FrmNextDevice - 1].NormNextSID != 0) //有样本正在路上
        {
            if ((Track[Frame->FrmNextDevice - 1].WaitSID != 0) || (Track[Frame->FrmNextDevice - 1].NormSID != 0))
            {
                return 1;
            }
        }
        else
        {
            if ((Track[Frame->FrmNextDevice - 1].WaitSID != 0) && (Track[Frame->FrmNextDevice - 1].NormSID != 0))
            {
                return 1;
            }
        }
    }
    else //急诊加样
    {
        if ((Track[Frame->FrmNextDevice - 1].EmgSID != 0) || (Track[Frame->FrmNextDevice - 1].EmgNextSID != 0))
        {
            return 1;
        }
    }

    //检查路途的情况
    for (uint8_t tid = 0; tid < (Frame->FrmNextDevice - 1); tid++)
    {
        //常规通道
        if (Track[tid].NormNextSID != 0) //有样本正在路上
        {
            if ((Track[tid].WaitSID != 0) || (Track[tid].NormSID != 0))
            {
                //如果急诊通道也拥堵
                if ((Track[tid].EmgSID != 0) || (Track[tid].EmgNextSID != 0))
                {
                    return 1;
                }
            }
        }
        else
        {
            if ((Track[tid].WaitSID != 0) && (Track[tid].NormSID != 0))
            {
                //如果急诊通道也拥堵
                if ((Track[tid].EmgSID != 0) || (Track[tid].EmgNextSID != 0))
                {
                    return 1;
                }
            }
        }
    }
    return 0;
}

/**
 * @brief  样本分发模块调度算法
 * @param  Frame：传入的样本信息地址
 * @retval None
 */
static void SampleDispatch(FrameDef *Frame)
{
    /*
    如果返回-返回
    否则（下一节）
    	下节加样
    		加样常规-常规 ，加样急诊-急诊
    	否则（下节不加样）
    		1如果是常规加样
    			如果下节常规空闲-常规
    			否则（常规不空闲）-急诊
    		2如果是急诊加样
    			如果下节急诊空闲-急诊
    			否则（急诊不空闲）
    				如果常规空闲-常规
    				否则（常规不空闲）-急诊
    */
    uint8_t NextDevice = 0;		  // 1-4
    if (Frame->FrameDevice == 12) //在样本台扫码区
    {
        NextDevice = 1;
    }
    else
    {
        NextDevice = Frame->FrameDevice + 1;
    }

    if (Frame->FrmNextDevice == 13) //返回样本台
    {
        Frame->SD.TrackExit = EBack;
    }
    else
    {
        if (Frame->FrmNextDevice == NextDevice) //下节加样
        {
            if (Frame->FrmAddingEmg == 0) //常规
            {
                Frame->SD.TrackExit = ENorm;
            }
            else //急诊
            {
                Frame->SD.TrackExit = EEmg;
            }
        }
        else //下节不加样
        {
            #if 0
            //优先走急诊超车通道
            if (Track[NextDevice - 1].EmgSID == 0) //下节急诊空闲
            {
                Frame->SD.TrackExit = EEmg;
            }
            else
            {
                if (Track[NextDevice - 1].WaitSID == 0 &&
                        Track[NextDevice - 1].NormSID == 0) //下节常规空闲
                {
                    Frame->SD.TrackExit = ENorm;
                }
                else
                {
                    Frame->SD.TrackExit = EEmg;
                }
            }
            #endif
			#if 1
            if (Frame->FrmAddingEmg == 0) //常规
            {
                if (Track[NextDevice - 1].WaitSID == 0 && Track[NextDevice - 1].NormSID == 0) //下节常规空闲
                {
                    Frame->SD.TrackExit = ENorm;
                }
                else
                {
                    if(Track[NextDevice - 1].EmgSID == 0)//下节急诊空闲
                    {
                        Frame->SD.TrackExit = EEmg;
                    }
                    else
                    {
                        Frame->SD.TrackExit = ENorm;
                    }
                }
            }
            else
            {
                if (Track[NextDevice - 1].EmgSID == 0) //下节急诊空闲
                {
                    Frame->SD.TrackExit = EEmg;
                }
                else
                {
                    if (Track[NextDevice - 1].WaitSID == 0 &&
                        Track[NextDevice - 1].NormSID == 0) //下节常规空闲
                    {
                        Frame->SD.TrackExit = ENorm;
                    }
                    else
                    {
                        Frame->SD.TrackExit = EEmg;
                    }
                }
            }
			#endif
        }
    }
}

/**
 * @brief  样本台样本控制线程
 * @param  None
 * @retval None
 */
static void TableSampleCtrlTask(void *pvParameters)
{
    //传入信息应该包含：样本信息
    FrameDef *FrameTemp = (FrameDef *)pvParameters;
    //	uint32_t timeoutcnt = 0;
    uint8_t u8Temp;
    ChainCtlStruDef ChainCtlTemp;
    uint32_t RandTemp = 0;
    uint32_t timeout = 0;
    //检查运行目标
    // 1.在扫码区准备进返回链条
    //  1.进入常规轨道
    //  2.进入急诊轨道
    //  3.直接进返回链条
    // 2.在第一节轨道准备进返回链条
    if (FrameTemp->FrameDevice == 12) //在样本台扫码区
    {
        //获取扫码区推板的互斥量   xUserSemaphoreTake(framemutex , portMAX_DELAY);
        if (xUserSemaphoreTake(Table.PBI.Mutex, portMAX_DELAY) != pdTRUE)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 25100);
        }
        if (Test_Mode == 0x00)
        {
            FrmBarcodeSend = 0;
            //等待上位机指令
            while (FrameTemp->FrmMovStart == 0)
            {
                if (FrmBarcodeSend == 0) //从未发送过数据，或上位机要求重新发送
                {
                    //发送给上位机样本信息
                    if (EthSendBarcode((uint8_t *)&(FrameTemp->FramSID), 0) == 0) //成功
                    {
                        FrmBarcodeSend = 1;
                    }
                }
                UserTaskDelay(100);
            }
            FrmBarcodeSend = 0;
        }
        else if (Test_Mode == 0x01) //随机测试
        {
            HAL_RNG_GenerateRandomNumber(&RngHandle, &RandTemp);
            FrameTemp->FrmNextDevice = 13;
            FrameTemp->FrmMovStart = 1;
            FrameTemp->ReStatus = (RandTemp % 2) + 1;
        }
        else if (Test_Mode == 0x02) //循环测试
        {
            FrameTemp->FrmNextDevice = 13;
            FrameTemp->FrmMovStart = 1;
            FrameTemp->ReStatus = 2;
        }
        else if (Test_Mode == 0x03) //回收测试
        {
            FrameTemp->FrmNextDevice = 13;
            FrameTemp->FrmMovStart = 1;
            FrameTemp->ReStatus = 1;
        }
        else if (Test_Mode == 0x10) //轨道循环测试
        {
            if (TrackNumber == 0)
            {
                FrameTemp->FrmNextDevice = 13;
                FrameTemp->FrmMovStart = 1;
                FrameTemp->ReStatus = 2;
            }
            else
            {
                HAL_RNG_GenerateRandomNumber(&RngHandle, &RandTemp);
                FrameTemp->FrmNextDevice = RandTemp % TrackNumber + 1;
                HAL_RNG_GenerateRandomNumber(&RngHandle, &RandTemp);
                RandTemp %= 3; //三分之一样本为急诊，其余常规
                if (RandTemp)
                {
                    FrameTemp->FrmAddingEmg = 0;
                }
                else
                {
                    FrameTemp->FrmAddingEmg = 1;
                }
                FrameTemp->FrmMovStart = 1;
            }
        }

        if (Test_Mode != 0)
        {
            if (EthSendBarcode((uint8_t *)&(FrameTemp->FramSID), 0) == 0) //成功
            {
                FrmBarcodeSend = 1;
            }
        }
		#if 1
        if(Frame->FrmNextDevice != 1)
        {
            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TBL FrameID %ld From SCAN To %d", FrameTemp->FrameID,Frame->FrmNextDevice);
            WorkRecordReadWrite(0, 0, LogBuffer);
            for (uint8_t tid = 0; tid < (Frame->FrmNextDevice - 1); tid++)
            {
                if ((Track[tid].WaitSID != 0) || (Track[tid].NormSID != 0))
                {
                    //如果急诊通道也拥堵
                    if ((Track[tid].EmgSID != 0) || (Track[tid].EmgNextSID != 0))
                    {
                        vTaskDelay(100);
                    }
                }
            }
        }
        #endif
        //样本出口调度
        SampleDispatch(FrameTemp);

        //判断样本是否可能被阻挡，等待
        while (SampleRouteCheck((FrameDef *)FrameTemp))
        {
            vTaskDelay(100);
        }

        //直接回回收区
        if (FrameTemp->FrmNextDevice == 13)
        {
            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TBL FrameID %ld From SCAN To WAIT", FrameTemp->FrameID);
            WorkRecordReadWrite(0, 0, LogBuffer);
            //首先检查链条上是否已经满20个样本了，如果已经满了，不再回收，等待

            while (ChainBack.FrameCount >= 20)
            {
                UserTaskDelay(100);
            }
            //置忙位
            ChainBack.SampleBackWaiting = 1;
            //获取返回链条的互斥量
            if (xUserSemaphoreTake(ChainBack.Mutex, portMAX_DELAY) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }

            WorkRecordReadWrite(0, 0, "INFO CHB SEM TAKE");

            //如果返回链条有样本，那么就往前移动2/3格
            if (ChainBack.FrameCount != 0)
            {
#ifdef __DeviceVersion01
                ChainCtlTemp.RunPos = 2; //走2格
#else
                ChainCtlTemp.RunPos = 3; //走3格
#endif
                ChainCtlTemp.StopCtl = 0; //不检测样本位置
                ChainCtlTemp.Align = 0;	  //对齐样本位置
                xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&ChainCtlTemp, ChainCtl_Priority, NULL);

                //等待运行完成

                while (1)
                {
                    UserTaskDelay(100);

                    if (ChainBack.Status == _chain_idle)
                    {
                        break;
                    }
                }
            }
            else if (ChainBack.CurAlign != 0)
            {

                ChainCtlTemp.RunPos = 1;  //走一步对齐
                ChainCtlTemp.StopCtl = 0; //不检测样本位置
                ChainCtlTemp.Align = 0;	  //对齐样本位置
                xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&ChainCtlTemp, ChainCtl_Priority, NULL);

                //等待运行完成

                while (1)
                {
                    UserTaskDelay(100);

                    if (ChainBack.Status == _chain_idle)
                    {
                        break;
                    }
                }
            }
            //推板推到返回链条上
            if (MotorCtrl((MotorStruDef *)&Table.PBI, _motor_run_pos, 12))
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 20110);
            }
            while (Table.PBI.Status != _motor_idle)
            {
                if (Table_Sensor_Get(Norm_SS)) //发现了非法样本
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 20111);
                }
                UserTaskDelay(10);
            }
            //到位后判断链条情况，推板直接回到上一位置，或者回0
            if (ChainBack.ReRunStatus == _crrs_CIReady || ChainBack.ReRunStatus == _crrs_PPCplt || ChainIn.RunCtl == _cctl_userpause)
            {
                if (MotorCtrl((MotorStruDef *)&Table.PBI, _motor_run_pos, 11))
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 20110);
                }
            }
            else
            {
                if (MotorCtrl((MotorStruDef *)&Table.PBI, _motor_home, 0))
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 20110);
                }
            }

#ifdef __DeviceVersion01
            ChainCtlTemp.RunPos = -3; //走3格
#else
            ChainCtlTemp.RunPos = -4;	 //走4格
#endif
            ChainCtlTemp.StopCtl = 0; //不检测样本位置
            ChainCtlTemp.Align = 0;	  //对齐样本位置
            xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&ChainCtlTemp, ChainCtl_Priority, NULL);
            //样本信息处理
            taskENTER_CRITICAL();
            // xUserSemaphoreTake(framemutex, portMAX_DELAY);
            ChainBack.FrameList[ChainBack.FrameCount] = FrameTemp->FramSID;
            ChainBack.RecoveryFullAlarm[ChainBack.FrameCount] = 0;
            ChainBack.FrameCount++;
            taskEXIT_CRITICAL();
            // xUserSemaphoreGive(framemutex);
            //等待运行完成

            while (1)
            {
                UserTaskDelay(50);
                if (ChainBack.Status == _chain_idle && Table.PBI.Status == _motor_idle)
                {
                    if (xUserSemaphoreGive(Table.PBI.Mutex) != pdTRUE)
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 25100);
                    }
                    break;
                }
            }
            //检测运行位置
            if (!(Table_Sensor_Get(ChainBack_SRS)) || (Table_Sensor_Get(ChainBack_SFS)))
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 20111);
            }
            //释放互斥量
            if (xUserSemaphoreGive(ChainBack.Mutex) != pdTRUE)
            {
                //错误处理

                User_Error_Handler(__FILE__, __LINE__, 25100);
            }

            WorkRecordReadWrite(0, 0, "INFO CHB SEM GIVE");

            //置空位
            ChainBack.SampleBackWaiting = 0;
            //置完成
            FrameTemp->FrmMovStart = 0;
            FrameTemp->FrameDevice = 13;
            FrameTemp->FrmNextDevice = 0;
            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TBL FrameID %ld From SCAN To WAIT Complete", FrameTemp->FrameID);
            WorkRecordReadWrite(0, 0, LogBuffer);
        }
        //移交轨道
        else
        {
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TBL FrameID %ld From SCAN To TRACK1", FrameTemp->FrameID);
            WorkRecordReadWrite(0, 0, LogBuffer);

            if (xQueueSend(TransferRequest, (void *)&(FrameTemp->FramSID), 0) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25104);
            }
            //等待下级进样许可
            while (FrameTemp->ReceiveStatus == 0)
            {
                UserTaskDelay(50);
            }
            //置等待返回推板位，如果可以，插队进样，避免等待回收
            // ChainBack.SampleInWaiting = 1;
            //获取返回链条的互斥量
            if (xUserSemaphoreTake(ChainBack.Mutex, portMAX_DELAY) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            //获取进样推板的互斥量
            if (xUserSemaphoreTake(Table.PPI.Mutex, portMAX_DELAY) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }

            WorkRecordReadWrite(0, 0, "INFO CHB PPI SEM TAKE");

            //判断返回链条是否对齐样本位
            //急诊样本，往后先走一格
            if (ChainBack.CurAlign != 0 || FrameTemp->SD.TrackExit == EEmg) //对齐了返回位或急诊样本
            {
                //对齐样本位
                if (FrameTemp->SD.TrackExit == EEmg)
                {
                    ChainCtlTemp.RunPos = -1; //对齐急诊
                }
                else
                {
                    ChainCtlTemp.RunPos = 1; //对齐常规
                }
                ChainCtlTemp.StopCtl = 0; //不检测样本位置
                ChainCtlTemp.Align = 0;	  //对齐样本位置
                xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&ChainCtlTemp, ChainCtl_Priority, NULL);
                //等待运行完成

                while (1)
                {
                    UserTaskDelay(100);

                    if (ChainBack.Status == _chain_idle)
                    {
                        break;
                    }
                }
            }
            //确认收到许可
            FrameTemp->ReceiveStatus = 0;
            //推板推到返回链条上
            if (MotorCtrl((MotorStruDef *)&Table.PBI, _motor_run_pos, 12))
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 20110);
            }
            while (Table.PBI.Status != _motor_idle)
            {
                if (Table_Sensor_Get(Norm_SS)) //发现了非法样本
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 20111);
                }
                UserTaskDelay(10);
            }

            //到位后判断链条情况，推板直接回到上一位置，或者回0，并释放互斥量
            if (ChainBack.ReRunStatus == _crrs_CIReady || ChainBack.ReRunStatus == _crrs_PPCplt)
            {
                if (MotorCtrl((MotorStruDef *)&Table.PBI, _motor_run_pos, 11))
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 20110);
                }
            }
            else
            {
                if (MotorCtrl((MotorStruDef *)&Table.PBI, _motor_home, 0))
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 20110);
                }
            }

            if (FrameTemp->SD.TrackExit == EEmg)
            {
                //走到急诊
                ChainCtlTemp.RunPos = 1;  //走1格
                ChainCtlTemp.StopCtl = 0; //不检测样本位置
                ChainCtlTemp.Align = 0;	  //对齐样本位置
                xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&ChainCtlTemp, ChainCtl_Priority, NULL);
                //等待运行完成

                while (1)
                {
                    UserTaskDelay(100);

                    if (ChainBack.Status == _chain_idle)
                    {
                        break;
                    }
                }

                UserTaskDelay(1000);
            }
            else
            {
                UserTaskDelay(1500);
            }

            //推板推到轨道
            if (MotorCtrl((MotorStruDef *)&Table.PPI, _motor_run_pos, 1))
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 20110);
            }
            while (Table.PPI.Status != _motor_idle)
            {
                UserTaskDelay(50);
            }

            //到位后推板直接返回，并释放互斥量
            if (MotorCtrl((MotorStruDef *)&Table.PPI, _motor_run_pos, 0))
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 20110);
            }
            u8Temp = 0x03;
            timeout = 0;
            while (u8Temp != 0)
            {
                UserTaskDelay(50);
                if (UserReadBit(u8Temp, 0))
                {
                    if (Table.PBI.Status == _motor_idle)
                    {
                        UserClrBit(u8Temp, 0);
                    }
                }
                if (UserReadBit(u8Temp, 1))
                {
                    if (Table.PPI.Status == _motor_idle)
                    {
                        UserClrBit(u8Temp, 1);
                    }
                }

                if(++timeout > 500)
                {
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                    break;
                }
            }

            if (xUserSemaphoreGive(Table.PBI.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }

            if (xUserSemaphoreGive(Table.PPI.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }

            //释放链条互斥量
            if (xUserSemaphoreGive(ChainBack.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            WorkRecordReadWrite(0, 0, "INFO CHB SEM GIVE");

            //置位恢复，
            ChainBack.SampleInWaiting = 0;
            //等待下级进样完成
            while (FrameTemp->ReceiveStatus == 0)
            {
                UserTaskDelay(50);
            }
            //确认收到完成
            FrameTemp->ReceiveStatus = 0;
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TBL FrameID %ld From SCAN To TRACK1 Complete", (uint32_t)FrameTemp->FrameID);
            WorkRecordReadWrite(0, 0, LogBuffer);
        }
    }
    else //在轨道，要回样本台，或在回收区
    {
        //工作记录
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "INFO TBL FrameID %ld From TRACK1 To WAIT", (uint32_t)FrameTemp->FrameID);
        WorkRecordReadWrite(0, 0, LogBuffer);
        //首先检查链条上是否已经满20个样本了，如果已经满了，不再回收，等待
        while (ChainBack.FrameCount >= 20)
        {
            UserTaskDelay(100);
        }
        //置忙位
        ChainBack.SampleBackWaiting = 1;
        while (1)
        {
            //获取返回链条的互斥量
            if (xUserSemaphoreTake(ChainBack.Mutex, portMAX_DELAY) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }

            WorkRecordReadWrite(0, 0, "INFO CHB SEM TAKE");

            //获取后判断是否需要归还给优先级高的线程
            if (ChainBack.SampleRecWaiting == 1 || ChainBack.SampleInWaiting == 1)
            {
                //归还返回链条的互斥量
                if (xUserSemaphoreGive(ChainBack.Mutex) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }

                WorkRecordReadWrite(0, 0, "INFO CHB SEM GIVE");
            }
            else
            {
                break;
            }
        }
        //如果返回链条有样本，那么就往前移动6/7格
        if (ChainBack.FrameCount != 0)
        {
#ifdef __DeviceVersion01
            ChainCtlTemp.RunPos = 6; //走6格
#else
            ChainCtlTemp.RunPos = 7;	 //走7格
#endif
            ChainCtlTemp.StopCtl = 0; //不检测样本位置
            ChainCtlTemp.Align = 1;	  //对齐返回位置
        }
        //没有样本，直接往前走一步，走到空位置即可
        else
        {
            // yfxiao add go home
            while (ChainBack.Status != _chain_idle)
            {
                UserTaskDelay(50);
            }
            if (MotorCtrl((MotorStruDef *)&Table.CHBR, _motor_rls, 0))
            {
            }
            while (1)
            {
                UserTaskDelay(50);
                if (Table.CHBR.Status == _motor_rlsd)
                {
                    break;
                }
            }

            if (MotorCtrl((MotorStruDef *)&Table.CHBF, _motor_home, 0))
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 26100);
            }
            while (1)
            {
                UserTaskDelay(50);
                if (Table.CHBF.Status == _motor_idle)
                {
                    break;
                }
            }

            ChainCtlTemp.RunPos = 1;  //走一格
            ChainCtlTemp.StopCtl = 0; //不检测样本位置
            ChainCtlTemp.Align = 1;	  //对齐返回位置
        }

        xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&ChainCtlTemp, ChainCtl_Priority, NULL);
        //等待运行完成

        while (1)
        {
            UserTaskDelay(100);

            if (ChainBack.Status == _chain_idle)
            {
                break;
            }
            if (ChainCtlTemp.RstReturn == _crst_Blocked)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 20111);
            }
        }
        //确认一下无样本
        while (Table_Sensor_Get(ChainBack_FS))
        {
            //未知样本处理
            ChainBackUnknownFrameDeal();
            //发送告警
            UnknownFrameAlarm.alarm = 1;
            UnknownFrameAlarm.pos = 1;
            //再向后走一格
            ChainCtlTemp.RunPos = -1; //走一格
            ChainCtlTemp.StopCtl = 0; //不检测样本位置
            ChainCtlTemp.Align = 1;	  //对齐返回位置
            xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&ChainCtlTemp, ChainCtl_Priority, NULL);
            //等待运行完成

            while (1)
            {
                UserTaskDelay(100);

                if (ChainBack.Status == _chain_idle)
                {
                    break;
                }
            }
        }
        //像第一节轨道发送进样许可
        FrameTemp->ReceiveStatus = 1;
        while (FrameTemp->ReceiveStatus == 1)
        {
            UserTaskDelay(50);
        }
        //等待样本到位
        while (1)
        {
            UserTaskDelay(50);
            if (Table_Sensor_Get(Back_SS)) //返回样本检测
            {
                break;
            }
        }
        //发送收到进样，允许样本推入
        FrameTemp->ReceiveStatus = 2;
        // while (FrameTemp->ReceiveStatus == 2)
        // {
        //     UserTaskDelay(50);
        // }
        //等待样本到位，当轨道推板推到位后，会将ReceiveStatus置0
        while (1)
        {
            UserTaskDelay(50);
            if (Table_Sensor_Get(ChainBack_FS) && FrameTemp->ReceiveStatus != 2)
            {
                break;
            }
        }
        //样本往后倒，走到样本检测位  检测挡板是否到位
#ifdef __DeviceVersion01
        ChainCtlTemp.RunPos = -7; //走7格
#else
        ChainCtlTemp.RunPos = -8;		 //走8格
#endif
        ChainCtlTemp.StopCtl = 0; //不检测样本位置
        ChainCtlTemp.Align = 0;	  //对齐样本位置
        xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&ChainCtlTemp, ChainCtl_Priority, NULL);
        //样本信息处理
        taskENTER_CRITICAL();
        // xUserSemaphoreTake(framemutex, portMAX_DELAY);
        ChainBack.FrameList[ChainBack.FrameCount] = FrameTemp->FramSID;
        ChainBack.RecoveryFullAlarm[ChainBack.FrameCount] = 0;
        ChainBack.FrameCount++;
        // xUserSemaphoreGive(framemutex);
        taskEXIT_CRITICAL();
        //等待运行完成

        while (1)
        {
            UserTaskDelay(50);

            if (ChainBack.Status == _chain_idle)
            {
                break;
            }
        }
        //检测运行位置 // ChainBack_SRS, //返回链条样本检测后  ChainBack_SFS, //返回链条样本检测前
        if (!(Table_Sensor_Get(ChainBack_SRS)) || (Table_Sensor_Get(ChainBack_SFS)))
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 20111);
        }
        //释放互斥量
        if (xUserSemaphoreGive(ChainBack.Mutex) != pdTRUE)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 25100);
        }
        WorkRecordReadWrite(0, 0, "INFO CHB SEM GIVE");

        //置空位
        ChainBack.SampleBackWaiting = 0;
        //置完成
        FrameTemp->FrmMovStart = 0;
        //工作记录
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "INFO TBL FrameID %ld From TRACK1 To WAIT Complete", FrameTemp->FrameID);
        WorkRecordReadWrite(0, 0, LogBuffer);
    }

    for (;;)
    {
        vTaskDelete(NULL);
    }
}

//传入信息应该包含：样本信息和所在轨道信息
//轨道信息应包含在本节轨道是否加样，本节轨道入口，本节轨道出口
//流程：
//接收样本信息，并进行处理
//如果是常规通道样本↓
//获取挡杆互斥量，获取常规传送带互斥量，关闭挡杆，打开传送带，向上一级返回进样许可，等待样本到来
//获取常规阻挡互斥量，如样本在本节加样，则常规阻挡走1位
//如样本不在本节加样，则常规阻挡走10位
//传感器检测到样本到位，向上一级返回进样完成
//打开挡杆，等待样本通过
//样本通过后，关闭挡杆，释放挡杆互斥量
//样本到位后 ，如在本节加样：
//设置到位标志位，等待分析仪器给信号
//同时获取轨道常规传送带互斥量，停止传送带，立即释放互斥量
//收到分析仪信号后，获取轨道常规传送带互斥量，打开传送带，根据样本信息继续向前走
//样本到位后，停止传送带，释放互斥量
//需要释放前，获取变轨和挡杆模块互斥量
//如不在本节加样
//获取变轨和挡杆模块互斥量
//检查样本出口信息
//如常规：
//变轨模块置常规位，变轨挡杆阻挡，等待样本到达变轨区
//等待样本到达，释放轨道常规阻挡互斥量
//向下一级发送样本信息，等待样本进样许可
//收到许可后，打开档杆
//等待样本走过后，释放变轨和挡杆互斥量
//如急诊：预留，目前不适用
//变轨模块置常规位，变轨挡杆阻挡，等待样本到达变轨区
//等待样本到达，释放轨道常规阻挡互斥量
//向下一级发送样本信息，等待样本进样许可
//变轨模块置急诊位，收到进样许可后，挡杆打开
//样本走过后，释放变轨和挡杆互斥量
//如返回：
//变轨模块置常规位，变轨挡杆阻挡，等待样本到达变轨区
//等待样本到达，释放轨道常规阻挡互斥量
//获取返回轨道阻挡互斥量，返回推板阻挡
//变轨模块置返回位
//样本走过后，释放变轨和挡杆互斥量
//向下一级发送样本信息，等待样本进样许可
//收到进样许可后，返回推板释放
//收到下一级进样到位后，返回推板推送
//返回推板复位，释放推板互斥量
//完成后删除线程
//常规通道样本↑

//如果是急诊通道样本↓
//获取急诊阻挡互斥量，获取急诊传送带互斥量，打开传送带
//如样本在本节加样，则急诊阻挡走1位
//如样本不在本节加样，则急诊阻挡走10位
//向上一级返回进样许可，等待样本到来
//传感器检测到样本到位，向上一级返回进样完成
//样本到位后 ，如在本节加样：
//设置到位标志位，等待分析仪器给信号
//同时获取轨道急诊传送带互斥量，停止传送带，立即释放互斥量
//收到分析仪信号后，获取轨道急诊传送带互斥量，打开传送带，根据样本信息继续向前走
//样本到位后，停止传送带，释放互斥量
//需要释放前，获取变轨和挡杆模块互斥量
//如不在本节加样
//获取变轨和挡杆模块互斥量
//检查样本出口信息
//如常规：
//变轨模块置急诊位，变轨挡杆阻挡，等待样本到达变轨区
//等待样本到达，释放轨道急诊阻挡互斥量
//向下一级发送样本信息，等待样本进样许可
//变轨模块置常规位，收到进样许可后，挡杆打开
//样本走过后，释放变轨和挡杆互斥量
//如急诊
//变轨模块置急诊位，变轨挡杆阻挡，等待样本到达变轨区
//等待样本到达，释放轨道急诊阻挡互斥量
//向下一级发送样本信息，等待样本进样许可
//收到进样许可后，挡杆打开
//样本走过后，释放变轨和挡杆互斥量
//如返回：
//变轨模块置急诊位，变轨挡杆阻挡，等待样本到达变轨区
//等待样本到达，释放轨道急诊阻挡互斥量
//获取返回轨道阻挡互斥量，返回推板阻挡
//变轨模块置返回位
//样本走过后，释放变轨和挡杆互斥量
//向下一级发送样本信息，等待样本进样许可
//收到进样许可后，返回推板释放
//收到下一级进样到位后，返回推板推送
//返回推板复位，释放推板互斥量
//完成后删除线程
//急诊通道样本↑

//如果是返回通道样本↓
//获取返回阻挡互斥量，获取变轨和挡杆互斥量
//变轨机构走到常规/急诊位
//向上一级返回进样许可，等待样本到来
//样本走过后，释放变轨和挡杆互斥量
//向下一级发送样本信息，等待样本进样许可
//收到进样许可后，返回推板释放
//收到下一级进样到位后，返回推板推送
//返回推板复位，释放推板互斥量
//完成后删除线程
//返回通道样本↑

/**
 * @brief
 *
 * @param pvParameters
 */
static void TrackSampleCtrlTask(void *pvParameters)
{
    //传入信息应该包含：样本信息和所在轨道信息
    //轨道信息应包含在本节轨道是否加样，本节轨道入口，本节轨道出口
    FrameDef *FrameTemp = (FrameDef *)pvParameters;
    uint8_t TrackID = FrameTemp->SD.TrackID;
    uint32_t timeout_cnt = 0;
    uint8_t u8Temp = 0;

    //    Track[TrackID].FrameCnt++;
    //流程：
    //接收样本信息，并进行处理
    //如果是常规通道样本↓
    if (FrameTemp->SD.TrackEntrance == ENorm)
    {
        //样本信息记录
        Track[TrackID].NormNextSID = FrameTemp->FramSID;
        // scanner_flg = 2;
        //工作记录
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Norm", FrameTemp->FrameID, TrackID + 1);
        WorkRecordReadWrite(0, 0, LogBuffer);

        //获取挡杆互斥量
        if (xUserSemaphoreTake(Track[TrackID].WBF.Mutex, portMAX_DELAY) != pdTRUE)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 25100);
        }

        //关闭挡杆
        if (MotorCtrl((MotorStruDef *)&Track[TrackID].WBF, _motor_run_pos, 1) != 0)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
        }
        //检测运行状态
        while (Track[TrackID].WBF.Status == _motor_running)
        {
            UserTaskDelay(10);
        }

        //传送带运行
        Track[TrackID].NLBelt.BeltRun++;

        //向上一级返回进样许可
        FrameTemp->ReceiveStatus = 1;
        while (FrameTemp->ReceiveStatus == 1)
        {
            UserTaskDelay(100);
        }

        //传感器检测到样本到位，
        timeout_cnt = 0;
        while (Track_Sensor_Get(TrackID, W_EN) == 0)
        {
            UserTaskDelay(10);
            timeout_cnt++;
            if (timeout_cnt > SampleTimeout)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20111 + (TrackID + 1) * 1000));
            }
        }

        //向上一级返回进样完成
        FrameTemp->ReceiveStatus = 2;
        while (FrameTemp->ReceiveStatus == 2)
        {
            UserTaskDelay(100);
        }

        //样本信息记录
        if (FrameTemp->SD.AddingEn == 1) //如在本节加样
        {
            FrameTemp->FrmSampleCurPos = 0xff;				  //等待位
            EthSendWaitpos((uint8_t *)&(FrameTemp->FramSID)); // yfxiao test
        }
        Track[TrackID].WaitSID = FrameTemp->FramSID;
        Track[TrackID].NormNextSID = 0;

        //传送带释放
        Track[TrackID].NLBelt.BeltRun--;

        //获取常规阻挡互斥量
        if (xUserSemaphoreTake(Track[TrackID].NPB.Mutex, portMAX_DELAY) != pdTRUE)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 25100);
        }

        //传送带运行
        Track[TrackID].NLBelt.BeltRun++;

        //如样本在本节加样，则常规阻挡走1位
        if (FrameTemp->SD.AddingEn == 1)
        {
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].NPB, _motor_run_pos, 1) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
        }
        //如样本不在本节加样，则常规阻挡走10位
        else
        {
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].NPB, _motor_run_pos, 10) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
        }
        //检测运行状态
        while (Track[TrackID].NPB.Status == _motor_running)
        {
            UserTaskDelay(10);
        }

        //打开挡杆，等待样本通过
        if (MotorCtrl((MotorStruDef *)&Track[TrackID].WBF, _motor_run_pos, 0) != 0)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
        }
        //检测运行状态
        while (Track[TrackID].WBF.Status == _motor_running)
        {
            UserTaskDelay(10);
        }

        //样本通过后，关闭挡杆，释放挡杆互斥量
        timeout_cnt = 0;
        while (Track_Sensor_Get(TrackID, N_MO) == 0)
        {
            UserTaskDelay(10);
            timeout_cnt++;
            if (timeout_cnt > SampleTimeout)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20112 + (TrackID + 1) * 1000));
            }
        }
        UserTaskDelay(100);
        if (MotorCtrl((MotorStruDef *)&Track[TrackID].WBF, _motor_run_pos, 1) != 0)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
        }
        //检测运行状态
        while (Track[TrackID].WBF.Status == _motor_running)
        {
            UserTaskDelay(10);
        }
        //释放电机控制权限
        if (xUserSemaphoreGive(Track[TrackID].WBF.Mutex) != pdTRUE)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 25100);
        }
        //传送带释放
        Track[TrackID].NLBelt.BeltRun--;

        //样本信息记录
        Track[TrackID].WaitSID = 0;
        Track[TrackID].NormSID = FrameTemp->FramSID;

        if (scanner_flg != 3)
        {
            scanner_flg = 1;
        }
        //样本到位后 ，如在本节加样：
        if (FrameTemp->SD.AddingEn == 1)
        {
            if (Test_Mode == 0x10) //自带测试模式
            {
                //传送带运行
                Track[TrackID].NLBelt.BeltRun++;
                for (uint8_t cnti = 0; cnti < 9; cnti++)
                {
                    UserTaskDelay(2000);
                    if (MotorCtrl((MotorStruDef *)&Track[TrackID].NPB, _motor_run_pos, cnti + 2) != 0)
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
                    }
                    //检测运行状态
                    while (Track[TrackID].NPB.Status == _motor_running)
                    {
                        UserTaskDelay(10);
                    }
                }
                // UserTaskDelay(500);
                FrameTemp->FrmNextDevice = 13;
                FrameTemp->ReStatus = 2;
            }
            else
            {
                FrameTemp->FrmSampleCurPos = 1;
                u8Temp = 0;
                while (1)
                {
                    UserTaskDelay(100);
                    if (FrameTemp->FrmNextDevice != (TrackID + 1) && FrameTemp->FrmMovStart == 1)
                    {
                        //工作记录
                        memset(LogBuffer, 0, sizeof(LogBuffer));
                        sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Norm Release", FrameTemp->FrameID, TrackID + 1);
                        WorkRecordReadWrite(0, 0, LogBuffer);
                        break;
                    }
                    else if (FrameTemp->FrmMovStart == 1)
                    {
                        //工作记录
                        memset(LogBuffer, 0, sizeof(LogBuffer));
                        sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Norm Move Pos %d", FrameTemp->FrameID, TrackID + 1, FrameTemp->FrmSampleNextPos);
                        WorkRecordReadWrite(0, 0, LogBuffer);
                        if (FrameTemp->FrmSampleNextPos < FrameTemp->FrmSampleCurPos)
                        {
                            FrameTemp->FrmMovStart = 2; //失败
                            //工作记录
                            memset(LogBuffer, 0, sizeof(LogBuffer));
                            sprintf(LogBuffer, "WARN TRK FrameID %ld TrackID %d Norm Move Pos %d Fail", FrameTemp->FrameID, TrackID + 1, FrameTemp->FrmSampleNextPos);
                            WorkRecordReadWrite(0, 0, LogBuffer);
                            continue;
                        }
                        //传送带运行
                        Track[TrackID].NLBelt.BeltRun++;
                        //走位
                        if (MotorCtrl((MotorStruDef *)&Track[TrackID].NPB, _motor_run_pos, FrameTemp->FrmSampleNextPos) != 0)
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
                        }
                        //检测运行状态
                        while (Track[TrackID].NPB.Status == _motor_running)
                        {
                            UserTaskDelay(10);
                        }
                        //等待传感器检测到样本
                        timeout_cnt = 0;
                        while (Track_Sensor_Get(TrackID, N_MO) == 0)
                        {
                            UserTaskDelay(10);
                            timeout_cnt++;
                            if (timeout_cnt > SampleTimeout)
                            {
                                //错误处理
                                User_Error_Handler(__FILE__, __LINE__, (20112 + (TrackID + 1) * 1000));
                            }
                        }
                        //工作记录
                        memset(LogBuffer, 0, sizeof(LogBuffer));
                        sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Norm Move Pos %d Complete", FrameTemp->FrameID, TrackID + 1, FrameTemp->FrmSampleNextPos);
                        WorkRecordReadWrite(0, 0, LogBuffer);
                        FrameTemp->FrmSampleCurPos = FrameTemp->FrmSampleNextPos;
                        FrameTemp->FrmMovStart = 0;
                        //传送带释放
                        Track[TrackID].NLBelt.BeltRun--;
                    }
                    else
                    {
                        if (u8Temp == 0)
                        {
                            if(Track_Sensor_Get(TrackID, N_MO) == 0)
                            {
                                Track[TrackID].NLBelt.BeltRun++;
                                u8Temp = 1;
                                timeout_cnt = 0;
                            }
                        }
                        else
                        {
                            if(Track_Sensor_Get(TrackID, N_MO) == 1)
                            {
                                Track[TrackID].NLBelt.BeltRun--;
                                u8Temp = 0;
                                timeout_cnt = 0;
                            } else
                            {
                                timeout_cnt++;
                                if (timeout_cnt > 50) // 5s左右
                                {
                                    //错误处理
                                    User_Error_Handler(__FILE__, __LINE__, (20112 + (TrackID + 1) * 1000));
                                }
                            }
                        }
                    }
                }
            }
        }

        //样本出口调度
        SampleDispatch((FrameDef *)FrameTemp);

        //需要释放前，获取变轨和挡杆模块互斥量
        if (xUserSemaphoreTake(Track[TrackID].MTC.Mutex, portMAX_DELAY) != pdTRUE)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 25100);
        }
        if (xUserSemaphoreTake(Track[TrackID].MBF.Mutex, portMAX_DELAY) != pdTRUE)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 25100);
        }

        //检查样本出口信息
        switch (FrameTemp->SD.TrackExit)
        {
        //如常规：
        case ENorm:
        {
            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Norm To Norm", FrameTemp->FrameID, TrackID + 1);
            WorkRecordReadWrite(0, 0, LogBuffer);
            // check wait && norm
            if (TrackID + 1 < 4)
            {
                if (xUserSemaphoreGive(Track[TrackID].MTC.Mutex) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }
                if (xUserSemaphoreGive(Track[TrackID].MBF.Mutex) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }

                while (Track[TrackID + 1].WaitSID != 0 && Track[TrackID + 1].NormSID != 0)
                {
                    UserTaskDelay(10);
                }

                if (xUserSemaphoreTake(Track[TrackID].MTC.Mutex, portMAX_DELAY) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }
                if (xUserSemaphoreTake(Track[TrackID].MBF.Mutex, portMAX_DELAY) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }
            }

            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Norm To Norm start", FrameTemp->FrameID, TrackID + 1);
            WorkRecordReadWrite(0, 0, LogBuffer);

            //传送带运行
            Track[TrackID].NLBelt.BeltRun++;
            Track[TrackID].NSBelt.BeltRun++;

            //变轨模块置常规位，变轨挡杆阻挡，等待样本到达变轨区
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_run_pos, 1) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MBF, _motor_run_pos, 1) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20112 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MTC.Status == _motor_running || Track[TrackID].MBF.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            //释放样本
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].NPB, _motor_run_pos, 11) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].NPB.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            //等待样本到达，释放轨道常规阻挡互斥量
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, N_MI) == 0)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    //User_Error_Handler(__FILE__, __LINE__, (20113 + (TrackID + 1) * 1000));
                    break;
                }
            }
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, N_MI) == 1)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    //User_Error_Handler(__FILE__, __LINE__, (20113 + (TrackID + 1) * 1000));
                    break;
                }
            }
            if (xUserSemaphoreGive(Track[TrackID].NPB.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            //传送带释放
            Track[TrackID].NLBelt.BeltRun--;

            //样本信息记录
            Track[TrackID].NormSID = 0;
            //向样本控制线程发送样本信息，等待下一级轨道的样本进样许可
            if (xQueueSend(TransferRequest, (void *)&FrameTemp->FramSID, 0) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25104);
            }
            //等待许可
            while (FrameTemp->ReceiveStatus != 1)
            {
                UserTaskDelay(100);
            }
            FrameTemp->ReceiveStatus = 0;

            //收到许可后，打开档杆
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MBF, _motor_run_pos, 0) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MBF.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            //等待样本走过后，释放变轨和挡杆互斥量
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, N_EX) == 0)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20114 + (TrackID + 1) * 1000));
                }
            }

            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, N_EX) == 1)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20114 + (TrackID + 1) * 1000));
                }
            }

            //变轨回0位
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_run_pos, 0) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MTC.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            if (xUserSemaphoreGive(Track[TrackID].MTC.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            if (xUserSemaphoreGive(Track[TrackID].MBF.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }

            //等待下一级进样完成
            while (FrameTemp->ReceiveStatus != 2)
            {
                UserTaskDelay(10);
            }
            FrameTemp->ReceiveStatus = 0;

            //传送带释放
            Track[TrackID].NSBelt.BeltRun--;

            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Norm To Norm Complete", FrameTemp->FrameID, TrackID + 1);
            WorkRecordReadWrite(0, 0, LogBuffer);
            break;
        }
        //如急诊：
        case EEmg:
        {
            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Norm To Emg", FrameTemp->FrameID, TrackID + 1);
            WorkRecordReadWrite(0, 0, LogBuffer);

            if (TrackID + 1 < 4)
            {
                if (xUserSemaphoreGive(Track[TrackID].MTC.Mutex) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }
                if (xUserSemaphoreGive(Track[TrackID].MBF.Mutex) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }

                while (Track[TrackID + 1].EmgSID != 0)
                {
                    UserTaskDelay(10);
                }

                if (xUserSemaphoreTake(Track[TrackID].MTC.Mutex, portMAX_DELAY) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }
                if (xUserSemaphoreTake(Track[TrackID].MBF.Mutex, portMAX_DELAY) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }
            }

            //变轨模块置常规位，变轨挡杆阻挡，等待样本到达变轨区
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_run_pos, 1) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MBF, _motor_run_pos, 1) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MTC.Status == _motor_running || Track[TrackID].MBF.Status == _motor_running)
            {
                UserTaskDelay(10);
            }

            //传送带运行
            Track[TrackID].NLBelt.BeltRun++;
            Track[TrackID].NSBelt.BeltRun++;

            //释放样本
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].NPB, _motor_run_pos, 11) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].NPB.Status == _motor_running)
            {
                UserTaskDelay(10);
            }

            //等待样本到达，释放轨道常规阻挡互斥量
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, N_MI) == 0)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    //User_Error_Handler(__FILE__, __LINE__, (20113 + (TrackID + 1) * 1000));
                    break;
                }
            }
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, N_MI) == 1)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    //User_Error_Handler(__FILE__, __LINE__, (20113 + (TrackID + 1) * 1000));
                    break;
                }
            }
            UserTaskDelay(300);
            if (xUserSemaphoreGive(Track[TrackID].NPB.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }

            //传送带释放
            Track[TrackID].NLBelt.BeltRun--;

            //样本信息记录
            Track[TrackID].NormSID = 0;

            //向样本控制线程发送样本信息，等待下一级轨道的样本进样许可
            if (xQueueSend(TransferRequest, (void *)&FrameTemp->FramSID, 0) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25104);
            }
            //变轨模块置急诊位
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_run_pos, 2) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MTC.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            //传送带释放
            Track[TrackID].NSBelt.BeltRun--;

            //等待许可
            while (FrameTemp->ReceiveStatus != 1)
            {
                UserTaskDelay(10);
            }
            FrameTemp->ReceiveStatus = 0;

            // 急诊短传送带开始运行
            Track[TrackID].ESBelt.BeltRun++;
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MBF, _motor_run_pos, 0) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MBF.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            //样本走过后，释放变轨和挡杆互斥量
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, E_EX) == 0)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20119 + (TrackID + 1) * 1000));
                }
            }
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, E_EX) == 1)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20119 + (TrackID + 1) * 1000));
                }
            }
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_run_pos, 0) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MTC.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            if (xUserSemaphoreGive(Track[TrackID].MTC.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            if (xUserSemaphoreGive(Track[TrackID].MBF.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }

            //等待下一级进样完成
            while (FrameTemp->ReceiveStatus != 2)
            {
                UserTaskDelay(100);
            }
            FrameTemp->ReceiveStatus = 0;

            //传送带释放
            Track[TrackID].ESBelt.BeltRun--;

            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Norm To Emg Complete", FrameTemp->FrameID, TrackID + 1);
            WorkRecordReadWrite(0, 0, LogBuffer);
            break;
        }
        //如返回：
        case EBack:
        {
            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Norm To Back", FrameTemp->FrameID, TrackID + 1);
            WorkRecordReadWrite(0, 0, LogBuffer);

            //变轨模块置常规位，变轨挡杆阻挡，等待样本到达变轨区
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_run_pos, 1) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MBF, _motor_run_pos, 1) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MTC.Status == _motor_running || Track[TrackID].MBF.Status == _motor_running)
            {
                UserTaskDelay(10);
            }

            //释放样本
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].NPB, _motor_run_pos, 11) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].NPB.Status == _motor_running)
            {
                UserTaskDelay(10);
            }

            //传送带运行
            Track[TrackID].NLBelt.BeltRun++;
            Track[TrackID].NSBelt.BeltRun++;

            //等待样本到达
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, N_MI) == 0)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    //User_Error_Handler(__FILE__, __LINE__, (20113 + (TrackID + 1) * 1000));
                    break;
                }
            }
            //等待样本走过
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, N_MI) == 1)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    //User_Error_Handler(__FILE__, __LINE__, (20113 + (TrackID + 1) * 1000));
                    break;
                }
            }
            UserTaskDelay(200);
            //释放轨道常规阻挡互斥量
            if (xUserSemaphoreGive(Track[TrackID].NPB.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            //传送带释放
            Track[TrackID].NLBelt.BeltRun--;

            //样本信息记录
            Track[TrackID].NormSID = 0;

            //获取返回轨道阻挡互斥量，返回推板阻挡
            if (xUserSemaphoreTake(Track[TrackID].BPB.Mutex, portMAX_DELAY) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].BPB, _motor_run_pos, 1) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].BPB.Status == _motor_running)
            {
                UserTaskDelay(10);
            }

            //传送带运行
            Track[TrackID].ESBelt.BeltRun++;
            Track[TrackID].BSBelt.BeltRun++;
            Track[TrackID].BLBelt.BeltRun++;

            //变轨模块置返回位
            if (MotorMTCCtrl(TrackID, 3))
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20130 + (TrackID + 1) * 1000));
            }
            // Track[TrackID].BSBelt.BeltRun++;
            //样本走过后，变轨回0位，释放变轨和挡杆互斥量
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, B_MI) == 0)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20123 + (TrackID + 1) * 1000));
                }
            }
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, B_MI) == 1)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20123 + (TrackID + 1) * 1000));
                }
            }
            //变轨模块置0位
            if (MotorMTCCtrl(TrackID, 0))
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20130 + (TrackID + 1) * 1000));
            }
            //传送带释放
            Track[TrackID].NSBelt.BeltRun--;
            Track[TrackID].ESBelt.BeltRun--;
            Track[TrackID].BSBelt.BeltRun--;

            if (xUserSemaphoreGive(Track[TrackID].MTC.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            if (xUserSemaphoreGive(Track[TrackID].MBF.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }

            //向样本控制线程发送样本信息，等待下一级轨道的样本进样许可
            if (xQueueSend(TransferRequest, (void *)&FrameTemp->FramSID, 0) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25104);
            }
            //等待许可
            while (FrameTemp->ReceiveStatus != 1)
            {
                UserTaskDelay(100);
            }
            FrameTemp->ReceiveStatus = 0;

            //收到进样许可后，返回推板释放
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].BPB, _motor_run_pos, 0) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].BPB.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            //收到下一级进样到位后，如果是第一节轨道，返回推板推送
            while (FrameTemp->ReceiveStatus != 2)
            {
                UserTaskDelay(50);
            }
            if (TrackID == 0)
            {
                // FrameTemp->ReceiveStatus = 0;
                UserTaskDelay(1000);
                if (MotorCtrl((MotorStruDef *)&Track[TrackID].BPB, _motor_run_pos, 2) != 0)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
                }
                //检测运行状态
                while (Track[TrackID].BPB.Status == _motor_running)
                {
                    UserTaskDelay(10);
                }
                //确定推板推送完成
                FrameTemp->ReceiveStatus = 0;
                //返回推板复位，释放推板互斥量
                if (MotorCtrl((MotorStruDef *)&Track[TrackID].BPB, _motor_run_pos, 0) != 0)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
                }
                //检测运行状态
                while (Track[TrackID].BPB.Status == _motor_running)
                {
                    UserTaskDelay(10);
                }
            }
            else
            {
                //确定推板推送完成
                FrameTemp->ReceiveStatus = 0;
                UserTaskDelay(2000);
            }

            if (xUserSemaphoreGive(Track[TrackID].BPB.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            //传送带释放
            Track[TrackID].BLBelt.BeltRun--;
            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Norm To Back Complete", FrameTemp->FrameID, TrackID + 1);
            WorkRecordReadWrite(0, 0, LogBuffer);
            break;
        }
        }
    }
    //常规通道样本↑

    //如果是急诊通道样本↓
    else if (FrameTemp->SD.TrackEntrance == EEmg)
    {
        //样本信息记录
        Track[TrackID].EmgNextSID = FrameTemp->FramSID;
        //工作记录
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Emg", FrameTemp->FrameID, TrackID + 1);
        WorkRecordReadWrite(0, 0, LogBuffer);

        //获取急诊阻挡互斥量
        if (xUserSemaphoreTake(Track[TrackID].EPB.Mutex, portMAX_DELAY) != pdTRUE)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 25100);
        }

        //如样本在本节加样，则急诊阻挡走1位
        if (FrameTemp->SD.AddingEn == 1)
        {
            if (MotorEPBCtrl(TrackID, 1) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20131 + (TrackID + 1) * 1000));
            }
        }
        //如样本不在本节加样，则急诊阻挡走10位
        else
        {
            if (MotorEPBCtrl(TrackID, 10) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20131 + (TrackID + 1) * 1000));
            }
        }
        //传送带运行
        Track[TrackID].ELBelt.BeltRun++;
        //向上一级返回进样许可
        FrameTemp->ReceiveStatus = 1;
        while (FrameTemp->ReceiveStatus == 1)
        {
            UserTaskDelay(100);
        }

        //传感器检测到样本到位，向上一级返回进样完成
        timeout_cnt = 0;
        while (Track_Sensor_Get(TrackID, E_EN) == 0)
        {
            UserTaskDelay(10);
            timeout_cnt++;
            if (timeout_cnt > SampleTimeout)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20116 + (TrackID + 1) * 1000));
            }
        }
        //样本信息记录
        Track[TrackID].EmgSID = FrameTemp->FramSID;
        Track[TrackID].EmgNextSID = 0;

        if (scanner_flg != 3)
        {
            scanner_flg = 1;
        }

        //向上一级返回进样完成
        FrameTemp->ReceiveStatus = 2;
        while (FrameTemp->ReceiveStatus == 2)
        {
            UserTaskDelay(10);
        }
        //等待样本到急诊挡板位置
        timeout_cnt = 0;
        while (Track_Sensor_Get(TrackID, E_MO) == 0)
        {
            UserTaskDelay(10);
            timeout_cnt++;
            if (timeout_cnt > SampleTimeout)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20116 + (TrackID + 1) * 1000));
            }
        }
        //传送带释放
        Track[TrackID].ELBelt.BeltRun--;
        //样本到位后 ，如在本节加样：
        if (FrameTemp->SD.AddingEn == 1)
        {
            if (Test_Mode == 0x10) //自带测试模式
            {
                //传送带运行
                Track[TrackID].ELBelt.BeltRun++;
                for (uint8_t cnti = 0; cnti < 9; cnti++)
                {
                    UserTaskDelay(2000);
                    if (MotorEPBCtrl(TrackID, cnti + 2) != 0)
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, (20131 + (TrackID + 1) * 1000));
                    }
                }
                FrameTemp->FrmNextDevice = 13;
                FrameTemp->ReStatus = 2;
            }
            else
            {
                FrameTemp->FrmSampleCurPos = 1;
                u8Temp = 0;
                while (1)
                {
                    UserTaskDelay(100);
                    if (FrameTemp->FrmNextDevice != (TrackID + 1) && FrameTemp->FrmMovStart == 1)
                    {
                        //工作记录
                        memset(LogBuffer, 0, sizeof(LogBuffer));
                        sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Emg Release", FrameTemp->FrameID, TrackID + 1);
                        WorkRecordReadWrite(0, 0, LogBuffer);
                        break;
                    }
                    else if (FrameTemp->FrmMovStart == 1)
                    {
                        if (FrameTemp->FrmSampleNextPos < FrameTemp->FrmSampleCurPos)
                        {
                            FrameTemp->FrmMovStart = 2; //失败
                            //工作记录
                            memset(LogBuffer, 0, sizeof(LogBuffer));
                            sprintf(LogBuffer, "WARN TRK FrameID %ld TrackID %d Emg Move Pos %d Fail", FrameTemp->FrameID, TrackID + 1, FrameTemp->FrmSampleNextPos);
                            WorkRecordReadWrite(0, 0, LogBuffer);
                            continue;
                        }
                        //传送带运行
                        Track[TrackID].ELBelt.BeltRun++;
                        //走位
                        if (MotorEPBCtrl(TrackID, FrameTemp->FrmSampleNextPos) != 0)
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, (20131 + (TrackID + 1) * 1000));
                        }

                        //工作记录
                        memset(LogBuffer, 0, sizeof(LogBuffer));
                        sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Emg Move Pos %d Complete", FrameTemp->FrameID, TrackID + 1, FrameTemp->FrmSampleNextPos);
                        WorkRecordReadWrite(0, 0, LogBuffer);
                        FrameTemp->FrmSampleCurPos = FrameTemp->FrmSampleNextPos;
                        FrameTemp->FrmMovStart = 0;
                        //传送带释放
                        Track[TrackID].ELBelt.BeltRun--;
                    }
                    else
                    {
                        if (u8Temp == 0)
                        {
                            if(Track_Sensor_Get(TrackID, E_MO) == 0)
                            {
                                Track[TrackID].ELBelt.BeltRun++;
                                u8Temp = 1;
                                timeout_cnt = 0;
                            }
                        }
                        else
                        {
                            if (Track_Sensor_Get(TrackID, E_MO) == 1)
                            {
                                Track[TrackID].ELBelt.BeltRun--;
                                u8Temp = 0;
                                timeout_cnt = 0;
                            } else
                            {
                                timeout_cnt++;
                                if (timeout_cnt > 50) // 5s左右
                                {
                                    //错误处理
                                    User_Error_Handler(__FILE__, __LINE__, (20117 + (TrackID + 1) * 1000));
                                }
                            }
                        }
                    }
                }
            }
        }

        //样本出口调度
        SampleDispatch((FrameDef *)FrameTemp);

        //需要释放前，获取变轨和挡杆模块互斥量
        if (xUserSemaphoreTake(Track[TrackID].MTC.Mutex, portMAX_DELAY) != pdTRUE)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 25100);
        }
        if (xUserSemaphoreTake(Track[TrackID].MBF.Mutex, portMAX_DELAY) != pdTRUE)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 25100);
        }

        //检查样本出口信息
        switch (FrameTemp->SD.TrackExit)
        {
        //如常规：
        case ENorm:
            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Emg To Norm", FrameTemp->FrameID, TrackID + 1);
            WorkRecordReadWrite(0, 0, LogBuffer);

            if (TrackID + 1 < 4)
            {
                if (xUserSemaphoreGive(Track[TrackID].MTC.Mutex) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }
                if (xUserSemaphoreGive(Track[TrackID].MBF.Mutex) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }
                while (Track[TrackID + 1].WaitSID != 0 && Track[TrackID + 1].NormSID != 0)
                {
                    UserTaskDelay(10);
                }

                if (xUserSemaphoreTake(Track[TrackID].MTC.Mutex, portMAX_DELAY) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }
                if (xUserSemaphoreTake(Track[TrackID].MBF.Mutex, portMAX_DELAY) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }
            }

            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "next track of Norm is free ");
            WorkRecordReadWrite(0, 0, LogBuffer);

            //传送带运行   2021-08-20
            Track[TrackID].ELBelt.BeltRun++;
            Track[TrackID].NSBelt.BeltRun++;
            Track[TrackID].ESBelt.BeltRun++;

            //变轨模块置急诊位，变轨挡杆阻挡，等待样本到达变轨区
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_run_pos, 2) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MBF, _motor_run_pos, 1) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MTC.Status == _motor_running || Track[TrackID].MBF.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            //释放样本
            if (MotorEPBCtrl(TrackID, 11) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20131 + (TrackID + 1) * 1000));
            }
            //等待样本到达，释放轨道急诊阻挡互斥量
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, E_MI) == 0)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    // User_Error_Handler(__FILE__, __LINE__, (20118 + (TrackID + 1) * 1000));
                    break;
                }
            }
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, E_MI) == 1)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    // User_Error_Handler(__FILE__, __LINE__, (20118 + (TrackID + 1) * 1000));
                    break;
                }
            }
            UserTaskDelay(300);
            if (xUserSemaphoreGive(Track[TrackID].EPB.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            //传送带释放
            Track[TrackID].ELBelt.BeltRun--;
            //样本信息记录
            Track[TrackID].EmgSID = 0;
            //向样本控制线程发送样本信息，等待下一级轨道的样本进样许可
            if (xQueueSend(TransferRequest, (void *)&FrameTemp->FramSID, 0) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25104);
            }

            //变轨模块置常规位
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_run_pos, 1) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MTC.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            //传送带释放
            Track[TrackID].ESBelt.BeltRun--;
            //等待许可
            while (FrameTemp->ReceiveStatus != 1)
            {
                UserTaskDelay(10);
            }
            FrameTemp->ReceiveStatus = 0;

            //收到进样许可后，挡杆打开
            UserTaskDelay(300);

            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MBF, _motor_run_pos, 0) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MBF.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            //样本走过后，释放变轨和挡杆互斥量
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, N_EX) == 0)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20114 + (TrackID + 1) * 1000));
                }
            }

            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, N_EX) == 1)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20114 + (TrackID + 1) * 1000));
                }
            }

            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_run_pos, 0) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MTC.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            if (xUserSemaphoreGive(Track[TrackID].MTC.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            if (xUserSemaphoreGive(Track[TrackID].MBF.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            //等待下一级进样完成
            while (FrameTemp->ReceiveStatus != 2)
            {
                UserTaskDelay(100);
            }
            FrameTemp->ReceiveStatus = 0;
            //传送带释放
            Track[TrackID].NSBelt.BeltRun--;
            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Emg To Norm Complete", FrameTemp->FrameID, TrackID + 1);
            WorkRecordReadWrite(0, 0, LogBuffer);
            break;

        //如急诊
        case EEmg:
            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Emg To Emg", FrameTemp->FrameID, TrackID + 1);
            WorkRecordReadWrite(0, 0, LogBuffer);

            if (TrackID + 1 < 4)
            {
                if (xUserSemaphoreGive(Track[TrackID].MTC.Mutex) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }
                if (xUserSemaphoreGive(Track[TrackID].MBF.Mutex) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }

                while (Track[TrackID + 1].EmgSID != 0)
                {
                    UserTaskDelay(10);
                }

                if (xUserSemaphoreTake(Track[TrackID].MTC.Mutex, portMAX_DELAY) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }
                if (xUserSemaphoreTake(Track[TrackID].MBF.Mutex, portMAX_DELAY) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }
            }

            //传送带运行
            Track[TrackID].ELBelt.BeltRun++;
            Track[TrackID].ESBelt.BeltRun++;

            //变轨模块置急诊位，变轨挡杆阻挡，等待样本到达变轨区
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_run_pos, 2) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MBF, _motor_run_pos, 1) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MTC.Status == _motor_running || Track[TrackID].MBF.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            //释放样本
            if (MotorEPBCtrl(TrackID, 11) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20131 + (TrackID + 1) * 1000));
            }
            //等待样本到达，释放轨道急诊阻挡互斥量
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, E_MI) == 0)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    // User_Error_Handler(__FILE__, __LINE__, (20118 + (TrackID + 1) * 1000));
                    break;
                }
            }
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, E_MI) == 1)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    // User_Error_Handler(__FILE__, __LINE__, (20118 + (TrackID + 1) * 1000));
                    break;
                }
            }
            if (xUserSemaphoreGive(Track[TrackID].EPB.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            //传送带释放
            Track[TrackID].ELBelt.BeltRun--;

            //样本信息记录
            Track[TrackID].EmgSID = 0;
            //向样本控制线程发送样本信息，等待下一级轨道的样本进样许可
            if (xQueueSend(TransferRequest, (void *)&FrameTemp->FramSID, 0) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25104);
            }
            //等待许可
            while (FrameTemp->ReceiveStatus != 1)
            {
                UserTaskDelay(10);
            }
            FrameTemp->ReceiveStatus = 0;

            //收到进样许可后，挡杆打开
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MBF, _motor_run_pos, 0) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MBF.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            //样本走过后，释放变轨和挡杆互斥量
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, E_EX) == 0)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20119 + (TrackID + 1) * 1000));
                }
            }
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, E_EX) == 1)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20119 + (TrackID + 1) * 1000));
                }
            }
            //变轨回0位
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_run_pos, 0) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MTC.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            if (xUserSemaphoreGive(Track[TrackID].MTC.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            if (xUserSemaphoreGive(Track[TrackID].MBF.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            //等待下一级进样完成
            while (FrameTemp->ReceiveStatus != 2)
            {
                UserTaskDelay(10);
            }
            FrameTemp->ReceiveStatus = 0;
            //传送带释放
            Track[TrackID].ESBelt.BeltRun--;
            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Emg To Emg Complete", FrameTemp->FrameID, TrackID + 1);
            WorkRecordReadWrite(0, 0, LogBuffer);
            break;
        //如返回：
        case EBack:
            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Emg To Back", FrameTemp->FrameID, TrackID + 1);
            WorkRecordReadWrite(0, 0, LogBuffer);

            //变轨模块置急诊位，变轨挡杆阻挡，等待样本到达变轨区
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_run_pos, 2) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MBF, _motor_run_pos, 1) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].MTC.Status == _motor_running || Track[TrackID].MBF.Status == _motor_running)
            {
                UserTaskDelay(10);
            }

            //传送带运行
            Track[TrackID].ELBelt.BeltRun++;
            Track[TrackID].ESBelt.BeltRun++;

            //释放样本
            if (MotorEPBCtrl(TrackID, 11) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20131 + (TrackID + 1) * 1000));
            }
            //等待样本到达
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, E_MI) == 0)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    // User_Error_Handler(__FILE__, __LINE__, (20118 + (TrackID + 1) * 1000));
                    break;
                }
            }
            //等待样本走过
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, E_MI) == 1)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    // User_Error_Handler(__FILE__, __LINE__, (20118 + (TrackID + 1) * 1000));
                    break;
                }
            }
            UserTaskDelay(500);
            //释放轨道急诊阻挡互斥量
            if (xUserSemaphoreGive(Track[TrackID].EPB.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            //传送带释放
            Track[TrackID].ELBelt.BeltRun--;
            //传送带运行
            Track[TrackID].BSBelt.BeltRun++;
            Track[TrackID].BLBelt.BeltRun++;
            //样本信息记录
            Track[TrackID].EmgSID = 0;
            //获取返回轨道阻挡互斥量，返回推板阻挡
            if (xUserSemaphoreTake(Track[TrackID].BPB.Mutex, portMAX_DELAY) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].BPB, _motor_run_pos, 1) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].BPB.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            //变轨模块置返回位
            if (MotorMTCCtrl(TrackID, 3))
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20130 + (TrackID + 1) * 1000));
            }
            //样本走过后，变轨回0位，释放变轨和挡杆互斥量
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, B_MI) == 0)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20123 + (TrackID + 1) * 1000));
                }
            }
            timeout_cnt = 0;
            while (Track_Sensor_Get(TrackID, B_MI) == 1)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > SampleTimeout)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20123 + (TrackID + 1) * 1000));
                }
            }
            //传送带释放
            Track[TrackID].ESBelt.BeltRun--;
            Track[TrackID].BSBelt.BeltRun--;
            //变轨模块置0位
            if (MotorMTCCtrl(TrackID, 0))
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20130 + (TrackID + 1) * 1000));
            }
            if (xUserSemaphoreGive(Track[TrackID].MTC.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            if (xUserSemaphoreGive(Track[TrackID].MBF.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }

            //向样本控制线程发送样本信息，等待下一级轨道的样本进样许可
            if (xQueueSend(TransferRequest, (void *)&FrameTemp->FramSID, 0) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25104);
            }
            //等待许可
            while (FrameTemp->ReceiveStatus != 1)
            {
                UserTaskDelay(100);
            }
            FrameTemp->ReceiveStatus = 0;

            //收到进样许可后，返回推板释放
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].BPB, _motor_run_pos, 0) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].BPB.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            //收到下一级进样到位后，如果是第一节轨道，返回推板推送
            while (FrameTemp->ReceiveStatus != 2)
            {
                UserTaskDelay(50);
            }
            if (TrackID == 0)
            {
                // FrameTemp->ReceiveStatus = 0;
                UserTaskDelay(1000);
                if (MotorCtrl((MotorStruDef *)&Track[TrackID].BPB, _motor_run_pos, 2) != 0)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
                }
                //检测运行状态
                while (Track[TrackID].BPB.Status == _motor_running)
                {
                    UserTaskDelay(10);
                }
                //确定推板推送完成
                FrameTemp->ReceiveStatus = 0;
                //返回推板复位，释放推板互斥量
                if (MotorCtrl((MotorStruDef *)&Track[TrackID].BPB, _motor_run_pos, 0) != 0)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
                }
                //检测运行状态
                while (Track[TrackID].BPB.Status == _motor_running)
                {
                    UserTaskDelay(10);
                }
            }
            else
            {
                //确定推板推送完成
                FrameTemp->ReceiveStatus = 0;
                UserTaskDelay(2000);
            }

            if (xUserSemaphoreGive(Track[TrackID].BPB.Mutex) != pdTRUE)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, 25100);
            }
            //传送带释放
            Track[TrackID].BLBelt.BeltRun--;
            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Emg To Back Complete", FrameTemp->FrameID, TrackID + 1);
            WorkRecordReadWrite(0, 0, LogBuffer);
            break;
        default:
            break;
        }
    }
    //急诊通道样本↑

    //如果是返回通道样本↓
    else
    {
        //工作记录
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Back", FrameTemp->FrameID, TrackID + 1);
        WorkRecordReadWrite(0, 0, LogBuffer);

        if (xUserSemaphoreTake(Track[TrackID].BPB.Mutex, portMAX_DELAY) != pdTRUE)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 25100);
        }
        //传送带运行
        Track[TrackID].BLBelt.BeltRun++;
        Track[TrackID].BSBelt.BeltRun++;

        if (MotorCtrl((MotorStruDef *)&Track[TrackID].BPB, _motor_run_pos, 1) != 0)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
        }

        while (Track[TrackID].BPB.Status == _motor_running)
        {
            UserTaskDelay(10);
        }
        //向上一级返回进样许可
        FrameTemp->ReceiveStatus = 1;
        while (FrameTemp->ReceiveStatus == 1)
        {
            UserTaskDelay(100);
        }

        //样本走到后，释放变轨和挡杆互斥量
        while (Track_Sensor_Get(TrackID, B_EN) == 0)
        {
            UserTaskDelay(10);
            timeout_cnt++;
            if (timeout_cnt > SampleTimeout)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20124 + (TrackID + 1) * 1000));
            }
        }
        //向上一级返回进样完成
        FrameTemp->ReceiveStatus = 2;
        while (FrameTemp->ReceiveStatus == 2)
        {
            UserTaskDelay(100);
        }

        timeout_cnt = 0;
        while (Track_Sensor_Get(TrackID, B_MI) == 0)
        {
            UserTaskDelay(10);
            timeout_cnt++;
            if (timeout_cnt > SampleTimeout)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20123 + (TrackID + 1) * 1000));
            }
        }
        timeout_cnt = 0;
        while (Track_Sensor_Get(TrackID, B_MI) == 1)
        {
            UserTaskDelay(10);
            timeout_cnt++;
            if (timeout_cnt > SampleTimeout)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20123 + (TrackID + 1) * 1000));
            }
        }
        //传送带释放
        Track[TrackID].BSBelt.BeltRun--;


        //向样本控制线程发送样本信息，等待下一级轨道的样本进样许可
        if (xQueueSend(TransferRequest, (void *)&FrameTemp->FramSID, 0) != pdTRUE)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 25104);
        }
        //等待许可
        while (FrameTemp->ReceiveStatus != 1)
        {
            UserTaskDelay(50);
        }
        FrameTemp->ReceiveStatus = 0;

        //收到进样许可后，返回推板释放
        if (MotorCtrl((MotorStruDef *)&Track[TrackID].BPB, _motor_run_pos, 0) != 0)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
        }
        //检测运行状态
        while (Track[TrackID].BPB.Status == _motor_running)
        {
            UserTaskDelay(10);
        }
        //收到下一级进样到位后，如果是第一节轨道，返回推板推送
        while (FrameTemp->ReceiveStatus != 2)
        {
            UserTaskDelay(50);
        }
        if (TrackID == 0)
        {
            // FrameTemp->ReceiveStatus = 0;
            UserTaskDelay(1000);
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].BPB, _motor_run_pos, 2) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].BPB.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
            //确定推板推送完成
            FrameTemp->ReceiveStatus = 0;
            //返回推板复位，释放推板互斥量
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].BPB, _motor_run_pos, 0) != 0)
            {
                //错误处理
                User_Error_Handler(__FILE__, __LINE__, (20110 + (TrackID + 1) * 1000));
            }
            //检测运行状态
            while (Track[TrackID].BPB.Status == _motor_running)
            {
                UserTaskDelay(10);
            }
        }
        else
        {
            //确定推板推送完成
            FrameTemp->ReceiveStatus = 0;
            UserTaskDelay(2000);
        }

        if (xUserSemaphoreGive(Track[TrackID].BPB.Mutex) != pdTRUE)
        {
            //错误处理
            User_Error_Handler(__FILE__, __LINE__, 25100);
        }
        //传送带释放
        Track[TrackID].BLBelt.BeltRun--;
        //工作记录
        memset(LogBuffer, 0, sizeof(LogBuffer));
        sprintf(LogBuffer, "INFO TRK FrameID %ld TrackID %d Back To Back Complete", FrameTemp->FrameID, TrackID + 1);
        WorkRecordReadWrite(0, 0, LogBuffer);
    }

    for (;;)
    {
        //完成后删除线程
        vTaskDelete(NULL);
    }
}

/**
 * @brief  电机复位
 * @param  init 0-仅复位操作,带传送带运行 1-复位加初始化操作，包括所有传送带的运行和急诊挡板的工作传感器扫描
 * @retval 0-OK,1-FAIL
 */
uint8_t Motor_All_Home(uint8_t init)
{
    volatile uint32_t runtemp = 0;
#ifndef __DeviceVersion01
    uint32_t cnttemp = 0;
    uint32_t sumtemp = 0;
#endif
    ChainCtlStruDef CBCtl;
    if (MotorCtrl((MotorStruDef *)&Table.PBI, _motor_home, 0))
    {
        return 1;
    }
    UserTaskDelay(20);
    if (MotorCtrl((MotorStruDef *)&Table.PPI, _motor_home, 0))
    {
        return 1;
    }
    UserTaskDelay(10);
    if (MotorCtrl((MotorStruDef *)&Table.PBB, _motor_home, 0))
    {
        return 1;
    }
    UserTaskDelay(10);
    if (MotorCtrl((MotorStruDef *)&Table.PPB, _motor_home, 0))
    {
        return 1;
    }
    UserTaskDelay(10);
    runtemp |= 0x0f;

    //轨道
    for (uint8_t Tid = 0; Tid < TrackNumber; Tid++)
    {
        //所有控制电机回HOME
        if (MotorCtrl((MotorStruDef *)&Track[Tid].WBF, _motor_home, 0))
        {
            return 1;
        }
        UserTaskDelay(10);
        if (MotorCtrl((MotorStruDef *)&Track[Tid].NPB, _motor_home, 0))
        {
            return 1;
        }
        UserTaskDelay(10);
        if (MotorCtrl((MotorStruDef *)&Track[Tid].EPB, _motor_home, 0))
        {
            return 1;
        }
        UserTaskDelay(10);
        if (MotorCtrl((MotorStruDef *)&Track[Tid].MTC, _motor_home, 0))
        {
            return 1;
        }
        UserTaskDelay(10);
        if (MotorCtrl((MotorStruDef *)&Track[Tid].MBF, _motor_home, 0))
        {
            return 1;
        }
        UserTaskDelay(10);
        if (MotorCtrl((MotorStruDef *)&Track[Tid].BPB, _motor_home, 0))
        {
            return 1;
        }
        UserTaskDelay(10);
        runtemp |= (0x3f << (Tid * 6 + 4)); //样本台4个，轨道每节6个
    }

    //等待复位完成
    while (runtemp != 0)
    {
        if (UserReadBit(runtemp, 0))
        {
            if (Table.PBI.Status == _motor_idle)
            {
                UserClrBit(runtemp, 0);
            }
            else if (Table.PBI.Status == _motor_error)
            {
                return 1;
            }
        }

        if (UserReadBit(runtemp, 1))
        {
            if (Table.PPI.Status == _motor_idle)
            {
                UserClrBit(runtemp, 1);
            }
            else if (Table.PPI.Status == _motor_error)
            {
                return 1;
            }
        }

        if (UserReadBit(runtemp, 2))
        {
            if (Table.PBB.Status == _motor_idle)
            {
                UserClrBit(runtemp, 2);
            }
            else if (Table.PBB.Status == _motor_error)
            {
                return 1;
            }
        }

        if (UserReadBit(runtemp, 3))
        {
            if (Table.PPB.Status == _motor_idle)
            {
                UserClrBit(runtemp, 3);
            }
            else if (Table.PPB.Status == _motor_error)
            {
                return 1;
            }
        }

        //轨道
        for (uint8_t Tid = 0; Tid < TrackNumber; Tid++)
        {
            if (UserReadBit(runtemp, Tid * 6 + 4 + 0))
            {
                if (Track[Tid].WBF.Status == _motor_idle)
                {
                    UserClrBit(runtemp, Tid * 6 + 4 + 0);
                }
                else if (Track[Tid].WBF.Status == _motor_error)
                {
                    return 1;
                }
            }

            if (UserReadBit(runtemp, Tid * 6 + 4 + 1))
            {
                if (Track[Tid].NPB.Status == _motor_idle)
                {
                    UserClrBit(runtemp, Tid * 6 + 4 + 1);
                }
                else if (Track[Tid].NPB.Status == _motor_error)
                {
                    return 1;
                }
            }

            if (UserReadBit(runtemp, Tid * 6 + 4 + 2))
            {
                if (Track[Tid].EPB.Status == _motor_idle)
                {
                    UserClrBit(runtemp, Tid * 6 + 4 + 2);
                }
                else if (Track[Tid].EPB.Status == _motor_error)
                {
                    return 1;
                }
            }

            if (UserReadBit(runtemp, Tid * 6 + 4 + 3))
            {
                if (Track[Tid].MTC.Status == _motor_idle)
                {
                    UserClrBit(runtemp, Tid * 6 + 4 + 3);
                }
                else if (Track[Tid].MTC.Status == _motor_error)
                {
                    return 1;
                }
            }

            if (UserReadBit(runtemp, Tid * 6 + 4 + 4))
            {
                if (Track[Tid].MBF.Status == _motor_idle)
                {
                    UserClrBit(runtemp, Tid * 6 + 4 + 4);
                }
                else if (Track[Tid].MBF.Status == _motor_error)
                {
                    return 1;
                }
            }

            if (UserReadBit(runtemp, Tid * 6 + 4 + 5))
            {
                if (Track[Tid].BPB.Status == _motor_idle)
                {
                    UserClrBit(runtemp, Tid * 6 + 4 + 5);
                }
                else if (Track[Tid].BPB.Status == _motor_error)
                {
                    return 1;
                }
            }
        }
        vTaskDelay(10);
    }

    //回收仓电机默认处于推出位置
    if (MotorCtrl((MotorStruDef *)&Table.PPB, _motor_run_pos, 1))
    {
        return 1;
    }

    //编码器模式下返回链条复位一次
#ifdef __USEChainBackEncoder
    //释放反向电机
    if (MotorCtrl((MotorStruDef *)&Table.CHBR, _motor_rls, 0))
    {
    }
    while (1)
    {
        UserTaskDelay(50);
        if (Table.CHBR.Status == _motor_rlsd)
        {
            break;
        }
    }
//正向电机HOME
#if 0
    if (STPM_HOffset_Set(Table.CHBF.BoardID, Table.CHBF.ChannelID, Table.CHBF.Param.HOffset))
    {
        //错误处理
        User_Error_Handler(__FILE__, __LINE__, 26100);
    }
#endif
    if (MotorCtrl((MotorStruDef *)&Table.CHBF, _motor_home, 0))
    {
        //错误处理
        User_Error_Handler(__FILE__, __LINE__, 26100);
    }
    while (1)
    {
        UserTaskDelay(50);
        if (Table.CHBF.Status == _motor_idle)
        {
            break;
        }
    }
#endif
    //返回链条向前一格对齐样本位
    CBCtl.RunPos = 1;
    CBCtl.StopCtl = 0;
    CBCtl.Align = 0;
    xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&CBCtl, ChainCtl_Priority, NULL);

    while (1)
    {
        UserTaskDelay(20);
        if (ChainBack.Status == _motor_idle && Table.PPB.Status == _motor_idle)
        {
            break;
        }
    }

    for (uint8_t Tid = 0; Tid < TrackNumber; Tid++)
    {
#ifndef __DeviceVersion01
        if (init == 1)
        {
            runtemp = 0;

            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                //急诊挡板扫描所有的位置
                // 0位置也是11号位，初始在0

                for (uint8_t i = 0; i < 11; i++)
                {

                    //先执行清零操作
                    if (STPM_EXINCnt_Set(Track[Tid].EPB.BoardID, Track[Tid].EPB.ChannelID))
                    {
                        break;
                    }

                    //走到下一个位置
                    if (MotorCtrl((MotorStruDef *)&Track[Tid].EPB, _motor_run_pos, 10 - i)) //运行顺序 10-1，再快速回0
                    {
                        break;
                    }
                    while (Track[Tid].EPB.Status != _motor_idle)
                    {
                        vTaskDelay(50);
                    }
                    //读取位置值
                    if (STPM_EXINCnt_Get(Track[Tid].EPB.BoardID, Track[Tid].EPB.ChannelID, &cnttemp))
                    {
                        break;
                    }
                    if (i == 10) //快回0判断
                    {
                        sumtemp = 0;
                        for (uint8_t j = 0; j < 10; j++)
                        {
                            sumtemp += Track[Tid].EPBWKCnt[j];
                        }
                        if (cnttemp == sumtemp)
                        {
                            runtemp = 1; //成功
                        }
                        else
                        {
                            //位置校验失败，清空
                            for (uint8_t j = 0; i < 10; j++)
                            {
                                Track[Tid].EPBWKCnt[j] = 0;
                            }
                        }
                    }
                    else
                    {
                        Track[Tid].EPBWKCnt[9 - i] = (uint8_t)cnttemp;
                    }
                }
                if (runtemp != 0)
                {
                    break;
                }
                if (cnt_try == MotorMaxTry - 1)
                {
                    //错误
                    return 1;
                }
            }
        }
#endif
        //所有传送带运行
        if (MotorCtrl((MotorStruDef *)&Track[Tid].NBL, _motor_run_spd, 0))
        {
            return 1;
        }
        UserTaskDelay(20);
        if (MotorCtrl((MotorStruDef *)&Track[Tid].NBS, _motor_run_spd, 0))
        {
            return 1;
        }
        UserTaskDelay(20);
        if (MotorCtrl((MotorStruDef *)&Track[Tid].EBL, _motor_run_spd, 0))
        {
            return 1;
        }
        UserTaskDelay(20);
        if (MotorCtrl((MotorStruDef *)&Track[Tid].EBS, _motor_run_spd, 0))
        {
            return 1;
        }
        UserTaskDelay(20);
        if (MotorCtrl((MotorStruDef *)&Track[Tid].BBL, _motor_run_spd, 0))
        {
            return 1;
        }
        UserTaskDelay(20);
        if (MotorCtrl((MotorStruDef *)&Track[Tid].BBS, _motor_run_spd, 0))
        {
            return 1;
        }
        UserTaskDelay(20);

        //等待传送带运行完成
        runtemp = 0x3f;
        while (runtemp != 0)
        {
            if (UserReadBit(runtemp, 0))
            {
                if (Track[Tid].NBL.Status == _motor_idle)
                {
                    UserClrBit(runtemp, 0);
                }
                else if (Track[Tid].NBL.Status == _motor_error)
                {
                    return 1;
                }
                else
                {
                    if (STPM_SpeedStatus_Get(Track[Tid].NBL.BoardID, Track[Tid].NBL.ChannelID) == 1)
                    {
                        Track[Tid].NBL.Status = _motor_idle;
                    }
                }
            }

            if (UserReadBit(runtemp, 1))
            {
                if (Track[Tid].NBS.Status == _motor_idle)
                {
                    UserClrBit(runtemp, 1);
                }
                else if (Track[Tid].NBS.Status == _motor_error)
                {
                    return 1;
                }
                else
                {
                    if (STPM_SpeedStatus_Get(Track[Tid].NBS.BoardID, Track[Tid].NBS.ChannelID) == 1)
                    {
                        Track[Tid].NBS.Status = _motor_idle;
                    }
                }
            }

            if (UserReadBit(runtemp, 2))
            {
                if (Track[Tid].EBL.Status == _motor_idle)
                {
                    UserClrBit(runtemp, 2);
                }
                else if (Track[Tid].EBL.Status == _motor_error)
                {
                    return 1;
                }
                else
                {
                    if (STPM_SpeedStatus_Get(Track[Tid].EBL.BoardID, Track[Tid].EBL.ChannelID) == 1)
                    {
                        Track[Tid].EBL.Status = _motor_idle;
                    }
                }
            }

            if (UserReadBit(runtemp, 3))
            {
                if (Track[Tid].EBS.Status == _motor_idle)
                {
                    UserClrBit(runtemp, 3);
                }
                else if (Track[Tid].EBS.Status == _motor_error)
                {
                    return 1;
                }
                else
                {
                    if (STPM_SpeedStatus_Get(Track[Tid].EBS.BoardID, Track[Tid].EBS.ChannelID) == 1)
                    {
                        Track[Tid].EBS.Status = _motor_idle;
                    }
                }
            }

            if (UserReadBit(runtemp, 4))
            {
                if (Track[Tid].BBL.Status == _motor_idle)
                {
                    UserClrBit(runtemp, 4);
                }
                else if (Track[Tid].BBL.Status == _motor_error)
                {
                    return 1;
                }
                else
                {
                    if (STPM_SpeedStatus_Get(Track[Tid].BBL.BoardID, Track[Tid].BBL.ChannelID) == 1)
                    {
                        Track[Tid].BBL.Status = _motor_idle;
                    }
                }
            }

            if (UserReadBit(runtemp, 5))
            {
                if (Track[Tid].BBS.Status == _motor_idle)
                {
                    UserClrBit(runtemp, 5);
                }
                else if (Track[Tid].BBS.Status == _motor_error)
                {
                    return 1;
                }
                else
                {
                    if (STPM_SpeedStatus_Get(Track[Tid].BBS.BoardID, Track[Tid].BBS.ChannelID) == 1)
                    {
                        Track[Tid].BBS.Status = _motor_idle;
                    }
                }
            }
            vTaskDelay(50);
        }
    }

    //工作记录
    memset(LogBuffer, 0, sizeof(LogBuffer));
    sprintf(LogBuffer, "INFO MOT ALL Home Complete");
    WorkRecordReadWrite(0, 0, LogBuffer);
    return 0;
}

/**
 * @brief  所有电机停止并释放
 * @param  mode 0停止后获取所有互斥量 1仅电机停止
 * @retval None
 */
void Motor_All_Stop(uint8_t mode)
{
    for (uint8_t cnt = 0; cnt < (mode == 0 ? 2 : 1); cnt++)
    {
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.CHIF, _motor_stop, 0))
            {
                break;
            }
        }
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.CHIR, _motor_stop, 0))
            {
                break;
            }
        }
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.CHBF, _motor_stop, 0))
            {
                break;
            }
        }
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.CHBR, _motor_stop, 0))
            {
                break;
            }
        }
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.PBI, _motor_stop, 0))
            {
                break;
            }
        }
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.PPI, _motor_stop, 0))
            {
                break;
            }
        }
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.PBB, _motor_stop, 0))
            {
                break;
            }
        }
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.PPB, _motor_stop, 0))
            {
                break;
            }
        }

        for (uint8_t Tid = 0; Tid < TrackNumber; Tid++)
        {
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].WBF, _motor_stop, 0))
                {
                    break;
                }
            }

            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].MTC, _motor_stop, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].MBF, _motor_stop, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].BPB, _motor_stop, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].NBL, _motor_stop, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].NBS, _motor_stop, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].EBL, _motor_stop, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].EBS, _motor_stop, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].BBL, _motor_stop, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].BBS, _motor_stop, 0))
                {
                    break;
                }
            }

            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
#if 0
                if (SystemEmergencyStop)
                {
                    SystemEmergencyStop=0;

                    if (!MotorCtrl((MotorStruDef *)&Track[Tid].NPB, _motor_run_pos, 11))
                    {
                        break;
                    }

                    while (Track[Tid].NPB.Status == _motor_running)
                    {
                        vTaskDelay(10);
                    }
                    SystemEmergencyStop=1;
                }
#endif

                if (!MotorCtrl((MotorStruDef *)&Track[Tid].NPB, _motor_stop, 0))
                {
                    break;
                }
            }

            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
#if 0
                if (SystemEmergencyStop)
                {
                    SystemEmergencyStop=0;

                    if (!MotorCtrl((MotorStruDef *)&Track[Tid].EPB, _motor_home, 0))
                    {
                        break;
                    }

                    while (Track[Tid].EPB.Status == _motor_running)
                    {
                        vTaskDelay(10);
                    }
                    SystemEmergencyStop=1;
                }
#endif
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].EPB, _motor_stop, 0))
                {
                    break;
                }
            }
        }

        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.CHIF, _motor_rls, 0))
            {
                break;
            }
        }
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.CHIR, _motor_rls, 0))
            {
                break;
            }
        }
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.CHBF, _motor_rls, 0))
            {
                break;
            }
        }
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.CHBR, _motor_rls, 0))
            {
                break;
            }
        }
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.PBI, _motor_rls, 0))
            {
                break;
            }
        }
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.PPI, _motor_rls, 0))
            {
                break;
            }
        }
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.PBB, _motor_rls, 0))
            {
                break;
            }
        }
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            if (!MotorCtrl((MotorStruDef *)&Table.PPB, _motor_rls, 0))
            {
                break;
            }
        }

        for (volatile uint8_t Tid = 0; Tid < TrackNumber; Tid++)
        {
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].WBF, _motor_rls, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].NPB, _motor_rls, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].EPB, _motor_rls, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].MTC, _motor_rls, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].MBF, _motor_rls, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].BPB, _motor_rls, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].NBL, _motor_rls, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].NBS, _motor_rls, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].EBL, _motor_rls, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].EBS, _motor_rls, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].BBL, _motor_rls, 0))
                {
                    break;
                }
            }
            for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
            {
                if (!MotorCtrl((MotorStruDef *)&Track[Tid].BBS, _motor_rls, 0))
                {
                    break;
                }
            }
        }

        if (cnt == 0 && mode == 0)
        {
            while (FullSemaphoreTake(portMAX_DELAY) == 1)
            {
                //拿到所有电机互斥量
            }
        }
    }
}

/**
 * @brief  轨道传送带运行
 * @param  trackid:轨道编号,0-3
 * @retval 0:成功 1：失败
 */
uint8_t Track_Belt_Run(uint8_t trackid)
{
    uint8_t runtemp = 0;
    uint32_t runcnt = 0;
    //所有传送带运行
    if (MotorCtrl((MotorStruDef *)&Track[trackid].NBL, _motor_run_spd, 0))
    {
        return 1;
    }
    if (MotorCtrl((MotorStruDef *)&Track[trackid].NBS, _motor_run_spd, 0))
    {
        return 1;
    }
    if (MotorCtrl((MotorStruDef *)&Track[trackid].EBL, _motor_run_spd, 0))
    {
        return 1;
    }
    if (MotorCtrl((MotorStruDef *)&Track[trackid].EBS, _motor_run_spd, 0))
    {
        return 1;
    }
    if (MotorCtrl((MotorStruDef *)&Track[trackid].BBL, _motor_run_spd, 0))
    {
        return 1;
    }
    if (MotorCtrl((MotorStruDef *)&Track[trackid].BBS, _motor_run_spd, 0))
    {
        return 1;
    }

    //等待传送带运行完成
    runtemp = 0x3f;
    while (runtemp != 0)
    {
        runcnt++;
        if (runcnt > MotorTimeout)
        {
            return 1;
        }
        if (UserReadBit(runtemp, 0))
        {
            if (Track[trackid].NBL.Status == _motor_idle)
            {
                UserClrBit(runtemp, 0);
            }
        }

        if (UserReadBit(runtemp, 1))
        {
            if (Track[trackid].NBS.Status == _motor_idle)
            {
                UserClrBit(runtemp, 1);
            }
        }

        if (UserReadBit(runtemp, 2))
        {
            if (Track[trackid].EBL.Status == _motor_idle)
            {
                UserClrBit(runtemp, 2);
            }
        }

        if (UserReadBit(runtemp, 3))
        {
            if (Track[trackid].EBS.Status == _motor_idle)
            {
                UserClrBit(runtemp, 3);
            }
        }

        if (UserReadBit(runtemp, 4))
        {
            if (Track[trackid].BBL.Status == _motor_idle)
            {
                UserClrBit(runtemp, 4);
            }
        }

        if (UserReadBit(runtemp, 5))
        {
            if (Track[trackid].BBS.Status == _motor_idle)
            {
                UserClrBit(runtemp, 5);
            }
        }
        vTaskDelay(10);
    }
    return 0;
}

/**
 * @brief  轨道传送带停止
 * @param  trackid:轨道编号,0-3
 * @retval 0:成功 1：失败
 */
uint8_t Track_Belt_Stop(uint8_t trackid)
{
    uint8_t runtemp = 0;
    uint32_t runcnt = 0;
    //所有传送带停止
    if (MotorCtrl((MotorStruDef *)&Track[trackid].NBL, _motor_stop, 0))
    {
        return 1;
    }
    if (MotorCtrl((MotorStruDef *)&Track[trackid].NBS, _motor_stop, 0))
    {
        return 1;
    }
    if (MotorCtrl((MotorStruDef *)&Track[trackid].EBL, _motor_stop, 0))
    {
        return 1;
    }
    if (MotorCtrl((MotorStruDef *)&Track[trackid].EBS, _motor_stop, 0))
    {
        return 1;
    }
    if (MotorCtrl((MotorStruDef *)&Track[trackid].BBL, _motor_stop, 0))
    {
        return 1;
    }
    if (MotorCtrl((MotorStruDef *)&Track[trackid].BBS, _motor_stop, 0))
    {
        return 1;
    }
    //所有传送带释放
    if (MotorCtrl((MotorStruDef *)&Track[trackid].NBL, _motor_rls, 0))
    {
        return 1;
    }
    if (MotorCtrl((MotorStruDef *)&Track[trackid].NBS, _motor_rls, 0))
    {
        return 1;
    }
    if (MotorCtrl((MotorStruDef *)&Track[trackid].EBL, _motor_rls, 0))
    {
        return 1;
    }
    if (MotorCtrl((MotorStruDef *)&Track[trackid].EBS, _motor_rls, 0))
    {
        return 1;
    }
    if (MotorCtrl((MotorStruDef *)&Track[trackid].BBL, _motor_rls, 0))
    {
        return 1;
    }
    if (MotorCtrl((MotorStruDef *)&Track[trackid].BBS, _motor_rls, 0))
    {
        return 1;
    }
    //等待传送带运行完成
    runtemp = 0x3f;
    while (runtemp != 0)
    {
        runcnt++;
        if (runcnt > MotorTimeout)
        {
            return 1;
        }
        if (UserReadBit(runtemp, 0))
        {
            if (Track[trackid].NBL.Status == _motor_rlsd)
            {
                UserClrBit(runtemp, 0);
            }
        }

        if (UserReadBit(runtemp, 1))
        {
            if (Track[trackid].NBS.Status == _motor_rlsd)
            {
                UserClrBit(runtemp, 1);
            }
        }

        if (UserReadBit(runtemp, 2))
        {
            if (Track[trackid].EBL.Status == _motor_rlsd)
            {
                UserClrBit(runtemp, 2);
            }
        }

        if (UserReadBit(runtemp, 3))
        {
            if (Track[trackid].EBS.Status == _motor_rlsd)
            {
                UserClrBit(runtemp, 3);
            }
        }

        if (UserReadBit(runtemp, 4))
        {
            if (Track[trackid].BBL.Status == _motor_rlsd)
            {
                UserClrBit(runtemp, 4);
            }
        }

        if (UserReadBit(runtemp, 5))
        {
            if (Track[trackid].BBS.Status == _motor_rlsd)
            {
                UserClrBit(runtemp, 5);
            }
        }
        vTaskDelay(10);
    }
    return 0;
}

/**
 * @brief  轨道急诊挡板控制
 * @param  TrackID：轨道编号0-3，posid：运行的位置
 * @retval 0-成功 1-失败
 */
static uint8_t MotorEPBCtrl(uint8_t TrackID, uint8_t posid)
{
#ifndef __DeviceVersion01 //第一版本机器不检测该运行位置
    uint8_t cur_pos = 0;
    uint8_t cnt_sum = 0;
    uint32_t cnt_temp = 0;
#endif
    uint8_t error = 0;
    uint8_t last_pos = 0;
    uint8_t belt_stop = 0;
    uint32_t timeout_cnt = 0;
    for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
    {
        error = 0;
        //清零位置值
        if (STPM_EXINCnt_Set(Track[TrackID].EPB.BoardID, Track[TrackID].EPB.ChannelID))
        {
            error = 1;
            continue;
        }
        last_pos = Track[TrackID].EPB.CurPos;
        if (last_pos == 0)
        {
            last_pos = 11;
        }
        if (MotorCtrl((MotorStruDef *)&Track[TrackID].EPB, _motor_run_pos, posid) != 0)
        {
            error = 1;
            continue;
        }
        //检测运行状态
        timeout_cnt = 0;
        while (Track[TrackID].EPB.Status == _motor_running)
        {
            UserTaskDelay(10);
            timeout_cnt++;
            if (timeout_cnt > MotorTimeout)
            {
                error = 1;
                break;
            }
        }
        if (error == 1)
        {
            continue;
        }
#ifdef __DeviceVersion01 //第一版本机器不检测该运行位置
        error = 0;
        break;
#else
        //读取位置值
        if (STPM_EXINCnt_Get(Track[TrackID].EPB.BoardID, Track[TrackID].EPB.ChannelID, &cnt_temp))
        {
            error = 1;
            continue;
        }
        //位置检测
        cur_pos = posid;
        if (cur_pos == 0)
        {
            cur_pos = 11;
        }
        cnt_sum = 0;
        if (cur_pos > last_pos)
        {
            for (uint8_t pos = last_pos; pos < cur_pos; pos++)
            {
                cnt_sum += Track[TrackID].EPBWKCnt[pos - 1];
            }
        }
        else if (cur_pos < last_pos)
        {
            for (uint8_t pos = cur_pos; pos < last_pos; pos++)
            {
                cnt_sum += Track[TrackID].EPBWKCnt[pos - 1];
            }
        }
        if (cnt_sum == cnt_temp)
        {
            //检验成功
            break;
        }

        //到这里说明出错了
        //急诊长传送带停止
        belt_stop = 1;
        MotorCtrl((MotorStruDef *)&Track[TrackID].EBL, _motor_stop, 0);
        while (Track[TrackID].EBL.Status != _motor_idle)
        {
            vTaskDelay(10);
        }

        //回HOME
        if (MotorCtrl((MotorStruDef *)&Track[TrackID].EPB, _motor_home, 0) != 0)
        {
            error = 1;
            continue;
        }
        //检测运行状态
        timeout_cnt = 0;
        while (Track[TrackID].EPB.Status == _motor_goinghome)
        {
            UserTaskDelay(10);
            timeout_cnt++;
            if (timeout_cnt > MotorTimeout)
            {
                error = 1;
                break;
            }
        }
        if (error == 1)
        {
            continue;
        }
#endif
    }
    if (error == 0 && belt_stop == 1)
    {
        MotorCtrl((MotorStruDef *)&Track[TrackID].EBL, _motor_run_spd, 0);
        while (Track[TrackID].EBL.Status != _motor_idle)
        {
            vTaskDelay(10);
        }
    }
    return error;
}

/**
 * @brief  轨道变轨电机返回轨道控制
 * @param  TrackID：轨道编号0-3，posid：运行的位置 0-由返回轨道复位 3-由其他位置走到返回轨道
 * @retval 0-成功 1-失败
 */
uint8_t MotorMTCCtrl(uint8_t TrackID, uint8_t posid)
{
    uint8_t error = 0;
    uint32_t timeout_cnt = 0;

    // 0-由返回轨道复位
    if (posid == 0)
    {
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            error = 0;
            //变轨模块置返回位
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_run_pos, 0) != 0)
            {
                error = 1;
                continue;
            }
            //检测运行状态
            timeout_cnt = 0;
            while (Track[TrackID].MTC.Status == _motor_running)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > MotorTimeout)
                {
                    error = 1;
                    break;
                }
            }
            if (error != 0)
            {
                continue;
            }
            //检测传感器状态
            if (Track_Sensor_Get(TrackID, MTC_SH) == 0)
            {
                error = 1;
                if (cnt_try == MotorMaxTry - 1)
                {
                    return 1;
                }
                //再走一次返回
                if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_run_pos, 3) != 0)
                {
                    error = 1;
                    continue;
                }
                //检测运行状态
                timeout_cnt = 0;
                while (Track[TrackID].MTC.Status == _motor_running)
                {
                    UserTaskDelay(10);
                    timeout_cnt++;
                    if (timeout_cnt > MotorTimeout)
                    {
                        error = 1;
                        break;
                    }
                }
            }
            if (error == 0)
            {
                break;
            }
        }
    }
    else if (posid == 3)
    {
        for (uint8_t cnt_try = 0; cnt_try < MotorMaxTry; cnt_try++)
        {
            error = 0;

            //停止三条短传送带
            Track[TrackID].NSBelt.BeltStop = 1;
            Track[TrackID].ESBelt.BeltStop = 1;
            Track[TrackID].BSBelt.BeltStop = 1;
            UserTaskDelay(100);

            //变轨模块置返回位
            if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_run_pos, 3) != 0)
            {
                error = 1;
                continue;
            }
            //检测运行状态
            timeout_cnt = 0;
            while (Track[TrackID].MTC.Status == _motor_running)
            {
                UserTaskDelay(10);
                timeout_cnt++;
                if (timeout_cnt > MotorTimeout)
                {
                    error = 1;
                    break;
                }
            }
            if (error != 0)
            {
                continue;
            }
            //检测传感器状态
            if (Track_Sensor_Get(TrackID, MTC_SW) == 0)
            {
                error = 1;
                //回HOME
                if (MotorCtrl((MotorStruDef *)&Track[TrackID].MTC, _motor_home, 0) != 0)
                {
                    error = 1;
                    continue;
                }
                //检测运行状态
                timeout_cnt = 0;
                while (Track[TrackID].MTC.Status == _motor_goinghome)
                {
                    UserTaskDelay(10);
                    timeout_cnt++;
                    if (timeout_cnt > MotorTimeout)
                    {
                        error = 1;
                        break;
                    }
                }
            }
            if (error == 0)
            {
                //重新启动三条短传送带
                Track[TrackID].NSBelt.BeltStop = 0;
                Track[TrackID].ESBelt.BeltStop = 0;
                Track[TrackID].BSBelt.BeltStop = 0;
                UserTaskDelay(100);
                break;
            }
            //出错
            if (cnt_try == MotorMaxTry - 1)
            {
                //错误处理
                return 1;
            }
        }
    }
    else
    {
        return 1;
    }
    return 0;
}

/**
 * @brief  样本台按键控制线程
 * @param  None
 * @retval None
 */
static void TableButCtrlTask(void *pvParameters)
{
    //    uint32_t LED_Cnt = 0;
    (void)pvParameters;
    static uint8_t backcount = 0;
    static uint8_t startflg = 55;
    startflg = 55;
    for (;;)
    {
        //按钮采集
        //暂停按钮
        if (ButPause.But_Status == 0)
        {
            if (Table_Sensor_Get(_BUT_Pause))
            {
                if (ButPause.But_Cnt == 0)
                {
                    ButPause.But_Cnt = 1;
                }
                else
                {
                    ButPause.But_Status = 1;
                    ButPause.But_Cnt = 0;
#if 1
                    if (startflg == 55)
                    {
                        scanner_flg = 3;
                        startflg = 0;
                    }
#else

                    EthSendButtonStart();

#endif

                    //工作记录
                    memset(LogBuffer, 0, sizeof(LogBuffer));
                    sprintf(LogBuffer, "INFO SYS Button Pause Pressed");
                    WorkRecordReadWrite(0, 0, LogBuffer);
                }
            }
            else
            {
                ButPause.But_Cnt = 0;
            }
        }
        else
        {
            if (!Table_Sensor_Get(_BUT_Pause))
            {
                if (ButPause.But_Cnt == 0)
                {
                    ButPause.But_Cnt = 1;
                }
                else
                {
                    ButPause.But_Status = 0;
                    ButPause.But_Cnt = 0;
                    //工作记录
                    memset(LogBuffer, 0, sizeof(LogBuffer));
                    sprintf(LogBuffer, "INFO SYS Button Pause Released");
                    WorkRecordReadWrite(0, 0, LogBuffer);
                }
            }
            else
            {
                ButPause.But_Cnt = 0;
            }
        }
        //返回按钮
        if (ButBack.But_Status == 0)
        {
            if (Table_Sensor_Get(_BUT_Back))
            {
                if (ButBack.But_Cnt == 0)
                {
                    ButBack.But_Cnt = 1;
                }
                else
                {
                    ButBack.But_Status = 1;
                    ButBack.But_Cnt = 0;
                    //工作记录
                    memset(LogBuffer, 0, sizeof(LogBuffer));
                    sprintf(LogBuffer, "INFO SYS Button Back Pressed");
                    WorkRecordReadWrite(0, 0, LogBuffer);
                }
            }
            else
            {
                ButBack.But_Cnt = 0;
            }
        }
        else
        {
            if (!Table_Sensor_Get(_BUT_Back))
            {
                if (ButBack.But_Cnt == 0)
                {
                    ButBack.But_Cnt = 1;
                }
                else
                {
                    ButBack.But_Status = 0;
                    ButBack.But_Cnt = 0;
                    //工作记录
                    memset(LogBuffer, 0, sizeof(LogBuffer));
                    sprintf(LogBuffer, "INFO SYS Button Back Released");
                    WorkRecordReadWrite(0, 0, LogBuffer);
                }
            }
            else
            {
                ButBack.But_Cnt = 0;
            }
        }
        //按钮处理
        //暂停按钮
        if (ButPause.But_Status == 1 && ButPause.But_Status_Prev == 0)
        {

            if (ChainIn.RunCtl == _cctl_userpause || ChainIn.RunCtl == _cctl_autopause) //用户暂停中
            {
                // ChainIn.RunCtl = _cctl_run; //切换为运行
                EthSendButtonStart();
                if (backcount)
                {
                    backcount = 0; // yfxiao 增加一个退回的计数变量，用于控制急诊，用户只要有后退就默认为需要急诊
                }
                //工作记录
                memset(LogBuffer, 0, sizeof(LogBuffer));
                sprintf(LogBuffer, "INFO CHI User Run");
                WorkRecordReadWrite(0, 0, LogBuffer);
            }
            else if (ChainIn.RunCtl == _cctl_run || ChainIn.RunCtl == _cctl_autopause) //运行中或自动暂停中
            {
                // ChainIn.RunCtl = _cctl_userpause; //切换为用户暂停
                EthSendButtonStart();
                //工作记录
                memset(LogBuffer, 0, sizeof(LogBuffer));
                sprintf(LogBuffer, "INFO CHI User Pause");
                WorkRecordReadWrite(0, 0, LogBuffer);
            }
            else if (ChainIn.RunCtl == _cctl_stop)
            {
                EthSendButtonStart();
            }

            ButPause.But_Status_Prev = 1;
        }
        if (ButPause.But_Status == 0 && ButPause.But_Status_Prev == 1)
        {
            ButPause.But_Status_Prev = 0;
        }

        //后退按钮
        if (ButBack.But_Status == 1 && ButBack.But_Status_Prev == 0 || emg_flg == 10)
        {
            // if (ChainIn.RunCtl == _cctl_userpause && DeviceStatus != Pause) //用户暂停中
            if (DeviceStatus != Pause) //用户暂停中
            {
                ChainIn.RunCtl = _cctl_userpause;
                // scanner_flg = 3;
                ChainIn.UserGoBack = 1;
                backcount++;
            }

            ButBack.But_Status_Prev = 1;
        }
        if (ButBack.But_Status == 0 && ButBack.But_Status_Prev == 1)
        {
            ButBack.But_Status_Prev = 0;
        }

        // LED状态处理,仅处理运行和暂停的状态
        if ((DeviceStatus == Run)) //|| (DeviceStatus == Pause))
        {
#if 0
            if (ChainIn.RunCtl == _cctl_stop || ChainIn.Status == _chain_idle||ChainIn.RunCtl == _cctl_autopause && ChainBack.ReRunStatus != _crrs_Prepared) //不运行，灭
            {
                ButPause.LED_Status = 0;
            }
            else if (ChainIn.RunCtl == _cctl_userpause && ChainIn.Status == _chain_idle) //用户暂停中，灭
            {
                ButPause.LED_Status = 0;
            }
            else
#endif

                // if   (DeviceFrameCnt == 0) &&  (  ChainIn.RunCtl == _cctl_run) //运行中，常亮 ChainIn.Status!= _chain_idle  ChainIn.RunCtl == _cctl_run && ChainIn.Status == _chain_running && ChainIn.UserGoBack != 1 &&
                if (ChainIn.RunCtl == _cctl_autopause || ChainIn.RunCtl == _cctl_userpause) //(DeviceStatus == Run || DeviceStatus == StandBy)&&
                {
                    ButPause.LED_Status = 0;
                }

                else if (ChainIn.UserGoBack == 0 && ChainIn.RunCtl == _cctl_run)
                {
                    ButPause.LED_Status = 1;
                }
                else
                {

                    ButPause.LED_Status = 0;
                }

            //后退按钮LED
            if (ChainIn.RunCtl == _cctl_userpause) //可操作
            {
                if (ChainIn.UserGoBack == 1) //操作中，闪
                {
                    ButBack.LED_Status = 2;
                }
                else //可操作，亮
                {
                    ButBack.LED_Status = 0;
                }
            }
            else //不可操作，灭
            {
                ButBack.LED_Status = 0;
            }
        }
#if 1
        else if (DeviceStatus == Pause) //暂停状态
        {
            if (DeviceStatusChg == Run) //切换中
            {
                //闪
                ButPause.LED_Status = 2;
            }
            else
            {
                //灭
                ButPause.LED_Status = 0;
            }
        }
#endif

        // LED控制
        //暂停按钮LED
        if (ButPause.LED_Status == 0 && ButPause.LED_Status_Prev != 0) //灭
        {
            LED_Set(0, 0);
            ButPause.LED_Status_Prev = 0;
        }
        else if (ButPause.LED_Status == 1 && ButPause.LED_Status_Prev != 1) //亮
        {
            LED_Set(0, 1);
            ButPause.LED_Status_Prev = 1;
        }
        else if (ButPause.LED_Status == 2) //闪
        {
            if (ButPause.LED_Status_Prev != 2) //首次进入
            {
                LED_Set(0, 1); //亮
                ButPause.LED_Cnt = 0;
                ButPause.LED_Status_Prev = 2;
            }
            else if (ButPause.LED_Cnt > 20) // 20*10ms
            {
                LED_Set(0, 2); //翻转
                ButPause.LED_Cnt = 0;
            }
            else
            {
                ButPause.LED_Cnt++;
            }
        }
        else if (ButPause.LED_Status == 3) //闪
        {
            if (ButPause.LED_Status_Prev != 3) //首次进入
            {
                LED_Set(0, 1); //亮
                ButPause.LED_Cnt = 0;
                ButPause.LED_Status_Prev = 3;
            }
            else if (ButPause.LED_Cnt > 100) // 100*10ms
            {
                LED_Set(0, 2); //翻转
                ButPause.LED_Cnt = 0;
            }
            else
            {
                ButPause.LED_Cnt++;
            }
        }
        //后退按钮LED
        if (ButBack.LED_Status == 0 && ButBack.LED_Status_Prev != 0) //灭
        {
            LED_Set(1, 0);
            ButBack.LED_Status_Prev = 0;
        }
        else if (ButBack.LED_Status == 1 && ButBack.LED_Status_Prev != 1) //亮
        {
            LED_Set(1, 1);
            ButBack.LED_Status_Prev = 1;
        }
        else if (ButBack.LED_Status == 2) //快闪
        {
            if (ButBack.LED_Status_Prev != 2) //首次进入
            {
                LED_Set(1, 1); //亮
                ButBack.LED_Cnt = 0;
                ButBack.LED_Status_Prev = 2;
            }
            else if (ButBack.LED_Cnt > 20) // 20*10ms
            {
                LED_Set(1, 2); //翻转
                ButBack.LED_Cnt = 0;
            }
            else
            {
                ButBack.LED_Cnt++;
            }
        }
        else if (ButBack.LED_Status == 3) //慢闪
        {
            if (ButBack.LED_Status_Prev != 3) //首次进入
            {
                LED_Set(1, 1); //亮
                ButBack.LED_Cnt = 0;
                ButBack.LED_Status_Prev = 3;
            }
            else if (ButBack.LED_Cnt > 100) // 100*10ms
            {
                LED_Set(1, 2); //翻转
                ButBack.LED_Cnt = 0;
            }
            else
            {
                ButBack.LED_Cnt++;
            }
        }
        //阻塞时间
        vTaskDelay(10);
    }
}

/**
 * @brief  当扫描为空时候，需要发送scanerror info to PC 仅仅报这个信息
 * @param  None
 * @retval None
 */
void SendScanErrorCode(void)
{
    memset(LogBuffer, 0, sizeof(LogBuffer));
    sprintf(LogBuffer, "INFO sendscanerrorcode F_ERROR Send Info");
    WorkRecordReadWrite(0, 0, LogBuffer);
    ScanErrorAlarm.alarm = 1;
    ScanErrorAlarm.code = ScanErrorCode;
    ScanErrorAlarm.level = 0; // info
    //发送info
    while (EthSendAlarm((DeviceAlarmStruDef *)&ScanErrorAlarm) != 0)
    {
        vTaskDelay(500);
    }
}
#if 0
/**
  * @brief  样本台进样部分控制线程
  * @param  None
  * @retval None
  */
uint32_t RunCnt = 0;
static void TableInCtrlTask(void *pvParameters)
{
    (void)pvParameters;
    ChainCtlStruDef CICtl;
    uint8_t SID_Temp = 0;
    uint8_t CNT_Temp = 0;
    uint32_t RunCntTemp = 0;
    TickType_t xStart, tmp;
    //uint8_t u8Temp = 0;

    uint8_t tid;

    for (;;)
    {
        if ((RunCnt != 0) && (ChainIn.RunCtl == _cctl_userpause))
        {
            RunCnt = 0;
        }

        if (ChainBack.ReRunStatus == _crrs_Prepared && ChainIn.RunCtl != _cctl_userpause) //准备复测
        {
            if (RunCnt >= 2)
            {
                ChainIn.RunCtl = _cctl_run;
                //先往前走1格对齐位置
                CICtl.RunPos = 1;
                CICtl.Align = 1;
                CICtl.StopCtl = 0;
                xUserTaskCreate(ChainInCtrlTask, "ChainInCtrlTask", MotorCtl_STACK_SIZE, (void *)&CICtl, ChainCtl_Priority, NULL);
                while (1)
                {
                    UserTaskDelay(20);
                    if (ChainIn.Status == _motor_idle)
                    {
                        break;
                    }
                }
                ChainBack.ReRunStatus = _crrs_CIReady;
                while (ChainBack.ReRunStatus != _crrs_PPCplt && ChainBack.ReRunStatus != _crrs_Idle)
                {
                    UserTaskDelay(20);
                }
                // 如果前两个传感器有信号，则往后走2格
                //if (Table_Sensor_Get(ChainIn_FS) || Table_Sensor_Get(ChainIn_SS))
                if (ChainBack.ReRunStatus != _crrs_Idle && ChainIn.RunCtl != _cctl_userpause)
                {
                    CICtl.RunPos = -2;
                    CICtl.StopCtl = 0;
                    xUserTaskCreate(ChainInCtrlTask, "ChainInCtrlTask", MotorCtl_STACK_SIZE, (void *)&CICtl, ChainCtl_Priority, NULL);
                    while (1)
                    {
                        UserTaskDelay(20);
                        if (ChainIn.Status == _chain_idle)
                        {
                            break;
                        }
                    }
                    vTaskDelay(300);
                    RunCnt = 0;
                }
                ChainBack.ReRunStatus = _crrs_Idle;
                if (ChainIn.RunCtl != _cctl_userpause)
                {
                    ChainIn.RunCtl = _cctl_run;
                }
            }
        }
        else if (ChainBack.ReRunStatus == _crrs_Prepared && ChainIn.RunCtl == _cctl_userpause)
        {
            //发送告警信息
            if (UserPausedRerunWaitingAlarm.alarm == 0)
            {
                UserPausedRerunWaitingAlarm.alarm = 1;
            }
        }
        if (ChainIn.RunCtl == _cctl_run)
        {
            while (DeviceStatus != Run)
            {
                vTaskDelay(100);
            }
            // //先检查第一位有没有样本，有的话后退
            // if (Table_Sensor_Get(ChainIn_FS))
            // {
            //
            // }
            // if (xUserSemaphoreTake(ChainIn.Mutex, portMAX_DELAY) != pdTRUE)
            // {
            //     //错误处理
            //     User_Error_Handler(__FILE__,__LINE__,25100);
            // }
            //前进30节，找样本
            CICtl.RunPos = 30;
            CICtl.Align = 0;
            CICtl.StopCtl = 1;
            xUserTaskCreate(ChainInCtrlTask, "ChainInCtrlTask", MotorCtl_STACK_SIZE, (void *)&CICtl, ChainCtl_Priority, NULL);
            RunCntTemp = RunCnt;

            xStart = xTaskGetTickCount();

            while (1)
            {
                UserTaskDelay(100);
                RunCnt = CICtl.RunCnt;

                tmp = xTaskGetTickCount();

                if (ChainBack.ReRunStatus == _crrs_Prepared && (RunCnt + RunCntTemp) >= 2) //准备复测
                {
                    ChainIn.RunCtl = _cctl_stop;
                    //break;
                }
                if (ChainIn.Status == _motor_idle || (tmp - xStart) > 10000) //yfxiao modfiy
                {
                    break;
                }
            }
            RunCnt += RunCntTemp;
            if (ChainBack.ReRunStatus == _crrs_Prepared && RunCnt >= 2) //准备复测
            {
                continue;
            }

            if (CICtl.RstReturn == _crst_Blocked) //如果最前排被挡死
            {
                RunCnt = 0;
                //往后走2格
                CICtl.RunPos = -2;
                CICtl.StopCtl = 0;
                xUserTaskCreate(ChainInCtrlTask, "ChainInCtrlTask", MotorCtl_STACK_SIZE, (void *)&CICtl, ChainCtl_Priority, NULL);
                while (1)
                {
                    UserTaskDelay(20);
                    if (ChainIn.Status == _chain_idle)
                    {
                        break;
                    }
                }
            }

            if (scanner_flg != 3)
            {
                if ((CICtl.RstReturn == _crst_SampleBlocked || CICtl.RstReturn == _crst_Sample))
                {

                    if (TrackNumber > 0)
                    {
                        for (tid = 0; tid < TrackNumber; tid++)
                        {
                            if (Track[tid].NormNextSID == 0 && (Track[tid].WaitSID == 0 || Track[tid].NormSID == 0))
                            {
                                scanner_flg = 3;
                                break;
                            }
                        }
                    }
                    else
                    {
                        scanner_flg = 3;
                    }

                    if (emg_flg == 20)
                    {
                        scanner_flg = 3;
                    }
                }
            }

            //检查是否找到样本
            if ((CICtl.RstReturn == _crst_SampleBlocked || CICtl.RstReturn == _crst_Sample) && (scanner_flg == 3))
            {

                //yfxiao 2021-04-21 增加新的需求，让扫码区一直处于空着的状态，除非上位机发送有急诊样本时候或者等待位有空闲
                //增加判断，轨道缓存区是否为空  检测缓冲器是否空闲，如果有空闲，跳出 Track

                //先获取推板信号量
                while (1)
                {
                    UserTaskDelay(100);
                    if (ChainIn.RunCtl != _cctl_run)
                    {
                        break;
                    }
                    if (xUserSemaphoreTake(Table.PBI.Mutex, 0) == pdTRUE)
                    {
                        break;
                    }
                }

                scanner_flg = 0;
                emg_flg = 0;
                waitscannerflg = 1;

                if (ChainIn.RunCtl != _cctl_run) //暂停
                {
                    continue;
                }
                if (Table_Sensor_Get(PBI_HM) == 0) //if (Table_Sensor_Get(PBI_HM) == 1)
                {
                    //再次回home
                    if (MotorCtrl((MotorStruDef *)&Table.PBI, _motor_home, 0))
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 20110);
                    }

                    while (Table.PBI.Status != _motor_idle)
                    {
                        UserTaskDelay(10);
                    }
                }

                waitscannerflg = 0;

                //建立样本
                //taskENTER_CRITICAL();
                xUserSemaphoreTake(framemutex, portMAX_DELAY);
                for (uint8_t i = 1; i < MaxFrameNumber; i++)
                {
                    if (Frame[i].Inuse == 0) //未使用,初始化样本架数据
                    {
                        Frame[i].Inuse = 1;
                        Frame[i].FramSID = i;
                        SID_Temp = i;
                        Frame[i].FrameDevice = 11;
                        Frame[i].ReceiveStatus = 0;
                        Frame[i].FrmMovStart = 1; //运行中
                        DeviceFrameCnt++;
                        break;
                    }
                }
                //taskEXIT_CRITICAL();
                xUserSemaphoreGive(framemutex);

#ifdef __DeviceVersion01

                //扫码
                if (ScannerF_Start_Scan() == F_ERROR)
                {
                    ScannerF_Stop_Scan();
                    Frame[SID_Temp].FrameID = 0xffffffff;
                    memset((void *)Frame[SID_Temp].FrameIDA, 0, 50);
                    //工作记录
                    WorkRecordReadWrite(0, 0, "WARN TBI Frame Barcode Scan Fail");
                }
                else //条码转换为样本架号
                {
                    Frame[SID_Temp].FrameID = 0;
                    memset((void *)Frame[SID_Temp].FrameIDA, 0, 50);
                    for (CNT_Temp = 0; CNT_Temp < Buffer_Length; CNT_Temp++)
                    {
                        if (ScannerF.RXD.Buffer[CNT_Temp] == 0x0d)
                        {
                            break;
                        }
                    }
                    for (uint8_t i = 0; i < CNT_Temp; i++)
                    {
                        Frame[SID_Temp].FrameID *= 10;
                        Frame[SID_Temp].FrameID += (ScannerF.RXD.Buffer[i] - 0x30);
                        Frame[SID_Temp].FrameIDA[i] = ScannerF.RXD.Buffer[i];
                    }
                    Frame[SID_Temp].FrameIDA[CNT_Temp] = 0x00; //字符串结尾
                    //工作记录
                    memset(LogBuffer, 0, sizeof(LogBuffer));
                    sprintf(LogBuffer, "INFO TBI Frame Barcode Scan Complete FrameID %ld", Frame[SID_Temp].FrameID);
                    WorkRecordReadWrite(0, 0, LogBuffer);
                }
#endif
                //再往前走一格
                CICtl.RunPos = 1;
                CICtl.StopCtl = 0;
                xUserTaskCreate(ChainInCtrlTask, "ChainInCtrlTask", MotorCtl_STACK_SIZE, (void *)&CICtl, ChainCtl_Priority, NULL);
                while (1)
                {
                    UserTaskDelay(20);
                    if (ChainIn.Status == _motor_idle)
                    {
                        RunCnt += CICtl.RunCnt;
                        break;
                    }
                }
                //检查样本是否在位,否则再走一格
                if (!Table_Sensor_Get(ChainIn_FS))
                {
                    //再往前走一格
                    CICtl.RunPos = 1;
                    CICtl.StopCtl = 0;
                    xUserTaskCreate(ChainInCtrlTask, "ChainInCtrlTask", MotorCtl_STACK_SIZE, (void *)&CICtl, ChainCtl_Priority, NULL);
                    while (1)
                    {
                        UserTaskDelay(20);
                        if (ChainIn.Status == _motor_idle)
                        {
                            RunCnt += CICtl.RunCnt;
                            break;
                        }
                    }
                    if (!Table_Sensor_Get(ChainIn_FS)) //样本丢失
                    {
                        if (DeviceFrameCnt > 0)
                        {
                            DeviceFrameCnt--;
                        }
                        FrameLostAlarm.alarm = 1;
                        FrameLostAlarm.info = Frame[SID_Temp].FrameID;
                        FrameLostAlarm.pos = 1;
                        //工作记录
                        memset(LogBuffer, 0, sizeof(LogBuffer));
                        sprintf(LogBuffer, "WARN TBI FrameID %ld Lost", Frame[SID_Temp].FrameID);
                        WorkRecordReadWrite(0, 0, LogBuffer);
                        //样本清空
                        Frame_Init(SID_Temp);
                        if (xUserSemaphoreGive(Table.PBI.Mutex) != pdTRUE)
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 25100);
                        }
                        continue;
                    }
                }

#ifdef __DeviceVersion01
                for (uint8_t cnti = 0; cnti < 11; cnti++)
                {
                    if (MotorCtrl((MotorStruDef *)&Table.PBI, _motor_run_pos, cnti + 1))
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 20110);
                    }
                    while (Table.PBI.Status != _motor_idle)
                    {
                        UserTaskDelay(10);
                    }
                    //UserTaskDelay(500);
                    //扫瓶类型
                    if (cnti < 10)
                    {
                        if (Table_Sensor_Get(Scan_LS)) //有效
                        {
                            if (Table_Sensor_Get(Scan_HS)) //高采血管
                            {
                                Frame[SID_Temp].Sample[cnti].Type = 2;
                            }
                            else
                            {
                                Frame[SID_Temp].Sample[cnti].Type = 1;
                            }
                        }
                        else
                        {
                            Frame[SID_Temp].Sample[cnti].Type = 0;
                        }
                    }

                    //扫码
                    if (((cnti != 0 && Frame[SID_Temp].Sample[cnti - 1].Type != 0) || (RACKRUNMODE == 0)) || (EthDbgMode == 1))
                    {
                        if (ScannerS_Start_Scan() == F_ERROR)
                        {
                            ScannerS_Stop_Scan();
                            //Frame[SID_Temp].Sample[cnti - 1].SampleID = 0;
                            memset((void *)Frame[SID_Temp].Sample[cnti - 1].SampleIDA, 0, 50);
                        }
                        else //条码转换为样本架号
                        {
                            //Frame[SID_Temp].Sample[cnti - 1].SampleID = 0;
                            memset((void *)Frame[SID_Temp].Sample[cnti - 1].SampleIDA, 0, 50);
                            for (CNT_Temp = 0; CNT_Temp < Buffer_Length; CNT_Temp++)
                            {
                                if (ScannerS.RXD.Buffer[CNT_Temp] == 0x0d)
                                {
                                    break;
                                }
                            }
                            for (uint8_t i = 0; i < CNT_Temp; i++)
                            {
                                //Frame[SID_Temp].Sample[cnti - 1].SampleID *= 10;
                                //Frame[SID_Temp].Sample[cnti - 1].SampleID += (ScannerS.RXD.Buffer[i] - 0x30);
                                Frame[SID_Temp].Sample[cnti - 1].SampleIDA[i] = ScannerS.RXD.Buffer[i];
                            }
                            Frame[SID_Temp].Sample[cnti - 1].SampleIDA[CNT_Temp] = 0x00; //字符串结尾
                        }
                    }
                }
#elif defined __DeviceVersion02
                //Leuze扫码器扫码程序，扫码电机闭环
                if (RACKRUNMODE == 0xff && EthDbgMode == 0)
                {
                    //获取上位机运行模式
                    EthSendGetEnableScanMode();
                }
                else if (EthDbgMode == 1)
                {
                    RACKRUNMODE = 0;
                }

                uint8_t timeout = 0;

                while (RACKRUNMODE == 0xFF && timeout < 100)
                {
                    timeout++;
                    UserTaskDelay(10);
                }

                if (timeout > 99)
                {
                    RACKRUNMODE = 0;
                }

                for (uint8_t cnti = 0; cnti < 11; cnti++)
                {

                    if (MotorCtrl((MotorStruDef *)&Table.PBI, _motor_run_pos, cnti + 1)) //
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 20110);
                    }

                    while (Table.PBI.Status != _motor_idle)
                    {
                        UserTaskDelay(10);
                    }

                    //扫码
                    if (cnti == 0) //扫架码
                    {

                        //扫码
                        if (ScannerCR100_Start_Scan() == F_ERROR) // 如果没有扫到，需要增加上报信息
                        {
                            NoBarcodeFrameID = U32T_LimitAdd(NoBarcodeFrameID, 1, 1000);
                            Frame[SID_Temp].FrameID = NoBarcodeFrameID + 9000; //临时的样本架ID 9xxx
                            memset((void *)Frame[SID_Temp].FrameIDA, 0, 50);
                            //工作记录
                            memset(LogBuffer, 0, sizeof(LogBuffer));
                            sprintf(LogBuffer, "WARN TBI Frame Barcode Scan Fail FrameID %ld", Frame[SID_Temp].FrameID);
                            WorkRecordReadWrite(0, 0, LogBuffer);
                            ///< 上传 info 信息
                            SendScanErrorCode();
                        }
                        else //条码转换为样本架号
                        {
                            Frame[SID_Temp].FrameID = 0;
                            memset((void *)Frame[SID_Temp].FrameIDA, 0, 50);
                            for (CNT_Temp = 0; CNT_Temp < Buffer_Length; CNT_Temp++)
                            {
                                if (ScannerCR100.RXD.Buffer[CNT_Temp] == 0x0d)
                                {
                                    break;
                                }
                            }
                            for (uint8_t i = 0; i < CNT_Temp - 1; i++)
                            {
                                Frame[SID_Temp].FrameID *= 10;
                                Frame[SID_Temp].FrameID += (ScannerCR100.RXD.Buffer[i + 1] - 0x30);
                                Frame[SID_Temp].FrameIDA[i] = ScannerCR100.RXD.Buffer[i + 1];
                            }
                            Frame[SID_Temp].FrameIDA[CNT_Temp] = 0x00; //字符串结尾
                            //工作记录
                            memset(LogBuffer, 0, sizeof(LogBuffer));
                            sprintf(LogBuffer, "INFO TBI Frame Barcode Scan Complete FrameID %ld", Frame[SID_Temp].FrameID);
                            WorkRecordReadWrite(0, 0, LogBuffer);
                        }
                    }
                    //扫瓶类型
                    else
                    {

                        if (Table_Sensor_Get(Scan_LS)) //有效
                        {
                            if (Table_Sensor_Get(Scan_HS)) //高采血管
                            {
                                if ((RACKRUNMODE == 0) || (EthDbgMode == 1))
                                {
                                    //扫瓶码
                                    if (ScannerCR100_Start_Scan() == F_ERROR) //yfxiao
                                    {
                                        memset((void *)Frame[SID_Temp].Sample[cnti - 1].SampleIDA, 0, 50);
                                    }
                                    else //条码转换为条码号
                                    {
                                        memset((void *)Frame[SID_Temp].Sample[cnti - 1].SampleIDA, 0, 50);
                                        for (CNT_Temp = 0; CNT_Temp < Buffer_Length; CNT_Temp++)
                                        {
                                            if (ScannerCR100.RXD.Buffer[CNT_Temp] == 0x0d)
                                            {
                                                break;
                                            }
                                        }
                                        for (uint8_t i = 0; i < CNT_Temp - 1; i++)
                                        {
                                            Frame[SID_Temp].Sample[cnti - 1].SampleIDA[i] = ScannerCR100.RXD.Buffer[i + 1];
                                        }
                                        Frame[SID_Temp].Sample[cnti - 1].SampleIDA[CNT_Temp] = 0x00; //字符串结尾
                                    }
                                }
                                else
                                {
                                    memset((void *)Frame[SID_Temp].Sample[cnti - 1].SampleIDA, 0, 50);
                                }

                                Frame[SID_Temp].Sample[cnti - 1].Type = 2;
                            }
                            else
                            {
                                Frame[SID_Temp].Sample[cnti - 1].Type = 1;
                                memset((void *)Frame[SID_Temp].Sample[cnti - 1].SampleIDA, 0, 50);
                            }
                        }
                        else
                        {
                            Frame[SID_Temp].Sample[cnti - 1].Type = 0;
                        }
                    }
                }
#endif
                Frame[SID_Temp].FrmMovStart = 0; //标记运行停止

                //工作记录
                memset(LogBuffer, 0, sizeof(LogBuffer));
                sprintf(LogBuffer, "INFO TBI FrameID %ld Sample Scan Complete", Frame[SID_Temp].FrameID);
                WorkRecordReadWrite(0, 0, LogBuffer);

                recodessid = SID_Temp;
                //直接移交
                if (xQueueSend(TransferRequest, (void *)&SID_Temp, 0) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25104);
                }
                //释放互斥量
                if (xUserSemaphoreGive(Table.PBI.Mutex) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }
            }
            //没有找到样本
            else if (CICtl.RstReturn == _crst_Idle)
            {
                if (DeviceFrameCnt == 0) //没有样本
                {
                    ChainIn.RunCtl = _cctl_userpause; //切换为普通暂停
                    //工作记录
                    memset(LogBuffer, 0, sizeof(LogBuffer));
                    sprintf(LogBuffer, "INFO CHI User Pause (Auto)");
                    WorkRecordReadWrite(0, 0, LogBuffer);
                }
                else
                {
                    ChainIn.RunCtl = _cctl_autopause; //自动暂停

                    //工作记录
                    memset(LogBuffer, 0, sizeof(LogBuffer));
                    sprintf(LogBuffer, "INFO CHI Auto Pause");
                    WorkRecordReadWrite(0, 0, LogBuffer);
                }
            }
        }
        else if (ChainIn.UserGoBack)
        {
            CICtl.RunPos = -5;
            CICtl.StopCtl = 0;
            xUserTaskCreate(ChainInCtrlTask, "ChainInCtrlTask", MotorCtl_STACK_SIZE, (void *)&CICtl, ChainCtl_Priority, NULL);
            while (1)
            {
                UserTaskDelay(20);
                if (ChainIn.Status == _chain_idle)
                {
                    ChainIn.UserGoBack = 0;
                    break;
                }
            }

            emg_flg = 20;
            //工作记录
            memset(LogBuffer, 0, sizeof(LogBuffer));
            sprintf(LogBuffer, "INFO CHI User Back");
            WorkRecordReadWrite(0, 0, LogBuffer);
        }
        UserTaskDelay(100);
    }
}
#endif

/**
 * @brief  样本台缓冲区部分控制线程
 * @param  None
 * @retval None
 */
void TableBackCtrlTask(void *pvParameters)
{
    (void)pvParameters;
    ChainCtlStruDef CICtl, CBCtl;
    uint8_t u8Temp = 0;
    uint32_t u32Temp = 0;
    uint8_t RecoveryStart = 0;
    uint8_t RecoveryCnt = 0;
    uint8_t EnterFlag = 0;
    uint32_t IdleCnt = 0;
    //    uint32_t timeoutcnt = 0;
    for (;;)
    {
        /*启动条件
        总样本数超过了20个,且第一个为回收
        或
        系统空闲超过一定时间,且第一个为回收
        或
        有样本需要复测，且之前的样本全部都为回收
        与
        样本回收仓不满
        */
        if (RecoveryStart == 0)
        {
            //刷新忙状态
            if (ChainBack.SampleBackWaiting == 0 && ChainBack.SampleInWaiting == 0 && Frame[ChainBack.FrameList[0]].ReStatus == 1)
            {
                if (IdleCnt++ > IdleCntAim)
                {
                    IdleCnt = IdleCntAim;
                }
            }
            else
            {
                IdleCnt = 0;
            }

            // 回收区满报警
            for (uint8_t i = 0; i < ChainBack.FrameCount; i++)
            {
                if (Table_Sensor_Get(Recy_FS) && ChainBack.RecoveryFullAlarm[i] == 0) //回收区满
                {
                    //发送告警信息
                    RecoveryFulltoStopAlarm.alarm = 1;
                    ChainBack.RecoveryFullAlarm[i] = 1;
                }
            }

            //判断是否启动
            //总样本数超过了20个,且第一个为回收
            if (ChainBack.FrameCount >= 20 && Frame[ChainBack.FrameList[0]].ReStatus == 1)
            {
                if (!Table_Sensor_Get(Recy_FS)) //回收区不满
                {
                    RecoveryStart = 1;
                    //工作记录
                    WorkRecordReadWrite(0, 0, "INFO TBB Start FrameCnt");
                }
            }
            //系统空闲超过一定时间,且第一个为回收
            else if (IdleCnt > IdleCntAim) //
            {
                if (!Table_Sensor_Get(Recy_FS)) //回收区不满
                {
                    RecoveryStart = 1;
                    //工作记录
                    WorkRecordReadWrite(0, 0, "INFO TBB Start IdleCnt");
                }
            }
            else
            {
                RecoveryCnt = 0;
                for (uint8_t i = 0; i < ChainBack.FrameCount; i++)
                {
                    if (Frame[ChainBack.FrameList[i]].ReStatus == 2) //复测
                    {
                        if (RecoveryCnt == i) //前面都是回收
                        {
                            if (Frame[ChainBack.FrameList[0]].ReStatus == 2) //第一个就是复测
                            {
                                if (ChainBack.ReRunStatus == _crrs_Idle)
                                {
                                    RecoveryStart = 1;
                                    //工作记录
                                    WorkRecordReadWrite(0, 0, "INFO TBB Start Rerun");
                                }
                            }
                            else //第一个是回收
                            {
                                if (!Table_Sensor_Get(Recy_FS)) //回收区不满
                                {
                                    RecoveryStart = 1;
                                    //工作记录
                                    WorkRecordReadWrite(0, 0, "INFO TBB Start ReadyRerun");
                                }
                            }
                            break;
                        }
                    }
                    else if (Frame[ChainBack.FrameList[i]].ReStatus == 1) //回收
                    {
                        RecoveryCnt++;
                        // if(RecoveryCnt >= SampleRecoveryCntAim) //到达样本数
                        // {
                        //     if(ChainBack.SampleBackWaiting == 0 && ChainBack.SampleInWaiting == 0)//系统不繁忙
                        //     {
                        //         RecoveryStart = 1;
                        //         break;
                        //     }

                        // }
                    }
                    else
                    {
                        RecoveryCnt = 0;
                        RecoveryStart = 0;
                        break;
                    }
                }
            }
        }

        if (RecoveryStart == 1)
        {
            //启动后直接将返回链条走到最后
            if (EnterFlag == 0) //首次进入
            {
                //获取推板电机、回收仓电机信号量
                if (xUserSemaphoreTake(Table.PBB.Mutex, portMAX_DELAY) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }

                if (xUserSemaphoreTake(Table.PPB.Mutex, portMAX_DELAY) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }

                //获取返回链条的互斥量
                if (xUserSemaphoreTake(ChainBack.Mutex, portMAX_DELAY) != pdTRUE)
                {
                    //错误处理
                    User_Error_Handler(__FILE__, __LINE__, 25100);
                }

                EnterFlag = 1;
                WorkRecordReadWrite(0, 0, "INFO CHB PBB PPB SEM TAKE");

                //确认推板电机、回收仓电机位置
                if (Table.PBB.CurPos != 0)
                {
                    if (MotorCtrl((MotorStruDef *)&Table.PBB, _motor_run_pos, 0))
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 20110);
                    }
                    while (Table.PBB.Status != _motor_idle)
                    {
                        UserTaskDelay(10);
                    }
                }

#ifdef __DeviceVersion01
                u32Temp = 25;
#else
                u32Temp = 24;
#endif
                //计数走到最后
                CBCtl.RunPos = -(u32Temp - ChainBack.FrameCount);
                CBCtl.StopCtl = 0;
                CBCtl.Align = 2; // 2021-02-26 增加
                xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&CBCtl, ChainCtl_Priority, NULL);
                while (1)
                {
                    UserTaskDelay(20);
                    if (ChainBack.Status == _motor_idle)
                    {
                        break;
                    }
                }
                if (CBCtl.RstReturn == _crst_Blocked) //出现了未知的样本
                {

                    //未知样本告警
                    UnknownFrameAlarm.alarm = 1;
                    UnknownFrameAlarm.pos = 3;

                    //记录已经运行的格数
                    u32Temp -= CBCtl.RunCnt;
                    u32Temp--;
                    //向前走一格
                    u32Temp++;
                    CBCtl.RunPos = 1;
                    CBCtl.StopCtl = 0;
                    CBCtl.Align = 2; // 2021-02-26 增加
                    xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&CBCtl, ChainCtl_Priority, NULL);
                    while (1)
                    {
                        UserTaskDelay(20);
                        if (ChainBack.Status == _motor_idle)
                        {
                            break;
                        }
                    }

                    //向后走一格，看看有没有样本
                    u32Temp--;
                    CBCtl.RunPos = -1;
                    CBCtl.StopCtl = 0;
                    CBCtl.Align = 2; // 2021-02-26 增加
                    xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&CBCtl, ChainCtl_Priority, NULL);
                    while (1)
                    {
                        UserTaskDelay(20);
                        if (ChainBack.Status == _motor_idle)
                        {
                            break;
                        }
                    }
                    if (!Table_Sensor_Get(ChainBack_RS))
                    {
                        //向后走一格，看看有没有样本
                        u32Temp--;
                        CBCtl.RunPos = -1;
                        CBCtl.StopCtl = 0;
                        CBCtl.Align = 2; // 2021-02-26 增加
                        xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&CBCtl, ChainCtl_Priority, NULL);
                        while (1)
                        {
                            UserTaskDelay(20);
                            if (ChainBack.Status == _motor_idle)
                            {
                                break;
                            }
                        }
                    }
                    if (Table_Sensor_Get(ChainBack_RS))
                    {
                        //回收该样本
                        //确认回收仓电机位置
                        // if (Table.PPB.CurPos != 0)
                        {
                            if (MotorCtrl((MotorStruDef *)&Table.PPB, _motor_home, 0))
                            {
                                //错误处理
                                User_Error_Handler(__FILE__, __LINE__, 20110);
                            }
                            while (Table.PPB.Status != _motor_idle)
                            {
                                UserTaskDelay(10);
                            }
                        }
                        UserTaskDelay(50);
                        //推板电机推到回收位
                        if (MotorCtrl((MotorStruDef *)&Table.PBB, _motor_run_pos, 1))
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 20110);
                        }
                        while (Table.PBB.Status != _motor_idle)
                        {
                            UserTaskDelay(10);
                        }
                        //推板电机复位
                        if (MotorCtrl((MotorStruDef *)&Table.PBB, _motor_run_pos, 0))
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 20110);
                        }
                        //回收仓电机推送
                        if (MotorCtrl((MotorStruDef *)&Table.PPB, _motor_run_pos, 1))
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 20110);
                        }
                        //检查运行状态
                        u8Temp = 0x03;
                        while (u8Temp != 0)
                        {
                            UserTaskDelay(50);
                            if (UserReadBit(u8Temp, 0))
                            {
                                if (Table.PBB.Status == _motor_idle)
                                {
                                    UserClrBit(u8Temp, 0);
                                }
                            }
                            if (UserReadBit(u8Temp, 1))
                            {
                                if (Table.PPB.Status == _motor_idle)
                                {
                                    UserClrBit(u8Temp, 1);
                                }
                            }
                        }
                    }
                    else
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 20112);
                    }
                    //回原位
#ifdef __DeviceVersion01
                    CBCtl.RunPos = 25 - u32Temp;
#else
                    CBCtl.RunPos = 24 - u32Temp;
#endif

                    CBCtl.StopCtl = 0;
                    CBCtl.Align = 0;
                    xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&CBCtl, ChainCtl_Priority, NULL);
                    while (1)
                    {
                        UserTaskDelay(20);
                        if (ChainBack.Status == _motor_idle)
                        {
                            break;
                        }
                    }
                    //检查一下样本状态
                    if (Table_Sensor_Get(ChainBack_SFS) || !Table_Sensor_Get(ChainBack_SRS))
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 20112);
                    }
                    EnterFlag = 0;
                    //释放推板电机、回收仓电机互斥量
                    if (xUserSemaphoreGive(Table.PBB.Mutex) != pdTRUE)
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 25100);
                    }
                    if (xUserSemaphoreGive(Table.PPB.Mutex) != pdTRUE)
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 25100);
                    }
                    //释放链条互斥量
                    if (xUserSemaphoreGive(ChainBack.Mutex) != pdTRUE)
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 25100);
                    }

                    WorkRecordReadWrite(0, 0, "INFO CHB PBB PPB SEM GIVE");
                }
            }
            else if (Frame[ChainBack.FrameList[0]].ReStatus == 1 && !Table_Sensor_Get(Recy_FS) && ChainBack.ReRunStatus == _crrs_Idle) //回收且回收区未满
            {
                //工作记录
                memset(LogBuffer, 0, sizeof(LogBuffer));
                sprintf(LogBuffer, "INFO TBB FrameID %ld To Recycle", Frame[ChainBack.FrameList[0]].FrameID);
                WorkRecordReadWrite(0, 0, LogBuffer);
                //检查是否有加样线程需要插队？
                if (ChainBack.SampleInWaiting == 1)
                {
                    //置位等待，避免样本回收线程提前拿走互斥量造成错误
                    ChainBack.SampleRecWaiting = 1;
                    //释放链条互斥量
                    if (xUserSemaphoreGive(ChainBack.Mutex) != pdTRUE)
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 25100);
                    }

                    WorkRecordReadWrite(0, 0, "INFO CHB SEM GIVE");

                    //等待进样完成
                    while (ChainBack.SampleInWaiting == 1)
                    {
                        UserTaskDelay(100);
                    }

                    //获取返回链条的互斥量
                    if (xUserSemaphoreTake(ChainBack.Mutex, portMAX_DELAY) != pdTRUE)
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 25100);
                    }
                    WorkRecordReadWrite(0, 0, "INFO CHB SEM TAKE");

                    //置位复位
                    ChainBack.SampleRecWaiting = 0;
                    //往后走1格
                    CBCtl.RunPos = -1;
                    CBCtl.StopCtl = 0;
                    CBCtl.Align = 2; // 2021-02-26 增加
                    xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&CBCtl, ChainCtl_Priority, NULL);
                    while (1)
                    {
                        UserTaskDelay(20);
                        if (ChainBack.Status == _motor_idle)
                        {
                            break;
                        }
                    }
                }
                //往后走1格
                CBCtl.RunPos = -1;
                CBCtl.StopCtl = 0;
                CBCtl.Align = 2; // 2021-02-26 增加
                xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&CBCtl, ChainCtl_Priority, NULL);
                while (1)
                {
                    UserTaskDelay(20);
                    if (ChainBack.Status == _motor_idle)
                    {
                        break;
                    }
                }

                //检查是否有样本
                if (!Table_Sensor_Get(ChainBack_RS))
                {
                    // if(DeviceFrameCnt > 0)
                    // {
                    //     DeviceFrameCnt--;
                    // }
                    //样本丢失告警
                    FrameLostAlarm.alarm = 1;
                    FrameLostAlarm.info = Frame[ChainBack.FrameList[0]].FrameID;
                    FrameLostAlarm.pos = 2;
                    //工作记录
                    memset(LogBuffer, 0, sizeof(LogBuffer));
                    sprintf(LogBuffer, "WARN TBB FrameID %ld Lost", Frame[ChainBack.FrameList[0]].FrameID);
                    WorkRecordReadWrite(0, 0, LogBuffer);
                }
                else
                {
                    //确认回收仓电机位置
                    // if (Table.PPB.CurPos != 0)
                    {
                        if (MotorCtrl((MotorStruDef *)&Table.PPB, _motor_home, 0))
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 20110);
                        }
                        while (Table.PPB.Status != _motor_idle)
                        {
                            UserTaskDelay(10);
                        }
                    }
                    //推板电机推到回收位
                    if (MotorCtrl((MotorStruDef *)&Table.PBB, _motor_run_pos, 1))
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 20110);
                    }
                    while (Table.PBB.Status != _motor_idle)
                    {
                        UserTaskDelay(10);
                    }
                    //推板电机复位
                    if (MotorCtrl((MotorStruDef *)&Table.PBB, _motor_run_pos, 0))
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 20110);
                    }
                    //回收仓电机推送
                    if (MotorCtrl((MotorStruDef *)&Table.PPB, _motor_run_pos, 1))
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 20110);
                    }
                    //检查运行状态
                    u8Temp = 0x03;
                    while (u8Temp != 0)
                    {
                        UserTaskDelay(50);
                        if (UserReadBit(u8Temp, 0))
                        {
                            if (Table.PBB.Status == _motor_idle)
                            {
                                UserClrBit(u8Temp, 0);
                            }
                        }
                        if (UserReadBit(u8Temp, 1))
                        {
                            if (Table.PPB.Status == _motor_idle)
                            {
                                UserClrBit(u8Temp, 1);
                                // UserSetBit(u8Temp, 2);
                                // //回收仓电机复位
                                // if (MotorCtrl((MotorStruDef *)&Table.PPB, _motor_home, 0))
                                // {
                                //     //错误处理
                                //     User_Error_Handler(__FILE__, __LINE__, 20110);
                                // }
                            }
                        }
                        // if (UserReadBit(u8Temp, 2))
                        // {
                        //     if (Table.PPB.Status == _motor_idle)
                        //     {
                        //         UserClrBit(u8Temp, 2);
                        //     }
                        // }
                    }
                }
                //工作记录
                memset(LogBuffer, 0, sizeof(LogBuffer));
                sprintf(LogBuffer, "INFO TBB FrameID %ld To Recycle Complete", Frame[ChainBack.FrameList[0]].FrameID);
                WorkRecordReadWrite(0, 0, LogBuffer);
                taskENTER_CRITICAL();
                // xUserSemaphoreTake(framemutex, portMAX_DELAY);
                //样本信息清空
                Frame_Init(ChainBack.FrameList[0]);
                //样本数记录减少
                if (DeviceFrameCnt > 0)
                {
                    DeviceFrameCnt--;
                }
                //回收链条样本信息编辑
                ChainBack.FrameCount--;
                for (uint8_t i = 0; i < ChainBack.FrameCount; i++)
                {
                    ChainBack.FrameList[i] = ChainBack.FrameList[i + 1];
                }
                ChainBack.FrameList[ChainBack.FrameCount] = 0;
                taskEXIT_CRITICAL();
                // xUserSemaphoreGive(framemutex);
            }
            else if (Frame[ChainBack.FrameList[0]].ReStatus == 2 && ChainBack.ReRunStatus == _crrs_Idle) //复测
            {
                //工作记录
                memset(LogBuffer, 0, sizeof(LogBuffer));
                sprintf(LogBuffer, "INFO TBB FrameID %ld To Rerun", Frame[ChainBack.FrameList[0]].FrameID);
                WorkRecordReadWrite(0, 0, LogBuffer);
                ChainBack.ReRunStatus = _crrs_Prepared;

                //返回链条往后走1格
                CBCtl.RunPos = -1;
                CBCtl.StopCtl = 0;
                CBCtl.Align = 2; // 2021-02-26 增加
                xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&CBCtl, ChainCtl_Priority, NULL);
                while (1)
                {
                    UserTaskDelay(20);
                    if (ChainBack.Status == _motor_idle && ChainIn.Status == _motor_idle)
                    {
                        break;
                    }
                }

                //检查是否有样本
                if (!Table_Sensor_Get(ChainBack_RS))
                {
                    //样本丢失告警
                    FrameLostAlarm.alarm = 1;
                    FrameLostAlarm.info = Frame[ChainBack.FrameList[0]].FrameID;
                    FrameLostAlarm.pos = 3;
                    //工作记录
                    memset(LogBuffer, 0, sizeof(LogBuffer));
                    sprintf(LogBuffer, "WARN TBB FrameID %ld Lost", Frame[ChainBack.FrameList[0]].FrameID);
                    WorkRecordReadWrite(0, 0, LogBuffer);
                    taskENTER_CRITICAL();
                    // xUserSemaphoreTake(framemutex, portMAX_DELAY);
                    //样本信息清空
                    Frame_Init(ChainBack.FrameList[0]);
                    //样本数记录减少
                    if (DeviceFrameCnt > 0)
                    {
                        DeviceFrameCnt--;
                    }
                    //回收链条样本信息编辑
                    ChainBack.FrameCount--;
                    for (uint8_t i = 0; i < ChainBack.FrameCount; i++)
                    {
                        ChainBack.FrameList[i] = ChainBack.FrameList[i + 1];
                    }
                    ChainBack.FrameList[ChainBack.FrameCount] = 0;
                    taskEXIT_CRITICAL();
                    // xUserSemaphoreGive(framemutex);
                    ChainBack.ReRunStatus = _crrs_Idle;
                }
                else
                {

                    //确认回收仓电机位置
                    // if (Table.PPB.CurPos != 0)
                    {
                        if (MotorCtrl((MotorStruDef *)&Table.PPB, _motor_home, 0))
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 20110);
                        }
                        while (Table.PPB.Status != _motor_idle)
                        {
                            UserTaskDelay(10);
                        }
                    }
                    //推板运行到中间位
                    if (MotorCtrl((MotorStruDef *)&Table.PBB, _motor_run_pos, 1))
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 20110);
                    }
                    while (Table.PBB.Status != _motor_idle)
                    {
                        UserTaskDelay(10);
                    }
                    //检测这时候进样链条是否已经到位
                    if (ChainBack.ReRunStatus != _crrs_CIReady) //还未到位
                    {
                        //推板电机推到中间等待位
                        if (MotorCtrl((MotorStruDef *)&Table.PBB, _motor_run_pos, 1))
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 20110);
                        }
                        while (Table.PBB.Status != _motor_idle)
                        {
                            UserTaskDelay(10);
                        }
                        //样本信息清空
                        taskENTER_CRITICAL();
                        // xUserSemaphoreTake(framemutex, portMAX_DELAY);
                        ChainBack.ReRunWaitingID = ChainBack.FrameList[0];
                        // Frame_Init(ChainBack.FrameList[0]);
                        //回收链条样本信息编辑
                        ChainBack.FrameCount--;
                        for (uint8_t i = 0; i < ChainBack.FrameCount; i++)
                        {
                            ChainBack.FrameList[i] = ChainBack.FrameList[i + 1];
                        }
                        ChainBack.FrameList[ChainBack.FrameCount] = 0;
                        taskEXIT_CRITICAL();
                        // xUserSemaphoreGive(framemutex);
                        //建立新的等待线程
                        xUserTaskCreate(ChainRerunWatingTask, "ChainRerunWatingTask", configMINIMAL_STACK_SIZE, NULL, SampleCtl_Priority, NULL);
                    }
                    else
                    {
                        //推板电机推到复测位
                        if (MotorCtrl((MotorStruDef *)&Table.PBB, _motor_run_pos, 2))
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 20110);
                        }
                        while (Table.PBB.Status != _motor_idle)
                        {
                            UserTaskDelay(10);
                        }
                        ChainBack.ReRunStatus = _crrs_PPCplt;
                        //推板电机复位
                        if (MotorCtrl((MotorStruDef *)&Table.PBB, _motor_run_pos, 0))
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 20110);
                        }
                        //确认回收仓电机阻挡
                        if (MotorCtrl((MotorStruDef *)&Table.PPB, _motor_run_pos, 1))
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 20110);
                        }
                        while (Table.PBB.Status != _motor_idle || Table.PPB.Status != _motor_idle)
                        {
                            UserTaskDelay(10);
                        }
                        //工作记录
                        memset(LogBuffer, 0, sizeof(LogBuffer));
                        sprintf(LogBuffer, "INFO TBB FrameID %ld To Rerun Complete", Frame[ChainBack.FrameList[0]].FrameID);
                        WorkRecordReadWrite(0, 0, LogBuffer);
                        taskENTER_CRITICAL();
                        // xUserSemaphoreTake(framemutex, portMAX_DELAY);
                        //样本信息清空
                        Frame_Init(ChainBack.FrameList[0]);
                        //样本数记录减少
                        if (DeviceFrameCnt > 0)
                        {
                            DeviceFrameCnt--;
                        }
                        //回收链条样本信息编辑
                        ChainBack.FrameCount--;
                        for (uint8_t i = 0; i < ChainBack.FrameCount; i++)
                        {
                            ChainBack.FrameList[i] = ChainBack.FrameList[i + 1];
                        }
                        ChainBack.FrameList[ChainBack.FrameCount] = 0;
                        taskEXIT_CRITICAL();
                        // xUserSemaphoreGive(framemutex);
                    }
                }
            }
            else //回原位
            {
                if (EnterFlag == 1) //需要返回
                {
                    EnterFlag = 0;
                    //释放推板电机、回收仓电机互斥量
                    if (xUserSemaphoreGive(Table.PBB.Mutex) != pdTRUE)
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 25100);
                    }
                    if (xUserSemaphoreGive(Table.PPB.Mutex) != pdTRUE)
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 25100);
                    }

                    WorkRecordReadWrite(0, 0, "INFO PBB PPB SEM GIVE");

                    if (ChainBack.FrameCount != 0) //返回链条上还有样本的情况
                    {
#ifdef __DeviceVersion01
                        CBCtl.RunPos = 25 - ChainBack.FrameCount;
#else
                        CBCtl.RunPos = 24 - ChainBack.FrameCount;
#endif

                        CBCtl.StopCtl = 0;
                        CBCtl.Align = 0;
                        xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&CBCtl, ChainCtl_Priority, NULL);
                        while (1)
                        {
                            UserTaskDelay(20);
                            if (ChainBack.Status == _motor_idle)
                            {
                                break;
                            }
                        }
                        //检查一下样本状态
                        if (Table_Sensor_Get(ChainBack_SFS) || !Table_Sensor_Get(ChainBack_SRS))
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 20112);
                        }
                    }
                    else
                    {
                        // yfxiao add go home
                        while (ChainBack.Status != _chain_idle)
                        {
                            UserTaskDelay(50);
                        }
                        if (MotorCtrl((MotorStruDef *)&Table.CHBR, _motor_rls, 0))
                        {
                        }
                        while (1)
                        {
                            UserTaskDelay(50);
                            if (Table.CHBR.Status == _motor_rlsd)
                            {
                                break;
                            }
                        }

                        if (MotorCtrl((MotorStruDef *)&Table.CHBF, _motor_home, 0))
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 26100);
                        }
                        while (1)
                        {
                            UserTaskDelay(50);
                            if (Table.CHBF.Status == _motor_idle)
                            {
                                break;
                            }
                        }
                    }
                    //释放链条互斥量
                    if (xUserSemaphoreGive(ChainBack.Mutex) != pdTRUE)
                    {
                        //错误处理
                        User_Error_Handler(__FILE__, __LINE__, 25100);
                    }
                    WorkRecordReadWrite(0, 0, "INFO CHB SEM GIVE");

                    RecoveryStart = 0;
                }
            }
        }
        UserTaskDelay(100);
    }
}

/**
 * @brief  样本台复测时，等待控制线程
 * @param  None
 * @retval None
 */
static void ChainRerunWatingTask(void *pvParameters)
{
    (void)pvParameters;
    //获取推板电机、回收仓电机信号量
    if (xUserSemaphoreTake(Table.PBB.Mutex, portMAX_DELAY) != pdTRUE)
    {
        //错误处理
        User_Error_Handler(__FILE__, __LINE__, 25100);
    }
    if (xUserSemaphoreTake(Table.PPB.Mutex, portMAX_DELAY) != pdTRUE)
    {
        //错误处理
        User_Error_Handler(__FILE__, __LINE__, 25100);
    }

    WorkRecordReadWrite(0, 0, "INFO PBB PPB SEM TAKE");

    while (ChainBack.ReRunStatus != _crrs_CIReady)
    {
        UserTaskDelay(100);
    }
    //推板电机推到复测位置
    if (MotorCtrl((MotorStruDef *)&Table.PBB, _motor_run_pos, 2))
    {
        //错误处理
        User_Error_Handler(__FILE__, __LINE__, 20110);
    }
    while (Table.PBB.Status != _motor_idle)
    {
        UserTaskDelay(10);
    }
    ChainBack.ReRunStatus = _crrs_PPCplt;
    //工作记录
    memset(LogBuffer, 0, sizeof(LogBuffer));
    sprintf(LogBuffer, "INFO TBB FrameID %ld To Rerun Complete", Frame[ChainBack.ReRunWaitingID].FrameID);
    WorkRecordReadWrite(0, 0, LogBuffer);
    //清空样本信息
    taskENTER_CRITICAL();
    // xUserSemaphoreTake(framemutex, portMAX_DELAY);
    Frame_Init(ChainBack.ReRunWaitingID);
    //样本数记录减少
    if (DeviceFrameCnt > 0)
    {
        DeviceFrameCnt--;
    }
    ChainBack.ReRunWaitingID = 0;
    taskEXIT_CRITICAL();
    // xUserSemaphoreGive(framemutex);
    //推板电机复位
    if (MotorCtrl((MotorStruDef *)&Table.PBB, _motor_run_pos, 0))
    {
        //错误处理
        User_Error_Handler(__FILE__, __LINE__, 20110);
    }
    //确认回收仓电机阻挡
    if (MotorCtrl((MotorStruDef *)&Table.PPB, _motor_run_pos, 1))
    {
        //错误处理
        User_Error_Handler(__FILE__, __LINE__, 20110);
    }
    while (Table.PBB.Status != _motor_idle || Table.PPB.Status != _motor_idle)
    {
        UserTaskDelay(10);
    }
    //释放推板电机、回收仓电机互斥量
    if (xUserSemaphoreGive(Table.PBB.Mutex) != pdTRUE)
    {
        //错误处理
        User_Error_Handler(__FILE__, __LINE__, 25100);
    }
    if (xUserSemaphoreGive(Table.PPB.Mutex) != pdTRUE)
    {
        //错误处理
        User_Error_Handler(__FILE__, __LINE__, 25100);
    }
    WorkRecordReadWrite(0, 0, "INFO PBB PPB SEM GIVE");

    for (;;)
    {
        vTaskDelete(NULL);
    }
}

/**
 * @brief  调试控制线程电机转换
 * @param  device设备号，motor电机号,MotorStruDef电机结构体
 * @retval 0-成功 1-失败
 */
uint8_t Debug_Motor_Conv(uint8_t device, uint8_t motor, MotorStruDef **MotorSt)
{
    if (device == 0) //样本台
    {
        switch (motor)
        {
        case 3:
            *MotorSt = (MotorStruDef *)&Table.PBI;
            break;
        case 4:
            *MotorSt = (MotorStruDef *)&Table.PPI;
            break;
        case 5:
            *MotorSt = (MotorStruDef *)&Table.CHBF;
            break;
        case 7:
            *MotorSt = (MotorStruDef *)&Table.PBB;
            break;
        case 8:
            *MotorSt = (MotorStruDef *)&Table.PPB;
            break;
        default:
            return 1;
        }
    }
    else if (device <= 4) //轨道
    {
        switch (motor)
        {
        case 1:
            *MotorSt = (MotorStruDef *)&Track[device - 1].WBF;
            break;
        case 2:
            *MotorSt = (MotorStruDef *)&Track[device - 1].BPB;
            break;
        case 3:
            *MotorSt = (MotorStruDef *)&Track[device - 1].NBL;
            break;
        case 4:
            *MotorSt = (MotorStruDef *)&Track[device - 1].EBL;
            break;
        case 5:
            *MotorSt = (MotorStruDef *)&Track[device - 1].MTC;
            break;
        case 6:
            *MotorSt = (MotorStruDef *)&Track[device - 1].NPB;
            break;
        case 7:
            *MotorSt = (MotorStruDef *)&Track[device - 1].EPB;
            break;
        case 8:
            *MotorSt = (MotorStruDef *)&Track[device - 1].BBL;
            break;
        case 9:
            *MotorSt = (MotorStruDef *)&Track[device - 1].NBS;
            break;
        case 10:
            *MotorSt = (MotorStruDef *)&Track[device - 1].EBS;
            break;
        case 11:
            *MotorSt = (MotorStruDef *)&Track[device - 1].BBS;
            break;
        case 12:
            *MotorSt = (MotorStruDef *)&Track[device - 1].MBF;
            break;
        default:
            return 1;
        }
    }
    else
    {
        return 1;
    }
    return 0;
}

uint8_t debug_cmd[200];
uint8_t debug_mode = 0;
/**
 * @brief  调试控制线程
 * @param  None
 * @retval None
 */
void DebugCtlTask(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    (void)pvParameters;
    FRESULT f_rst;
    uint8_t tempbuff2[200];
    TimeStructDef DebugTime;
    MotorStruDef **MotorStruTemp = NULL;
    MotorStruDef *MotorStruDebug = NULL;
    ChainCtlStruDef DebugTemp;
    uint32_t u32temp = 0;
    uint8_t device = 0;
    uint8_t motor = 0;
    int_fast32_t pos = 0;
    uint32_t pos_id = 0;
    uint8_t alig = 0;
    uint8_t logbuf[125];
    char *tmp = NULL;
    memset(debug_cmd, 0, sizeof(debug_cmd));
    printf("Debug Mode:\n");
    for (;;)
    {
        printf("CMD:DT Data&Time\n");
        printf("CMD:AMPW AllMotorParamWrite\n");
        printf("CMD:M(device) (motorid) MotorPosSet\n");
        printf("CMD:C(chainid) ChainPosSet\n");
        printf("CMD:T(tracknum) TrackNumberSet\n");
        printf("CMD:S(device) SensorGet\n");
        printf("CMD:SAVE ParamSave\n");
        printf("CMD:EXIT ExitDebugMode\n");

        while (debug_cmd[0] == 0) //等待输入
        {
            vTaskDelay(100);
        }
        if (memcmp(debug_cmd, "SAVE", 4) == 0) //电机参数保存到flash
        {
            printf("Confirm:");
            memset(debug_cmd, 0, sizeof(debug_cmd));
            while (debug_cmd[0] == 0) //等待输入
            {
                vTaskDelay(100);
            }
            if (debug_cmd[0] == 0x0d)
            {
                memset(debug_cmd, 0, sizeof(debug_cmd));
                //保存参数
                if (UserParamInfoRW(0))
                {
                    printf("ERROR\n");
                }
                else
                {
                    printf("OK\n");
                }
            }
        }
        else if (memcmp(debug_cmd, "LOGADD", 6) == 0)
        {
            u32temp = atol((char *)&debug_cmd[6]);
            memset(debug_cmd, 0, sizeof(debug_cmd));
            for (uint8_t i = 0; i < u32temp; i++)
            {
                memset(logbuf, 0, sizeof(logbuf));
                sprintf((char *)logbuf, "TEST LOG No%u All%u", i, u32temp);
                WorkRecordReadWrite(0, 0, (char *)logbuf);
                vTaskDelay(10);
            }
        }
        else if (memcmp(debug_cmd, "LOGRAM", 6) == 0) // SDRAMlog
        {
            memset(debug_cmd, 0, sizeof(debug_cmd));
            for (;;)
            {
                printf("Input:");
                while (debug_cmd[0] == 0) //等待输入
                {
                    vTaskDelay(100);
                }
                if (memcmp(debug_cmd, "EXIT", 4) == 0) //退出
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    break;
                }
                else if (memcmp(debug_cmd, "R", 1) == 0) //读取
                {
                    memset(logbuf, 0, sizeof(logbuf));
                    u32temp = atol((char *)&debug_cmd[1]);
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    if (u32temp >= 10000)
                    {
                        printf("ERROR!\n");
                    }
                    else
                    {
                        WorkRecordReadWrite(1, u32temp, (char *)logbuf);
                        printf("Idx:%u:%s\n", u32temp, logbuf);
                    }
                }
                else
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                }
            }
        }
        else if (memcmp(debug_cmd, "LOGSD", 5) == 0) //存储卡log
        {
            memset(debug_cmd, 0, sizeof(debug_cmd));
            for (;;)
            {
                printf("Input:");
                while (debug_cmd[0] == 0) //等待输入
                {
                    vTaskDelay(100);
                }
                if (memcmp(debug_cmd, "EXIT", 4) == 0) //退出
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    break;
                }
                else if (memcmp(debug_cmd, "IDX", 3) == 0) //读取索引文件
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    f_rst = f_open(&fil, "0:index.txt", FA_READ | FA_OPEN_EXISTING);
                    while (1)
                    {
                        memset(tempbuff2, 0, sizeof(tempbuff2));
                        if (f_gets((char *)tempbuff2, 200, &fil) == NULL)
                        {
                            break;
                        }
                        printf("%s", tempbuff2);
                    }
                    f_rst = f_close(&fil);
                }
                else if (memcmp(debug_cmd, "OPEN", 4) == 0) //打开文件
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    printf("Input Date(YYYYMMDD):");
                    while (debug_cmd[0] == 0) //等待输入
                    {
                        vTaskDelay(100);
                    }
                    if (memcmp(debug_cmd, "EXIT", 4) == 0) //退出
                    {
                        memset(debug_cmd, 0, sizeof(debug_cmd));
                        continue;
                    }
                    memset(tempbuff2, 0, sizeof(tempbuff2));
                    SDPrintFileList(debug_cmd);
                    tmp = strstr((char *)debug_cmd, "\r");
                    if (tmp != NULL)
                    {
                        *tmp = '\0';
                    }
                    sprintf((char *)tempbuff2, "0:%s_", debug_cmd);

                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    printf("Input Time(HHMMSS):");
                    while (debug_cmd[0] == 0) //等待输入
                    {
                        vTaskDelay(100);
                    }
                    if (memcmp(debug_cmd, "EXIT", 4) == 0) //退出
                    {
                        memset(debug_cmd, 0, sizeof(debug_cmd));
                        continue;
                    }
                    tmp = strstr((char *)debug_cmd, "\r");
                    if (tmp != NULL)
                    {
                        *tmp = '\0';
                    }
                    strcat((char *)tempbuff2, (char *)debug_cmd);
                    strcat((char *)tempbuff2, ".txt");
                    // printf("%s\n",tempbuff2);
                    f_rst = SDPrintFile(tempbuff2);
                    if (f_rst == FR_OK)
                    {
                        printf("OK\n");
                    }
                    else
                    {
                        printf("ERROR\n");
                    }
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                }
                else if (memcmp(debug_cmd, "DEL", 3) == 0) //删除全部日志
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    printf("Confirm:");
                    while (debug_cmd[0] == 0) //等待输入
                    {
                        vTaskDelay(100);
                    }
                    if (debug_cmd[0] == 0x0d)
                    {
                        if (SDDeleteAllFiles() == FR_OK)
                        {

                            printf("OK\n");
                        }
                        else
                        {
                            printf("ERROR\n");
                        }
                    }
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                }
                else
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                }
            }
        }
        else if (memcmp(debug_cmd, "DT", 2) == 0) //时间读取/设置
        {
            memset(debug_cmd, 0, sizeof(debug_cmd));
            printf("Date(YYYY-MM-DD):%04ld-%02d-%02d\n", SysTime.year + 2000, SysTime.month, SysTime.day);
            printf("Time(HH:MM:SS):%02d:%02d:%02d\n", SysTime.hour, SysTime.minute, SysTime.second);
            for (;;)
            {
                printf("Input:");
                while (debug_cmd[0] == 0) //等待输入
                {
                    vTaskDelay(100);
                }
                if (memcmp(debug_cmd, "EXIT", 4) == 0) //退出
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    break;
                }
                else if (memcmp(debug_cmd, "READ", 4) == 0) //从RTC重新读取系统时间
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    if (RTC_Time_Read())
                    {
                        printf("ERROR!\n");
                    }
                    else
                    {
                        printf("OK!\n");
                    }
                }
                else if (memcmp(debug_cmd, "SAVE", 4) == 0) //保存时间到RTC
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    RTC_Time_Write();
                    printf("OK!\n");
                }
                else if (memcmp(debug_cmd, "GET", 3) == 0) //打印日期时间
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    printf("Date(YYYY-MM-DD):%04ld-%02d-%02d\n", SysTime.year + 2000, SysTime.month, SysTime.day);
                    printf("Time(HH:MM:SS):%02d:%02d:%02d\n", SysTime.hour, SysTime.minute, SysTime.second);
                }
                else if (memcmp(debug_cmd, "SET", 3) == 0) //设置日期时间
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    printf("Year:");
                    while (debug_cmd[0] == 0) //等待输入
                    {
                        vTaskDelay(100);
                    }
                    u32temp = atol((char *)debug_cmd);
                    if (u32temp < 2000 || u32temp > 2200)
                    {
                        printf("ERROR!\n");
                        continue;
                    }
                    DebugTime.year = u32temp - 2000;

                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    printf("Month:");
                    while (debug_cmd[0] == 0) //等待输入
                    {
                        vTaskDelay(100);
                    }
                    u32temp = atol((char *)debug_cmd);
                    if (u32temp < 1 || u32temp > 12)
                    {
                        printf("ERROR!\n");
                        continue;
                    }
                    DebugTime.month = u32temp;

                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    printf("Day:");
                    while (debug_cmd[0] == 0) //等待输入
                    {
                        vTaskDelay(100);
                    }
                    u32temp = atol((char *)debug_cmd);
                    if (u32temp < 1 || u32temp > 31)
                    {
                        printf("ERROR!\n");
                        continue;
                    }
                    DebugTime.day = u32temp;

                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    printf("Hour:");
                    while (debug_cmd[0] == 0) //等待输入
                    {
                        vTaskDelay(100);
                    }
                    u32temp = atol((char *)debug_cmd);
                    if (u32temp > 23)
                    {
                        printf("ERROR!\n");
                        continue;
                    }
                    DebugTime.hour = u32temp;

                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    printf("Minute:");
                    while (debug_cmd[0] == 0) //等待输入
                    {
                        vTaskDelay(100);
                    }
                    u32temp = atol((char *)debug_cmd);
                    if (u32temp > 59)
                    {
                        printf("ERROR!\n");
                        continue;
                    }
                    DebugTime.minute = u32temp;

                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    printf("Second:");
                    while (debug_cmd[0] == 0) //等待输入
                    {
                        vTaskDelay(100);
                    }
                    u32temp = atol((char *)debug_cmd);
                    if (u32temp > 59)
                    {
                        printf("ERROR!\n");
                        continue;
                    }
                    DebugTime.second = u32temp;

                    SysTime.year = DebugTime.year;
                    SysTime.month = DebugTime.month;
                    SysTime.day = DebugTime.day;
                    SysTime.hour = DebugTime.hour;
                    SysTime.minute = DebugTime.minute;
                    SysTime.second = DebugTime.second;
                    printf("OK!\n");
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                }
                else
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                }
            }
        }
        else if (memcmp(debug_cmd, "AMPW", 4) == 0) //写入电机参数
        {
            memset(debug_cmd, 0, sizeof(debug_cmd));
            if (Motor_Param_Set() == 0)
            {
                printf("OK\n");
            }
            else
            {
                printf("ERROR\n");
            }
        }
        else if (memcmp(debug_cmd, "EXIT", 4) == 0) //结束
        {
            memset(debug_cmd, 0, sizeof(debug_cmd));
            printf("Exit Debug Mode\n");
            debug_mode = 0;
            vTaskDelete(NULL);
        }
        else if (memcmp(debug_cmd, "M", 1) == 0) //电机
        {
            device = debug_cmd[1] - 0x30;
            motor = atol((char *)&debug_cmd[3]);
            memset(debug_cmd, 0, sizeof(debug_cmd));
            printf("Device %d,Motor %02d\n", device, motor);
            for (;;)
            {
                printf("Input:");
                while (debug_cmd[0] == 0) //等待输入
                {
                    vTaskDelay(100);
                }
                if (memcmp(debug_cmd, "SCNTSET", 7) == 0)
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    Debug_Motor_Conv(device, motor, MotorStruTemp);
                    MotorStruDebug = *MotorStruTemp;
                    STPM_EXINCnt_Set((MotorStruDebug)->BoardID, (MotorStruDebug)->ChannelID);
                }
                else if (memcmp(debug_cmd, "SCNTGET", 7) == 0)
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    Debug_Motor_Conv(device, motor, MotorStruTemp);
                    MotorStruDebug = *MotorStruTemp;
                    STPM_EXINCnt_Get((MotorStruDebug)->BoardID, (MotorStruDebug)->ChannelID, &u32temp);
                    printf("SCNT%u\n", u32temp);
                }
                else if (memcmp(debug_cmd, "EXIT", 4) == 0) //退出
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    break;
                }
                else if (memcmp(debug_cmd, "POS", 3) == 0) //展示存的位置
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    Debug_Motor_Conv(device, motor, MotorStruTemp);
                    MotorStruDebug = *MotorStruTemp;
                    for (uint8_t i = 1; i <= 12; i++)
                    {
                        printf("POS%d\t%ld\n", i, (MotorStruDebug)->Pos[i]);
                    }
                }
                else if (memcmp(debug_cmd, "SAVEPOS", 7) == 0) //保存当前位置
                {
                    pos_id = atoi((char *)&debug_cmd[7]);
                    Debug_Motor_Conv(device, motor, MotorStruTemp);
                    MotorStruDebug = *MotorStruTemp;
                    (MotorStruDebug)->Pos[pos_id] = pos;
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    printf("OK\n");
                }
                else if (memcmp(debug_cmd, "HOME", 4) == 0) //回HOME
                {
                    Debug_Motor_Conv(device, motor, MotorStruTemp);
                    MotorStruDebug = *MotorStruTemp;
                    printf("HOME Confirm:");
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    while (debug_cmd[0] == 0) //等待输入
                    {
                        vTaskDelay(100);
                    }
                    if (debug_cmd[0] == 0x0d)
                    {
                        memset(debug_cmd, 0, sizeof(debug_cmd));
                        //电机运行
                        if (MotorCtrl(MotorStruDebug, _motor_home, 0))
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 25110);
                        }
                    }
                    else
                    {
                        memset(debug_cmd, 0, sizeof(debug_cmd));
                    }
                }
                else if (memcmp(debug_cmd, "B", 1) == 0) //条码
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
#ifdef __DeviceVersion02
                    ScannerCR100_Start_Scan();
                    printf("%s\n", &ScannerCR100.RXD.Buffer[1]);
#endif
                }
                else if (memcmp(debug_cmd, "S", 1) == 0) //扫码区高低传感器
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    printf("Scan_HS:\t%d\n", Table_Sensor_Get(Scan_HS));
                    printf("Scan_LS:\t%d\n", Table_Sensor_Get(Scan_LS));
                }
                else //位置
                {
                    Debug_Motor_Conv(device, motor, MotorStruTemp);
                    MotorStruDebug = *MotorStruTemp;
                    pos = atol((char *)debug_cmd);
                    printf("GoPos%ld:\t", pos);
                    printf("Confirm:");
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    while (debug_cmd[0] == 0) //等待输入
                    {
                        vTaskDelay(100);
                    }
                    if (debug_cmd[0] == 0x0d)
                    {
                        memset(debug_cmd, 0, sizeof(debug_cmd));
                        //电机运行
                        (MotorStruDebug)->PosTemp = pos;
                        if (MotorCtrl(MotorStruDebug, _motor_run_pos, 0xff))
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 25110);
                        }
                    }
                    else
                    {
                        memset(debug_cmd, 0, sizeof(debug_cmd));
                    }
                }
            }
        }
        else if (memcmp(debug_cmd, "C", 1) == 0) //链条
        {
            motor = atol((char *)&debug_cmd[1]);
            memset(debug_cmd, 0, sizeof(debug_cmd));
            printf("Chain %d\n", motor);
            for (;;)
            {
                printf("Input:");
                while (debug_cmd[0] == 0) //等待输入
                {
                    vTaskDelay(100);
                }
                if (memcmp(debug_cmd, "EXIT", 4) == 0) //退出
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    break;
                }
                else if (memcmp(debug_cmd, "HOME", 4) == 0) //找一次HOME（正向）
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    if (motor == 1) //仅返回链条
                    {
                        //释放反向电机
                        if (MotorCtrl((MotorStruDef *)&Table.CHBR, _motor_rls, 0))
                        {
                        }
                        while (1)
                        {
                            UserTaskDelay(50);
                            if (Table.CHBR.Status == _motor_rlsd)
                            {
                                break;
                            }
                        }
//正向电机HOME
#if 0
                        if (STPM_HOffset_Set(Table.CHBF.BoardID, Table.CHBF.ChannelID, Table.CHBF.Param.HOffset))
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 26100);
                        }
#endif
                        if (MotorCtrl((MotorStruDef *)&Table.CHBF, _motor_home, 0))
                        {
                            //错误处理
                            User_Error_Handler(__FILE__, __LINE__, 26100);
                        }
                    }
                }
                else if (memcmp(debug_cmd, "POS", 3) == 0) //展示存的位置
                {
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    if (motor == 0) //进样链条
                    {
                        printf("OFFSET1\t%ld\n", ChainIn.Offset[0]);
                        printf("OFFSET2\t%ld\n", ChainIn.Offset[1]);
                    }
                    else //返回链条
                    {
                        printf("OFFSET1\t%ld\n", ChainBack.Offset[0][0]);
                        printf("OFFSET2\t%ld\n", ChainBack.Offset[0][1]);
                        printf("OFFSET3\t%ld\n", ChainBack.Offset[1][0]);
                        printf("OFFSET4\t%ld\n", ChainBack.Offset[1][1]);
                    }
                }
                else if (memcmp(debug_cmd, "SAVEPOS", 7) == 0) //保存当前位置
                {
                    pos_id = atoi((char *)&debug_cmd[7]);
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                    if (motor == 0) //进样链条
                    {
                        if (pos_id == 1)
                        {
                            ChainIn.Offset[0] = pos;
                            printf("OK\n");
                        }
                        else if (pos_id == 2)
                        {
                            ChainIn.Offset[1] = pos;
                            printf("OK\n");
                        }
                        else
                        {
                            printf("ERROR\n");
                        }
                    }
                    else //返回链条
                    {
                        if (pos_id == 1)
                        {
                            ChainBack.Offset[0][0] = pos;
                            printf("OK\n");
                        }
                        else if (pos_id == 2)
                        {
                            ChainBack.Offset[0][1] = pos;
                            printf("OK\n");
                        }
                        else if (pos_id == 3)
                        {
                            ChainBack.Offset[1][0] = pos;
                            printf("OK\n");
                        }
                        else if (pos_id == 4)
                        {
                            ChainBack.Offset[1][1] = pos;
                            printf("OK\n");
                        }
                        else
                        {
                            printf("ERROR\n");
                        }
                    }
                }
                else if (memcmp(debug_cmd, "RUN", 3) == 0) //运行 RUN0 10
                {
                    if (debug_cmd[3] == '1')
                    {
                        alig = 1;
                    }
                    else if (debug_cmd[3] == '2')
                    {
                        alig = 2;
                    }
                    else
                    {
                        alig = 0;
                    }
                    pos_id = atol((char *)&debug_cmd[5]);
                    memset(debug_cmd, 0, sizeof(debug_cmd));

                    DebugTemp.RunPos = pos_id;
                    DebugTemp.Align = alig;
                    if (motor == 0)
                    {
                        ChainIn.RunCtl = _cctl_run;
                        xUserTaskCreate(ChainInCtrlTask, "ChainInCtrlTask", MotorCtl_STACK_SIZE, (void *)&DebugTemp, ChainCtl_Priority, NULL);
                    }
                    else
                    {
                        xUserTaskCreate(ChainBackCtrlTask, "ChainBackCtrlTask", configMINIMAL_STACK_SIZE * 3, (void *)&DebugTemp, ChainCtl_Priority, NULL);
                    }
                }
                else //位置
                {
                    pos = atol((char *)debug_cmd);
                    printf("Offset%ld:\n", pos);
                    memset(debug_cmd, 0, sizeof(debug_cmd));
                }
            }
        }
        else if (memcmp(debug_cmd, "T", 1) == 0) //轨道数量
        {
            pos_id = atol((char *)&debug_cmd[1]);
            memset(debug_cmd, 0, sizeof(debug_cmd));
            if (pos_id > 4)
            {
                printf("ERROR\n");
            }
            else
            {
                TrackNumber = pos_id;
                printf("OK\n");
            }
        }
        else if (memcmp(debug_cmd, "S", 1) == 0) //传感器
        {
            device = debug_cmd[1] - 0x30;
            memset(debug_cmd, 0, sizeof(debug_cmd));
            for (;;)
            {
                if (device == 0) //样本台
                {
                    printf("\n");
                    printf("CHIF_S:\t\t%d\n", Table_Sensor_Get(CHIF_S));
                    printf("CHIR_S:\t\t%d\n", Table_Sensor_Get(CHIR_S));
                    printf("CHBF_S:\t\t%d\n", Table_Sensor_Get(CHBF_S));
                    printf("CHBR_S:\t\t%d\n", Table_Sensor_Get(CHBR_S));
                    printf("PBI_HM:\t\t%d\n", Table_Sensor_Get(PBI_HM));
                    printf("PBI_WK1:\t%d\n", Table_Sensor_Get(PBI_WK1));
                    printf("PBI_WK2:\t%d\n", Table_Sensor_Get(PBI_WK2));
                    printf("PPI_HM:\t\t%d\n", Table_Sensor_Get(PPI_HM));
                    printf("PPI_WK:\t\t%d\n", Table_Sensor_Get(PPI_WK));
                    printf("PBB_HM:\t\t%d\n", Table_Sensor_Get(PBB_HM));
                    printf("PBB_WK1:\t%d\n", Table_Sensor_Get(PBB_WK1));
                    printf("PBB_WK2:\t%d\n", Table_Sensor_Get(PBB_WK2));
                    printf("PPB_HM:\t\t%d\n", Table_Sensor_Get(PPB_HM));
                    printf("ChainIn_FS:\t%d\n", Table_Sensor_Get(ChainIn_FS));
                    printf("ChainIn_RS:\t%d\n", Table_Sensor_Get(ChainIn_RS));
                    printf("ChainIn_SS:\t%d\n", Table_Sensor_Get(ChainIn_SS));
                    printf("ChainBack_FS:\t%d\n", Table_Sensor_Get(ChainBack_FS));
                    printf("ChainBack_RS:\t%d\n", Table_Sensor_Get(ChainBack_RS));
                    printf("ChainBack_SFS:\t%d\n", Table_Sensor_Get(ChainBack_SFS));
                    printf("ChainBack_SRS:\t%d\n", Table_Sensor_Get(ChainBack_SRS));
                    printf("Scan_HS:\t%d\n", Table_Sensor_Get(Scan_HS));
                    printf("Scan_LS:\t%d\n", Table_Sensor_Get(Scan_LS));
                    printf("Scan_OS:\t%d\n", Table_Sensor_Get(Scan_OS));
                    printf("Recy_ES:\t%d\n", Table_Sensor_Get(Recy_ES));
                    printf("Recy_FS:\t%d\n", Table_Sensor_Get(Recy_FS));
                    printf("Norm_SS:\t%d\n", Table_Sensor_Get(Norm_SS));
                    printf("Emg_SS:\t\t%d\n", Table_Sensor_Get(Emg_SS));
                    printf("Back_SS:\t%d\n", Table_Sensor_Get(Back_SS));
                    printf("Pause_BUT:\t%d\n", Table_Sensor_Get(_BUT_Pause));
                    printf("Back_BUT:\t%d\n", Table_Sensor_Get(_BUT_Back));
                }
                else if (device <= 4)
                {
                    printf("\n");
                    printf("WBF_HM:\t%d\n", Track_Sensor_Get(device - 1, WBF_HM));
                    printf("WBF_WK:\t%d\n", Track_Sensor_Get(device - 1, WBF_WK));
                    printf("NPB_HM:\t%d\n", Track_Sensor_Get(device - 1, NPB_HM));
                    printf("NPB_WK:\t%d\n", Track_Sensor_Get(device - 1, NPB_WK));
                    printf("EPB_HM:\t%d\n", Track_Sensor_Get(device - 1, EPB_HM));
                    printf("EPB_WK:\t%d\n", Track_Sensor_Get(device - 1, EPB_WK));
                    printf("MTC_HM:\t%d\n", Track_Sensor_Get(device - 1, MTC_HM));
                    printf("MTC_WK:\t%d\n", Track_Sensor_Get(device - 1, MTC_WK));
                    printf("MTC_SH:\t%d\n", Track_Sensor_Get(device - 1, MTC_SH));
                    printf("MTC_SW:\t%d\n", Track_Sensor_Get(device - 1, MTC_SW));
                    printf("MBF_HM:\t%d\n", Track_Sensor_Get(device - 1, MBF_HM));
                    printf("MBF_WK:\t%d\n", Track_Sensor_Get(device - 1, MBF_WK));
                    printf("BPB_HM:\t%d\n", Track_Sensor_Get(device - 1, BPB_HM));
                    printf("BPB_WK:\t%d\n", Track_Sensor_Get(device - 1, BPB_WK));
                    printf("W_EN:\t%d\n", Track_Sensor_Get(device - 1, W_EN));
                    printf("N_MO:\t%d\n", Track_Sensor_Get(device - 1, N_MO));
                    printf("N_MI:\t%d\n", Track_Sensor_Get(device - 1, N_MI));
                    printf("N_EX:\t%d\n", Track_Sensor_Get(device - 1, N_EX));
                    printf("E_EN:\t%d\n", Track_Sensor_Get(device - 1, E_EN));
                    printf("E_MO:\t%d\n", Track_Sensor_Get(device - 1, E_MO));
                    printf("E_MI:\t%d\n", Track_Sensor_Get(device - 1, E_MI));
                    printf("E_EX:\t%d\n", Track_Sensor_Get(device - 1, E_EX));
                    printf("B_EN:\t%d\n", Track_Sensor_Get(device - 1, B_EN));
                    printf("B_MI:\t%d\n", Track_Sensor_Get(device - 1, B_MI));
                }
                else
                {
                    printf("ERROR\n");
                    break;
                }

                vTaskDelay(50);
                if (debug_cmd[0] != 0) //任意键退出
                {
                    break;
                }
            }
        }
        else if (memcmp(debug_cmd, "RUNTEST00", 9) == 0) //整机运行测试，随机
        {
            printf("Confirm:");
            memset(debug_cmd, 0, sizeof(debug_cmd));
            while (debug_cmd[0] == 0) //等待输入
            {
                vTaskDelay(100);
            }
            if (debug_cmd[0] == 0x0d)
            {
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
                memset(debug_cmd, 0, sizeof(debug_cmd));
                printf("TEST00 Running...\n");

                Motor_Param_Set();
                Motor_All_Home(1);
                DeviceStatusChg = StandBy;
                while (DeviceStatus != StandBy)
                {
                    vTaskDelay(100);
                }
                DeviceStatusChg = Run;
                while (DeviceStatus != Run)
                {
                    vTaskDelay(100);
                }
                Test_Mode = 0x01;
                ChainIn.RunCtl = _cctl_run;
                while (1)
                {
                    vTaskDelay(1000);
                }
            }
        }
        else if (memcmp(debug_cmd, "RUNTEST01", 9) == 0) //整机运行测试，回收
        {
            printf("Confirm:");
            memset(debug_cmd, 0, sizeof(debug_cmd));
            while (debug_cmd[0] == 0) //等待输入
            {
                vTaskDelay(100);
            }
            if (debug_cmd[0] == 0x0d)
            {
                memset(debug_cmd, 0, sizeof(debug_cmd));
                printf("TEST01 Running...\n");

                Motor_Param_Set();
                Motor_All_Home(1);
                DeviceStatusChg = StandBy;
                while (DeviceStatus != StandBy)
                {
                    vTaskDelay(100);
                }
                DeviceStatusChg = Run;
                while (DeviceStatus != Run)
                {
                    vTaskDelay(100);
                }
                Test_Mode = 0x03; //回收
                ChainIn.RunCtl = _cctl_run;
                while (1)
                {
                    vTaskDelay(1000);
                }
            }
        }
        else if (memcmp(debug_cmd, "RUNTEST02", 9) == 0) //整机运行测试，复测循环
        {
            printf("Confirm:");
            memset(debug_cmd, 0, sizeof(debug_cmd));
            while (debug_cmd[0] == 0) //等待输入
            {
                vTaskDelay(100);
            }
            if (debug_cmd[0] == 0x0d)
            {
                memset(debug_cmd, 0, sizeof(debug_cmd));
                printf("TEST02 Running...\n");
                Motor_Param_Set();
                Motor_All_Home(1);
                DeviceStatusChg = StandBy;
                while (DeviceStatus != StandBy)
                {
                    vTaskDelay(100);
                }
                DeviceStatusChg = Run;
                while (DeviceStatus != Run)
                {
                    vTaskDelay(100);
                }
                Test_Mode = 0x02; //复测
                ChainIn.RunCtl = _cctl_run;
                while (1)
                {
                    vTaskDelay(1000);
                }
            }
        }
        else if (memcmp(debug_cmd, "RUNTEST10", 9) == 0) //轨道测试，循环
        {
            printf("Confirm:");
            memset(debug_cmd, 0, sizeof(debug_cmd));
            while (debug_cmd[0] == 0) //等待输入
            {
                vTaskDelay(100);
            }
            if (debug_cmd[0] == 0x0d)
            {
                memset(debug_cmd, 0, sizeof(debug_cmd));
                printf("TEST10 Running...\n");
                Motor_Param_Set();
                Motor_All_Home(1);
                DeviceStatusChg = StandBy;
                while (DeviceStatus != StandBy)
                {
                    vTaskDelay(100);
                }
                DeviceStatusChg = Run;
                while (DeviceStatus != Run)
                {
                    vTaskDelay(100);
                }
                Test_Mode = 0x10; //轨道测试
                ChainIn.RunCtl = _cctl_run;
                while (1)
                {
                    vTaskDelay(1000);
                }
            }
        }
        else if (memcmp(debug_cmd, "DTEST", 5) == 0)
        {
            memset(debug_cmd, 0, sizeof(debug_cmd));
        }
        else
        {
            memset(debug_cmd, 0, sizeof(debug_cmd));
        }
    }
}

/**
 * @brief  初始化获取所有电机互斥量
 * @param  NULL
 * @retval 0-成功，1-错误
 */
uint8_t FullSemaphoreTake(TickType_t block_time)
{
    //获取所有电机的信号量
    if (xUserSemaphoreTake(ChainIn.Mutex, block_time) != pdTRUE)
    {
        return 1;
    }
    if (xUserSemaphoreTake(ChainBack.Mutex, block_time) != pdTRUE)
    {
        return 1;
    }
    if (xUserSemaphoreTake(Table.PBI.Mutex, block_time) != pdTRUE)
    {
        return 1;
    }
    if (xUserSemaphoreTake(Table.PPI.Mutex, block_time) != pdTRUE)
    {
        return 1;
    }
    if (xUserSemaphoreTake(Table.PBB.Mutex, block_time) != pdTRUE)
    {
        return 1;
    }
    if (xUserSemaphoreTake(Table.PPB.Mutex, block_time) != pdTRUE)
    {
        return 1;
    }

    for (uint8_t Tid = 0; Tid < TrackNumber; Tid++)
    {
        if (xUserSemaphoreTake(Track[Tid].WBF.Mutex, block_time) != pdTRUE)
        {
            return 1;
        }
        if (xUserSemaphoreTake(Track[Tid].NPB.Mutex, block_time) != pdTRUE)
        {
            return 1;
        }
        if (xUserSemaphoreTake(Track[Tid].EPB.Mutex, block_time) != pdTRUE)
        {
            return 1;
        }
        if (xUserSemaphoreTake(Track[Tid].MTC.Mutex, block_time) != pdTRUE)
        {
            return 1;
        }
        if (xUserSemaphoreTake(Track[Tid].MBF.Mutex, block_time) != pdTRUE)
        {
            return 1;
        }
        if (xUserSemaphoreTake(Track[Tid].BPB.Mutex, block_time) != pdTRUE)
        {
            return 1;
        }
        if (xUserSemaphoreTake(Track[Tid].NBL.Mutex, block_time) != pdTRUE)
        {
            return 1;
        }
        if (xUserSemaphoreTake(Track[Tid].NBS.Mutex, block_time) != pdTRUE)
        {
            return 1;
        }
        if (xUserSemaphoreTake(Track[Tid].EBL.Mutex, block_time) != pdTRUE)
        {
            return 1;
        }
        if (xUserSemaphoreTake(Track[Tid].EBS.Mutex, block_time) != pdTRUE)
        {
            return 1;
        }
        if (xUserSemaphoreTake(Track[Tid].BBL.Mutex, block_time) != pdTRUE)
        {
            return 1;
        }
        if (xUserSemaphoreTake(Track[Tid].BBS.Mutex, block_time) != pdTRUE)
        {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief  初始化释放所有电机互斥量
 * @param  NULL
 * @retval 0-成功，1-错误
 */
uint8_t FullSemaphoreGive(void)
{
    uint8_t ErrorFlag = 0;
    //释放所有电机的信号量
    if (xUserSemaphoreGive(ChainIn.Mutex) != pdTRUE)
    {
        ErrorFlag |= 1;
    }
    if (xUserSemaphoreGive(ChainBack.Mutex) != pdTRUE)
    {
        ErrorFlag |= 1;
    }
    if (xUserSemaphoreGive(Table.PBI.Mutex) != pdTRUE)
    {
        ErrorFlag |= 1;
    }
    if (xUserSemaphoreGive(Table.PPI.Mutex) != pdTRUE)
    {
        ErrorFlag |= 1;
    }
    if (xUserSemaphoreGive(Table.PBB.Mutex) != pdTRUE)
    {
        ErrorFlag |= 1;
    }
    if (xUserSemaphoreGive(Table.PPB.Mutex) != pdTRUE)
    {
        ErrorFlag |= 1;
    }

    for (uint8_t Tid = 0; Tid < TrackNumber; Tid++)
    {
        if (xUserSemaphoreGive(Track[Tid].WBF.Mutex) != pdTRUE)
        {
            ErrorFlag |= 1;
        }
        if (xUserSemaphoreGive(Track[Tid].NPB.Mutex) != pdTRUE)
        {
            ErrorFlag |= 1;
        }
        if (xUserSemaphoreGive(Track[Tid].EPB.Mutex) != pdTRUE)
        {
            ErrorFlag |= 1;
        }
        if (xUserSemaphoreGive(Track[Tid].MTC.Mutex) != pdTRUE)
        {
            ErrorFlag |= 1;
        }
        if (xUserSemaphoreGive(Track[Tid].MBF.Mutex) != pdTRUE)
        {
            ErrorFlag |= 1;
        }
        if (xUserSemaphoreGive(Track[Tid].BPB.Mutex) != pdTRUE)
        {
            ErrorFlag |= 1;
        }

        if (xUserSemaphoreGive(Track[Tid].NBL.Mutex) != pdTRUE)
        {
            ErrorFlag |= 1;
        }
        if (xUserSemaphoreGive(Track[Tid].NBS.Mutex) != pdTRUE)
        {
            ErrorFlag |= 1;
        }
        if (xUserSemaphoreGive(Track[Tid].EBL.Mutex) != pdTRUE)
        {
            ErrorFlag |= 1;
        }
        if (xUserSemaphoreGive(Track[Tid].EBS.Mutex) != pdTRUE)
        {
            ErrorFlag |= 1;
        }
        if (xUserSemaphoreGive(Track[Tid].BBL.Mutex) != pdTRUE)
        {
            ErrorFlag |= 1;
        }
        if (xUserSemaphoreGive(Track[Tid].BBS.Mutex) != pdTRUE)
        {
            ErrorFlag |= 1;
        }
    }
    return ErrorFlag;
}

/**
 * @brief  扫码器初始化线程
 * @param  None
 * @retval None
 */
static void ScannerInitTask(void *pvParameters)
{
    //初始化扫码器
#ifdef __DeviceVersion01
    // Honeywell扫码器
    ScannerF_Reset();
    ScannerS_Reset();
#elif defined __DeviceVersion02
    // Leuze扫码器

    ScannerCR100_INIT();
    // ScannerF_INIT();
#endif

    for (;;)
    {
        vTaskDelete(NULL);
    }
}

/**
 * @brief  初始化轨道数
 * @param  None
 * @retval None
 */
void TrakIDInit(void)
{
    uint8_t flag = 0;
    TrackNumber = 0;
    //初始化CANID
    //样本台
    Table.BdCanID[0] = 0x10;
    Table.BdCanID[1] = 0x11;
    //轨道
    for (uint8_t Tid = 0; Tid < MaxTrackNumber; Tid++)
    {
        Track[Tid].BdCanID[0] = 0x14 + (Tid * 4);
        Track[Tid].BdCanID[1] = Track[Tid].BdCanID[0] + 1;
        Track[Tid].BdCanID[2] = Track[Tid].BdCanID[0] + 2;
    }
    //尝试连接各个轨道CAN板卡
    //样本台
    if (STPM_Comm_Test(Table.BdCanID[0]) || STPM_Comm_Test(Table.BdCanID[1]))
    {
        //错误
        User_Error_Handler(__FILE__, __LINE__, 26100);
        return ;
    }
    //轨道
    for (uint8_t Tid = 0; Tid < MaxTrackNumber; Tid++)
    {
        for (uint8_t Bid = 0; Bid < 3; Bid++)
        {
            if (STPM_Comm_Test(Track[Tid].BdCanID[Bid]))
            {
                flag = 1;
                TrackNumber = Tid;
                break;
            }
        }
        if (flag)
        {
            break;
        }
        else if (Tid == MaxTrackNumber)
        {
            TrackNumber = 4;
        }
    }
}

/**
 * @brief  整机控制线程
 * @param  None
 * @retval None
 */
void DeviceCtrlTask(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    (void)pvParameters;

    framemutex = xSemaphoreCreateMutex();

    //等待系统启动
    while (SysOnTime == 0)
    {
        vTaskDelay(1);
    }
    //初始化板卡传感器IO
    MAIN_EXIN_Init();
    //初始化系统状态
    DeviceStatus = None;
    DeviceStatusChg = None;
    //启动工作日志记录
    memset(LogBuffer, 0, sizeof(LogBuffer));
    sprintf(LogBuffer, "INFO SYS On %s TrackNumber %d", ProjectVersion, TrackNumber);
    WorkRecordReadWrite(0, 0, LogBuffer);
    WorkRecordReadWrite(0, 0, "INFO SYS Status None");
    //建立扫码器初始化线程
    xUserTaskCreate(ScannerInitTask, "ScannerInitTask", configMINIMAL_STACK_SIZE, NULL, SampleCtl_Priority, NULL);
    //建立按键、灯控制线程
    xUserTaskCreate(TableButCtrlTask, "TableButCtrlTask", configMINIMAL_STACK_SIZE, NULL, SampleCtl_Priority, NULL);
    initflg = 0;
    for (;;)
    {
        //系统状态刷新和处理
        switch (DeviceStatusChg)
        {
        case None:
            if (DeviceStatus == _Reset) //初始化错误导致
            {
                //状态转变
                DeviceStatus = None;
                //灭灯
                ButPause.LED_Status = 0;
                ButBack.LED_Status = 0;
                //工作记录
                WorkRecordReadWrite(0, 0, "INFO SYS Status None");
            }
            // DeviceStatusChg = _NULL;
            DeviceStatusChg = _NULL;
            break;

        case StandBy:
            if (DeviceStatus == _Reset || DeviceStatus == None)
            {
                //状态转变
                DeviceStatus = StandBy;
                //灭灯
                ButPause.LED_Status = 0;
                ButBack.LED_Status = 0;
                //建立进样线程
                xUserTaskCreate(TableInCtrlTask, "TableInCtrlTask", configMINIMAL_STACK_SIZE * 2, NULL, SampleCtl_Priority, &TableInCtrlTaskHandle);
                //建立样本移交线程
                xUserTaskCreate(TrackSampleTransferTask, "TrackSampleTransferTask", configMINIMAL_STACK_SIZE, NULL, SampleCtl_Priority, &TrackSampleTransferTaskHandle);
                //建立返回处理线程
                xUserTaskCreate(TableBackCtrlTask, "TableBackCtrlTask", configMINIMAL_STACK_SIZE * 3, NULL, SampleCtl_Priority, &TableBackCtrlTaskTaskHandle);
                //工作记录
                WorkRecordReadWrite(0, 0, "INFO SYS Status StandBy");
            }
            DeviceStatusChg = _NULL;
            break;

        case _Reset:
            if (DeviceStatus == None)
            {
                //状态转变
                DeviceStatus = _Reset;
                //闪烁
                ButPause.LED_Status = 2;
                ButBack.LED_Status = 0;
                //工作记录
                WorkRecordReadWrite(0, 0, "INFO SYS Status Reset");
            }
            DeviceStatusChg = _NULL;
            break;

        case Run:
            if (DeviceStatus == StandBy)
            {
                //状态转变
                DeviceStatus = Run;
                //工作记录
                WorkRecordReadWrite(0, 0, "INFO SYS Status Run");
            }
            if (DeviceStatus == Pause) //唤醒
            {

                Motor_All_Home(0);
                for (uint8_t Tid = 0; Tid < TrackNumber; Tid++)
                {
                    Track[Tid].NLBelt.BeltPause = 0;
                    Track[Tid].ELBelt.BeltPause = 0;
                    Track[Tid].BLBelt.BeltPause = 0;
                    Track[Tid].NSBelt.BeltPause = 0;
                    Track[Tid].ESBelt.BeltPause = 0;
                    Track[Tid].BSBelt.BeltPause = 0;
                }
                //状态转变
                DeviceStatus = Run;
                //工作记录
                WorkRecordReadWrite(0, 0, "INFO SYS Status Run");
            }
            DeviceStatusChg = _NULL;
            break;

        case Pause:
            if (DeviceStatus == Run || DeviceStatus == StandBy)
            {
                Motor_All_Stop(1);
                ChainIn.RunCtl = _cctl_userpause; //切换为用户暂停
                //状态转变
                DeviceStatus = Pause;
                //工作记录
                WorkRecordReadWrite(0, 0, "INFO SYS Status Pause");
            }
            DeviceStatusChg = _NULL;
            break;

        case Stop:
            if (DeviceStatus != None)
            {
                //状态转变
                DeviceStatus = Stop;
                //灭灯
                ButPause.LED_Status = 0;
                ButBack.LED_Status = 0;

                SystemEmergencyStop = 1;
                //删除进样线程
                if (TableInCtrlTaskHandle != NULL)
                {
                    vTaskDelete(TableInCtrlTaskHandle);
                }
                //删除样本移交线程
                if (TrackSampleTransferTaskHandle != NULL)
                {
                    vTaskDelete(TrackSampleTransferTaskHandle);
                }
                //删除返回处理线程
                if (TableBackCtrlTaskTaskHandle != NULL)
                {
                    vTaskDelete(TableBackCtrlTaskTaskHandle);
                }

                //停止所有电机
                Motor_All_Stop(0);
                //工作记录
                WorkRecordReadWrite(0, 0, "INFO SYS Status Stop");
            }
            DeviceStatusChg = _NULL;
            break;

        case Error: //严重错误
            //状态转变
            DeviceStatus = Error;
            //运行和急诊闪烁代表错误
            ButPause.LED_Status = 2;
            ButBack.LED_Status = 2;

            SystemEmergencyStop = 1;
            //删除进样线程
            if (TableInCtrlTaskHandle != NULL)
            {
                vTaskDelete(TableInCtrlTaskHandle);
            }
            //删除样本移交线程
            if (TrackSampleTransferTaskHandle != NULL)
            {
                vTaskDelete(TrackSampleTransferTaskHandle);
            }
            //删除返回处理线程
            if (TableBackCtrlTaskTaskHandle != NULL)
            {
                vTaskDelete(TableBackCtrlTaskTaskHandle);
            }
            //工作记录
            WorkRecordReadWrite(0, 0, "INFO SYS Status Error");
            //停止所有电机
            Motor_All_Stop(0);
            DeviceStatusChg = _NULL;
            vTaskDelete(NULL);
            break;

        default:
            break;
        }

        //轨道空闲管理
        if (DeviceStatus == StandBy || DeviceStatus == Run)
        {
            for (uint8_t Tid = 0; Tid < TrackNumber; Tid++)
            {
                //传送带空闲管理
                //常规长
                if (Track[Tid].NLBelt.BeltRun == 0 && Track[Tid].NLBelt.BeltPause == 0)
                {
                    Track[Tid].NLBelt.BeltCnt++;
                }
                if (Track[Tid].NLBelt.BeltCnt > 100 ||
                        (Track[Tid].NLBelt.BeltStop == 1 && Track[Tid].NLBelt.BeltPause == 0)) // 10*100ms
                {
                    Track[Tid].NLBelt.BeltCnt = 0;
                    Track[Tid].NLBelt.BeltPause = 1;
                    //传送带停止
                    MotorCtrl((MotorStruDef *)&Track[Tid].NBL, _motor_stop, 0);
                    //工作记录
                    memset(LogBuffer, 0, sizeof(LogBuffer));
                    sprintf(LogBuffer, "INFO TRK TrackID %d NormBelt Paused", Tid + 1);
                    WorkRecordReadWrite(0, 0, LogBuffer);
                }
                if (Track[Tid].NLBelt.BeltRun != 0 && Track[Tid].NLBelt.BeltStop != 1)
                {
                    Track[Tid].NLBelt.BeltCnt = 0;
                    if (Track[Tid].NLBelt.BeltPause == 1)
                    {
                        Track[Tid].NLBelt.BeltPause = 0;
                        //传送带运行
                        MotorCtrl((MotorStruDef *)&Track[Tid].NBL, _motor_run_spd, 0);
                        // while (Track[Tid].NBL.Status != _motor_idle)
                        // {
                        //     vTaskDelay(1);
                        // }
                        //工作记录
                        memset(LogBuffer, 0, sizeof(LogBuffer));
                        sprintf(LogBuffer, "INFO TRK TrackID %d NormLongBelt Run", Tid + 1);
                        WorkRecordReadWrite(0, 0, LogBuffer);
                    }
                }

                //急诊长
                if (Track[Tid].ELBelt.BeltRun == 0 && Track[Tid].ELBelt.BeltPause == 0)
                {
                    Track[Tid].ELBelt.BeltCnt++;
                }
                if (Track[Tid].ELBelt.BeltCnt > 100 ||
                        (Track[Tid].ELBelt.BeltStop == 1 && Track[Tid].ELBelt.BeltPause == 0)) // 10*100ms
                {
                    Track[Tid].ELBelt.BeltCnt = 0;
                    Track[Tid].ELBelt.BeltPause = 1;
                    //传送带停止
                    MotorCtrl((MotorStruDef *)&Track[Tid].EBL, _motor_stop, 0);
                    //工作记录
                    memset(LogBuffer, 0, sizeof(LogBuffer));
                    sprintf(LogBuffer, "INFO TRK TrackID %d EmgLongBelt Paused", Tid + 1);
                    WorkRecordReadWrite(0, 0, LogBuffer);
                }
                if (Track[Tid].ELBelt.BeltRun != 0 && Track[Tid].ELBelt.BeltStop != 1)
                {
                    Track[Tid].ELBelt.BeltCnt = 0;
                    if (Track[Tid].ELBelt.BeltPause == 1)
                    {
                        Track[Tid].ELBelt.BeltPause = 0;
                        //传送带运行
                        MotorCtrl((MotorStruDef *)&Track[Tid].EBL, _motor_run_spd, 0);
                        // while (Track[Tid].EBL.Status != _motor_idle)
                        // {
                        //     vTaskDelay(1);
                        // }
                        //工作记录
                        memset(LogBuffer, 0, sizeof(LogBuffer));
                        sprintf(LogBuffer, "INFO TRK TrackID %d EmgLongBelt Run", Tid + 1);
                        WorkRecordReadWrite(0, 0, LogBuffer);
                    }
                }

                //返回长
                if (Track[Tid].BLBelt.BeltRun == 0 && Track[Tid].BLBelt.BeltPause == 0)
                {
                    Track[Tid].BLBelt.BeltCnt++;
                }
                if (Track[Tid].BLBelt.BeltCnt > 1000 ||
                        (Track[Tid].BLBelt.BeltStop == 1 && Track[Tid].BLBelt.BeltPause == 0)) // 10*1000ms
                {
                    Track[Tid].BLBelt.BeltCnt = 0;
                    Track[Tid].BLBelt.BeltPause = 1;
                    //传送带停止
                    MotorCtrl((MotorStruDef *)&Track[Tid].BBL, _motor_stop, 0);
                    //工作记录
                    memset(LogBuffer, 0, sizeof(LogBuffer));
                    sprintf(LogBuffer, "INFO TRK TrackID %d BackLongBelt Paused", Tid + 1);
                    WorkRecordReadWrite(0, 0, LogBuffer);
                }
                if (Track[Tid].BLBelt.BeltRun != 0 && Track[Tid].BLBelt.BeltStop != 1)
                {
                    Track[Tid].BLBelt.BeltCnt = 0;
                    if (Track[Tid].BLBelt.BeltPause == 1)
                    {
                        Track[Tid].BLBelt.BeltPause = 0;
                        //传送带运行
                        MotorCtrl((MotorStruDef *)&Track[Tid].BBL, _motor_run_spd, 0);
                        // while (Track[Tid].BBL.Status != _motor_idle)
                        // {
                        //     vTaskDelay(1);
                        // }
                        //工作记录
                        memset(LogBuffer, 0, sizeof(LogBuffer));
                        sprintf(LogBuffer, "INFO TRK TrackID %d BackLongBelt Run", Tid + 1);
                        WorkRecordReadWrite(0, 0, LogBuffer);
                    }
                }
                //常规短
                if (Track[Tid].NSBelt.BeltRun == 0 && Track[Tid].NSBelt.BeltPause == 0)
                {
                    Track[Tid].NSBelt.BeltCnt++;
                }
                if (Track[Tid].NSBelt.BeltCnt > 1000 ||
                        (Track[Tid].NSBelt.BeltStop == 1 && Track[Tid].NSBelt.BeltPause == 0)) // 10*1000ms
                {
                    Track[Tid].NSBelt.BeltCnt = 0;
                    Track[Tid].NSBelt.BeltPause = 1;
                    //传送带停止
                    MotorCtrl((MotorStruDef *)&Track[Tid].NBS, _motor_stop, 0);
                    //工作记录
                    memset(LogBuffer, 0, sizeof(LogBuffer));
                    sprintf(LogBuffer, "INFO TRK TrackID %d NormShortBelt Paused", Tid + 1);
                    WorkRecordReadWrite(0, 0, LogBuffer);
                }
                if (Track[Tid].NSBelt.BeltRun != 0 && Track[Tid].NSBelt.BeltStop != 1)
                {
                    Track[Tid].NSBelt.BeltCnt = 0;
                    if (Track[Tid].NSBelt.BeltPause == 1)
                    {
                        Track[Tid].NSBelt.BeltPause = 0;
                        //传送带运行
                        MotorCtrl((MotorStruDef *)&Track[Tid].NBS, _motor_run_spd, 0);
                        // while (Track[Tid].NBS.Status != _motor_idle)
                        // {
                        //     vTaskDelay(1);
                        // }
                        //工作记录
                        memset(LogBuffer, 0, sizeof(LogBuffer));
                        sprintf(LogBuffer, "INFO TRK TrackID %d NormShortBelt Run", Tid + 1);
                        WorkRecordReadWrite(0, 0, LogBuffer);
                    }
                }

                //急诊短
                if (Track[Tid].ESBelt.BeltRun == 0 && Track[Tid].ESBelt.BeltPause == 0)
                {
                    Track[Tid].ESBelt.BeltCnt++;
                }
                if (Track[Tid].ESBelt.BeltCnt > 1000 ||
                        (Track[Tid].ESBelt.BeltStop == 1 && Track[Tid].ESBelt.BeltPause == 0)) // 10*1000ms
                {
                    Track[Tid].ESBelt.BeltCnt = 0;
                    Track[Tid].ESBelt.BeltPause = 1;
                    //传送带停止
                    MotorCtrl((MotorStruDef *)&Track[Tid].EBS, _motor_stop, 0);
                    //工作记录
                    memset(LogBuffer, 0, sizeof(LogBuffer));
                    sprintf(LogBuffer, "INFO TRK TrackID %d EmgShortBelt Paused", Tid + 1);
                    WorkRecordReadWrite(0, 0, LogBuffer);
                }
                if (Track[Tid].ESBelt.BeltRun != 0 && Track[Tid].ESBelt.BeltStop != 1)
                {
                    Track[Tid].ESBelt.BeltCnt = 0;
                    if (Track[Tid].ESBelt.BeltPause == 1)
                    {
                        Track[Tid].ESBelt.BeltPause = 0;
                        //传送带运行
                        MotorCtrl((MotorStruDef *)&Track[Tid].EBS, _motor_run_spd, 0);
                        // while (Track[Tid].EBS.Status != _motor_idle)
                        // {
                        //     vTaskDelay(1);
                        // }
                        //工作记录
                        memset(LogBuffer, 0, sizeof(LogBuffer));
                        sprintf(LogBuffer, "INFO TRK TrackID %d EmgShortBelt Run", Tid + 1);
                        WorkRecordReadWrite(0, 0, LogBuffer);
                    }
                }

                //返回短
                if (Track[Tid].BSBelt.BeltRun == 0 && Track[Tid].BSBelt.BeltPause == 0)
                {
                    Track[Tid].BSBelt.BeltCnt++;
                }
                if (Track[Tid].BSBelt.BeltCnt > 1000 ||
                        (Track[Tid].BSBelt.BeltStop == 1 && Track[Tid].BSBelt.BeltPause == 0)) // 10*1000ms
                {
                    Track[Tid].BSBelt.BeltCnt = 0;
                    Track[Tid].BSBelt.BeltPause = 1;
                    //传送带停止
                    MotorCtrl((MotorStruDef *)&Track[Tid].BBS, _motor_stop, 0);
                    //工作记录
                    memset(LogBuffer, 0, sizeof(LogBuffer));
                    sprintf(LogBuffer, "INFO TRK TrackID %d BackShortBelt Paused", Tid + 1);
                    WorkRecordReadWrite(0, 0, LogBuffer);
                }
                if (Track[Tid].BSBelt.BeltRun != 0 && Track[Tid].BSBelt.BeltStop != 1)
                {
                    Track[Tid].BSBelt.BeltCnt = 0;
                    if (Track[Tid].BSBelt.BeltPause == 1)
                    {
                        Track[Tid].BSBelt.BeltPause = 0;
                        //传送带运行
                        MotorCtrl((MotorStruDef *)&Track[Tid].BBS, _motor_run_spd, 0);
                        // while (Track[Tid].BBS.Status != _motor_idle)
                        // {
                        //     vTaskDelay(1);
                        // }
                        //工作记录
                        memset(LogBuffer, 0, sizeof(LogBuffer));
                        sprintf(LogBuffer, "INFO TRK TrackID %d BackShortBelt Run", Tid + 1);
                        WorkRecordReadWrite(0, 0, LogBuffer);
                    }
                }
            }
        }

        //整机空闲管理
        if ((DeviceFrameCnt == 0) && (ChainIn.RunCtl != _cctl_run) && (DeviceStatus == Run || DeviceStatus == StandBy)) //没有样本了
        {
            DeviceIdleCnt++;
        }
        if (DeviceIdleCnt > 6000) // 10*6000ms
        {
            DeviceIdleCnt = 0;
            initflg = 0;
            DeviceStatusChg = Pause;
            //工作记录
            WorkRecordReadWrite(0, 0, "INFO SYS Pause");
        }
        if (ChainIn.RunCtl == _cctl_run)
        {
            DeviceIdleCnt = 0;
            if (DeviceStatus == Pause)
            {
                DeviceStatusChg = Run;
                //工作记录
                WorkRecordReadWrite(0, 0, "INFO SYS Run");
            }
        }
        //告警信息管理
        //初始化告警
        if ((DeviceStatus == None || DeviceStatus == _Reset) && InitAlarm.alarm == 1)
        {
            InitAlarm.alarm = 0; //一次性发送
            InitAlarm.code = InitAlarmCode + InitAlarm.pos;
            InitAlarm.level = 1; // warn
            //发送告警
            while (EthSendAlarm((DeviceAlarmStruDef *)&InitAlarm) != 0)
            {
                vTaskDelay(500);
            }
            //工作记录
            WorkRecordReadWrite(0, 0, "INFO SYS Alarm InitAlarm");
        }

        //垃圾箱满告警
        if (Table_Sensor_Get(Recy_FS))
        {
            if (RecoveryFulltoStopAlarm.alarm == 1) //报warn
            {
                RecoveryFulltoStopAlarm.alarm = 2;
                RecoveryFullAlarm.alarm = 0;
                RecoveryFulltoStopAlarm.code = RecoveryFullCode;
                RecoveryFulltoStopAlarm.level = 1; // warn
                //发送告警
                while (EthSendAlarm((DeviceAlarmStruDef *)&RecoveryFulltoStopAlarm) != 0)
                {
                    vTaskDelay(500);
                }
                //工作记录
                WorkRecordReadWrite(0, 0, "INFO SYS Alarm RecoveryFulltoStopAlarm");
            }
            else if (RecoveryFullAlarm.alarm == 0 && RecoveryFulltoStopAlarm.alarm == 0) //报info
            {
                RecoveryFullAlarm.alarm = 2;
                RecoveryFullAlarm.code = RecoveryFullCode;
                RecoveryFullAlarm.level = 0; // info
                //发送告警
                while (EthSendAlarm((DeviceAlarmStruDef *)&RecoveryFullAlarm) != 0)
                {
                    vTaskDelay(500);
                }
                //工作记录
                WorkRecordReadWrite(0, 0, "INFO SYS Alarm RecoveryFullAlarm");
            }
        }
        else //错误解除
        {


            if (RecoveryFullAlarm.alarm != 0)
            {
                RecoveryFullAlarm.alarm = 0;
                EthSendRecoveryFulltoStopAlarm();
                //工作记录
                WorkRecordReadWrite(0, 0, "INFO SYS Reset RecoveryFullAlarm");
            }
            if (RecoveryFulltoStopAlarm.alarm != 0)
            {
                RecoveryFulltoStopAlarm.alarm = 0;
                EthSendRecoveryFulltoStopAlarm();
                //工作记录
                WorkRecordReadWrite(0, 0, "INFO SYS Reset RecoveryFulltoStopAlarm");
            }
        }

        if (DeviceStatus != None)
        {
#if 0
            //垃圾箱满告警
            if (Table_Sensor_Get(Recy_FS))
            {
                if (RecoveryFulltoStopAlarm.alarm == 1) //报warn
                {
                    RecoveryFulltoStopAlarm.alarm = 2;
                    RecoveryFullAlarm.alarm = 0;
                    RecoveryFulltoStopAlarm.code = RecoveryFullCode;
                    RecoveryFulltoStopAlarm.level = 1; //warn
                    //发送告警
                    while (EthSendAlarm((DeviceAlarmStruDef *)&RecoveryFulltoStopAlarm) != 0)
                    {
                        vTaskDelay(500);
                    }
                    //工作记录
                    WorkRecordReadWrite(0, 0, "INFO SYS Alarm RecoveryFulltoStopAlarm");
                }
                else if (RecoveryFullAlarm.alarm == 0 && RecoveryFulltoStopAlarm.alarm == 0) //报info
                {
                    RecoveryFullAlarm.alarm = 2;
                    RecoveryFullAlarm.code = RecoveryFullCode;
                    RecoveryFullAlarm.level = 0; //info
                    //发送告警
                    while (EthSendAlarm((DeviceAlarmStruDef *)&RecoveryFullAlarm) != 0)
                    {
                        vTaskDelay(500);
                    }
                    //工作记录
                    WorkRecordReadWrite(0, 0, "INFO SYS Alarm RecoveryFullAlarm");
                }
            }
            else //错误解除
            {
                if (RecoveryFullAlarm.alarm != 0)
                {
                    RecoveryFullAlarm.alarm = 0;
                    //工作记录
                    WorkRecordReadWrite(0, 0, "INFO SYS Reset RecoveryFullAlarm");
                }
                if (RecoveryFulltoStopAlarm.alarm != 0)
                {
                    RecoveryFulltoStopAlarm.alarm = 0;
                    //工作记录
                    WorkRecordReadWrite(0, 0, "INFO SYS Reset RecoveryFulltoStopAlarm");
                }
            }
#endif
            //用户暂停影响复测通知
            if (ChainIn.RunCtl == _cctl_userpause && ChainBack.ReRunStatus == _crrs_Prepared)
            {
                if (UserPausedRerunWaitingAlarm.alarm == 1)
                {
                    UserPausedRerunWaitingAlarm.alarm = 2;
                    UserPausedRerunWaitingAlarm.code = UserPausedRerunWaitingCode;
                    UserPausedRerunWaitingAlarm.level = 0; // info
                    //发送告警
                    while (EthSendAlarm((DeviceAlarmStruDef *)&UserPausedRerunWaitingAlarm) != 0)
                    {
                        vTaskDelay(500);
                    }
                    //工作记录
                    WorkRecordReadWrite(0, 0, "INFO SYS Alarm UserPausedRerunWaitingAlarm");
                }
            }
            else if (UserPausedRerunWaitingAlarm.alarm != 0) //错误解除
            {
                UserPausedRerunWaitingAlarm.alarm = 0;
                //工作记录
                WorkRecordReadWrite(0, 0, "INFO SYS Reset UserPausedRerunWaitingAlarm");
            }

            //样本丢失告警
            if (FrameLostAlarm.alarm == 1)
            {
                FrameLostAlarm.alarm = 0; //一次性发送
                FrameLostAlarm.code = FrameLostAlarmCode + FrameLostAlarm.pos;
                FrameLostAlarm.level = 1; // warn
                //发送告警
                while (EthSendAlarm((DeviceAlarmStruDef *)&FrameLostAlarm) != 0)
                {
                    vTaskDelay(500);
                }
                //工作记录
                WorkRecordReadWrite(0, 0, "INFO SYS Alarm FrameLostAlarm");
            }

            //出现未知样本告警
            if (UnknownFrameAlarm.alarm == 1)
            {
                UnknownFrameAlarm.alarm = 0; //一次性发送
                UnknownFrameAlarm.code = UnknownFrameAlarmCode + UnknownFrameAlarm.pos;
                UnknownFrameAlarm.level = 1; // warn
                //发送告警
                while (EthSendAlarm((DeviceAlarmStruDef *)&UnknownFrameAlarm) != 0)
                {
                    vTaskDelay(500);
                }
                //工作记录
                WorkRecordReadWrite(0, 0, "INFO SYS Alarm UnknownFrameAlarm");
            }
        }
        vTaskDelay(10);
    }
}
/*****************************END OF FILE*****************************/
