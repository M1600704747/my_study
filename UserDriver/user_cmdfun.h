/**
******************************************************************************
* @文件    user_cmdfun.h
* @作者    
* @版本    V0.0.1
******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_CMDFUN_H
#define __USER_CMDFUN_H

#ifdef __cplusplus
extern "C" {
#endif
    
/* Includes ------------------------------------------------------------------*/

#include "user_app.h"


/* Private types ------------------------------------------------------------*/
typedef struct
{
    uint8_t device;
    uint8_t motor;
    uint8_t pos;
    int_fast32_t value;
}EthMotParamDef;//电机参数传递结构体

typedef struct
{
    uint8_t sid;
    uint8_t isauto;

} BarcodeInfo;

typedef enum
{
    NoneParam = 0,
    TrackNumberParam,
    TimeParam,
    MacSNParam,
    CtrlBoardSNParam,
}DbgParamDef;//调试参数枚举

extern __IO uint8_t EthDbgMode; //网络调试模式
extern __IO uint8_t RACKRUNMODE; //上位机运行模式

/* Exported constants --------------------------------------------------------*/
 
/* Exported macro ------------------------------------------------------------*/
/* Exported variables ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t EthRecvCmdDeal(EthPkgDef *pkg, char *str);
void EthCmdDeviceStatusTask(void *pvParameters);
//void EthCmdDeviceVersionTask(void *pvParameters);
void EthCmdDeviceInfoTask(void *pvParameters);
//void EthCmdDeviceSetTimeTask(void *pvParameters);
uint8_t EthCmdAlarmReset(uint8_t* data);
uint8_t EthSendAlarm(DeviceAlarmStruDef *AlarmStru);
//void EthCmdAlarmTask(void *pvParameters);



uint8_t EthSendWaitpos(uint8_t *sid);
void EthCmdTrackWaitposTask(void *pvParameters);
//void EthCmdTrackGotoRecycleTask(void *pvParameters);


void EthQuickInitTask(void *pvParameters);
uint8_t EthCmdTestStart(void);
void EthCmdCancelDo(void);
uint8_t EthSendBarcode(uint8_t *sid, uint8_t autotmp);
uint8_t EthSendButtonStart(void);

uint8_t EthSendChainInStatus(void);

    uint8_t EthCmdScanTrackBarcode(void);
void EthOTATask(void *pvParameters);

//uint8_t EthDbgEnterDebugMode(void);
uint8_t EthDbgExitDebugMode(void);
uint8_t EthDbgSensorGet(EthPkgDef *pkg);
void EthDbgSensorGetTask(void *pvParameters);
uint8_t EthDbgSetPos(EthPkgDef *pkg);
uint8_t EthDbgGetPos(EthPkgDef *pkg);
void EthDbgGetPosTask(void *pvParameters);
uint8_t EthDbgMotorRun(EthPkgDef *pkg);
//void EthDbgMotorRunTask(void *pvParameters);

//uint8_t EthDbgSetParam(EthPkgDef *pkg);
uint8_t EthDbgGetParam(EthPkgDef *pkg);
void EthDbgGetParamTask(void *pvParameters);
uint8_t EthDbgParamSave(void);
uint8_t EthDbgDeviceTestRun(EthPkgDef *pkg);
void EthDbgDeviceTestRunTask(void *pvParameters);

uint8_t EthSendGetEnableScanMode(void);




#endif 
/*****************************END OF FILE*****************************/
