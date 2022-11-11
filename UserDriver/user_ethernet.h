/*
 * @Author: your name
 * @Date: 2020-09-01 14:10:50
 * @LastEditTime: 2020-09-02 19:59:14
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \MDK-ARMd:\Tempfiles\FLM19XX\FLM1901\Program\UDProjectA2\UserDriver\user_ethernet.h
 */
/**
******************************************************************************
* @文件    user_ethernet.h
* @作者    GENGXU
* @版本    V0.0.1
******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_ETHERNET_H
#define __USER_ETHERNET_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "user_gpm.h"

#include "lwip/opt.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/apps/fs.h"
#include "string.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "ethernetif.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "app_ethernet.h"
#include "lan8742.h"
#include "lwip/tcp.h"

#ifdef __USE_TFTP
#include "tftpserver.h"
#endif
  /* Private types ------------------------------------------------------------*/

#define EthDataLen 2048
#define EthPkgNum 240
#define EthHandleNum 240

  typedef struct
  {
    uint8_t state;         //0--初始化，1--使用中（接收）/待写入（发送），2--已写入，待写入队列（发送），3--已写入队列（发送），4--已发送（发送）
    uint32_t datelen;      //数据长度
    char data[EthDataLen]; //数据

    uint8_t ackflag;        //0--非响应包，1--ack响应，2--nak响应
    uint32_t cmdid;         //数据包指令id，与data中的<id>字段应一致
    char target[25];        //指令目标，XXX.XXX.XXX.XXX:YYYYY
    char source[25];        //指令来源
    TaskHandle_t *dealtask; //发送线程，如果不为NULL，需要发送通知
  } EthPkgDef;              //数据包结构体

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define TCPECHO_THREAD_PRIO (osPriorityAboveNormal  + 5 )
#define EthComPort 7 //端口号

#define EthMaxTxTimes 3    //发送次数
#define EthTxOvertime 3000 //超时重发间隔时间，单位ms

#define RevAck 1
#define RevNak 2
  
#define SendTaskPriority  6//发送线程优先级

/* Exported variables ------------------------------------------------------------*/
extern uint32_t TxID;
extern uint32_t EthErrCode; //网络错误代码
extern TaskHandle_t EthCMDTaskHandle[EthHandleNum];
extern EthPkgDef EthPkgBuf_Send[EthPkgNum]; //发送数据包缓存
extern EthPkgDef EthPkgBuf_Recv[EthPkgNum]; //接收数据包缓存
extern char EthSource[25]; //板卡自身的IP地址，初始化后自动从IP_ADDRn更新，用于指令收发
extern char LastSource[25]; //最近一条的指令来源
extern uint8_t emg_flg;
extern uint8_t scanner_flg;
extern uint8_t initflg;

extern uint8_t waitscannerflg;
extern QueueHandle_t SendList; //发送队列，保存EthPkgBuf的索引

/* Exported functions ------------------------------------------------------- */
void UserEthernetInit(void);

uint8_t EthXmlCmpFind(const char *data_s, const char *lable, char *slice_data);
uint8_t EthXmlCmpAdd(char *data_s, const char *lable, const uint8_t mode, char *slice_data);
uint8_t EthXmlAddLable(char *data_s, const char *lable);
uint8_t EthPkgEncode(const char *data_s, uint32_t *data_exp_len);
uint8_t EthPkgDecode(const char *data_s, const uint32_t data_s_len, char *data_exp);
uint32_t EthSendIndexGet(void);


uint32_t EthHandleIndexGet(void);
uint32_t EthCurrentHandleIndex(void);
void EthDeleteCurrentTask(uint32_t handle_idx,EthPkgDef *recvpkg,EthPkgDef *sendpkg);



///**
//* @brief  数据包发送操作线程
//* @param  
//* @retval 
//*/
//void EthSendOperateTask(void *pvParameters)
//{
//  char *send_data = (char *)pvParameters;
//  char strtmp[EthDataLen] = {0};
//  uint32_t SendIdx = 0;
//  uint32_t NotifyReturn = 0;
//  uint8_t CurrentTaskHandleIndex = EthCurrentHandleIndex();
//  while (1)
//  {
//    SendIdx = EthSendIndexGet();
//    if (SendIdx != UINT32_MAX)
//    {
//      break;
//    }
//    vTaskDelay(10);
//  }
//  EthPkgBuf_Send[SendIdx].cmdid = TxID++;
//  EthPkgBuf_Send[SendIdx].dealtask = &EthCMDTaskHandle[CurrentTaskHandleIndex]; //传入当前任务句柄
//  strcpy(EthPkgBuf_Send[SendIdx].source, EthSource);
//  strcpy(EthPkgBuf_Send[SendIdx].target, LastSource);
//  strcpy(EthPkgBuf_Send[SendIdx].data, send_data);

//  memset(strtmp, 0, EthDataLen);
//  sprintf((char *)strtmp, "%u", EthPkgBuf_Send[SendIdx].cmdid);
//  if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "id", 4, strtmp) == 1)
//  {
//    //错误处理
//    WorkRecordReadWrite(0, 0, __LINE__, "WARN ETH XmlCmpAdd Error");
//    EthDeleteCurrentTask(CurrentTaskHandleIndex);
//  }
//  if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "source", 4, EthPkgBuf_Send[SendIdx].source) == 1)
//  {
//    //错误处理
//    WorkRecordReadWrite(0, 0, __LINE__, "WARN ETH XmlCmpAdd Error");
//    EthDeleteCurrentTask(CurrentTaskHandleIndex);
//  }
//  if (EthXmlCmpAdd(EthPkgBuf_Send[SendIdx].data, "target", 4, EthPkgBuf_Send[SendIdx].target) == 1)
//  {
//    //错误处理
//    WorkRecordReadWrite(0, 0, __LINE__, "WARN ETH XmlCmpAdd Error");
//    EthDeleteCurrentTask(CurrentTaskHandleIndex);
//  }

//  /*数据打包*/
//  if (EthPkgEncode(EthPkgBuf_Send[SendIdx].data, &EthPkgBuf_Send[SendIdx].datelen) == 1)
//  {
//    //错误处理
//    WorkRecordReadWrite(0, 0, __LINE__, "WARN ETH Encode Error");
//    EthDeleteCurrentTask(CurrentTaskHandleIndex);
//  }

//  /*写入完成，等待管理线程自动发送*/
//  EthPkgBuf_Send[SendIdx].state = 2;

//  /*等待通知*/
//  for (uint8_t i = 0; i < EthMaxTxTimes; i++)
//  {
//    NotifyReturn = ulTaskNotifyTake(pdTRUE, EthTxOvertime / portTICK_RATE_MS);
//    if (NotifyReturn == RevAck)
//    {
//      EthDeleteCurrentTask(CurrentTaskHandleIndex);
//    }
//    else
//    {
//      //重发
//      EthPkgBuf_Send[SendIdx].state = 2;
//    }
//  }
//  EthPkgBagInit(&EthPkgBuf_Send[SendIdx]);

//  /*删除线程*/
//  for (;;)
//  {
//    EthDeleteCurrentTask(CurrentTaskHandleIndex);
//  }
//}

#ifdef __cplusplus
}
#endif

#endif
  /*****************************END OF FILE*****************************/
