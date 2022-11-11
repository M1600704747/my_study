/**
******************************************************************************
* @文件    user_ethernet.c
* @作者    GENGXU
* @版本    V0.0.1
******************************************************************************


******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "user_ethernet.h"
#include "./user_cmdfun.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
extern void tftpd_init(void);
extern uint8_t Table_Sensor_Get(TableSensorDef SensorID);
/* Private variables ---------------------------------------------------------*/
struct netif gnetif; /* network interface structure */

/*Static IP ADDRESS*/
uint32_t IP_ADDR0 = 192;
uint32_t IP_ADDR1 = 168;
uint32_t IP_ADDR2 = 1;
uint32_t IP_ADDR3 = 10;

/*NETMASK*/
uint32_t NETMASK_ADDR0 = 255;
uint32_t NETMASK_ADDR1 = 255;
uint32_t NETMASK_ADDR2 = 255;
uint32_t NETMASK_ADDR3 = 0;

/*Gateway Address*/
uint32_t GW_ADDR0 = 192;
uint32_t GW_ADDR1 = 168;
uint32_t GW_ADDR2 = 1;
uint32_t GW_ADDR3 = 1;

uint32_t TxID = 0; //发送ID累加值，0-uint32max，在建立发送时由线程给出并累加

uint32_t RxID[100] = {0}; //接收包ID
uint8_t RxIDIdx = 0;

EthPkgDef EthPkgBuf_Send[EthPkgNum] __attribute__((at(RTOSSTARTADDR + 28 * 1024 * 1024)));
// __attribute__((at(0x20000000)))    发送数据包缓存
EthPkgDef EthPkgBuf_Recv[EthPkgNum] __attribute__((at(RTOSSTARTADDR + 28 * 1024 * 1024 + 2 * 1024 * 1024))); // __attribute__((at(0x20000000 + 0x10000)));      //接收数据包缓存

SemaphoreHandle_t SendMutex; //发送数据包缓冲区操作管理互斥量，同时管理TxID。
SemaphoreHandle_t RecvMutex; //接收数据包缓冲区操作管理互斥量

QueueHandle_t SendList; //发送队列，保存EthPkgBuf的索引
QueueHandle_t RecvList; //接收队列，保存EthPkgBuf的索引

TaskHandle_t EthCMDTaskHandle[EthHandleNum] = {NULL}; //网络通讯线程句柄库

uint32_t EthErrCode = 0; //网络错误代码
// bit0：驱动故障
// bit1：网线断开
// bit2：套接字申请失败
// bit3：套接字绑定失败
// bit4：数据接收出错
// bit5：信息获取错误
// bit6：套接字配置错误

char EthSource[25]; //板卡自身的IP地址，初始化后自动从IP_ADDRn更新，用于指令收发

char LastSource[25]; //最近一条的指令来源

extern lan8742_Object_t LAN8742;

static TaskHandle_t EthManageTaskHandle;
// static TaskHandle_t TCPIPTaskHandle;

extern struct tcp_pcb *tcp_active_pcbs;

uint8_t emg_flg;
uint8_t scanner_flg;    //3表示可以扫码，它判断轨道的加样位和等待位是否空
uint8_t initflg;
uint8_t waitscannerflg;

/* Private function prototypes -----------------------------------------------*/
// static void EthPkgBagInit(EthPkgDef *pkg);
static void EthManageTask(void *pvParameters);
static uint8_t EthRecvDeal(EthPkgDef *pkg);
static uint8_t EthAckPack(EthPkgDef *pkg, uint32_t re_cmd_id, char *target);
static uint8_t EthNakPack(EthPkgDef *pkg, uint32_t re_cmd_id, char *target);
static void EthPkgBagInit(EthPkgDef *pkg);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Initializes the lwIP stack
 * @param  None
 * @retval None
 */
static void Netif_Config(void)
{
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw;

#if LWIP_DHCP
    ip_addr_set_zero_ip4(&ipaddr);
    ip_addr_set_zero_ip4(&netmask);
    ip_addr_set_zero_ip4(&gw);
#else
    IP_ADDR4(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    IP_ADDR4(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
    IP_ADDR4(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
#endif /* LWIP_DHCP */

    /* add the network interface */
    netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

    /*  Registers the default network interface. */
    netif_set_default(&gnetif);

    ethernet_link_status_updated(&gnetif);

#if LWIP_NETIF_LINK_CALLBACK
    netif_set_link_callback(&gnetif, ethernet_link_status_updated);

    osThreadDef(EthLink, ethernet_link_thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 1);
    osThreadCreate(osThread(EthLink), &gnetif);
#endif

#if LWIP_DHCP
    /* Start DHCPClient */
    osThreadDef(DHCP, DHCP_Thread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE * 2);
    osThreadCreate(osThread(DHCP), &gnetif);
#endif
}

/**
 * @brief  数据包初始化
 * @param  None
 * @retval None
 */
static void EthPkgBagInit(EthPkgDef *pkg)
{

    memset(pkg, 0, sizeof(EthPkgDef));
#if 0
    uint32_t cntj = 0;

    pkg->ackflag = 0;
    pkg->cmdid = 0;
    for (cntj = 0; cntj < 25; cntj++)
    {
        pkg->target[cntj] = 0;
        pkg->source[cntj] = 0;
    }
    pkg->dealtask = NULL;

    pkg->datelen = 0;
    for (cntj = 0; cntj < EthDataLen; cntj++)
    {
        pkg->data[cntj] = 0;
    }
    pkg->state = 0;
#endif
}

/**
* @brief  服务器数据处理线程
该线程负责接收监听端口的数据，并将数据写入接收队列中。同时，对发送队列的数据进行发送处理。
该线程同时负责断联的监听。
  * @param  None
  * @retval None
  */
static void TcpIpServerTask(void *arg)
{
    LWIP_UNUSED_ARG(arg);

    /*--------------------参数定义-------------------*/
    uint32_t rp_last = 0;
    int sock, size, newconn;
    int ret;
    int on = 1;
    struct sockaddr_in address, remotehost;
    struct tcp_pcb *pcb;
    uint32_t cnti = 0;

    uint32_t RecvIndex = 0;
    uint8_t RecvData[EthDataLen];
    uint32_t RecvDataLen = 0;
    uint32_t SendIndex = 0;
    uint8_t SendData[EthDataLen];
    uint32_t SendDataLen = 0;
    uint32_t RecvByteIdx = 0;
    uint8_t RecvContinue = 0;
    DeviceAlarmStruDef tcp_ipalarm;

    /*--------------------线程初始化-------------------*/
    /*建立收发管理线程*/
    xUserTaskCreate(EthManageTask, "EthManageTask", configMINIMAL_STACK_SIZE * 4, NULL, 4, (TaskHandle_t *)&EthManageTaskHandle);

    /* create a TCP socket */
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        Bit_Set_U32t(&EthErrCode, 2, 1);
        vTaskDelete(NULL);
    }

    if (setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (const void *)&on, sizeof(on)) < 0)
    {
        Bit_Set_U32t(&EthErrCode, 2, 1);
        vTaskDelete(NULL);
    }

    /* bind to port EthComPort at any interface */
    address.sin_family = AF_INET;
    address.sin_port = htons(EthComPort);
    address.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        Bit_Set_U32t(&EthErrCode, 3, 1);
        vTaskDelete(NULL);
    }

    /* listen for incoming connections (TCP listen backlog = 5) */
    if (listen(sock, 5) < 0)
    {
        Bit_Set_U32t(&EthErrCode, 0, 1);
        vTaskDelete(NULL);
    }

    /*--------------------线程工作循环-------------------*/
    while (1)
    {
        size = sizeof(remotehost);
        newconn = accept(sock, (struct sockaddr *)&remotehost, (socklen_t *)&size);

        if(newconn == -1)
        {
            WorkRecordReadWrite(0, 0, "INFO ETH accept error.");
            continue;
        }

        WorkRecordReadWrite(0, 0, "INFO ETH Connection Established");
        rp_last = tcp_active_pcbs->remote_port; //记录客户端端口号
        for (cnti = 0; cnti < 100; cnti++)
        {
            RxID[cnti] = UINT32_MAX;
        }
        RxIDIdx = 0;

        emg_flg = 0;
        scanner_flg = 0;

        initflg = 5;

        // xUserTaskCreate(CheckFrameIsOk, "CheckFrameIsOk", configMINIMAL_STACK_SIZE , NULL, 4, NULL);

        //建立新的任务

        // 查看是否有样本在缓存区或者在扫码区
        if (Table_Sensor_Get(Recy_FS))
        {
            tcp_ipalarm.alarm = 2;
            tcp_ipalarm.code = RecoveryFullCode;
            tcp_ipalarm.level = 0; // info
            //发送告警
            while (EthSendAlarm((DeviceAlarmStruDef *)&tcp_ipalarm) != 0)
            {
                vTaskDelay(10);
            }
            //工作记录
            WorkRecordReadWrite(0, 0, "INFO SYS Alarm tcp_ipalarm");
        }

        while (1)
        {
            /*初始化接收缓冲区，启动接收*/
            RecvDataLen = 0;
            for (cnti = 0; cnti < EthDataLen; cnti++)
            {
                RecvData[cnti] = 0;
            }

            ret = recv(newconn, RecvData, EthDataLen, MSG_DONTWAIT); //接收

            if (ret == 0) //客户端主动断开连接
            {
                WorkRecordReadWrite(0, 0, "INFO ETH Connection Shutdown");
                break;
            }
            else if (rp_last != tcp_active_pcbs->remote_port) //客户端的端口号改变，可能发生了断线，重连
            {
                WorkRecordReadWrite(0, 0, "WARN ETH Connection Linkdown");
                break;
            }
            else if (ret > 0)
            {
                RecvDataLen = ret;
                RecvByteIdx = 0;
                RecvContinue = 1;
                while (RecvContinue == 1)
                {
                    RecvContinue = 0;
                    if (xUserSemaphoreTake(RecvMutex, portMAX_DELAY) == pdTRUE) //获取操作权限
                    {
                        RecvIndex = 0;
                        for (cnti = 0; cnti < EthPkgNum; cnti++)
                        {
                            if (EthPkgBuf_Recv[cnti].state == 0)
                            {
                                EthPkgBuf_Recv[cnti].state = 1;
                                RecvIndex = cnti;
                                break;
                            }
                            if (cnti == EthPkgNum - 1)
                            {
                                RecvIndex = UINT32_MAX;
                            }
                        }
                        if (RecvIndex != UINT32_MAX)
                        {
                            for (cnti = 0; cnti < RecvDataLen; cnti++)
                            {
                                //包溢出处理
                                if (RecvByteIdx + cnti > EthDataLen - 1)
                                {
                                    EthPkgBuf_Recv[RecvIndex].datelen = cnti;
                                    break;
                                }
                                EthPkgBuf_Recv[RecvIndex].data[cnti] = RecvData[RecvByteIdx + cnti];
                                if (EthPkgBuf_Recv[RecvIndex].data[cnti] == 0x03)
                                {
                                    EthPkgBuf_Recv[RecvIndex].datelen = cnti + 1;
                                    if (RecvData[RecvByteIdx + cnti + 1] == 0x02) //还有未收完的数据包
                                    {
                                        RecvByteIdx += (cnti + 1);
                                        RecvContinue = 1;
                                        break;
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }
                            }

                            if (xQueueSend(RecvList, (void *)&RecvIndex, 0) != pdPASS) //加入接收队列
                            {
                                EthPkgBagInit(&EthPkgBuf_Recv[RecvIndex]);
                            }
                        }

                        if (xUserSemaphoreGive(RecvMutex) != pdTRUE) //释放操作权限
                        {
                            Bit_Set_U32t(&EthErrCode, 0, 1);
                            vTaskDelete(NULL);
                        }
                    }
                    else
                    {
                        Bit_Set_U32t(&EthErrCode, 0, 1);
                        vTaskDelete(NULL);
                    }
                }
            }
            else
            {
                if (!(errno == EINTR || errno == EWOULDBLOCK || errno == EAGAIN))
                {
                    Bit_Set_U32t(&EthErrCode, 4, 1);
                    break;
                }
            }

            /*处理发送 yfxiao*/
            while (xQueueReceive(SendList, (void *)&SendIndex, 0) == pdPASS) //阻塞一段时间读取队列
            {
                SendDataLen = 0;
                for (cnti = 0; cnti < EthDataLen; cnti++)
                {
                    SendData[cnti] = 0;
                }

                if (xUserSemaphoreTake(SendMutex, portMAX_DELAY) == pdTRUE) //获取操作权限
                {
                    if (EthPkgBuf_Send[SendIndex].state == 3)
                    {
                        SendDataLen = EthPkgBuf_Send[SendIndex].datelen;
                        for (cnti = 0; cnti < SendDataLen; cnti++)
                        {
                            SendData[cnti] = EthPkgBuf_Send[SendIndex].data[cnti];
                        }
                        EthPkgBuf_Send[SendIndex].state = 4;
                        if (xUserSemaphoreGive(SendMutex) != pdTRUE) //释放操作权限
                        {
                            Bit_Set_U32t(&EthErrCode, 0, 1);
                            vTaskDelete(NULL);
                        }

                        // ret = write(newconn, (const uint8_t *)SendData, SendDataLen);
                        ret = lwip_send(newconn, (const uint8_t *)SendData, SendDataLen, MSG_DONTWAIT); //非阻塞发送，返回值为发送字节
                        if (ret != SendDataLen)                                                         //发送错误或缓冲区满
                        {

                            break; // yxiao write
                        }
                        vTaskDelay(1);
                    }
                    else
                    {
                        if (xUserSemaphoreGive(SendMutex) != pdTRUE) //释放操作权限
                        {
                            Bit_Set_U32t(&EthErrCode, 0, 1);
                            vTaskDelete(NULL);
                        }
                    }
                }
                else
                {
                    Bit_Set_U32t(&EthErrCode, 0, 1);
                    vTaskDelete(NULL);
                }
            }

            if (EthErrCode != 0)
            {
                break;
            }
            
            //处理因为网络断线可能引起问题的PCB
            for (pcb = tcp_active_pcbs; pcb != NULL; pcb = pcb->next)
            {
                if (pcb->state == FIN_WAIT_1 || pcb->state == FIN_WAIT_2)
                {
                    tcp_abort(pcb);
                }
            }

            vTaskDelay(3);
        }
        /* Close connection socket */
        close(newconn);
        newconn = -1;
        vTaskDelay(1000);
    }
}

/**
 * @brief  网络数据收发管理线程
 * @param  None
 * @retval None
 */
static void EthManageTask(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    (void)pvParameters;

    uint32_t cnti = 0;
    uint32_t RecvIndex = 0;
    uint32_t SendIndex = 0;
    uint8_t recv_ret = 0;
    uint32_t ACK_NAKIndex = 0;
    char str_tmp[EthDataLen] = {0};
    for (;;)
    {
        /*------------------------------------------独占发送缓冲区处理权限-------------------------------------*/
        if (xUserSemaphoreTake(SendMutex, portMAX_DELAY) != pdTRUE) //获取操作权限
        {
            Bit_Set_U32t(&EthErrCode, 0, 1);
            vTaskDelete(NULL);
        }

        /*------------------------------------------接收缓冲区数据处理-------------------------------------*/
        /*获取操作权限*/
        if (xUserSemaphoreTake(RecvMutex, portMAX_DELAY) != pdTRUE)
        {
            Bit_Set_U32t(&EthErrCode, 0, 1);
            vTaskDelete(NULL);
        }

        /*从接收队列获取数据并处理*/
        while (xQueueReceive(RecvList, (void *)&RecvIndex, 0) == pdPASS) //读取接收队列的数据
        {
            /*数据包完整性校验*/
            if ((EthPkgBuf_Recv[RecvIndex].data[0] == 0x02) && (EthPkgBuf_Recv[RecvIndex].data[EthPkgBuf_Recv[RecvIndex].datelen - 1] == 0x03))
            {
                recv_ret = EthRecvDeal(&EthPkgBuf_Recv[RecvIndex]);
            }
            else
            {
                //尝试解析一次ID
                memset((char *)str_tmp, 0, EthDataLen);
                if (EthXmlCmpFind(EthPkgBuf_Recv[RecvIndex].data, "id", str_tmp) == 0)
                {
                    EthPkgBuf_Recv[RecvIndex].cmdid = atoi((char *)str_tmp);
                }
                /*回复NAK，删除当前包*/
                recv_ret = 1; //回NAK
            }

            /*根据返回值确定发送队列的内容，如接收到ACK/NAK，对发送缓冲区cmd相同、ID相同的发送包进行复位，同时将ACK或NAK通知给到相应的线程*/
            if (recv_ret == 0) //回ACK
            {
                WorkRecordReadWrite(0, 0, "INFO ETH Send ACK");
                for (cnti = 0; cnti < EthPkgNum; cnti++)
                {
                    if (EthPkgBuf_Send[cnti].state == 0)
                    {
                        EthPkgBuf_Send[cnti].state = 1;
                        SendIndex = cnti;
                        break;
                    }
                    if (cnti == EthPkgNum - 1)
                    {
                        SendIndex = UINT32_MAX;
                    }
                }
                if (SendIndex != UINT32_MAX)
                {
                    if (EthAckPack(&EthPkgBuf_Send[SendIndex], EthPkgBuf_Recv[RecvIndex].cmdid, EthPkgBuf_Recv[RecvIndex].source) == 1)
                    {
                        EthPkgBagInit(&EthPkgBuf_Send[SendIndex]);
                    }
                }
                if (xQueueSend(SendList, (void *)&SendIndex, 0) != pdPASS) //加入发送队列
                {
                    EthPkgBagInit(&EthPkgBuf_Send[SendIndex]);
                }
            }
            else if (recv_ret == 1) //回NAK
            {
                WorkRecordReadWrite(0, 0, "WARN ETH Send NAK");
                //首先将NAK的包ID从接收ID库中删除，避免上位机重发后包被丢弃
                for (uint32_t i = 0; i < 100; i++)
                {
                    if (RxID[i] == EthPkgBuf_Recv[RecvIndex].cmdid) //找到包id
                    {
                        RxID[i] = UINT32_MAX;
                        break;
                    }
                }
                for (cnti = 0; cnti < EthPkgNum; cnti++)
                {
                    if (EthPkgBuf_Send[cnti].state == 0)
                    {
                        EthPkgBuf_Send[cnti].state = 1;
                        SendIndex = cnti;
                        break;
                    }
                    if (cnti == EthPkgNum - 1)
                    {
                        SendIndex = UINT32_MAX;
                    }
                }
                if (SendIndex != UINT32_MAX)
                {
                    if (EthNakPack(&EthPkgBuf_Send[SendIndex], EthPkgBuf_Recv[RecvIndex].cmdid, EthPkgBuf_Recv[RecvIndex].source) == 1)
                    {
                        EthPkgBagInit(&EthPkgBuf_Send[SendIndex]);
                    }
                }
                if (xQueueSend(SendList, (void *)&SendIndex, 0) != pdPASS) //加入发送队列
                {
                    EthPkgBagInit(&EthPkgBuf_Send[SendIndex]);
                }
            }
            else if (recv_ret == 2) //收到ACK
            {
                // WorkRecordReadWrite(0, 0, "INFO ETH Recv ACK");
                memset((char *)str_tmp, 0, EthDataLen);
                if (EthXmlCmpFind(EthPkgBuf_Recv[RecvIndex].data, "data", str_tmp) == 0)
                {
                    ACK_NAKIndex = atoi((char *)str_tmp);
                    for (cnti = 0; cnti < EthPkgNum; cnti++)
                    {
                        if (EthPkgBuf_Send[cnti].cmdid == ACK_NAKIndex)
                        {
                            if (EthPkgBuf_Send[cnti].dealtask != NULL)
                            {
                                if (xTaskNotify(*EthPkgBuf_Send[cnti].dealtask, RevAck, eSetValueWithOverwrite) != pdTRUE) // 1--收到ACK
                                {
                                    Bit_Set_U32t(&EthErrCode, 0, 1);
                                    vTaskDelete(NULL);
                                }
                            }
                            EthPkgBagInit(&EthPkgBuf_Send[cnti]);
                            break;
                        }
                    }
                }
            }
            else if (recv_ret == 3) // NAK
            {
                WorkRecordReadWrite(0, 0, "WARN ETH Recv NAK");
                memset((char *)str_tmp, 0, EthDataLen);
                if (EthXmlCmpFind(EthPkgBuf_Recv[RecvIndex].data, "data", str_tmp) == 0)
                {
                    ACK_NAKIndex = atoi((char *)str_tmp);
                    for (cnti = 0; cnti < EthPkgNum; cnti++)
                    {
                        if (EthPkgBuf_Send[cnti].cmdid == ACK_NAKIndex)
                        {
                            if (EthPkgBuf_Send[cnti].dealtask != NULL)
                            {
                                if (xTaskNotify(*EthPkgBuf_Send[cnti].dealtask, RevNak, eSetValueWithOverwrite) != pdTRUE) // 2--收到NAK
                                {
                                    Bit_Set_U32t(&EthErrCode, 0, 1);
                                    vTaskDelete(NULL);
                                }
                            }
                            break;
                        }
                    }
                }
            }
            /*处理完成当前接收包进行复位*/
            /*如果接收由线程处理，则由线程负责接受包复位*/
            if (EthPkgBuf_Recv[RecvIndex].dealtask == NULL)
            {
                EthPkgBagInit(&EthPkgBuf_Recv[RecvIndex]);
            }
        }

        if (xUserSemaphoreGive(RecvMutex) != pdTRUE) //释放操作权限
        {
            Bit_Set_U32t(&EthErrCode, 0, 1);
            vTaskDelete(NULL);
        }

        /*------------------------------------------发送缓冲区操作-------------------------------------*/
        /*查找发送缓冲区中待复位（state==4）的NAK或ACK数据包，清空数据*/
        for (cnti = 0; cnti < EthPkgNum; cnti++)
        {
            if ((EthPkgBuf_Send[cnti].ackflag == RevAck) || (EthPkgBuf_Send[cnti].ackflag == RevNak))
            {
                if (EthPkgBuf_Send[cnti].state == 4)
                {
                    EthPkgBagInit(&EthPkgBuf_Send[cnti]);
                }
            }
        }

        /*对发送缓冲区待复位（state==4）的包进行检查，如sendtask为NULL，将该包复位，如sendtask不为NULL，表明线程尚未接收到回应*/
        for (cnti = 0; cnti < EthPkgNum; cnti++)
        {
            if ((EthPkgBuf_Send[cnti].state == 4) && (EthPkgBuf_Send[cnti].dealtask == NULL))
            {
                EthPkgBagInit(&EthPkgBuf_Send[cnti]);
            }
        }

        /*查找发送缓冲区待写入（state==2）的数据包，写入发送队列*/
        for (cnti = 0; cnti < EthPkgNum; cnti++)
        {
            if (EthPkgBuf_Send[cnti].state == 2)
            {
                if (xQueueSend(SendList, (void *)&cnti, 0) == pdPASS) //加入发送队列
                {
                    EthPkgBuf_Send[cnti].state = 3;
                }
            }
        }

        /*------------------------------------------释放发送缓冲区独占-------------------------------------*/
        if (xUserSemaphoreGive(SendMutex) != pdTRUE) //释放操作权限
        {
            Bit_Set_U32t(&EthErrCode, 0, 1);
            vTaskDelete(NULL);
        }
        vTaskDelay(3);
    }
}

/**
 * @brief  返回当前可用的发送缓冲区索引
 * @param
 * @retval 索引值,返回UINT32_MAX时表示数据不可用
 */
uint32_t EthSendIndexGet(void)
{
    for (uint32_t cnti = 0; cnti < EthPkgNum; cnti++)
    {
        if (EthPkgBuf_Send[cnti].state == 0)
        {
            EthPkgBuf_Send[cnti].state = 1;
            return cnti;
        }
        if (cnti == EthPkgNum - 1)
        {
            return UINT32_MAX;
        }
    }
    return UINT32_MAX;
}

/**
 * @brief  获取一个空的句柄索引
 * @param
 * @retval 索引值,返回UINT32_MAX时表示数据不可用
 */
uint32_t EthHandleIndexGet(void)
{
    taskENTER_CRITICAL(); //进入临界区
    for (uint32_t cnti = 0; cnti < EthHandleNum; cnti++)
    {
        if (EthCMDTaskHandle[cnti] == NULL)
        {
            taskEXIT_CRITICAL(); // 退出临界区
            return cnti;
        }
        if (cnti == EthPkgNum - 1)
        {
            taskEXIT_CRITICAL(); // 退出临界区
            return UINT32_MAX;
        }
    }
    taskEXIT_CRITICAL(); // 退出临界区
    return UINT32_MAX;
}

/**
 * @brief  返回当前线程使用句柄的索引
 * @param
 * @retval 索引值,返回UINT32_MAX时表示数据不可用
 */
uint32_t EthCurrentHandleIndex(void)
{
    TaskHandle_t CurrentTaskHandle = xTaskGetCurrentTaskHandle();
    for (uint32_t cnti = 0; cnti < EthHandleNum; cnti++)
    {
        if (CurrentTaskHandle == EthCMDTaskHandle[cnti])
        {
            return cnti;
        }
    }
    return UINT32_MAX;
}

/**
* @brief  删除当前线程，并删除对应句柄库的句柄和接收包/发送包
* @param  索引值,UINT32_MAX时表示数据不可用
recvpkg：接收包，无此项使用NULL
sendpkg：发送包，无此项使用NULL
* @retval None
*/
void EthDeleteCurrentTask(uint32_t handle_idx, EthPkgDef *recvpkg, EthPkgDef *sendpkg)
{
    if (recvpkg != NULL)
    {
        EthPkgBagInit(recvpkg);
    }

    if (sendpkg != NULL)
    {
        EthPkgBagInit(sendpkg);
    }

    if (handle_idx != UINT32_MAX)
    {
        EthCMDTaskHandle[handle_idx] = NULL;
    }
    vTaskDelete(NULL);
}

/**
 * @brief  接收数据包指令处理
 *对接收数据包中的数据进行处理，分析指令，如收到ACK或NAK直接进行自动处理，否则转入EthRecvCmdDeal中
 *处理后，对pkg的cmd和cmdid进行更新
 * @param
 **pkg：等待处理的数据包
 * @retval 0--需要返回ack，1--需要返回nak，2--收到ack，3--接收到nak，4--收到重复包
 */
static uint8_t EthRecvDeal(EthPkgDef *pkg)
{
    char str_tmp[EthDataLen] = {0};
    char logbuf[100] = {0};

    /*提取cmdid------------------------------------------------------------------------------------*/
    memset((char *)str_tmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "id", str_tmp) != 0)
    {
        return 1;
    }
    pkg->cmdid = atoi(str_tmp);
    // ID重复性判断
    for (uint32_t i = 0; i < 20; i++)
    {
#if 0
        if (RxID[i] == pkg->cmdid) //包重复
        {
            memset((char *)logbuf, 0, sizeof(char) * 100);
            sprintf(logbuf, "WARN ETH RXID %u Duplicated, Ignored", pkg->cmdid);
            WorkRecordReadWrite(0, 0, logbuf);

            return 4; //丢弃包
        }
#else
        if (RxID[i] == pkg->cmdid) //包重复
        {
            memset((char *)logbuf, 0, sizeof(char) * 100);
            sprintf(logbuf, "user_ethernet: WARN ETH RXID %u Duplicated, Ignored", pkg->cmdid);
            WorkRecordReadWrite(0, 0, LogBuffer);
            //提取source用于回ACK
            memset((char *)str_tmp, 0, EthDataLen);
            if (EthXmlCmpFind(pkg->data, "source", str_tmp) != 0)
            {
                return 1;
            }
            strcpy((char *)pkg->source, str_tmp);
            memset((char *)LastSource, 0, 25);
            strcpy((char *)LastSource, (char *)str_tmp); //保存最近一次的数据来源
            return 0;                                    //回ACK
        }
#endif
    }
    memset((char *)logbuf, 0, sizeof(char) * 100);
    sprintf(logbuf, "INFO ETH RXID %u", pkg->cmdid);
    WorkRecordReadWrite(0, 0, logbuf);
    RxID[RxIDIdx] = pkg->cmdid;
    RxIDIdx = U32T_LimitAdd(RxIDIdx, 1, 100);

    /*提取source------------------------------------------------------------------------------------*/
    memset((char *)str_tmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "source", str_tmp) != 0)
    {
        return 1;
    }
    strcpy((char *)pkg->source, str_tmp);
    memset((char *)LastSource, 0, 25);
    strcpy((char *)LastSource, (char *)str_tmp); //保存最近一次的数据来源

    /*提取target------------------------------------------------------------------------------------*/
    memset((char *)str_tmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "target", str_tmp) != 0)
    {
        return 1;
    }
    strcpy((char *)pkg->target, str_tmp);

    /*提取cmd------------------------------------------------------------------------------------*/
    memset((char *)str_tmp, 0, EthDataLen);
    if (EthXmlCmpFind(pkg->data, "command", str_tmp) != 0)
    {
        return 1;
    }

    if (strcmp(str_tmp, "ACK") == 0) //收到ACK应答
    {
        pkg->ackflag = 1;
        return 2;
    }
    else if (strcmp(str_tmp, "NAK") == 0) //收到NAK应答
    {
        pkg->ackflag = 2;
        return 3;
    }
    else
    {
        return EthRecvCmdDeal(pkg, str_tmp);
    }
}

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @brief  xml标签匹配查找（查找范围最大不超过EthDataLen）
 * @param
 **data_s：需要查找数据区，以\0结尾，超出查找范围返回1
 **lable：需要查找的标签字符串，如字符串为“root”，则匹配<root>和</root>之间的数据
 **slice_data：返回标签之间的数据，字符串形式，以\0结束，建议最大长度为EthDataLen，如超出最大长度，返回1
 * @retval 0--匹配成功，1--匹配失败，返回值不可用，2--匹配成功，标签之间无数据
 */
uint8_t EthXmlCmpFind(const char *data_s, const char *lable, char *slice_data)
{
    uint32_t lable_len = 0;
    uint32_t data_s_len = 0;
    uint32_t cnti = 0;
    uint32_t cntj = 0;
    uint32_t cntk = 0;
    uint32_t slice_front = 0;
    uint32_t slice_end = 0;

    for (cnti = 0; cnti < EthDataLen; cnti++)
    {
        if (*(data_s + cnti) == 0)
        {
            data_s_len = cnti;
            break;
        }
        if (cnti == EthDataLen - 1)
        {
            return 1;
        }
    }

    for (cnti = 0; cnti < EthDataLen; cnti++)
    {
        if (*(lable + cnti) == 0)
        {
            lable_len = cnti;
            break;
        }
        if (cnti == EthDataLen - 1)
        {
            return 1;
        }
    }

    if ((data_s_len == 0) || (lable_len == 0) || (data_s_len < lable_len))
        return 1;

    for (cnti = 0; cnti < data_s_len; cnti++)
    {
        if ((data_s_len - cnti) < lable_len)
            return 1;

        if (*(data_s + cnti) == '<')
        {
            for (cntj = 0; cntj < lable_len; cntj++)
            {
                if (*(lable + cntj) != *(data_s + cnti + 1 + cntj))
                {
                    break;
                }
            }
            if ((cntj == lable_len) && (*(data_s + cnti + 1 + lable_len) == '>'))
            {
                slice_front = cnti + 1 + lable_len + 1;

                for (cntk = slice_front; cntk < data_s_len; cntk++)
                {
                    if (*(data_s + cntk) == '<')
                    {
                        if ((data_s_len - cntk) < (lable_len + 1))
                            return 1;

                        if (*(data_s + cntk + 1) != '/')
                            continue;

                        for (cntj = 0; cntj < lable_len; cntj++)
                        {
                            if (*(lable + cntj) != *(data_s + cntk + 2 + cntj))
                            {
                                break;
                            }
                        }
                        if ((cntj == lable_len) && (*(data_s + cntk + 2 + lable_len) == '>'))
                        {
                            slice_end = cntk - 1;
                            break;
                        }
                    }
                }
                break;
            }
        }
    }

    if (slice_end < slice_front)
        return 2;

    for (cnti = 0; cnti <= EthDataLen; cnti++)
    {
        *(slice_data + cnti) = *(data_s + slice_front + cnti);
        if (slice_front + cnti == slice_end)
        {
            *(slice_data + cnti + 1) = 0;
            break;
        }
    }

    return 0;
}

/**
* @brief  xml标签匹配添加
*(生成的xml格式不考虑标签间的换行和缩进)
* @param
**data_s：需要查找数据区匹配添加的数据缓冲区，以\0结尾，若无结尾，直接返回1（长度最大不超过EthDataLen）
**lable：需要查找的标签字符串，如字符串为“root”，则匹配<root>和</root>之间的数据
*mode：添加方式， 0--直接在data_s数据区前部添加，忽略*lable；
                  1--直接在data_s数据区后部添加，忽略*lable；
                  2--在标签后添加，如lable为<ID>...</ID>，待添加的为<DATA>...</DATA>，添加后为<ID>...</ID><DATA>...</DATA>；
                  3--在标签前添加，如lable为<ID>...</ID>，待添加的为<DATA>...</DATA>，添加后为<DATA>...</DATA><ID>...</ID>；
                  4--在标签内添加，插入标签前部，如lable为<ID>...</ID>，待添加的为<DATA>...</DATA>，添加后为<ID><DATA>...</DATA>...</ID>；
                  5--在标签内添加，插入标签后部，如lable为<ID>...</ID>，待添加的为<DATA>...</DATA>，添加后为<ID>...<DATA>...</DATA></ID>；
                  6--在标签内添加，替换标签内已有的数据，如lable为<ID>...</ID>，待添加的为<DATA>...</DATA>，添加后为<ID><DATA>...</DATA></ID>；
**slice_data：标签之间的数据，字符串形式，以\0结束，插入后，删除不符合规则的\0
* @retval 0--添加成功，1--添加失败
*/
uint8_t EthXmlCmpAdd(char *data_s, const char *lable, const uint8_t mode, char *slice_data)
{
    uint32_t lable_len = 0;
    uint32_t data_s_len = 0;
    uint32_t slice_len = 0;
    uint32_t cnti = 0;
    uint32_t cntk = 0;
    int_fast32_t data_s_index = 0;
    char data_tmp[EthDataLen] = {0};

    for (cnti = 0; cnti < EthDataLen; cnti++)
    {
        if (*(data_s + cnti) == 0)
        {
            data_s_len = cnti;
            break;
        }
        if (cnti == EthDataLen - 1)
        {
            return 1;
        }
    }

    for (cnti = 0; cnti < EthDataLen; cnti++)
    {
        if (*(lable + cnti) == 0)
        {
            lable_len = cnti;
            break;
        }
        if (cnti == EthDataLen - 1)
        {
            return 1;
        }
    }

    for (cnti = 0; cnti < EthDataLen; cnti++)
    {
        if (*(slice_data + cnti) == 0)
        {
            slice_len = cnti;
            break;
        }
        if (cnti == EthDataLen - 1)
        {
            return 1;
        }
    }

    if (data_s_len + slice_len + 1 > EthDataLen)
    {
        return 1;
    }

    switch (mode)
    {
    case 0:
        strcat(data_tmp, slice_data);
        strcat(data_tmp, data_s);
        break;

    case 1:
        strcat(data_tmp, data_s);
        strcat(data_tmp, slice_data);
        break;

    case 2:
        for (cnti = 0; cnti < data_s_len; cnti++)
        {
            if (*(data_s + cnti) == '<')
            {
                if ((data_s_len - cnti) < (lable_len + 1))
                    return 1;
                if (*(data_s + cnti + 1) != '/')
                    continue;

                for (cntk = 0; cntk < lable_len; cntk++)
                {
                    if (*(lable + cntk) != *(data_s + cnti + 2 + cntk))
                    {
                        break;
                    }
                }
                if ((cntk == lable_len) && (*(data_s + cnti + 2 + lable_len) == '>'))
                {
                    data_s_index = cnti + 2 + lable_len;
                    break;
                }
            }
        }

        strncat(data_tmp, data_s, data_s_index + 1);
        strcat(data_tmp, slice_data);
        strcat(data_tmp, (data_s + data_s_index + 1));

        break;

    case 3:
        for (cnti = 0; cnti < data_s_len; cnti++)
        {
            if (*(data_s + cnti) == '<')
            {
                if ((data_s_len - cnti) < (lable_len + 1))
                    return 1;

                for (cntk = 0; cntk < lable_len; cntk++)
                {
                    if (*(lable + cntk) != *(data_s + cnti + 1 + cntk))
                    {
                        break;
                    }
                }
                if ((cntk == lable_len) && (*(data_s + cnti + 1 + lable_len) == '>'))
                {
                    data_s_index = cnti - 1;
                    break;
                }
            }
        }

        strncat(data_tmp, data_s, data_s_index + 1);
        strcat(data_tmp, slice_data);
        strcat(data_tmp, (data_s + data_s_index + 1));
        break;

    case 4:
        for (cnti = 0; cnti < data_s_len; cnti++)
        {
            if (*(data_s + cnti) == '<')
            {
                if ((data_s_len - cnti) < (lable_len + 1))
                    return 1;

                for (cntk = 0; cntk < lable_len; cntk++)
                {
                    if (*(lable + cntk) != *(data_s + cnti + 1 + cntk))
                    {
                        break;
                    }
                }
                if ((cntk == lable_len) && (*(data_s + cnti + 1 + lable_len) == '>'))
                {
                    data_s_index = cnti + 1 + lable_len;
                    break;
                }
            }
        }

        strncat(data_tmp, data_s, data_s_index + 1);
        strcat(data_tmp, slice_data);
        strcat(data_tmp, (data_s + data_s_index + 1));
        break;

    case 5:
        for (cnti = 0; cnti < data_s_len; cnti++)
        {
            if (*(data_s + cnti) == '<')
            {
                if ((data_s_len - cnti) < (lable_len + 1))
                    return 1;
                if (*(data_s + cnti + 1) != '/')
                    continue;

                for (cntk = 0; cntk < lable_len; cntk++)
                {
                    if (*(lable + cntk) != *(data_s + cnti + 2 + cntk))
                    {
                        break;
                    }
                }
                if ((cntk == lable_len) && (*(data_s + cnti + 2 + lable_len) == '>'))
                {
                    data_s_index = cnti - 1;
                    break;
                }
            }
        }

        strncat(data_tmp, data_s, data_s_index + 1);
        strcat(data_tmp, slice_data);
        strcat(data_tmp, (data_s + data_s_index + 1));
        break;

    case 6:
        for (cnti = 0; cnti < data_s_len; cnti++)
        {
            if (*(data_s + cnti) == '<')
            {
                if ((data_s_len - cnti) < (lable_len + 1))
                    return 1;

                for (cntk = 0; cntk < lable_len; cntk++)
                {
                    if (*(lable + cntk) != *(data_s + cnti + 1 + cntk))
                    {
                        break;
                    }
                }
                if ((cntk == lable_len) && (*(data_s + cnti + 1 + lable_len) == '>'))
                {
                    data_s_index = cnti + 1 + lable_len;
                    break;
                }
            }
        }

        strncat(data_tmp, data_s, data_s_index + 1);
        strcat(data_tmp, slice_data);
        for (cnti = 0; cnti < data_s_len; cnti++)
        {
            if (*(data_s + cnti) == '<')
            {
                if ((data_s_len - cnti) < (lable_len + 1))
                    return 1;
                if (*(data_s + cnti + 1) != '/')
                    continue;

                for (cntk = 0; cntk < lable_len; cntk++)
                {
                    if (*(lable + cntk) != *(data_s + cnti + 2 + cntk))
                    {
                        break;
                    }
                }
                if ((cntk == lable_len) && (*(data_s + cnti + 2 + lable_len) == '>'))
                {
                    data_s_index = cnti - 1;
                    break;
                }
            }
        }

        strcat(data_tmp, (data_s + data_s_index + 1));
        break;

    default:
        return 1;
    }

    for (cnti = 0; cnti < EthDataLen; cnti++)
    {
        *(data_s + cnti) = *(data_tmp + cnti);
        if (*(data_tmp + cnti) == 0)
        {
            break;
        }
    }
    return 0;
}

/**
* @brief  数据字段添加标签
* @param
**data_s：需要添加标签的数据字段，长度最大不超过EthDataLen，以\0结尾，添加标签后，\0移位到标签后
**lable：需要被添加的标签，以\0结尾

* @retval 0--添加成功，1--添加失败
*/
uint8_t EthXmlAddLable(char *data_s, const char *lable)
{
    uint32_t lable_len = 0;
    uint32_t data_s_len = 0;
    uint32_t cnti = 0;
    char data_tmp[EthDataLen] = {0};

    for (cnti = 0; cnti < EthDataLen; cnti++)
    {
        if (*(data_s + cnti) == 0)
        {
            data_s_len = cnti;
            break;
        }
        if (cnti == EthDataLen - 1)
        {
            return 1;
        }
    }

    for (cnti = 0; cnti < EthDataLen; cnti++)
    {
        if (*(lable + cnti) == 0)
        {
            lable_len = cnti;
            break;
        }
        if (cnti == EthDataLen - 1)
        {
            return 1;
        }
    }

    if (data_s_len + 2 * lable_len + 5 + 1 > EthDataLen)
    {
        return 1;
    }

    /*标签添加*/
    strcat(data_tmp, "<");
    strcat(data_tmp, lable);
    strcat(data_tmp, ">");
    strcat(data_tmp, data_s);
    strcat(data_tmp, "<");
    strcat(data_tmp, "/");
    strcat(data_tmp, lable);
    strcat(data_tmp, ">");

    for (cnti = 0; cnti < EthDataLen; cnti++)
    {
        *(data_s + cnti) = *(data_tmp + cnti);
        if (*(data_tmp + cnti) == 0)
        {
            break;
        }
    }
    return 0;
}

/**
* @brief  数据包编码
将数据包增加包头和包尾，增加分隔符和校验字段，删除data_s结尾的\0
* @param
**data_s：需要打包的数据，以\0结尾
*data_exp_len：打包完成后数据包长度，不含\0
* @retval 0--打包成功，1--打包失败
*/
uint8_t EthPkgEncode(const char *data_s, uint32_t *data_exp_len)
{
    uint32_t data_s_len = 0;
    uint32_t data_exp_len_tmp = 0;
    uint32_t cnti = 0;
    char data_tmp[EthDataLen] = {0};

    for (cnti = 0; cnti < EthDataLen; cnti++)
    {
        if (*(data_s + cnti) == 0)
        {
            data_s_len = cnti;
            break;
        }
        if (cnti == EthDataLen - 1)
        {
            return 1;
        }
    }

    if (data_s_len + 8 > EthDataLen)
        return 1;

    *data_tmp = 0x02;
    *(data_tmp + 1) = 0;
    strcat(data_tmp, data_s);
    data_exp_len_tmp = data_s_len + 1;
    *(data_tmp + data_exp_len_tmp) = 0x17;
    *(data_tmp + data_exp_len_tmp + 1) = 'a';
    *(data_tmp + data_exp_len_tmp + 2) = 'b';
    *(data_tmp + data_exp_len_tmp + 3) = 'c';
    *(data_tmp + data_exp_len_tmp + 4) = 'd';
    *(data_tmp + data_exp_len_tmp + 5) = 0x03;
    *(data_tmp + data_exp_len_tmp + 6) = 0;
    *data_exp_len = data_exp_len_tmp + 6;

    memcpy((char *)data_s, data_tmp, *data_exp_len);

    return 0;
}

/**
* @brief  数据包解码
删除数据包的包头、包尾、校验字段、分隔符，在xml文件最后增加\0
* @param
**data_s：需要解码的数据
*data_s_len：需要解码的数据长度
**data_exp：解码后的数据包，以\0结尾
* @retval 0--打包成功，1--打包失败
*/
uint8_t EthPkgDecode(const char *data_s, const uint32_t data_s_len, char *data_exp)
{
    uint32_t cnti = 0;

    /*数据合法性校验*/
    if (data_s_len < 7)
    {
        return 1;
    }
    if ((*data_s != 0x02) || (*(data_s + data_s_len - 1) != 0x03) || (*(data_s + data_s_len - 2) != 'd') || (*(data_s + data_s_len - 3) != 'c') || (*(data_s + data_s_len - 4) != 'b') || (*(data_s + data_s_len - 5) != 'a') || (*(data_s + data_s_len - 6) != 0x17))
    {
        return 1;
    }

    /*数据包解码*/
    for (cnti = 0; cnti < data_s_len - 7; cnti++)
    {
        *(data_exp + cnti) = *(data_s + 1 + cnti);
    }

    *(data_exp + cnti) = 0;

    return 0;
}
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
/**
 * @brief  生成一个ACK包
 * @param
 * @retval 0--打包成功，1--打包失败
 */
static uint8_t EthAckPack(EthPkgDef *pkg, uint32_t re_cmd_id, char *target)
{
    char strtmp[EthDataLen] = {0};

    pkg->ackflag = RevAck;
    pkg->cmdid = TxID++;
    pkg->dealtask = NULL;
    strcpy((char *)pkg->source, (char *)EthSource);
    strcpy((char *)pkg->target, (char *)target);
    strcpy((char *)pkg->data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>ACK</command>\n\
  <priority>9</priority>\n\
  <data></data>\n\
</root>");

    sprintf((char *)strtmp, "%d", pkg->cmdid);
    if (EthXmlCmpAdd(pkg->data, "id", 4, strtmp) == 1)
    {
        return 1;
    }
    if (EthXmlCmpAdd(pkg->data, "source", 4, EthSource) == 1)
    {
        return 1;
    }
    if (EthXmlCmpAdd(pkg->data, "target", 4, target) == 1)
    {
        return 1;
    }

    memset((char *)strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%d", re_cmd_id);
    if (EthXmlCmpAdd(pkg->data, "data", 4, strtmp) == 1)
    {
        return 1;
    }

    if (EthPkgEncode(pkg->data, &pkg->datelen) == 1)
    {
        return 1;
    }

    pkg->state = 2;
    return 0;
}

/**
 * @brief  生成一个NAK包
 * @param
 * @retval 0--打包成功，1--打包失败
 */
static uint8_t EthNakPack(EthPkgDef *pkg, uint32_t re_cmd_id, char *target)
{
    char strtmp[EthDataLen] = {0};

    pkg->ackflag = RevNak;
    pkg->cmdid = TxID++;
    pkg->dealtask = NULL;
    strcpy((char *)pkg->source, (char *)EthSource);
    strcpy((char *)pkg->target, (char *)target);
    strcpy((char *)pkg->data, "\
<root>\n\
  <version>1</version>\n\
  <id></id>\n\
  <source></source>\n\
  <target></target>\n\
  <command>NAK</command>\n\
  <priority>9</priority>\n\
  <data></data>\n\
</root>");

    sprintf((char *)strtmp, "%d", pkg->cmdid);
    if (EthXmlCmpAdd(pkg->data, "id", 4, strtmp) == 1)
    {
        return 1;
    }
    if (EthXmlCmpAdd(pkg->data, "source", 4, EthSource) == 1)
    {
        return 1;
    }
    if (EthXmlCmpAdd(pkg->data, "target", 4, target) == 1)
    {
        return 1;
    }
    memset((char *)strtmp, 0, EthDataLen);
    sprintf((char *)strtmp, "%d", re_cmd_id);
    if (EthXmlCmpAdd(pkg->data, "data", 4, strtmp) == 1)
    {
        return 1;
    }

    if (EthPkgEncode(pkg->data, &pkg->datelen) == 1)
    {
        return 1;
    }

    pkg->state = 2;
    return 0;
}

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @brief  网络初始化
 * @param  None
 * @retval None
 */
void UserEthernetInit(void)
{
    uint32_t cnti = 0;
    uint8_t str1[25] = {0};

    /*初始化参数*/
    for (cnti = 0; cnti < EthPkgNum; cnti++)
    {
        EthPkgBagInit(&EthPkgBuf_Send[cnti]);
        EthPkgBagInit(&EthPkgBuf_Recv[cnti]);
    }

    /*初始化接收和发送管理互斥量*/
    SendMutex = xSemaphoreCreateMutex();
    RecvMutex = xSemaphoreCreateMutex();
    if ((SendMutex == NULL) || (RecvMutex == NULL))
    {
        Bit_Set_U32t(&EthErrCode, 0, 1);
        vTaskDelete(NULL);
    }

    /*初始化接收和发送队列*/
    SendList = xQueueCreate(EthPkgNum, sizeof(uint32_t));
    RecvList = xQueueCreate(EthPkgNum, sizeof(uint32_t));

    if ((SendList == NULL) || (RecvList == NULL))
    {
        Bit_Set_U32t(&EthErrCode, 0, 1);
        vTaskDelete(NULL);
    }

    /*初始化自身IP标示字段*/
    sprintf((char *)str1, "%d", IP_ADDR0);
    strcat((char *)EthSource, (char *)str1);
    strcat((char *)EthSource, ".");
    sprintf((char *)str1, "%d", IP_ADDR1);
    strcat((char *)EthSource, (char *)str1);
    strcat((char *)EthSource, ".");
    sprintf((char *)str1, "%d", IP_ADDR2);
    strcat((char *)EthSource, (char *)str1);
    strcat((char *)EthSource, ".");
    sprintf((char *)str1, "%d", IP_ADDR3);
    strcat((char *)EthSource, (char *)str1);
    strcat((char *)EthSource, ":");
    sprintf((char *)str1, "%d", EthComPort);
    strcat((char *)EthSource, (char *)str1);

    /* Create tcp_ip stack thread */
    tcpip_init(NULL, NULL);

    /* Initialize the LwIP stack */
    Netif_Config();

#ifdef __USE_TFTP
    /* Initialize the TFTP server */
    tftpd_init();
#endif

    /*服务器线程启动*/
    // xUserTaskCreate(TcpIpServerTask, "TCPServer", configMINIMAL_STACK_SIZE * 1, NULL, TCPECHO_THREAD_PRIO, (TaskHandle_t *)&TCPIPTaskHandle);
    sys_thread_new("TCPServer", TcpIpServerTask, NULL, (configMINIMAL_STACK_SIZE), TCPECHO_THREAD_PRIO);
}

/*-----------------------------------------------------------------------------------*/
