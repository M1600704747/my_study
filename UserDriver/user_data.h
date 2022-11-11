/**
******************************************************************************
* @文件    user_data.H
* @作者    GENGXU
* @版本    V0.0.1
******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_DATA_H
#define __USER_DATA_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "user_gpm.h"
#include "../Tasks/user_motor_api.h"


    /* Private types ------------------------------------------------------------*/
    /* Exported constants --------------------------------------------------------*/
    /* Exported macro ------------------------------------------------------------*/
    /* Exported variables ------------------------------------------------------------*/
    /* Exported functions ------------------------------------------------------- */
    uint8_t UserFlashCheck(void);
    uint8_t UserFwInfoRW(uint8_t rw);
    uint8_t UserEthernetInfoRW(uint8_t rw);
    uint8_t UserMacInfoRW(uint8_t rw);
    uint8_t UserParamInfoRW(uint8_t rw);
    

#endif
    /*****************************END OF FILE*****************************/
