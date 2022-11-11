/**
******************************************************************************
* @文件    user_can.H
* @作者    GENGXU
* @版本    V0.0.1
******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_CAN_H
#define __USER_CAN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "user_gpm.h"
#include "cmsis_os.h"
/* Private types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor FDCANx instance used and associated
   resources */
/* Definition for FDCANx clock resources */
#define FDCANx FDCAN1
#define FDCANx_CLK_ENABLE() __HAL_RCC_FDCAN_CLK_ENABLE()
#define FDCANx_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define FDCANx_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

#define FDCANx_FORCE_RESET() __HAL_RCC_FDCAN_FORCE_RESET()
#define FDCANx_RELEASE_RESET() __HAL_RCC_FDCAN_RELEASE_RESET()

/* Definition for FDCANx Pins */
#define FDCANx_TX_PIN GPIO_PIN_9
#define FDCANx_TX_GPIO_PORT GPIOB
#define FDCANx_TX_AF GPIO_AF9_FDCAN1
#define FDCANx_RX_PIN GPIO_PIN_8
#define FDCANx_RX_GPIO_PORT GPIOB
#define FDCANx_RX_AF GPIO_AF9_FDCAN1

/* Definition for FDCANx's NVIC IRQ and IRQ Handlers */
#define FDCANx_IRQn FDCAN1_IT0_IRQn
#define FDCANx_IRQHandler FDCAN1_IT0_IRQHandler

#define FDCANTryCnt 10

   /* Exported macro ------------------------------------------------------------*/
   /* Exported variables ------------------------------------------------------------*/

   /* Exported functions ------------------------------------------------------- */
   uint8_t FDCAN1_Moudle_Init(void);
   uint8_t FDCAN1_Communicate(const uint32_t Timeout, uint32_t *CommState, const uint32_t TargetID,
                              const uint8_t TxLen, uint8_t *TxBuf, uint8_t *RxLen, uint8_t *RxBuf);


#endif
   /*****************************END OF FILE*****************************/
