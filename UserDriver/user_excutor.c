/**
******************************************************************************
* @文件    user_excutor.C
* @作者    GENGXU
* @版本    V0.0.1
******************************************************************************
*执行机构控制函数，包括泵、阀、注射器、步进电机、直流电机、电磁抓手




******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "user_excutor.h"
#include "user_app.h"
#include "user_can.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile ChainInStruDef ChainIn;
volatile ChainBackStruDef ChainBack;
extern volatile TableStruDef Table;


#ifndef __BoardVersion02 //Version03 or later
uint32_t MainSenGPIOPin[] = {GPIO_PIN_5, GPIO_PIN_5, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_14, GPIO_PIN_12, GPIO_PIN_8};
GPIO_TypeDef *MainSenGPIOPort[] = {GPIOA, GPIOK, GPIOI, GPIOI, GPIOI, GPIOJ, GPIOA};

uint32_t MainLedGPIOPIN[] = {GPIO_PIN_7, GPIO_PIN_6};
GPIO_TypeDef *MainLedGPIOPort[] = {GPIOB, GPIOB};

uint32_t MainDirGPIOPIN[] = {GPIO_PIN_6, GPIO_PIN_0};
GPIO_TypeDef *MainDirGPIOPort[] = {GPIOA, GPIOA};

#else //Version02
uint32_t MainSenGPIOPin[] = {GPIO_PIN_6, GPIO_PIN_0, GPIO_PIN_0, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_6, GPIO_PIN_4};
GPIO_TypeDef *MainSenGPIOPort[] = {GPIOA, GPIOA, GPIOB, GPIOA, GPIOA, GPIOB, GPIOH};

uint32_t MainLedGPIOPIN[] = {GPIO_PIN_7, GPIO_PIN_0};
GPIO_TypeDef *MainLedGPIOPort[] = {GPIOC, GPIOC};

uint32_t MainDirGPIOPIN[] = {GPIO_PIN_0};
GPIO_TypeDef *MainDirGPIOPort[] = {GPIOK};
#endif
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void LED_Set(uint8_t LEDID, uint8_t LED_Status)
{
    if (LED_Status == 0) //灭
    {
        HAL_GPIO_WritePin(MainLedGPIOPort[LEDID], MainLedGPIOPIN[LEDID], GPIO_PIN_RESET);
    }
    else if (LED_Status == 1) //亮
    {
        HAL_GPIO_WritePin(MainLedGPIOPort[LEDID], MainLedGPIOPIN[LEDID], GPIO_PIN_SET);
    }
    else //翻转
    {
        HAL_GPIO_TogglePin(MainLedGPIOPort[LEDID], MainLedGPIOPIN[LEDID]);
    }
}








uint8_t MAIN_EXIN_Get(uint8_t sensorid)
{
    return HAL_GPIO_ReadPin(MainSenGPIOPort[sensorid], MainSenGPIOPin[sensorid]);
}

void MAIN_EXIN_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOJ_CLK_ENABLE();
    __HAL_RCC_GPIOK_CLK_ENABLE();

    for (uint8_t i = 0; i < 7; i++)
    {
        GPIO_InitStruct.Pin = MainSenGPIOPin[i];
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

        HAL_GPIO_Init(MainSenGPIOPort[i], &GPIO_InitStruct);
    }

    for (uint8_t i = 0; i < 2; i++)
    {
        HAL_GPIO_WritePin(MainLedGPIOPort[i], MainLedGPIOPIN[i], GPIO_PIN_RESET);
        GPIO_InitStruct.Pin = MainLedGPIOPIN[i];
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

        HAL_GPIO_Init(MainLedGPIOPort[i], &GPIO_InitStruct);
    }

    HAL_GPIO_WritePin(MainDirGPIOPort[0], MainDirGPIOPIN[0], GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = MainDirGPIOPIN[0];
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_Init(MainDirGPIOPort[0], &GPIO_InitStruct);

#ifndef __BoardVersion02
    HAL_GPIO_WritePin(MainDirGPIOPort[1], MainDirGPIOPIN[1], GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = MainDirGPIOPIN[1];
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_Init(MainDirGPIOPort[1], &GPIO_InitStruct);
#endif
}




/*****************************END OF FILE*****************************/
