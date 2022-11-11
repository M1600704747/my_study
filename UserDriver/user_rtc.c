/**
******************************************************************************
* @文件    RTC.C
* @作者    
* @版本    V0.0.1
* @日期    
* @摘要    
******************************************************************************
* @attention
*
*
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "user_rtc.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint8_t BQ32002_AW=0xD0;
const uint8_t BQ32002_AR=0xD1;
#define FAC_us          400//单位Mhz

TimeStructDef RTC_Time;


/* Private function prototypes -----------------------------------------------*/
static void I2C_Init(void);
static void I2C_START(void);
static void I2C_STOP(void);
static uint8_t I2C_WAIT_ACK(const uint16_t  Timeout);
static void I2C_ACK(void);
static void I2C_NACK(void);
static void I2C_Send_Byte(uint8_t data_tx);
static uint8_t I2C_Read_Byte(const uint8_t ack_status);
static void I2C_Delay_us(uint32_t  n_us);
static void SCL_LOW(void);
static void SCL_HIGH(void);
static void SDA_LOW(void);
static void SDA_HIGH(void);
static void SDA_IN(void);
static void SDA_OUT(void);
static GPIO_PinState Read_SDA(void);
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  I2C延时函数
  * @param  None
  * @retval None
  */
static void I2C_Delay_us(uint32_t  n_us)
{		
	uint32_t ticks=0;
	uint32_t t_old=0;
  uint32_t t_new=0;
  uint32_t t_cnt=0;
	uint32_t reload=SysTick->LOAD;				//LOAD的值	    	 
  
	ticks=n_us*FAC_us; 						//需要的节拍数 
	t_old=SysTick->VAL;        				//刚进入时的计数器值
  
	while(1)
	{
		t_new=SysTick->VAL;	
		if(t_new!=t_old)
		{	    
			if(t_new<t_old)
      {
        t_cnt+=(t_old-t_new);	//这里注意一下SYSTICK是一个递减的计数器就可以了.
      }
			else 
      {
        t_cnt+=(reload-t_new+t_old);	    
      }
			t_old=t_new;
			if(t_cnt>=ticks)
      {
        break;			//时间超过/等于要延迟的时间,则退出.
      }
		}  
	}
}  

/**
  * @brief  I2C输入输出端口设置
  * @param  None
  * @retval None
  */
static void SCL_LOW(void)
{
  HAL_GPIO_WritePin(RTC_SCL_GPIO_Port,RTC_SCL_Pin,GPIO_PIN_RESET);
}

static void SCL_HIGH(void)
{
  HAL_GPIO_WritePin(RTC_SCL_GPIO_Port,RTC_SCL_Pin,GPIO_PIN_SET);
}

static void SDA_LOW(void)
{
  HAL_GPIO_WritePin(RTC_SDA_GPIO_Port,RTC_SDA_Pin,GPIO_PIN_RESET);
}

static void SDA_HIGH(void)
{
  HAL_GPIO_WritePin(RTC_SDA_GPIO_Port,RTC_SDA_Pin,GPIO_PIN_SET);
}

static void SDA_IN(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  GPIO_InitStruct.Pin =      RTC_SDA_Pin;
  GPIO_InitStruct.Mode =     GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull =     GPIO_NOPULL;
  GPIO_InitStruct.Speed =    GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RTC_SDA_GPIO_Port, &GPIO_InitStruct);
}

static void SDA_OUT(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  GPIO_InitStruct.Pin =      RTC_SDA_Pin;
  GPIO_InitStruct.Mode =     GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull =     GPIO_NOPULL;
  GPIO_InitStruct.Speed =    GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RTC_SDA_GPIO_Port, &GPIO_InitStruct);
}

static GPIO_PinState Read_SDA(void)
{
  return HAL_GPIO_ReadPin(RTC_SDA_GPIO_Port,RTC_SDA_Pin);
}

/**
  * @brief  USER_I2C1_Init
  * @param  None
  * @retval None
  */
static void I2C_Init(void)
{ 
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*初始化输出端口*/
	__HAL_RCC_GPIOF_CLK_ENABLE();
  /* 配置SDA引脚 */
  GPIO_InitStruct.Pin =      RTC_SDA_Pin;
  GPIO_InitStruct.Mode =     GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull =     GPIO_NOPULL;
  GPIO_InitStruct.Speed =    GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RTC_SDA_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(RTC_SDA_GPIO_Port, RTC_SDA_Pin, GPIO_PIN_SET);
  
  /*配置SCL引脚*/
  GPIO_InitStruct.Pin =      RTC_SCL_Pin;
  GPIO_InitStruct.Mode =     GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull =     GPIO_NOPULL;
  GPIO_InitStruct.Speed =    GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RTC_SCL_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(RTC_SCL_GPIO_Port, RTC_SCL_Pin, GPIO_PIN_SET);
}

/**
  * @brief  产生起始信号
  * @param  None
  * @retval None
  */
static void I2C_START(void)
{
  SDA_OUT();
  SDA_HIGH();
  SCL_HIGH();
  I2C_Delay_us(4);
  SDA_LOW();//START:when CLK is high,DATA change form high to low 
  I2C_Delay_us(4);
  SCL_LOW();//钳住I2C总线，准备发送或接收数据 
}

/**
  * @brief  产生停止信号
  * @param  None
  * @retval None
  */
static void I2C_STOP(void)
{
  SDA_OUT();//sda线输出
  SCL_LOW();
  SDA_LOW();//STOP:when CLK is high DATA change form low to high
  I2C_Delay_us(4);
  SCL_HIGH();
  SDA_HIGH();//发送I2C总线结束信号
  I2C_Delay_us(4);
}

/**
  * @brief  等待应答信号到来
  * @param  None
  * @retval 0，接收应答失败
            1，接收应答成功
  */
static uint8_t I2C_WAIT_ACK(const uint16_t  Timeout)
{
  uint16_t  ERR_TIME=0;
  
  SDA_IN();//SDA设置为输入  
  SDA_HIGH();
  I2C_Delay_us(1);
  SCL_HIGH();
  I2C_Delay_us(1);
  while(Read_SDA())
  {
    ERR_TIME++;
    if(ERR_TIME>Timeout)
    {
      I2C_STOP();
      return 0;
    }
  }
  SCL_LOW();//时钟输出0 	   
  return 1;
}

/**
  * @brief  产生应答信号
  * @param  None
  * @retval None
  */
static void I2C_ACK(void)
{
  SCL_LOW();
  SDA_OUT();
  SDA_LOW();
  I2C_Delay_us(2);
  SCL_HIGH();
  I2C_Delay_us(2);
  SCL_LOW();
}

/**
* @brief  不产生应答信号
  * @param  None
  * @retval None
  */
static void I2C_NACK(void)
{
  SCL_LOW();
  SDA_OUT();
  SDA_HIGH();
  I2C_Delay_us(2);
  SCL_HIGH();
  I2C_Delay_us(2);
  SCL_LOW();
}

/**
* @brief  发送一个字节
  * @param  None
  * @retval None
  */
static void I2C_Send_Byte(uint8_t data_tx)
{
  uint8_t counti;
  
  SDA_OUT();
  SCL_LOW();//拉低时钟开始数据传输
  for(counti=0;counti<8;counti++)
  {
    if((data_tx&0x80)>>7)
    {
      SDA_HIGH();
    }
    else
    {
      SDA_LOW();
    }
    data_tx<<=1;
    I2C_Delay_us(2);
    SCL_HIGH();
    I2C_Delay_us(2);
    SCL_LOW();
    I2C_Delay_us(2);
  }
}

/**
* @brief  读取一个字节
* @param  1：发送ack，0：不发送ack
  * @retval 读取的字节
  */
static uint8_t I2C_Read_Byte(const uint8_t ack_status)
{
  uint8_t counti=0;
  uint8_t data_rx=0;
  
  SDA_IN();//SDA设置为输入
  for(counti=0;counti<8;counti++)
  {
    SCL_LOW();
    I2C_Delay_us(2);
    SCL_HIGH();
    data_rx<<=1;
    if(Read_SDA())
    {
      data_rx++;
    }
    I2C_Delay_us(1);
  }
  if(ack_status)
  {
    I2C_ACK();
  }
  else
  {
    I2C_NACK();
  }
  return data_rx;
}

/**
* @brief  RTC寄存器读取
* @param  addr：地址
* @retval 读取的字节
  */
uint8_t RTC_Register_Read(uint8_t addr)
{
	uint8_t data;
	I2C_START();
	I2C_Send_Byte(BQ32002_AW);
	if(I2C_WAIT_ACK(10000)==0) 
  {
    return 0;
  }
  I2C_Send_Byte(addr);
  if(I2C_WAIT_ACK(10000)==0) 
  {
    return 0;
  }
	I2C_START();
  I2C_Send_Byte(BQ32002_AR);
  if(I2C_WAIT_ACK(10000)==0) 
  {
    return 0;
  }
	
	data = I2C_Read_Byte(0);
  I2C_STOP();
	return data;
}

/**
* @brief  RTC寄存器写入
* @param  addr：地址，data：数据
* @retval None
  */
void RTC_Register_Write(uint8_t addr,uint8_t data)
{
	I2C_START();
	I2C_Send_Byte(BQ32002_AW);
	if(I2C_WAIT_ACK(10000)==0) 
  {
    return;
  }
  I2C_Send_Byte(addr);
  if(I2C_WAIT_ACK(10000)==0) 
  {
    return;
  }
  I2C_Send_Byte(data);
  if(I2C_WAIT_ACK(10000)==0) 
  {
    return;
  }
	I2C_STOP();
}

/**
* @brief  RTC初始化
* @param  None
* @retval 0成功
  */
uint8_t RTC_Init(void)
{
	I2C_Init();
	return 0;
}

/**
* @brief  RTC时间读取
* @param  None
* @retval 0成功，1失败
  */
uint8_t RTC_Time_Read(void)
{
  uint8_t temp;
  temp =  RTC_Register_Read(RTC_YEAR);
  RTC_Time.year = ((temp>>4)&0x0f)*10+(temp&0x0f);
  if(RTC_Time.year > 200 )
  {
    return 1;
  }

  temp =  RTC_Register_Read(RTC_MONTH);
  RTC_Time.month = ((temp>>4)&0x01)*10+(temp&0x0f);
  if(RTC_Time.month > 12 ||  RTC_Time.month < 1)
  {
    return 1;
  }

  temp =  RTC_Register_Read(RTC_DATE);
  RTC_Time.day = ((temp>>4)&0x03)*10+(temp&0x0f);
  if(RTC_Time.day > 31 ||  RTC_Time.day < 1)
  {
    return 1;
  }  

  temp =  RTC_Register_Read(RTC_CENT_HOURS);
  RTC_Time.hour = ((temp>>4)&0x03)*10+(temp&0x0f);
  if(RTC_Time.hour > 23)
  {
    return 1;
  } 

  temp =  RTC_Register_Read(RTC_MINUTES);
  RTC_Time.minute = ((temp>>4)&0x07)*10+(temp&0x0f);
  if(RTC_Time.minute > 59)
  {
    return 1;
  }   
 
  temp =  RTC_Register_Read(RTC_SECONDS) & 0x7f;
  RTC_Time.second = ((temp>>4)&0x07)*10+(temp&0x0f);
  if(RTC_Time.second > 59)
  {
    return 1;
  } 

  SysTime.year = RTC_Time.year;
  SysTime.month = RTC_Time.month;
  SysTime.day = RTC_Time.day;
  SysTime.hour = RTC_Time.hour;
  SysTime.minute = RTC_Time.minute;
  SysTime.second = RTC_Time.second;

  if (SysTime.year > 33 || SysTime.year < 20)
  {
    SysTime.year = PROGRAMYEAR % 200;
    SysTime.month = PROGRAMMONTH;
    SysTime.day = PROGRAMDAY;
    SysTime.hour = PROGRAMHOUR;
    SysTime.minute = PROGRAMMINTER;//atol((char *)%60);//;  //  motorname = atol((char *)strtmp);
    SysTime.second = PROGRAMSECOND;

    RTC_Time_Write();
  }
  
  return 0;
}

/**
* @brief  RTC时间写入
* @param  None
* @retval 0成功
  */
uint8_t RTC_Time_Write(void)
{
  uint8_t temp = 0;

  temp = RTC_Register_Read(RTC_SECONDS) & 0x80;
  temp |= (((SysTime.second)/10)<<4) + (SysTime.second%10);
  RTC_Register_Write(RTC_SECONDS,temp);

  temp = RTC_Register_Read(RTC_MINUTES) & 0x80;
  temp |= (((SysTime.minute)/10)<<4) + (SysTime.minute%10);
  RTC_Register_Write(RTC_MINUTES,temp);

  temp = RTC_Register_Read(RTC_CENT_HOURS) & 0xc0;
  temp |= (((SysTime.hour)/10)<<4) + (SysTime.hour%10);  
  RTC_Register_Write(RTC_CENT_HOURS,temp);

  temp = RTC_Register_Read(RTC_DATE) & 0xc0;
  temp |= (((SysTime.day)/10)<<4) + (SysTime.day%10);  
  RTC_Register_Write(RTC_DATE,temp);

  temp = RTC_Register_Read(RTC_MONTH) & 0xe0;
  temp |= (((SysTime.month)/10)<<4) + (SysTime.month%10);  
  RTC_Register_Write(RTC_MONTH,temp);

  temp = 0;
  temp |= (((SysTime.year)/10)<<4) + (SysTime.year%10);  
  RTC_Register_Write(RTC_YEAR,temp);

  return 0;
}


/*****************************END OF FILE*****************************/


