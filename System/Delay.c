#include "stm32f10x.h"

uint32_t Count;

/**
  * @brief	滴答计数，在需要更换的时基(定时器)中调用,
  * @param  无
  * @retval 无
  */

void Delay_Tick(void)
{	
	Count++;
}


/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
//void Delay_us(uint32_t xus)
//{
//	SysTick->LOAD = 72 * xus;				//设置定时器重装值
//	SysTick->VAL = 0x00;					//清空当前计数值
//	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
//	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
//	SysTick->CTRL = 0x00000004;				//关闭定时器
//}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{	
	uint32_t Start=Count;
	
	while(Count-Start<xms){}

//	xms=Count+xms;
//	while(Count<xms)
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 
