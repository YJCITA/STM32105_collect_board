/**
  ******************************************************************************
  * @file    	systick.c
  * @author  	Stone
  * @version 	V0.1
  * @date    	2015-09-08
  * @brief   	
  * 
  ******************************************************************************
***/	
	
//===============================================================================
// 													包含头文件																	 
//===============================================================================
//#include "includes.h"
#include "stm32f10x.h"
#include "systick.h"


//===============================================================================
// 													全局变量声明 																	
//===============================================================================



//===============================================================================
// 													本地全局变量声明 																	
//===============================================================================
static volatile uint32_t sysTickMillis = 0;
static uint32_t sysTickPerUs = 72;// 本地晶振
static void (*systickUserCallback)(void);

//===============================================================================
// 													私有函数原型声明 																	
//===============================================================================
static __INLINE int systick_check_underflow(void);





/*****************************************************************************
*	Function:		static __INLINE int systick_check_underflow(void)
*	Input:
*			
*	Output:
*			
*	Describe:
*			
*
******************************************************************************/
static __INLINE int systick_check_underflow(void)
{
    return SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk;
}

/*****************************************************************************
*	Function:		extern uint32_t micros(void)
*	Input:
*			
*	Output:
*			
*	Describe:
*		返回以us为单位的时间计数值		
*
******************************************************************************/
extern uint32_t micros(void)
{
	uint32_t cycle, timeMs;
	
	do
	{
        timeMs 	= sysTickMillis;
        cycle 	= SysTick->VAL;
        __ASM volatile("nop");
        __ASM volatile("nop");
        __ASM volatile("nop");
        __ASM volatile("nop");
	}
	while( timeMs != sysTickMillis );
	
	return (timeMs * 1000) + (SysTick->LOAD + 1 - cycle) / sysTickPerUs;
}

/*****************************************************************************
*	Function:		extern double GetSystickCounterValudeRAW(void )
*	Input:
*			
*	Output: us
*			
*	Describe:
*		返回	
*
******************************************************************************/
extern double GetSystickCounterValudeRAW(void )
{
	float cycle, timeMs;
	
	do
	{
			timeMs 	= (float)sysTickMillis;
			cycle 	= (float)SysTick->VAL;
			__ASM volatile("nop");
			__ASM volatile("nop");
			__ASM volatile("nop");
			__ASM volatile("nop");
	}
	while( timeMs != sysTickMillis );
	
	return (timeMs * 1000.0f) + (SysTick->LOAD + 1.0f - cycle)/(double)sysTickPerUs;
}

/*****************************************************************************
*	Function:		extern unsigned int millis(void)
*	Input:
*			
*	Output:
*			
*	Describe:
*		返回以ms为单位的时间计数值		
*
******************************************************************************/
extern unsigned int millis(void)
{
    return sysTickMillis;
}

/*****************************************************************************
*	Function:		extern void SysTickAttachCallback(void (*callback)(void))
*	Input:
*			
*	Output:
*			
*	Describe:
*				
*
******************************************************************************/
extern void SysTickAttachCallback(void (*callback)(void))
{
    systickUserCallback = callback;
}

/*****************************************************************************
*	Function:		extern void InitSysTick(void)
*	Input:
*			
*	Output:
*			
*	Describe:
*				
*
******************************************************************************/
extern void InitSysTick(void)
{
    sysTickPerUs = SystemCoreClock / 1000000;
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_SetPriority(SysTick_IRQn, 0);					//set systick interrupt priority, 0 is the highest for all
}

/*****************************************************************************
*	Function:		extern void SysTick_Handler(void)
*	Input:
*			
*	Output:
*			
*	Describe:
*		系统时基中断处理函数		
*
******************************************************************************/
extern void SysTick_Handler(void)
{
    __disable_irq();
    sysTickMillis++;
    __enable_irq();

    if( systickUserCallback )
    {
    	systickUserCallback();
    }
}

