/**
  ******************************************************************************
  * @file    	usart.h 
  * @author  	Stone
  * @version 	V0.1
  * @date    	2015-09-08
  * @brief   	
  * 
  ******************************************************************************
	*/	
#ifndef SYSTICK_H_
#define SYSTICK_H_

//===============================================================================
// 													全局变量声明 																	
//===============================================================================	
#define SYSCLK_FREQ_8MHz 8000000


//===============================================================================
// 													全局函数原型声明 																	
//===============================================================================
extern void InitSysTick(void);
extern uint32_t micros(void);
extern double GetSystickCounterValudeRAW(void );
extern unsigned int millis(void);
extern void SysTick_Handler(void);
extern void SysTickAttachCallback(void (*callback)(void));

#endif /* SYSTICK_H_ */
