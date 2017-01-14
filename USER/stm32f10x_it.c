/**
  ******************************************************************************
  * @file GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and 
  *         peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_can.h"
#include "can.h"

#include "usart.h"

//extern u8 RxBuffer1[1000];	
//extern u16 RxCounter1;	// 接收计数
//extern u8 ReceiveState1;  // 接收完一帧数据的标志

extern u8 RxBuffer2[1000];	 
extern u16 RxCounter2 ;	// 接收计数
extern u16 RxCounter2_frame;	// 接收完一整帧之后的直接数量
extern u8 ReceiveState2;

extern u8 RxBuffer3[1000];	 
extern u16 RxCounter3 ;	// 接收计数
extern u8 ReceiveState3;

u8 GPS_data_farme_counter = 0; // 记录当前GPS数据的帧号(因为数据分成3包发送)
u8 GPS_data_array[35] = {0x00};


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}


//void USART1_IRQHandler(void)
//{
//	u8 Clear=Clear;	//这种定义方法，用来消除编译器的"没有用到"提醒
//	
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 	// 如果接收到1个字节
//	{ 	
//		RxBuffer1[RxCounter1++] = USART1->DR;				// 把接收到的字节保存，数组地址加1
//	} 
//	else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)	// 如果接收到1帧数据
//	{
//		Clear=USART1->SR;		// 读SR寄存器
//		Clear=USART1->DR;		// 读DR寄存器(先读SR再读DR，就是为了清除IDLE中断)
//		ReceiveState1 = 1;			// 标记接收到了1帧数据
//	}
//}


/*
  USART2中断服务程序
*/				  
void USART2_IRQHandler(void)
{
    u8 Clear = Clear;		// 用来消除编译器的"没有用到"提醒
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
		RxBuffer2[RxCounter2++] = USART_ReceiveData(USART2);	//接收数据 逐个字节接收											   				 

    }
	else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)	// 如果接收到1帧数据
	{
		Clear = USART2->SR;		// 读SR寄存器
		Clear = USART2->DR;		// 读DR寄存器(先读SR再读DR，就是为了清除IDLE中断)
		ReceiveState2 = 1;			// 标记接收到了1帧数据

        RxCounter2_frame = RxCounter2;
        RxCounter2 = 0;            
	}
	

    /*禁止发送中断 */
    if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)                    
    { 
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }
}


/*
  USART3中断服务程序
*/				  
void USART3_IRQHandler(void)
{
    u8 Clear = Clear;		// 用来消除编译器的"没有用到"提醒
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
		RxBuffer3[RxCounter3++] = USART_ReceiveData(USART3);	//接收数据												   				 

    }
	else if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)	// 如果接收到1帧数据
	{
		Clear = USART3->SR;		// 读SR寄存器
		Clear = USART3->DR;		// 读DR寄存器(先读SR再读DR，就是为了清除IDLE中断)
		ReceiveState3 = 1;		// 标记接收到了1帧数据
	}	

    /*禁止发送中断 */
    if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)                    
    { 
        USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
    }
}




/*******************************************************************************
* Function Name  : USB_LP_CAN_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts 
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;

    RxMessage.StdId=0x00;
    RxMessage.ExtId=0x00;
	RxMessage.RTR=CAN_RTR_DATA;  // 数据帧or远程
    RxMessage.IDE=CAN_ID_STD;	 // 标准or扩展
    RxMessage.DLC=0;
    RxMessage.FMI=0;
    RxMessage.Data[0]=0x00;
    RxMessage.Data[1]=0x00;

    CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);	 // CAN接收数据		

//	Comm_Receive_CANmsg_str(USART3, 1, &RxMessage);
}

/***********************
 CAN2 FIFO1
***********************/
void CAN2_RX1_IRQHandler(void)
{
    CanRxMsg RxMessage;

    RxMessage.StdId=0x00;
    RxMessage.ExtId=0x00;
	RxMessage.RTR=CAN_RTR_DATA;  // 数据帧or远程
    RxMessage.IDE=CAN_ID_STD;	 // 标准or扩展
    RxMessage.DLC=0;
    RxMessage.FMI=0;
    RxMessage.Data[0]=0x00;
    RxMessage.Data[1]=0x00;

    CAN_Receive(CAN2, CAN_FIFO1, &RxMessage);	 // CAN接收数据	

//	USART_Send(USART3, RxMessage.Data, (u16)RxMessage.DLC);
	Comm_Receive_CANmsg_str(USART3, 2, &RxMessage);

}


/***********************
 CAN2 FIFO    暂不使用
***********************/								   								  
//void CAN2_RX0_IRQHandler(void)
//{
//    CanRxMsg RxMessage;

//    RxMessage.StdId=0x00;
//    RxMessage.ExtId=0x00;
//	RxMessage.RTR=CAN_RTR_DATA;  // 数据帧or远程
//    RxMessage.IDE=CAN_ID_STD;	 // 标准or扩展
//    RxMessage.DLC=0;
//    RxMessage.FMI=0;
//    RxMessage.Data[0]=0x00;
//    RxMessage.Data[1]=0x00;

//    CAN_Receive(CAN2,CAN_FIFO0, &RxMessage);	 // CAN接收数据

//	//	USART_STR(USART1,"CAN2 RX0 Get\r\n");   //////////
//}								 								



/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
//void SysTick_Handler(void)
//{
//}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval : None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
