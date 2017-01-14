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
//extern u16 RxCounter1;	// ���ռ���
//extern u8 ReceiveState1;  // ������һ֡���ݵı�־

extern u8 RxBuffer2[1000];	 
extern u16 RxCounter2 ;	// ���ռ���
extern u16 RxCounter2_frame;	// ������һ��֮֡���ֱ������
extern u8 ReceiveState2;

extern u8 RxBuffer3[1000];	 
extern u16 RxCounter3 ;	// ���ռ���
extern u8 ReceiveState3;

u8 GPS_data_farme_counter = 0; // ��¼��ǰGPS���ݵ�֡��(��Ϊ���ݷֳ�3������)
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
//	u8 Clear=Clear;	//���ֶ��巽��������������������"û���õ�"����
//	
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 	// ������յ�1���ֽ�
//	{ 	
//		RxBuffer1[RxCounter1++] = USART1->DR;				// �ѽ��յ����ֽڱ��棬�����ַ��1
//	} 
//	else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)	// ������յ�1֡����
//	{
//		Clear=USART1->SR;		// ��SR�Ĵ���
//		Clear=USART1->DR;		// ��DR�Ĵ���(�ȶ�SR�ٶ�DR������Ϊ�����IDLE�ж�)
//		ReceiveState1 = 1;			// ��ǽ��յ���1֡����
//	}
//}


/*
  USART2�жϷ������
*/				  
void USART2_IRQHandler(void)
{
    u8 Clear = Clear;		// ����������������"û���õ�"����
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
		RxBuffer2[RxCounter2++] = USART_ReceiveData(USART2);	//�������� ����ֽڽ���											   				 

    }
	else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)	// ������յ�1֡����
	{
		Clear = USART2->SR;		// ��SR�Ĵ���
		Clear = USART2->DR;		// ��DR�Ĵ���(�ȶ�SR�ٶ�DR������Ϊ�����IDLE�ж�)
		ReceiveState2 = 1;			// ��ǽ��յ���1֡����

        RxCounter2_frame = RxCounter2;
        RxCounter2 = 0;            
	}
	

    /*��ֹ�����ж� */
    if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)                    
    { 
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }
}


/*
  USART3�жϷ������
*/				  
void USART3_IRQHandler(void)
{
    u8 Clear = Clear;		// ����������������"û���õ�"����
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
		RxBuffer3[RxCounter3++] = USART_ReceiveData(USART3);	//��������												   				 

    }
	else if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)	// ������յ�1֡����
	{
		Clear = USART3->SR;		// ��SR�Ĵ���
		Clear = USART3->DR;		// ��DR�Ĵ���(�ȶ�SR�ٶ�DR������Ϊ�����IDLE�ж�)
		ReceiveState3 = 1;		// ��ǽ��յ���1֡����
	}	

    /*��ֹ�����ж� */
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
	RxMessage.RTR=CAN_RTR_DATA;  // ����֡orԶ��
    RxMessage.IDE=CAN_ID_STD;	 // ��׼or��չ
    RxMessage.DLC=0;
    RxMessage.FMI=0;
    RxMessage.Data[0]=0x00;
    RxMessage.Data[1]=0x00;

    CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);	 // CAN��������		

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
	RxMessage.RTR=CAN_RTR_DATA;  // ����֡orԶ��
    RxMessage.IDE=CAN_ID_STD;	 // ��׼or��չ
    RxMessage.DLC=0;
    RxMessage.FMI=0;
    RxMessage.Data[0]=0x00;
    RxMessage.Data[1]=0x00;

    CAN_Receive(CAN2, CAN_FIFO1, &RxMessage);	 // CAN��������	

//	USART_Send(USART3, RxMessage.Data, (u16)RxMessage.DLC);
	Comm_Receive_CANmsg_str(USART3, 2, &RxMessage);

}


/***********************
 CAN2 FIFO    �ݲ�ʹ��
***********************/								   								  
//void CAN2_RX0_IRQHandler(void)
//{
//    CanRxMsg RxMessage;

//    RxMessage.StdId=0x00;
//    RxMessage.ExtId=0x00;
//	RxMessage.RTR=CAN_RTR_DATA;  // ����֡orԶ��
//    RxMessage.IDE=CAN_ID_STD;	 // ��׼or��չ
//    RxMessage.DLC=0;
//    RxMessage.FMI=0;
//    RxMessage.Data[0]=0x00;
//    RxMessage.Data[1]=0x00;

//    CAN_Receive(CAN2,CAN_FIFO0, &RxMessage);	 // CAN��������

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
