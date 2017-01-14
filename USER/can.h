
#ifndef __CAN_H__
#define __CAN_H__

#include "stm32f10x.h"


// CAN波特率  BaudRate = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler 
#define    SET_CAN_SJW   CAN_SJW_1tq
#define    SET_CAN_BS1   CAN_BS1_4tq	// 8
#define    SET_CAN_BS2   CAN_BS2_4tq	// 7
#define    SET_CAN_PRES  8				// 波特率分频器 9-250K 18-125K 

void CAN1_Config(uint8_t sjw,uint8_t bs1,uint8_t bs2,uint16_t pres);
void CAN2_Config(uint8_t sjw,uint8_t bs1,uint8_t bs2,uint16_t pres);
void CAN_SendData(CAN_TypeDef* CANx,CanTxMsg* CanData);
void Comm_Send_CANmsg_str(USART_TypeDef* USARTx, uint8_t port,CanTxMsg *RxMsg);
void Comm_Receive_CANmsg_str(USART_TypeDef* USARTx, uint8_t port, CanRxMsg *RxMsg);


#endif
