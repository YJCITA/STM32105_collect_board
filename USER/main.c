/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_can.h"
#include "usart.h"
#include "can.h"

#include "stdio.h"
#include "string.h"
#include "stdlib.h"

//adc
#include "stm32f10x_conf.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "misc.h"

#define CAN_GPS 0
#define CAN_ATT 1
#define CAN_ADC 1

void init(void);
void RCC_Configuration(void);
void ADC_Configuration(void);
void RCC_Configuration1(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void DMA_Configuration(void);
void LED_Config(void);
void Delay(__IO uint32_t nCount);

void GPS_USART2CAN(void);
void USART2CAN(u32 CAN_send_ID_t );

void LED_life(void);
void AD_filter(void);


/* Private variables ---------------------------------------------------------*/
// ADC
#define SAMPLES_N 20 //ÿͨ����20��
#define CHANEELS_M 2 //Ϊ12��ͨ��
#define ADC1_DR_Address    ((u32)0x4001244C)

vu16 AD_value[SAMPLES_N][CHANEELS_M]; //�������ADCת�������Ҳ��DMA��Ŀ���ַ
vu16 AD_value_filter[CHANEELS_M]; //���������ƽ��ֵ֮��Ľ��
vu16 AD_value_out[CHANEELS_M]; 


//CanTxMsg TxMsg1={0xAB,0,CAN_ID_STD,CAN_RTR_DATA,8,{0xAB,0,0,0,0,0,0,0}};
//CanTxMsg TxMsg2={0xCD,1,CAN_ID_STD,CAN_RTR_DATA,8,{0xCD,0,0,0,0,0,0,0}};

// usart		
volatile u8 RxBuffer2[1000] = {0x00};	 	
volatile u16 RxCounter2 = 0;	// ���ռ���
volatile u16 RxCounter2_frame = 0;	// ������һ��֮֡���ֱ������
volatile u8 ReceiveState2 = 0;		
volatile u8 RxBuffer3[1000] = {0x00};	 	
volatile u16 RxCounter3 = 0;	// ���ռ���
volatile u8 ReceiveState3 = 0;

u32 led_life = 0;  // ϵͳ������
u8 led_counter = 0;  // ���������գ�led��˸
u8 str_data[10] = {0x00};

// usart2can
CanTxMsg TxMsg;
u8 last_left_data_nums = 0; // ��8B�������ʣ����ֽ�����
u32 CAN_send_ID = 190;  //��190��ʼ��Ϊ��һ��
int16_t adc_data[2] = {0x00};


int main(void)
{	
    u16 adc_update_counter = 0;
    
    init( );    
    while (1)
    {
        if(CAN_GPS)
        {
            GPS_USART2CAN();	
        }else if(CAN_ATT)
        {  // att
            CAN_send_ID = 0xBC;
            USART2CAN(CAN_send_ID);
        }

        if(CAN_ADC)
        {
            if(adc_update_counter++>50)
            {
                adc_update_counter = 0;
                // ADC
                AD_filter();
            	adc_data[0] = AD_value_filter[0]; // DMA �������е�����
            	adc_data[1] = AD_value_filter[1]; 

                if(adc_data[0] > 10 || adc_data[1] > 10)
                {
                      // CAN ת�� ADC 			
            		TxMsg.StdId=0x00; 
            		TxMsg.ExtId=0x00;  
            		TxMsg.IDE=CAN_ID_STD;  //ʹ�ñ�׼id
            		TxMsg.RTR=CAN_RTR_DATA;
            		CAN_send_ID = 0xBB;
            		TxMsg.DLC = 4; // data length
            		memcpy(&TxMsg.Data[0], 0, 8); // ������
            		memcpy(&TxMsg.Data[0], &adc_data[0], 4);
            		TxMsg.StdId = CAN_send_ID; 
            		CAN_SendData(CAN1,	&TxMsg);
                }

//                Comm_Send_CANmsg_str(USART3, 1, &TxMsg);
            }
        }

        LED_life();
    }

	
}

void init(void)
{
    //	/* System Clocks Configuration **********************************************/
	RCC_Configuration();     
	LED_Config();// LED		
	
	USART2_Configuration();// ��������	
	Delay(500);	
	USART3_Configuration();	  												   
	
	CAN1_Config(SET_CAN_SJW,SET_CAN_BS1,SET_CAN_BS2,SET_CAN_PRES);// CAN1 ����
	Delay(500); 	
	CAN2_Config(SET_CAN_SJW,SET_CAN_BS1,SET_CAN_BS2,SET_CAN_PRES);	// CAN2 ����

     /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration1();
    GPIO_Configuration();
    DMA_Configuration();
    ADC_Configuration();
}
    

void AD_filter(void)
{
    int sum = 0;
    u8 i, count;
    for(i=0; i<CHANEELS_M; i++)
    {
        for(count=0; count<SAMPLES_N; count++)
        {
            sum += AD_value[count][i];
        }
        AD_value_filter[i] = sum/SAMPLES_N;
        sum=0;
    }
}

void LED_life(void)
{    
    // ϵͳ������ LED ��˸
    led_life++;
    if(led_life<200000)
    {
        GPIO_SetBits(GPIOC,GPIO_Pin_0); 
    }else
    {
        GPIO_ResetBits(GPIOC,GPIO_Pin_0);
        if(led_life > 400000)
        {   
            led_life = 0; 
        }
    }
}

// �Ӵ���2��FMU���ݣ�ͨ��CAN1����
// ����: euler(3*2B) + vel(3*2B) + timestamp(4B) + pos(3*2B) = 22B
void GPS_USART2CAN(void)
{
    u8 i, j;
    u16 GPS_msg_length = 0; // CAN�ϴ����͵����ݳ���
    int16_t GPS_hex_data[30] = {0x00};
	if(ReceiveState2 == 1)//������յ�1֡����
		{			
			if(RxBuffer2[0] == 'A' && RxBuffer2[1] == 'r' && RxBuffer2[2] == 'd' )// ���ݴ�0-11Ϊ֡ͷ
			{
				// ����2�յ�һ֡���ݣ�LED ��˸
				if(led_counter==0)
				{
					GPIO_SetBits(GPIOC,GPIO_Pin_15); 
					led_counter = 1;
				}else
				{
					GPIO_ResetBits(GPIOC,GPIO_Pin_15);
					led_counter = 0;
				}

				// CAN ת�� 			
				TxMsg.StdId=0x00; 
				TxMsg.ExtId=0x00;  
				TxMsg.IDE=CAN_ID_STD;  //ʹ�ñ�׼id
				TxMsg.RTR=CAN_RTR_DATA; 
				
				GPS_msg_length = 0; // �����͵�GPS���ݳ���
				CAN_send_ID = 0xB8;
				for(i=0; i<(RxCounter2_frame-12)/5; i++)
				{
					// FMU��������һ��������5���ַ�����
					j = i*5;
					str_data[0] = RxBuffer2[12+j];
					str_data[1] = RxBuffer2[12+j+1];
					str_data[2] = RxBuffer2[12+j+2];
					str_data[3] = RxBuffer2[12+j+3];
					str_data[4] = RxBuffer2[12+j+4];
					str_data[5] = '\0';
					sscanf(str_data, "%hd", &GPS_hex_data[i]);
					GPS_msg_length+=1;
				}

				last_left_data_nums = GPS_msg_length; // �����͵����ݳ���  2BΪ��λ
				for(i=0; i<GPS_msg_length/4; i++) // һ�����ݳ���Ϊ2Byte,һ��CAN��Ϊ8B
				{ 
					TxMsg.DLC = 8; // data length
					memcpy(&TxMsg.Data[0], &GPS_hex_data[i*4], 8);
					TxMsg.StdId = CAN_send_ID++; 
					CAN_SendData(CAN1,	&TxMsg);
					last_left_data_nums = last_left_data_nums - 4; // һ�η���8B
					Delay(1800);			
					//Comm_Send_CANmsg_str(USART3, 1, &TxMsg);
				}				
				if(last_left_data_nums>0)
				{
					TxMsg.DLC = last_left_data_nums*2; // data length
					memcpy(&TxMsg.Data[0], 0, 8); // ������
					memcpy(&TxMsg.Data[0], &GPS_hex_data[GPS_msg_length-last_left_data_nums], last_left_data_nums*2);
					TxMsg.StdId=CAN_send_ID++; 
					CAN_SendData(CAN1,	&TxMsg);	
					//Comm_Send_CANmsg_str(USART3, 1, &TxMsg);
				}
							
			}
			ReceiveState2 = 0;
			RxCounter2_frame = 0;
		}

		if(ReceiveState3 == 1)//������յ�1֡����
		{
			//USART_Send(USART3, RxBuffer3, RxCounter3);	// �ѽ��յ����ݷ��ͻش���
			ReceiveState3 = 0;
			RxCounter3 = 0;
		}
}


// �Ӵ���2��att���ݣ�ͨ��CAN1����
// ����: euler(3*2B) 
void USART2CAN(u32 CAN_send_ID_t )
{
    u8 i, j;
    u8 uart_msg_length = 0;// �����͵�GPS���ݳ���
    int16_t uart_hex_data[30] = {0x00};
	u32 CAN_send_ID = CAN_send_ID_t;
	if(ReceiveState2 == 1)//������յ�1֡����
		{			
			if(RxBuffer2[0] == 'A' && RxBuffer2[1] == 'r' && RxBuffer2[2] == 'd' )// ���ݴ�0-11Ϊ֡ͷ
			{
				// ����2�յ�һ֡���ݣ�LED ��˸
				if(led_counter==0)
				{
					GPIO_SetBits(GPIOC,GPIO_Pin_15); 
					led_counter = 1;
				}else
				{
					GPIO_ResetBits(GPIOC,GPIO_Pin_15);
					led_counter = 0;
				}

				// CAN ת�� 			
				TxMsg.StdId=0x00; 
				TxMsg.ExtId=0x00;  
				TxMsg.IDE=CAN_ID_STD;  //ʹ�ñ�׼id
				TxMsg.RTR=CAN_RTR_DATA; 

				for(i=0; i<(RxCounter2_frame-12)/5; i++)
				{
					// FMU��������һ��������5���ַ�����
					j = i*5;
					str_data[0] = RxBuffer2[12+j];
					str_data[1] = RxBuffer2[12+j+1];
					str_data[2] = RxBuffer2[12+j+2];
					str_data[3] = RxBuffer2[12+j+3];
					str_data[4] = RxBuffer2[12+j+4];
					str_data[5] = '\0';
					sscanf(str_data, "%hd", &uart_hex_data[i]);
					uart_msg_length+=1;
				}

				last_left_data_nums = uart_msg_length; // �����͵����ݳ���  2BΪ��λ
				for(i=0; i<uart_msg_length/4; i++) // һ�����ݳ���Ϊ2Byte,һ��CAN��Ϊ8B
				{ 
					TxMsg.DLC = 8; // data length
					memcpy(&TxMsg.Data[0], &uart_hex_data[i*4], 8);
					TxMsg.StdId = CAN_send_ID++; 
					CAN_SendData(CAN1,	&TxMsg);
					last_left_data_nums = last_left_data_nums - 4; // һ�η���8B
					Delay(1800);			
					//Comm_Send_CANmsg_str(USART3, 1, &TxMsg);
				}				
				if(last_left_data_nums>0)
				{
					TxMsg.DLC = last_left_data_nums*2; // data length
					memcpy(&TxMsg.Data[0], 0, 8); // ������
					memcpy(&TxMsg.Data[0], &uart_hex_data[uart_msg_length-last_left_data_nums], last_left_data_nums*2);
					TxMsg.StdId=CAN_send_ID++; 
					CAN_SendData(CAN1,	&TxMsg);	
					//Comm_Send_CANmsg_str(USART3, 1, &TxMsg);
				}							
			}
			ReceiveState2 = 0;
			RxCounter2_frame = 0;
		}

		if(ReceiveState3 == 1)//������յ�1֡����
		{
			ReceiveState3 = 0;
			RxCounter3 = 0;
		}
}


/*******************************************************************************
* Function Name  : DMA_Configuration
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    /* DMA channel1 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1); //��λDMAͨ��1
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;//���� DMAͨ���������ַ=ADC1_DR_Address
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_value; //ADC_ConvertedValue;//����DMAͨ���洢����ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//ָ������ΪԴ��ַ
    DMA_InitStructure.DMA_BufferSize = SAMPLES_N*CHANEELS_M;//����DMA��������СN*M
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//��ǰ����Ĵ�����ַ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //��ǰ�洢����ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//�����������ݿ��16λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//����洢�����ݿ��16λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMAͨ������ģʽλ���λ���ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMAͨ�����ȼ���
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//��ֹDMAͨ���洢�����洢������
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);//��ʼ��DMAͨ��1
    DMA_Cmd(DMA1_Channel1, ENABLE);   // Enable DMA channel1 
}
/*******************************************************************************
* Function Name  : ADC_Configuration
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_Configuration(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    /* ADC1 configuration ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//ADC1��ADC2�����ڶ���ģʽ
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//ɨ��ģʽʹ��
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//ADCת������������ģʽ
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//����ת�����ⲿ�¼�--��  CR2
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ת����������Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel = CHANEELS_M;	//ת����ͨ����ΪM
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_TempSensorVrefintCmd(ENABLE);
    //����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
    //ADC1,ADCͨ��x,�������˳��ֵΪy,����ʱ��Ϊ239.5����
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_239Cycles5 );
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 2, ADC_SampleTime_239Cycles5);
  
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_55Cycles5);	  //ͨ��14����ʱ��
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 2, ADC_SampleTime_55Cycles5);	  //ͨ��15����ʱ��
													  
    ADC_DMACmd(ADC1, ENABLE);	 //����ADC1����DMA����  
    ADC_Cmd(ADC1, ENABLE); //ʹ��ADC1
    /* Enable ADC1 reset calibaration register */   
    ADC_ResetCalibration(ADC1);	//����ADC1У׼�Ĵ���
    while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ�ADC1У׼�������
    ADC_StartCalibration(ADC1);	 //����ADC1 У׼
    while(ADC_GetCalibrationStatus(ADC1));//�ȴ�ADC1У׼���    
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//�����������ADC1ת��

}


/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //PC4/5 ��Ϊģ��ͨ����������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //ģ����������
    GPIO_Init(GPIOC, &GPIO_InitStructure);

   /* Configure USART1 Tx (PA.09) as alternate function push-pull 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);		 */
    
  /* Configure USART1 Rx (PA.10) as input floating 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);	   */
													
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{	
//�¼�������ݣ���
//	NVIC_InitTypeDef NVIC_InitStructure;
    /* Configure the NVIC Preemption Priority Bits */  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
#ifdef  VECT_TAB_RAM  
  /* Set the Vector Table base location at 0x20000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif
}


void RCC_Configuration(void)
{   
	/* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
	 initialize the PLL and update the SystemFrequency variable. */
	SystemInit();
								  			 
}


/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration1(void)
{
    ErrorStatus HSEStartUpStatus;

    /* RCC system reset(for debug purpose) */
    RCC_DeInit();//��λRCC�ⲿ�豸�Ĵ�����Ĭ��ֵ
    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);//���ⲿ���پ���
    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();//�ȴ��ⲿ����ʱ��׼����
    if(HSEStartUpStatus == SUCCESS)
    {
        /* HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1); //����AHB(HCLK)ʱ��=SYSCLK  
        /* PCLK2 = HCLK */
        RCC_PCLK2Config(RCC_HCLK_Div1); //����APB2(PCLK2)��=AHBʱ��
        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config(RCC_HCLK_Div2);//����APB1(PCLK1)��=AHB 1/2ʱ��
        /* ADCCLK = PCLK2/4 */
        RCC_ADCCLKConfig(RCC_PCLK2_Div4); //����ADCʱ��=PCLK2 1/4  

        /* PLLCLK = 8MHz * 9 = 56 MHz */
    //    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);//����PLLʱ�� == �ⲿ���پ���ʱ��*9
    //    RCC_ADCCLKConfig(RCC_PCLK2_Div4);//����ADCʱ��= PCLK2/4

        /* Enable PLL */ 
        RCC_PLLCmd(ENABLE);//ʹ��PLLʱ��
        /* Wait till PLL is ready */
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)//�ȴ�PLLʱ�Ӿ���
        {
        }

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//����ϵͳʱ�� = PLLʱ��
        /* Wait till PLL is used as system clock source */
        while(RCC_GetSYSCLKSource() != 0x08) //���PLLʱ���Ƿ���Ϊϵͳʱ��
        {
        }
    }

/* Enable peripheral clocks --------------------------------------------------*/
  /* Enable DMA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //ʹ��DMAʱ��
  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
  /* Enable USART1 and GPIOA clock */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE); //ʹ�ܴ���1ʱ��
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE); //ʹ�ܴ���2ʱ��
}


void Delay(__IO uint32_t nCount)
{
    uint8_t x;
    for(; nCount != 0; nCount--)
	    for(x=0;x<100;x++);
}

void LED_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_14|GPIO_Pin_15;				   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	// led off
	GPIO_SetBits(GPIOC,GPIO_Pin_0);
	GPIO_SetBits(GPIOC,GPIO_Pin_1);
	GPIO_SetBits(GPIOC,GPIO_Pin_14);
	GPIO_SetBits(GPIOC,GPIO_Pin_15);
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif



// ���ܣ����ڽ��յ��������ַ������ݣ���ASCII�룬��������Ҫ�������������͵�  
// �磺���ڽ��յ����ǣ�"-22.11"����ַ���
void PutCharTOdata(float *dTabDest,unsigned char *rev_tab,unsigned char rev_count)//   tab[ ] Ϊת��������ݣ�rev_tab[ ]�ַ����飬rev_countΪ�ַ�������(�ַ��ĸ���)  
{  
    unsigned char i;  
    unsigned char pflag  = 0;  
    unsigned char point  = 0;  
    unsigned char F_flag  =0;  
  
    // ��ս������  
    for(i = 0; i < 10; i++) dTabDest[i] =0.0;  
  
    for(i = 0; i < rev_count; i++)  
    {  
        if((F_flag==0) && (rev_tab[i] == '-'))//�ж��Ƿ�Ϊ����  
        {  
            F_flag = 1;  
        }    
        else if(rev_tab[i] != ',')//�ж��Ƿ���������  
        {  
            if(rev_tab[i] != '.')//�ж��Ƿ�����С����  
            {  
                if(pflag)  
                {  
                    dTabDest[point] = (double)(rev_tab[i]-0x30)/(pow((long double)10,(long)pflag))+dTabDest[point];  
                    pflag ++;  
                }  
                else  
                {                      
                    dTabDest[point] = (rev_tab[i]-0x30)+dTabDest[point]*10;  
                }  
            }  
            else  
            {  
                pflag ++;  
            }  
        }  
        else if(rev_tab[i]==',')  
        {  
  
            if(F_flag ==1) dTabDest[point] =  -dTabDest[point];  
            point++;  
            pflag =0;  
            F_flag = 0;  
        }  
    }  
  
    if(F_flag ==1) dTabDest[point] = -dTabDest[point];  
}  






/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
