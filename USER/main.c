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
#define SAMPLES_N 20 //每通道采20次
#define CHANEELS_M 2 //为12个通道
#define ADC1_DR_Address    ((u32)0x4001244C)

vu16 AD_value[SAMPLES_N][CHANEELS_M]; //用来存放ADC转换结果，也是DMA的目标地址
vu16 AD_value_filter[CHANEELS_M]; //用来存放求平均值之后的结果
vu16 AD_value_out[CHANEELS_M]; 


//CanTxMsg TxMsg1={0xAB,0,CAN_ID_STD,CAN_RTR_DATA,8,{0xAB,0,0,0,0,0,0,0}};
//CanTxMsg TxMsg2={0xCD,1,CAN_ID_STD,CAN_RTR_DATA,8,{0xCD,0,0,0,0,0,0,0}};

// usart		
volatile u8 RxBuffer2[1000] = {0x00};	 	
volatile u16 RxCounter2 = 0;	// 接收计数
volatile u16 RxCounter2_frame = 0;	// 接收完一整帧之后的直接数量
volatile u8 ReceiveState2 = 0;		
volatile u8 RxBuffer3[1000] = {0x00};	 	
volatile u16 RxCounter3 = 0;	// 接收计数
volatile u8 ReceiveState3 = 0;

u32 led_life = 0;  // 系统生命灯
u8 led_counter = 0;  // 串口数据收，led闪烁
u8 str_data[10] = {0x00};

// usart2can
CanTxMsg TxMsg;
u8 last_left_data_nums = 0; // 按8B发送完后，剩余的字节数据
u32 CAN_send_ID = 190;  //从190开始作为第一包
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
            	adc_data[0] = AD_value_filter[0]; // DMA 缓存区中的数据
            	adc_data[1] = AD_value_filter[1]; 

                if(adc_data[0] > 10 || adc_data[1] > 10)
                {
                      // CAN 转发 ADC 			
            		TxMsg.StdId=0x00; 
            		TxMsg.ExtId=0x00;  
            		TxMsg.IDE=CAN_ID_STD;  //使用标准id
            		TxMsg.RTR=CAN_RTR_DATA;
            		CAN_send_ID = 0xBB;
            		TxMsg.DLC = 4; // data length
            		memcpy(&TxMsg.Data[0], 0, 8); // 先清零
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
	
	USART2_Configuration();// 串口配置	
	Delay(500);	
	USART3_Configuration();	  												   
	
	CAN1_Config(SET_CAN_SJW,SET_CAN_BS1,SET_CAN_BS2,SET_CAN_PRES);// CAN1 配置
	Delay(500); 	
	CAN2_Config(SET_CAN_SJW,SET_CAN_BS1,SET_CAN_BS2,SET_CAN_PRES);	// CAN2 配置

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
    // 系统生命灯 LED 闪烁
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

// 从串口2收FMU数据，通过CAN1发送
// 数据: euler(3*2B) + vel(3*2B) + timestamp(4B) + pos(3*2B) = 22B
void GPS_USART2CAN(void)
{
    u8 i, j;
    u16 GPS_msg_length = 0; // CAN上待发送的数据长度
    int16_t GPS_hex_data[30] = {0x00};
	if(ReceiveState2 == 1)//如果接收到1帧数据
		{			
			if(RxBuffer2[0] == 'A' && RxBuffer2[1] == 'r' && RxBuffer2[2] == 'd' )// 数据从0-11为帧头
			{
				// 串口2收到一帧数据，LED 闪烁
				if(led_counter==0)
				{
					GPIO_SetBits(GPIOC,GPIO_Pin_15); 
					led_counter = 1;
				}else
				{
					GPIO_ResetBits(GPIOC,GPIO_Pin_15);
					led_counter = 0;
				}

				// CAN 转发 			
				TxMsg.StdId=0x00; 
				TxMsg.ExtId=0x00;  
				TxMsg.IDE=CAN_ID_STD;  //使用标准id
				TxMsg.RTR=CAN_RTR_DATA; 
				
				GPS_msg_length = 0; // 待发送的GPS数据长度
				CAN_send_ID = 0xB8;
				for(i=0; i<(RxCounter2_frame-12)/5; i++)
				{
					// FMU发过来的一个数据是5个字符长度
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

				last_left_data_nums = GPS_msg_length; // 待发送的数据长度  2B为单位
				for(i=0; i<GPS_msg_length/4; i++) // 一个数据长度为2Byte,一个CAN包为8B
				{ 
					TxMsg.DLC = 8; // data length
					memcpy(&TxMsg.Data[0], &GPS_hex_data[i*4], 8);
					TxMsg.StdId = CAN_send_ID++; 
					CAN_SendData(CAN1,	&TxMsg);
					last_left_data_nums = last_left_data_nums - 4; // 一次发送8B
					Delay(1800);			
					//Comm_Send_CANmsg_str(USART3, 1, &TxMsg);
				}				
				if(last_left_data_nums>0)
				{
					TxMsg.DLC = last_left_data_nums*2; // data length
					memcpy(&TxMsg.Data[0], 0, 8); // 先清零
					memcpy(&TxMsg.Data[0], &GPS_hex_data[GPS_msg_length-last_left_data_nums], last_left_data_nums*2);
					TxMsg.StdId=CAN_send_ID++; 
					CAN_SendData(CAN1,	&TxMsg);	
					//Comm_Send_CANmsg_str(USART3, 1, &TxMsg);
				}
							
			}
			ReceiveState2 = 0;
			RxCounter2_frame = 0;
		}

		if(ReceiveState3 == 1)//如果接收到1帧数据
		{
			//USART_Send(USART3, RxBuffer3, RxCounter3);	// 把接收到数据发送回串口
			ReceiveState3 = 0;
			RxCounter3 = 0;
		}
}


// 从串口2收att数据，通过CAN1发送
// 数据: euler(3*2B) 
void USART2CAN(u32 CAN_send_ID_t )
{
    u8 i, j;
    u8 uart_msg_length = 0;// 待发送的GPS数据长度
    int16_t uart_hex_data[30] = {0x00};
	u32 CAN_send_ID = CAN_send_ID_t;
	if(ReceiveState2 == 1)//如果接收到1帧数据
		{			
			if(RxBuffer2[0] == 'A' && RxBuffer2[1] == 'r' && RxBuffer2[2] == 'd' )// 数据从0-11为帧头
			{
				// 串口2收到一帧数据，LED 闪烁
				if(led_counter==0)
				{
					GPIO_SetBits(GPIOC,GPIO_Pin_15); 
					led_counter = 1;
				}else
				{
					GPIO_ResetBits(GPIOC,GPIO_Pin_15);
					led_counter = 0;
				}

				// CAN 转发 			
				TxMsg.StdId=0x00; 
				TxMsg.ExtId=0x00;  
				TxMsg.IDE=CAN_ID_STD;  //使用标准id
				TxMsg.RTR=CAN_RTR_DATA; 

				for(i=0; i<(RxCounter2_frame-12)/5; i++)
				{
					// FMU发过来的一个数据是5个字符长度
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

				last_left_data_nums = uart_msg_length; // 待发送的数据长度  2B为单位
				for(i=0; i<uart_msg_length/4; i++) // 一个数据长度为2Byte,一个CAN包为8B
				{ 
					TxMsg.DLC = 8; // data length
					memcpy(&TxMsg.Data[0], &uart_hex_data[i*4], 8);
					TxMsg.StdId = CAN_send_ID++; 
					CAN_SendData(CAN1,	&TxMsg);
					last_left_data_nums = last_left_data_nums - 4; // 一次发送8B
					Delay(1800);			
					//Comm_Send_CANmsg_str(USART3, 1, &TxMsg);
				}				
				if(last_left_data_nums>0)
				{
					TxMsg.DLC = last_left_data_nums*2; // data length
					memcpy(&TxMsg.Data[0], 0, 8); // 先清零
					memcpy(&TxMsg.Data[0], &uart_hex_data[uart_msg_length-last_left_data_nums], last_left_data_nums*2);
					TxMsg.StdId=CAN_send_ID++; 
					CAN_SendData(CAN1,	&TxMsg);	
					//Comm_Send_CANmsg_str(USART3, 1, &TxMsg);
				}							
			}
			ReceiveState2 = 0;
			RxCounter2_frame = 0;
		}

		if(ReceiveState3 == 1)//如果接收到1帧数据
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
    DMA_DeInit(DMA1_Channel1); //复位DMA通道1
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;//定义 DMA通道外设基地址=ADC1_DR_Address
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_value; //ADC_ConvertedValue;//定义DMA通道存储器地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//指定外设为源地址
    DMA_InitStructure.DMA_BufferSize = SAMPLES_N*CHANEELS_M;//定义DMA缓冲区大小N*M
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//当前外设寄存器地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //当前存储器地址不变
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//定义外设数据宽度16位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//定义存储器数据宽度16位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA通道操作模式位环形缓冲模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA通道优先级高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//禁止DMA通道存储器到存储器传输
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);//初始化DMA通道1
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
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//ADC1和ADC2工作在独立模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//扫描模式使能
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//ADC转换工作在连续模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//启动转换的外部事件--无  CR2
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//转换后的数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = CHANEELS_M;	//转换的通道数为M
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_TempSensorVrefintCmd(ENABLE);
    //设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
    //ADC1,ADC通道x,规则采样顺序值为y,采样时间为239.5周期
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_239Cycles5 );
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 2, ADC_SampleTime_239Cycles5);
  
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_55Cycles5);	  //通道14采样时间
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 2, ADC_SampleTime_55Cycles5);	  //通道15采样时间
													  
    ADC_DMACmd(ADC1, ENABLE);	 //允许ADC1进行DMA传送  
    ADC_Cmd(ADC1, ENABLE); //使能ADC1
    /* Enable ADC1 reset calibaration register */   
    ADC_ResetCalibration(ADC1);	//重置ADC1校准寄存器
    while(ADC_GetResetCalibrationStatus(ADC1));	//等待ADC1校准重置完成
    ADC_StartCalibration(ADC1);	 //启动ADC1 校准
    while(ADC_GetCalibrationStatus(ADC1));//等待ADC1校准完成    
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//软件触发启动ADC1转换

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

    //PC4/5 作为模拟通道输入引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //模拟输入引脚
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
//新加入的内容：：
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
    RCC_DeInit();//复位RCC外部设备寄存器到默认值
    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);//打开外部高速晶振
    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();//等待外部高速时钟准备好
    if(HSEStartUpStatus == SUCCESS)
    {
        /* HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1); //配置AHB(HCLK)时钟=SYSCLK  
        /* PCLK2 = HCLK */
        RCC_PCLK2Config(RCC_HCLK_Div1); //配置APB2(PCLK2)钟=AHB时钟
        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config(RCC_HCLK_Div2);//配置APB1(PCLK1)钟=AHB 1/2时钟
        /* ADCCLK = PCLK2/4 */
        RCC_ADCCLKConfig(RCC_PCLK2_Div4); //配置ADC时钟=PCLK2 1/4  

        /* PLLCLK = 8MHz * 9 = 56 MHz */
    //    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);//配置PLL时钟 == 外部高速晶体时钟*9
    //    RCC_ADCCLKConfig(RCC_PCLK2_Div4);//配置ADC时钟= PCLK2/4

        /* Enable PLL */ 
        RCC_PLLCmd(ENABLE);//使能PLL时钟
        /* Wait till PLL is ready */
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)//等待PLL时钟就绪
        {
        }

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//配置系统时钟 = PLL时钟
        /* Wait till PLL is used as system clock source */
        while(RCC_GetSYSCLKSource() != 0x08) //检查PLL时钟是否作为系统时钟
        {
        }
    }

/* Enable peripheral clocks --------------------------------------------------*/
  /* Enable DMA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能DMA时钟
  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
  /* Enable USART1 and GPIOA clock */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE); //使能串口1时钟
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE); //使能串口2时钟
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



// 功能：串口接收的数据是字符型数据，是ASCII码，而我们需要解析的是数据型的  
// 如：串口接收到的是："-22.11"这个字符串
void PutCharTOdata(float *dTabDest,unsigned char *rev_tab,unsigned char rev_count)//   tab[ ] 为转换后的数据，rev_tab[ ]字符数组，rev_count为字符串长度(字符的个数)  
{  
    unsigned char i;  
    unsigned char pflag  = 0;  
    unsigned char point  = 0;  
    unsigned char F_flag  =0;  
  
    // 清空结果数组  
    for(i = 0; i < 10; i++) dTabDest[i] =0.0;  
  
    for(i = 0; i < rev_count; i++)  
    {  
        if((F_flag==0) && (rev_tab[i] == '-'))//判断是否为负号  
        {  
            F_flag = 1;  
        }    
        else if(rev_tab[i] != ',')//判断是否碰到逗号  
        {  
            if(rev_tab[i] != '.')//判断是否碰到小数点  
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
