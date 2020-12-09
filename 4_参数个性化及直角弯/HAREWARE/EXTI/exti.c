#include "exti.h"
#include "key.h"


/**************************************************************************
函数功能：外部中断初始化
入口参数：无
返回  值：无 
**************************************************************************/
void EXTIX_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;
	KEY_Init();	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //外部中断，需要使能AFIO时钟
	
	//GPIOA.5 中断线以及中断初始化配置
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource5);
  EXTI_InitStructure.EXTI_Line=EXTI_Line5;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
	//设置IO口与中断线的映射关系
	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;         //使能按键所在的外部中断通道
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;            //使能外部中断通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;  //抢占优先级2 
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;         //响应优先级1
	NVIC_Init(&NVIC_InitStruct);

}

//外部中断9~5服务程序
void EXTI9_5_IRQHandler(void)
{			
	delay_ms(4);     //消抖		
	
   if(KEY==0)	   //开/关电机
	{
		EXTI_ClearITPendingBit(EXTI_Line5);  //清除LINE5上的中断标志位 
		Flag_Next=1;
	}		
}





