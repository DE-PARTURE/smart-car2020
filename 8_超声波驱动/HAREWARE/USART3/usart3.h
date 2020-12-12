#ifndef __USRAT3_H
#define __USRAT3_H 
#include "sys.h"	  	


extern uint8_t TxBuffer1[]; 
extern uint8_t RxBuffer1[];
extern __IO uint8_t TxCounter1;
extern __IO uint8_t RxCounter1; 
extern uint8_t NbrOfDataToTransfer1;
extern uint8_t NbrOfDataToRead1;
extern u8 TxReady;
extern u8 TX_Flag;
extern u8 RxReady;

void uart3_init(u32 bound);
void USART3_IRQHandler(void);
void USART3_SendString(void);
#endif

