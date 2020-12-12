#include "US.h"
#include "usart3.h"


#define DisCapCMD 0x55
//u8 RXRecivedFlag = 0;//


void DisCap()
{
	TxBuffer1[0] = DisCapCMD;//送入待发送指令
	memset(RxBuffer1,0x00,sizeof(uint8_t)*255);//清空接收
	USART3_SendString();
	//while(RxBuffer1[0] == 0);//未接收到数据则等待
	//接收数据处理
	
	Dis = RxBuffer1[0]*256+RxBuffer1[1];
}
