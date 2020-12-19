#include "show.h"
#include "sys.h"
  /**************************************************************************
作者：
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
unsigned char i;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
float Vol;
/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{
		if(Flag_Way==0)	
		{
		  OLED_ShowString(00,0,"SPEED");
			OLED_ShowNumber(45,0, Bluetooth_Velocity,3,12);  //PS2的数据	
		  OLED_ShowString(00,10,"RX");
			OLED_ShowNumber(30,10, PID_Send,3,12);  //PS2的数据			
		}
		else if(Flag_Way==2)
		{
			OLED_Show_CCD(); 
			OLED_ShowString(00,10,"Z");
			OLED_ShowNumber(35,10, CCD_Zhongzhi,3,12);
			OLED_ShowString(70,10,"Y");
			OLED_ShowNumber(95,10, CCD_Yuzhi,3,12);
		}
									
//		                      OLED_ShowString(00,20,"EncoLEFT");    //编码器数据
//		if( Encoder_Left<0)		OLED_ShowString(80,20,"-"),
//		                      OLED_ShowNumber(95,20,-Encoder_Left,5,12);
//		else                 	OLED_ShowString(80,20,"+"),
//		                      OLED_ShowNumber(95,20, Encoder_Left,5,12);

//		                      OLED_ShowString(00,30,"EncoRIGHT");
//		if(Encoder_Right<0)	  OLED_ShowString(80,30,"-"),
//		                      OLED_ShowNumber(95,30,-Encoder_Right,5,12);
//		else               		OLED_ShowString(80,30,"+"),
//		                      OLED_ShowNumber(95,30,Encoder_Right,5,12);
		if(PARAIndex == 2){
			OLED_ShowString(00,20,"-L:"); OLED_ShowNumber(25,20,TurnLeft,5,12);
		}else{
			OLED_ShowString(00,20," L:"); OLED_ShowNumber(25,20,TurnLeft,5,12);
		}
		if(PARAIndex == 3){
			OLED_ShowString(00,30,"-R:"); OLED_ShowNumber(25,30,TurnRight,5,12);
		}else{
			OLED_ShowString(00,30," R:"); OLED_ShowNumber(25,30,TurnRight,5,12);
		}
		if(PARAIndex == 4){
			OLED_ShowString(70,20,"-Le:"); OLED_ShowNumber(95,20,LeftBias,5,12);
		}else{
			OLED_ShowString(70,20," Le:"); OLED_ShowNumber(95,20,LeftBias,5,12);
		}
		if(PARAIndex == 5){
			OLED_ShowString(70,30,"-La:"); OLED_ShowNumber(95,30,LastBias,5,12);
		}else{
			OLED_ShowString(70,30," La:"); OLED_ShowNumber(95,30,LastBias,5,12);
		}

		//电压值及开关状态
		                      OLED_ShowString(00,40,"VOLTAGE");
		                      OLED_ShowString(68,40,".");
		                      OLED_ShowString(90,40,"V");
		                      OLED_ShowNumber(55,40,Voltage/100,2,12);
		                      OLED_ShowNumber(78,40,Voltage%100,2,12);
		 if(Voltage%100<10) 	OLED_ShowNumber(72,40,0,2,12);
		                    
													if(Flag_Stop==0)
													{
														if(PARAIndex == 1){
															OLED_ShowString(94,40,"-O-N");
														}else{
															OLED_ShowString(94,40," O-N");
														}
													}
													if(Flag_Stop==1)
													{
														if(PARAIndex == 1){
															OLED_ShowString(94,40,"-OFF");
														}else{
															OLED_ShowString(94,40," OFF");
														}
														
													}
													
		
		//个性化参数
													OLED_ShowString(00,50,"PARA-");
													if(Flag_Way==2)				  OLED_ShowNumber(40,50, UsefulPoint2,3,12);
													
													if(PARAIndex == 0){
														OLED_ShowString(80,50,"-B");
													}else{
														OLED_ShowString(80,50," B");
													}
													OLED_ShowNumber(95,50,CCD_YuzhiBias,4,12);
		//=============刷新=======================//
		OLED_Refresh_Gram();	
	}
/**************************************************************************
函数功能：向APP发送数据
入口参数：无
返回  值：无
作    者：
**************************************************************************/
void APP_Show(void)
{    
	  static u8 flag;
	  int app_2,app_3,app_4;
		app_4=(Voltage-740)*2/3;		if(app_4<0)app_4=0;if(app_4>100)app_4=100;   //对电压数据进行处理
		app_3=Encoder_Right*1.1; if(app_3<0)app_3=-app_3;			                   //对编码器数据就行数据处理便于图形化
		app_2=Encoder_Left*1.1;  if(app_2<0)app_2=-app_2;
	  flag=!flag;
		if(PID_Send==1)//发送PID参数
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)Bluetooth_Velocity,(int)Velocity_KP,(int)Velocity_KI,0,0,0,0,0,0);//打印到APP上面	
		PID_Send=0;	
	}	
  else  if(flag==0)// 
   printf("{A%d:%d:%d:%d}$",(u8)app_2,(u8)app_3,app_4,(int)((Servo-SERVO_INIT)*0.16)); //打印到APP上面
	 else
	 printf("{B%d:%d:%d:%d}$",(int)Servo,(int)(Voltage),Encoder_Left,Encoder_Right);//打印到APP上面 显示波形
}

void OLED_DrawPoint_Shu(u8 x,u8 y,u8 t)
{ 
	 u8 i=0;
  OLED_DrawPoint(x,y,t);
	OLED_DrawPoint(x,y,t);
	  for(i = 0;i<8; i++)
  {
      OLED_DrawPoint(x,y+i,t);
  }
}

void OLED_Show_CCD(void)
{ 
	 u8 i,t;
	 for(i = 0;i<128; i++)
  {
		if(ADV[i]<CCD_Yuzhi) t=1; else t=0;
		OLED_DrawPoint_Shu(i,0,t);
  }
}

//开机显示一次的内容
void oled_show_once(void)
{
   OLED_ShowString(0,00,"Turn Right Wheel");
   OLED_ShowString(0,10,"TO Select Mode");
	 OLED_ShowString(0,20,"Current Mode Is");
	if(Flag_Way==0)         OLED_ShowString(50,30,"APP");
	//if(Flag_Way==1)         OLED_ShowString(50,30,"PS2");
	if(Flag_Way==2)				  OLED_ShowString(50,30,"CCD");
	//if(Flag_Way==3)				  OLED_ShowString(50,30,"PARA");
	
	OLED_ShowString(0,40,"Press User Key");
  OLED_ShowString(0,50,"TO End Selection");

		OLED_Refresh_Gram();	
	}
