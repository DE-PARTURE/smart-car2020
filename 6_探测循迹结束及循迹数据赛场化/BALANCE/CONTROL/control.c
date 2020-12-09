#include "control.h"		

//车身参数无需修改
#define T 0.156f
#define L 0.1445f
#define K 622.8f
//
#define RightAngleTimeCountPre 80//直角转弯时间
#define PointDifference 25//CCD直角判断80-118有效Point数量
#define RightAngleJudgeCountPre 2//CCD直角判断次数
#define AcuteAngleTimeCountPre 120//锐角转弯时间
#define PointDifference2 2 //循迹结束检测黑点数量
#define StopJudgeCountPre 200//循迹结束条件判断次数
u8 Flag_Target;
int Voltage_Temp,Voltage_Count,Voltage_All,sum;

/**************************************************************************
函数功能：小车运动数学模型
入口参数：速度和转角
返回  值：无
**************************************************************************/
void Kinematic_Analysis(float velocity,float angle)
{
		Target_A=velocity*(1+T*tan(angle)/2/L); 
		Target_B=velocity*(1-T*tan(angle)/2/L);      //后轮差速
		Servo=SERVO_INIT+angle*K;    								//舵机转向//1000-1650-2500
}
/**************************************************************************
函数功能：所有的控制代码都在这里面
          100ms定时中断由MPU6050的INT引脚触发
          严格保证采样和数据处理的时间同步				 
**************************************************************************/
void TIM1_UP_IRQHandler(void)  
{    
	if(TIM_GetFlagStatus(TIM1,TIM_FLAG_Update)==SET)//5ms定时中断
	{   
		 TIM_ClearITPendingBit(TIM1,TIM_IT_Update);                             //===清除定时器1中断标志位	      
		 if(delay_flag==1)
		 {
			 if(++delay_50==5)
			 {
			    delay_50=0,delay_flag=0;                      //给主函数提供50ms的精准延时
			 }
		 }
		 ///////////////直角延时////////////////////////////
		 if((RightAngleFlag) && (!Flag_Stop))//方向转死延时
		 {
			 //直角弯操作
			 //持续100ms*RightAngleTimeCount时间的直角弯模式
			 RightAngleTimeCount--;
			 if(RightAngleTimeCount == 0)
			 {
				 RightAngleFlag = 0;
				 ActionIndex = 1;//结束直角弯
			 }		
		 }
		 ///////////////锐角延时////////////////////////////
		 if((ActionIndex == 1) && (!Flag_Stop))
		 {
			 //锐角弯操作
				AcuteAngleTimeCount--;
			 if(AcuteAngleTimeCount == 0)
			 {
				 ActionIndex = 2;//结束锐角弯
				 ActionFlag = 0;
			 }
		 }
		 /////////////////延时结束//////////////////////////
			Encoder_Left=Read_Encoder(2);                                       //===读取编码器的值							 //为了保证M法测速的时间基准，首先读取编码器数据
			Encoder_Right=-Read_Encoder(3);                                      //===读取编码器的值
			if(Flag_Way==2 )//▲▲▲▲▲
		   {	 
				 RD_TSL();                           		 //===读取线性CCD数据 
				 Find_CCD_Zhongzhi();			          		 //===提取中线 
		   }				 
			 Get_RC();																												//速度计算及舵机角度angle计算
			 if(!ActionFlag){
				 Kinematic_Analysis(Velocity,Angle);     														//小车运动学分析,后轮差速计算   
			 }else if(ActionIndex == 3){
				 //循迹结束
				 Kinematic_Analysis(0,0);
			 }
			 //运动参数赋值
			 if(Turn_Off(Voltage)==0)                              							 //===如果不存在异常
			 {
				/////////直角转弯//////////////////////////////////
				if((RightAngleFlag) && (!Flag_Stop))									//直角弯模式
				{
					Kinematic_Analysis(15,PI/3);
				}
				/////////锐角转弯//////////////////////////////////
				if((ActionIndex == 1) && (!Flag_Stop))//锐角弯
				{
					Kinematic_Analysis(15,-PI/3);
				}
				//////////////////////////////////////////////////////
				Motor_A=Incremental_PI_A(Encoder_Left,Target_A);                   //===速度闭环控制计算电机A最终PWM
				Motor_B=Incremental_PI_B(Encoder_Right,Target_B);                  //===速度闭环控制计算电机B最终PWM 
				Xianfu_Pwm();                                                      //===PWM限幅
				Set_Pwm(Motor_A,Motor_B,Servo);                                 	 //===赋值给PWM寄存器 
				
			}
			else  Set_Pwm(0,0,SERVO_INIT);                                 						 //===赋值给PWM寄存器  	
					
				     Voltage_Temp=Get_battery_volt();		                                 //=====读取电池电压		
					   Voltage_Count++;                                                     //=====平均值计数器
					   Voltage_All+=Voltage_Temp;                                           //=====多次采样累积
			if(Voltage_Count==100) 
			{
				//=====求平均值		 
				Voltage=Voltage_All/100;
				Voltage = (int)((float)Voltage*1.0065);//对电压检测电路电阻阻值不精准的补偿
				Voltage_All=0;
				Voltage_Count=0;                                  
			}
			if(Flag_Show==0)				Led_Flash(100);
			else if(Flag_Show==1)	Led_Flash(0);  //led闪烁
			 Key();    //===扫描按键状态 单击双击可以改变小车运行状态
	}
} 
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int servo)
{
	    if(Flag_Way>=2)//巡线模式下，只允许电机正转
			{
			  if(motor_a<0)motor_a=0;
				if(motor_b<0)motor_b=0;
			}
    	if(motor_a<0)			PWMA1=7200,PWMA2=7200+motor_a;
			else 	            PWMA2=7200,PWMA1=7200-motor_a;
		
		  if(motor_b<0)			PWMB1=7200,PWMB2=7200+motor_b;
			else 	            PWMB2=7200,PWMB1=7200-motor_b;
     SERVO=servo;	
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=3000;    //===PWM满幅是7200 限制在6900
    if(Motor_A<-Amplitude) Motor_A=-Amplitude;	
		if(Motor_A>Amplitude)  Motor_A=Amplitude;	
	  if(Motor_B<-Amplitude) Motor_B=-Amplitude;	
		if(Motor_B>Amplitude)  Motor_B=Amplitude;		
		if(Servo<(SERVO_INIT-650))     Servo=SERVO_INIT-650;	  //舵机限幅//1000-1650-2500
		if(Servo>(SERVO_INIT+850))     Servo=SERVO_INIT+850;		  //舵机限幅
}
/**************************************************************************
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp,tmp2;
	tmp=click_N_Double(40); 
	if(tmp==2)Flag_Stop=!Flag_Stop;//双击控制小车的启停
	if(tmp==1&&Flag_Way==2)
	{
		if(CCD_YuzhiBias < 6)
		{
			CCD_YuzhiBias ++;
		}
		else
		{
			CCD_YuzhiBias = 0;
		}
	}//单击增加偏置
	tmp2=Long_Press();          
  if(tmp2==1)Flag_Show=!Flag_Show;//控制小车的显示状态
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<740||Flag_Stop==1)//电池电压低于7.4V关闭电机
			{	                                                
				temp=1;                                            
      }
			else
				temp=0;
      return temp;			
}
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 Last_bias=Bias;	                   //保存上一次偏差 
	 return Pwm;                         //增量输出
}
/**************************************************************************
函数功能：遥控和CCD模式的速度设定和角度计算
入口参数：无
返回  值：无
**************************************************************************/
#define ANGLE_P 0.015f//0.010f
#define ANGLE_D 0.085f//0.085f 0.020f
void Get_RC(void)
{
	static float Bias,Last_Bias;
	if(Flag_Way==0)//蓝牙控制
	{
		if(Flag_Direction==0) Velocity=0,Angle=0;   //停止
		else if(Flag_Direction==1) Velocity=Bluetooth_Velocity,Angle=0;  //前进
		else if(Flag_Direction==2) Velocity=Bluetooth_Velocity,Angle=PI/5;  //右前
		else if(Flag_Direction==3) Velocity=0,Angle=0;   //舵机向右
		else if(Flag_Direction==4) Velocity=-Bluetooth_Velocity,Angle=PI/5;  // 右后
		else if(Flag_Direction==5) Velocity=-Bluetooth_Velocity,Angle=0;    //后退
		else if(Flag_Direction==6) Velocity=-Bluetooth_Velocity,Angle=-PI/5;  //左后
		else if(Flag_Direction==7) Velocity=0,Angle=0;                       //舵机向左
		else if(Flag_Direction==8) Velocity=Bluetooth_Velocity,Angle=-PI/5;  //左前
	}
	else	if(Flag_Way==2)//CCD巡线
	{
		 Velocity=15;	   //CCD巡线模式的速度//40
		 Bias=CCD_Zhongzhi-64;   //提取偏差
		 Angle=Bias*ANGLE_P+(Bias-Last_Bias)*ANGLE_D; //PD控制
		 Last_Bias=Bias;   //保存上一次的偏差
	}
}
/**************************************************************************
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/

void  Find_CCD_Zhongzhi(void)
{ 
	 static u16 i,j,Left,Right,Last_CCD_Zhongzhi;
	 static u16 value1_max,value1_min;
   u32 temp = 0;
	 UsefulPoint = 0;
	 UsefulPoint2 = 0;
	
	   value1_max=ADV[0];  //动态阈值算法，读取最大和最小值
     for(i=5;i<123;i++)   //两边各去掉5个点
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //最小值
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }

	   for(i = 5;i<118; i++)	//计算电压平均值
	   {
				temp += ADV[i];
	   }	
		 
		 CCD_Yuzhi = 	temp/113 - CCD_YuzhiBias;
		 
	for(i = 5;i<118; i++)   //寻找左边跳变沿
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi
					&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}

	}
	 for(j = 118;j>5; j--)//寻找右边跳变沿
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi
				&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
		////////////黑点数量计数/////////////
		if(ADV[j]<=CCD_Yuzhi)//6~118
		{
			UsefulPoint2 ++;
		}
		if((j>=80) && (ADV[j]<CCD_Yuzhi))//80~118
		{
			UsefulPoint++;
		}
		///////////黑点数量计数结束///////////////
  }
	
	CCD_Zhongzhi=(Right+Left)/2;//计算中线位置
	
	///////////直角弯探测//////////////////
	if((UsefulPoint > PointDifference) 
										&& (ActionIndex == 0) && (++RightAngleJudgeCount==RightAngleJudgeCountPre) 
												&& (Flag_Stop == 0))//直角弯检测
	{
			RightAngleFlag = 1;//方向转死标志位
			RightAngleTimeCount = RightAngleTimeCountPre;//方向转死延时赋值
			ActionFlag = 1;
			AcuteAngleTimeCount = AcuteAngleTimeCountPre;//方向转死延时赋值
	}
	//////////直角弯探测结束///////////////////////
	//////////循迹结束探测/////////////////////////
	if(ActionIndex == 2)
	{
		if(UsefulPoint < PointDifference2){
			if(++StopJudgeCount > StopJudgeCountPre)//多次判断
			{
				ActionFlag = 1;//摆脱CCD数据控制
				ActionIndex = 3;//进入循迹结束后的操作
			}
		}
		else
		{
			StopJudgeCount = 0;//判断中断则判断有效次数清零
		}
	}
	///////////循迹结束探测结束////////////////////
	
//if((Right-Left < 100)&&RightAngleFlag) RightAngleFlag = 0;//结束直角
		
	
//	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //计算中线的偏差，如果太大
//	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //则取上一次的值
//	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //保存上一次的偏差

}
