#include "pid.h"
#include "encoder.h"
# include "IIC.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"

//直立环变量 Kp：0-1000  Kd:0-10
float Vertical_Kp=70,Vertical_Kd=1; 
//速度环变量 Kp:0-1
float Velocity_Kp=0,Velocity_Ki;
uint8_t stop;
//转向环变量
float Turn_Kp,Turn_Kd;
// 编码器 
int Encoder_left,Encoder_right;
extern TIM_HandleTypeDef htim2,htim4;
// 陀螺仪
float pitch,roll,yaw;
short gyrox,gyroy,gyroz;//角速度
short aacx,aacy,aacz;//角加速度
//PID控制中间变量
int Vertical_out,Velocity_out,Turn_out,Target_Speed,Target_Turn;
float Med_angle= 13 ;//平衡时重心角度偏移量——恒定值
int MOTO1,MOTO2;



// 直立闭环PD控制器,用于控制角度稳定
// 参数 MED:期望的角度 Angle：真实的角度 gy：角速度值（积分后变为角度）
int Vertical(float Med,float Angle,float gyro_Y)
{
	int temp;//加载给电机的值
	temp = Vertical_Kp*(Med-Angle)+Vertical_Kd*gyro_Y;
	return temp;
}

// 速度PI控制器,用于控制速度稳定
// 参数 MED:期望的速度 左编码器速度，右编码器速度
int Velocity(int Target,int encoder_L,int enconder_R)
{
	//计算偏差
	int Err,Err_LowOut;
	Err = encoder_L+enconder_R-Target;
	//软件滤波
	static int Err_LowOut_last;
	static float a = 0.7;
Velocity_Ki = Velocity_Kp/200;
	Err_LowOut = (1-a)*Err+a*Err_LowOut_last;
	Err_LowOut_last=Err_LowOut;
	//编码器积分
	static int Encoder_S;//保存编码器积分
	Encoder_S += Err_LowOut_last;
	//积分限幅
	int Integration_Limit = 10000; 
    if (Encoder_S > Integration_Limit)  Encoder_S = Integration_Limit;
    if (Encoder_S < -Integration_Limit) Encoder_S = -Integration_Limit;
	if(stop==1)Encoder_S=0,stop=0;//停止的时候去除积分
	//速度环计算
	int temp;
	temp = Velocity_Kp*Err_LowOut + Velocity_Ki*Encoder_S;
	return temp;
}
// 转向PD控制器,用于控制速度稳定
// 参数 角速度 角度值
int Turn(float gyro_Z,int Target_turn)
{
	int temp;//加载给电机的值
	temp = Turn_Kp*Target_turn+Turn_Kd*gyro_Z;
	return temp;
}

void Control(void)
{
	int PWM_out;//控制转速

	//读取传感器的值
	// 1 读取编码器
	Encoder_left = Read_Speed(&htim2);
	Encoder_right = -Read_Speed(&htim4);
	//读取陀螺仪数据
	mpu_dmp_get_data(&pitch,&roll,&yaw);
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);

	//传入PID，得到左右电机转速
	Velocity_out = Velocity(Target_Speed,Encoder_left,Encoder_right);
	Vertical_out = Vertical(Velocity_out+Med_angle,roll,gyrox);
	Turn_out = Turn(gyroz,Target_Turn);

	PWM_out = Vertical_out;//前进后退的速度
	MOTO1 = PWM_out - Turn_out;
	MOTO2 = PWM_out + Turn_out;
	//限幅操作
	Limit(&MOTO1,&MOTO2);
	Load(MOTO1,MOTO2);

	
}