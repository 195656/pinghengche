#include "motor.h"
#define PWM_MAX 7200
#define PWM_MIN -7200
extern TIM_HandleTypeDef htim1;

int abs(int p)
{
	if(p>0)
	{
		return p;
	}
	else
		return -p;
}
void Load(int moto1,int moto2)
{
	if(moto1<0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,abs(moto1));
	
	
	if(moto2<0)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,abs(moto2));
	
}

void Limit(int *moto1,int *moto2)
{
	// 对第一个电机进行限幅
    if (*moto1 > PWM_MAX)       *moto1 = PWM_MAX;
    else if (*moto1 < PWM_MIN)  *moto1 = PWM_MIN;

    // 对第二个电机进行限幅
    if (*moto2 > PWM_MAX)       *moto2 = PWM_MAX;
    else if (*moto2 < PWM_MIN)  *moto2 =PWM_MIN;
}