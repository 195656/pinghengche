#ifndef __MOTOR_H__
#define __MOTOR_H__


#include "stm32f1xx_hal.h"
void Load(int moto1,int moto2);
void Limit(int *moto1,int *moto2);
#endif
