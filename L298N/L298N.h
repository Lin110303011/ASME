#include "stm32f1xx_hal.h"
void forward(TIM_HandleTypeDef htim);
void backward(TIM_HandleTypeDef htim);
uint16_t r_speed_setting(TIM_HandleTypeDef htim, float pid ,float last_set);

uint16_t l_speed_setting(TIM_HandleTypeDef htim, float pid ,float last_set);

void stop(TIM_HandleTypeDef htim);

