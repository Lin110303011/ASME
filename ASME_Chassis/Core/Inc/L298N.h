#include "stm32f4xx_hal.h"
#include <stdint.h>

uint16_t Speed_setting(TIM_HandleTypeDef htim, float pid ,float last_set);

void stop(TIM_HandleTypeDef htim);

