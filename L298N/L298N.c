#include "L298N.h"
#include "stm32f1xx_hal.h"

uint32_t check1;
void forward(TIM_HandleTypeDef htim)
{

    uint32_t ARR =  htim.Init.Period +1;
	TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    check1 = ARR * 1.35/20;

    sConfigOC.Pulse = check1;
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_2);
        HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_2); // enable timer3 PWM
        HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_3); // enable timer3 PWM

}

void backward(TIM_HandleTypeDef htim)
{
	uint32_t ARR =  htim.Init.Period +1;
	TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    check1 =ARR * 1.41/20;
    sConfigOC.Pulse = check1;
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_2);
        HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_3);
       HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_2); // enable timer3 PWM
        HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_3); // enable timer3 PWM
}

uint16_t r_speed_setting(TIM_HandleTypeDef htim, float pid ,float last_set)
{
    uint32_t ARR =  htim.Init.Period +1;
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = last_set+ pid;
    last_set = sConfigOC.Pulse;
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_3); // enable timer3 PWM
    return last_set;

}

uint16_t l_speed_setting(TIM_HandleTypeDef htim, float pid ,float last_set)
{
    uint32_t ARR =  htim.Init.Period +1;
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = last_set+ pid;
    last_set = sConfigOC.Pulse;
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_2); // enable timer3 PWM
    return last_set;

}

void stop(TIM_HandleTypeDef htim)
{
    uint32_t ARR =  htim.Init.Period +1;
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    check1 = ARR * 1.35/20;
    sConfigOC.Pulse = check1;
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_2); // enable timer3 PWM
}
