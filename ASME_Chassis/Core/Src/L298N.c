#include "L298N.h"
/**
  * @brief  Set the speed of the motor
  * @param  htim: TIM_HandleTypeDef
  * @param  pid: float
  * @param  last_set: float
  * @retval uint16_t
*/

uint16_t Speed_setting(TIM_HandleTypeDef htim, float pid ,float last_set)
{
    //uint32_t ARR =  htim.Init.Period +1;
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = last_set+ pid;
    last_set = sConfigOC.Pulse;
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1); // enable timer2 PWM
    return last_set;

}// End of Speed_setting


/**
  * @brief  Stop the motor
  * @param  htim: TIM_HandleTypeDef
  * @retval None
*/
void stop(TIM_HandleTypeDef htim)
{
    uint32_t ARR =  htim.Init.Period +1;
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = ARR * 1.35/20;
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1); // enable timer3 PWM
}
