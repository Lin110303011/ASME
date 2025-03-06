#include "L298N.h"
/**
  * @brief  Set the speed of the motor
  * @param  htim: TIM_HandleTypeDef
  * @param  pid: float
  * @param  last_set: float
  * @retval uint16_t
*/

uint16_t Speed_setting(TIM_HandleTypeDef htim, float duty_ratio ,float last_set)
{
    uint32_t ARR =  htim.Init.Period +1;
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = ARR * duty_ratio / 100;
    last_set = sConfigOC.Pulse;

    // Enable PWM Left wheel
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1); // enable timer2 PWM
    // Enable PWM Right wheel
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_2); // enable timer2 PWM

    // Speed Equalization
    int left_wheel_speed = TIM3->CNT;
    int right_wheel_speed = TIM4->CNT;

    while (left_wheel_speed != right_wheel_speed)
    {
        if (left_wheel_speed > right_wheel_speed)
        {
            // Increase the right wheel speed
            float rignt_wheel_duty_ratio = duty_ratio + 1;
            sConfigOC.Pulse = ARR * rignt_wheel_duty_ratio / 100;

            // Enable PWM Right wheel
            HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_2);
          
        }
        else
        {
            // Increase the left wheel speed
            float left_wheel_duty_ratio = duty_ratio + 1;
            sConfigOC.Pulse = ARR * left_wheel_duty_ratio / 100;

            // Enable PWM Left wheel
            HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1);
        }

        // Refresh wheel speed
        left_wheel_speed = TIM3->CNT;
        right_wheel_speed = TIM4->CNT;
    }// End of speed equalization

    return last_set;

}// End of Speed_setting


/**
  * @brief  Stop the motor
  * @param  htim: TIM_HandleTypeDef
  * @retval None
*/
void stop(TIM_HandleTypeDef htim)
{
    //uint32_t ARR =  htim.Init.Period +1;
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1); // enable timer3 PWM
}

void PWM_test(TIM_HandleTypeDef htim)
{
    uint32_t ARR =  htim.Init.Period +1;
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = ARR * 60/100;
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1); // enable timer3 PWM
}