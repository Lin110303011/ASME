# L298N 控制程式


## :small_blue_diamond:*L298N接口*
### 硬體簡述

## :small_blue_diamond:*L298N控制*
### 實作
#### 以STM32f103微控制器輸入PWM波形(定速)
```c=
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
```
#### 以STM32f103微控制器輸入PWM波形(以PID控制)
```c=
uint16_t r_speed_setting(TIM_HandleTypeDef htim, float pid ,float last_set)
{

    //setting PWM mode
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    //setting Duty Ratio
    sConfigOC.Pulse = last_set+ pid;
    last_set = sConfigOC.Pulse;
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_3); // enable timer3 PWM
    return last_set;

}
```
