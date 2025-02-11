# PID Controller
## PID Controller 設計

## PID Controller 實作

### 函式設計
以STM32 讀取輪速
```c=

float calculate_wheel_speed(char wheel_choose, int16_t time_scale_ms)
{

  // get the wheel interrupt
  int16_t wheel_interrupt;
  float wheel_speed;
  char right;
  right = 'r';

  if(wheel_choose == right)
  {

    wheel_interrupt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);  // Get right wheel encoder value

    //clear the counter
    __HAL_TIM_SET_COUNTER(&htim3, 0);
  }

  else
  {
    wheel_interrupt = (int16_t) __HAL_TIM_GET_COUNTER(&htim1);  // Get left wheel encoder value
    //clear the counter
    __HAL_TIM_SET_COUNTER(&htim1, 0);
  }

  // calculate wheel speed(rps)
  wheel_speed = (wheel_interrupt / 24)/(time_scale_ms/1000);
  return wheel_speed;

}// end calculate_wheel_speed
```
PID 控制器實作
```c=
float PID(float expected_rps, float real_rps, float Kp, float Ki, float Kd, float error, float PID_output)
{

	error = expected_rps - real_rps;

	// Proportional
	proportional = error;
	// Integral
	integral = (integral * 0.9 + error) * 0.1;
	// Derivative
	derivative = (error - previous_error) / 0.1;
	previous_error = error;

	PID_output = Kp * proportional + Ki * integral + Kd * derivative;
	return PID_output;
}// end PID
```