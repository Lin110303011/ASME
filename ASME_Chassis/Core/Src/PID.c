#include "pid.h"

void PID_Init(PIDController_t *pid, float Kp, float Ki, float Kd, float outMin, float outMax)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->outMin = outMin;
    pid->outMax = outMax;

    // Reset internal state
    pid->prevError = 0.0f;
    pid->integral  = 0.0f;
}

float PID_Compute(PIDController_t *pid, float setpoint, float measurement, float dt)
{
    // Calculate current error
    float error = setpoint - measurement;

    // Update integral term
    pid->integral += (error * dt);

    // Calculate derivative term
    float derivative = 0.0f;
    if (dt > 1e-6f) { // Avoid division by zero or extremely small dt
        derivative = (error - pid->prevError) / dt;
    }

    // Compute PID output
    float output = (pid->Kp * error)
                 + (pid->Ki * pid->integral)
                 + (pid->Kd * derivative);

    // Save current error as previous for next iteration
    pid->prevError = error;

    // Apply output saturation
    if (output > pid->outMax) {
        output = pid->outMax;
        // Optional anti-windup: adjust integral if needed
        // pid->integral -= error * dt;
    } else if (output < pid->outMin) {
        output = pid->outMin;
        // Optional anti-windup: adjust integral if needed
        // pid->integral -= error * dt;
    }

    return output;
}
