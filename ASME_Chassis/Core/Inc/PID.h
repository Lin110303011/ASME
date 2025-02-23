#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief PID controller structure.
 *
 * This structure holds the PID gains and internal state necessary
 * for the PID computation.
 */
typedef struct {
    float Kp;         /** Proportional gain */
    float Ki;         /** Integral gain     */
    float Kd;         /** Derivative gain   */
    
    float prevError;  /** Previous error (for derivative term) */
    float integral;   /** Accumulated integral term            */
    
    float outMin;     /** Minimum allowed output (saturation) */
    float outMax;     /** Maximum allowed output (saturation) */
} PIDController_t;

/**
 * @brief Initializes the PID controller structure.
 *
 * @param pid       Pointer to a PIDController_t structure
 * @param Kp        Proportional gain
 * @param Ki        Integral gain
 * @param Kd        Derivative gain
 * @param outMin    Minimum output 
 * @param outMax    Maximum output
 */
void PID_Init(PIDController_t *pid, float Kp, float Ki, float Kd, float outMin, float outMax);

/**
 * @brief Performs a PID computation and returns the control output.
 *
 * @param pid         Pointer to a PIDController_t structure
 * @param setpoint    Desired target value 
 * @param measurement Current measured value 
 * @param dt          Time interval since the last update
 * @return            The PID output
 */
float PID_Compute(PIDController_t *pid, float setpoint, float measurement, float dt);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
