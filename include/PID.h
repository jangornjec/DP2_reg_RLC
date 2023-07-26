/*************************************
 * PID.h
 *
 *  Created on: 26 Oct 2022
 *      Author: Jan Gornjec
 *
 *
 *************************************/

#ifndef INCLUDE_PID_H_
#define INCLUDE_PID_H_


// PID structure
typedef struct {

    // Gains
    float Kp;
    float Ki;
    float Kd;

    // Derivative low-pass filter time constant
    float tau;

    // Sample time (seconds)
    float T;

    // Output limits
    float limMin;
    float limMax;

    // Persistent data
    float integrator;
    float prevError;
    float differentiator;
    float prevMeasurement;

    // Output
    float out;

} PIDController;


// function declaration
void PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);



#endif /* INCLUDE_PID_H_ */
