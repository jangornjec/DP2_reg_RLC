/*****************************************************************
 * PID.c
 *
 *  Created on: 26 Oct 2022
 *      Author: Jan Gornjec
 *
 *****************************************************************/


#include    "PID.h"


/*****************************************************************
* PID Initialise function
******************************************************************/

void PIDController_Init(PIDController *pid) {

    // Clear persistent data
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;

    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;
}




/*****************************************************************
 * PID update function
 * returns PID output
 *****************************************************************/

float PIDController_Update(PIDController *pid, float setpoint, float measurement){

    // Error signal
    float error = setpoint - measurement;

    // Proportional
    float proportional = pid->Kp * error;


    // Integral
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);


    // Derivative (filtered)
    if (pid->Kp == 0.0f){

        pid->differentiator = 0.0f;

    } else {

        pid->differentiator = (2.0f * pid->Kd * (error - pid->prevError)
                               + (2.0f * pid->tau - pid->T) * pid->differentiator)
                               / (2.0f * pid->tau + pid->T);
    }


    // Compute output
    pid->out = proportional + pid->integrator + pid->differentiator;

    // Saturation, anti-wind-up
    if (pid->out > pid->limMax) {

        pid->integrator = 0.0f;
        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->integrator = 0.0f;
        pid->out = pid->limMin;

    }

    // Store error and measurement
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    // Return controller output
    return pid->out;
}



