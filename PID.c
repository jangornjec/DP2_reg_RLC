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
    float tempIntegrator = pid->Ki * pid->T * error;

    // Derivative (filtered)
    if (pid->Kp == 0.0f){

        pid->differentiator = 0.0f;

    } else {

        pid->differentiator = (2.0f * pid->Kd * (error - pid->prevError)
                               + (2.0f * pid->tau - pid->T) * pid->differentiator)
                               / (2.0f * pid->tau + pid->T);
    }


    // Compute temporary output
    pid->out = proportional + (pid->integrator + tempIntegrator) + pid->differentiator;

    // Check controller limits, set integrator in this iteration to 0
    if (pid->out > pid->limMax || pid->out < pid->limMin) {

        tempIntegrator = 0.0f;

    }

    // New integral
    pid->integrator += tempIntegrator;


    // Compute final output
    pid->out = proportional + pid->integrator + pid->differentiator;


    // Clamp output
    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }


    // Store error
    pid->prevError = error;

    // Return controller output
    return pid->out;
}



