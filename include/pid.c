#include "pid.h"

void PID_Init(PID_InfoTypeDef* pid)
{
    /* Clear controller variables */
    pid->fIntegrator = 0.0f;
    pid->fPrevError  = 0.0f;
    pid->fDifferentiator  = 0.0f;
    pid->fPrevMeasurement = 0.0f;
    pid->fOutput = 0.0f;
}

double PID_Calculate(PID_InfoTypeDef* pid, float setpoint, float measurement, float sampleTime) 
{
    /*
    * Error signal
    */
    float error = setpoint - measurement;
    /*
    * Proportional
    */
    float proportional = pid->fKp * error;
    /*
    * Integral
    */
    pid->fIntegrator = pid->fIntegrator + 0.5f * pid->fKi * sampleTime * (error + pid->fPrevError);
    /* Anti-wind-up via integrator clamping */
    if (pid->fIntegrator > pid->fIntegratorLimitMax) 
    {
        pid->fIntegrator = pid->fIntegratorLimitMax;
    }
    else if (pid->fIntegrator < pid->fIntegratorLimitMin) 
    {
        pid->fIntegrator = pid->fIntegratorLimitMin;
    }
    /*
    * Derivative (band-limited differentiator)
    */
    pid->fDifferentiator = -(2.0f * pid->fKd * (measurement - pid->fPrevMeasurement)    /* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->fTau - sampleTime) * pid->fDifferentiator)
                        / (2.0f * pid->fTau + sampleTime);
    /*
    * Compute output and apply limits
    */
    pid->fOutput = proportional + pid->fIntegrator + pid->fDifferentiator;
    if (pid->fOutput > pid->fOutputLimitMax) 
    {
        pid->fOutput = pid->fOutputLimitMax;
    }
    else if (pid->fOutput < pid->fOutputLimitMin) 
    {
        pid->fOutput = pid->fOutputLimitMin;
    }
    /* Store error and measurement for later use */
    pid->fPrevError       = error;
    pid->fPrevMeasurement = measurement;
    /* Return controller output */
    return pid->fOutput;
}
