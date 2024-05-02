#pragma once
#include <stdint.h>

typedef struct {
    /* Controller gains */
    float fKp;
    float fKi;
    float fKd;
    /* Derivative low-pass filter time constant */
    float fTau;
    /* Output limits */
    float fOutputLimitMin;
    float fOutputLimitMax;
    /* Integrator limits */
    float fIntegratorLimitMin;
    float fIntegratorLimitMax;
    /* Controller "memory" */
    float fIntegrator;
    float fPrevError;            /* Required for integrator */
    float fDifferentiator;
    float fPrevMeasurement;        /* Required for differentiator */
    /* Controller output */
    float fOutput;
} PID_InfoTypeDef;

void PID_Init(PID_InfoTypeDef* pid);
double PID_Calculate(PID_InfoTypeDef* pid, float setpoint, float measurement, float sampleTime); //sample time in seconds
