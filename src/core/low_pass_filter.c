#include <math.h>
#include "low_pass_filter.h"
#include <stm32f1xx_hal.h>

#define ORDER 2

float a[ORDER];
float b[ORDER+1];
float omega0;
float dt;
bool adapt;
float tn1 = 0;
float x[ORDER+1]; // Raw values
float y[ORDER+1]; // Filtered values

extern uint32_t micros(void);

static void LowPassFilter_SetCoef()
{
    if(adapt)
    {
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
    }
    
    float alpha = omega0*dt;
    if(ORDER==1)
    {
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
    }
    if(ORDER==2)
    {
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
    }
}

void LowPassFilter_Init(float f0, float fs, bool adaptive)
{
    omega0 = 6.28318530718*f0;
    dt = 1.0/fs;
    adapt = adaptive;
    tn1 = -dt;
    for(int k = 0; k < ORDER+1; k++) {
        x[k] = 0;
        y[k] = 0;        
    }
    LowPassFilter_SetCoef();
}

float LowPassFilter_Filt(float xn)
{
    // Provide me with the current raw value: x
    // I will give you the current filtered value: y
    if(adapt) 
    {
        LowPassFilter_SetCoef(); // Update coefficients if necessary      
    }
    y[0] = 0;
    x[0] = xn;
    // Compute the filtered values
    for(int k = 0; k < ORDER; k++) 
    {
        y[0] += a[k]*y[k+1] + b[k]*x[k];
    }
    y[0] += b[ORDER]*x[ORDER];
    // Save the historical values
    for(int k = ORDER; k > 0; k--) 
    {
        y[k] = y[k-1];
        x[k] = x[k-1];
    }
    // Return the filtered value    
    return y[0];
}