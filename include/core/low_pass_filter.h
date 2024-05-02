#pragma once
#include <stdint.h>
#include <stdbool.h>
// f0: cutoff frequency (Hz)
// fs: sample frequency (Hz)
// adaptive: boolean flag, if set to 1, the code will automatically set
// the sample frequency based on the time history.
void LowPassFilter_Init(float f0, float fs, bool adaptive);
float LowPassFilter_Filt(float xn);